/*
 * drvTimRx.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Jan. 18, 2018
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include "drvTimRx.h"
#include <epicsExport.h>

#define SERVICE_NAME_SIZE               50

static const boardMap_t boardMap[MAX_TIM_RXS+1] = {
         /* board, timRx*/
    /* 0 (INVALID)  */ {-1, -1},
    /* 1            */ {1,   0},
    /* 2            */ {1,   1},
    /* 3            */ {2,   0},
    /* 4            */ {2,   1},
    /* 5            */ {3,   0},
    /* 6            */ {3,   1},
    /* 7            */ {4,   0},
    /* 8            */ {4,   1},
    /* 9            */ {5,   0},
    /* 10           */ {5,   1},
    /* 11           */ {6,   0},
    /* 12           */ {6,   1},
    /* 13           */ {7,   0},
    /* 14           */ {7,   1},
    /* 15           */ {8,   0},
    /* 16           */ {8,   1},
    /* 17           */ {9,   0},
    /* 18           */ {9,   1},
    /* 19           */ {10,  0},
    /* 20           */ {10,  1},
    /* 21           */ {11,  0},
    /* 22           */ {11,  1},
    /* 23           */ {12,  0},
    /* 24           */ {12,  1}
};

/* Double functions mapping */
static const functionsAny_t timRxSetGetLinkStatusFunc = {functionsInt32_t{"LNLS_AFC_TIMING", NULL, afc_timing_get_link_status}};
static const functionsAny_t timRxRxEnStatusFunc =       {functionsInt32_t{"LNLS_AFC_TIMING", NULL, afc_timing_get_rxen_status}};
static const functionsAny_t timRxRefClkLockedFunc =     {functionsInt32_t{"LNLS_AFC_TIMING", NULL, afc_timing_get_ref_clk_locked}};

static const char *driverName="drvTimRx";
void acqTask(void *drvPvt);

static void exitHandlerC(void *pPvt)
{
    drvTimRx *pdrvTimRx = (drvTimRx *)pPvt;
    pdrvTimRx->~drvTimRx();
}

asynStatus drvTimRx::getServiceID (int timRxNumber, int addr, const char *serviceName,
        int *serviceIDArg) const
{
    static const char *functionName = "getServiceID";
    asynStatus status = asynSuccess;

    *serviceIDArg = boardMap[timRxNumber].timRx;

    return status;
}

asynStatus drvTimRx::getFullServiceName (int timRxNumber, int addr, const char *serviceName,
        char *fullServiceName, int fullServiceNameSize) const
{
    static const char *functionName = "getFullServiceName";
    int coreID = 0;
    int errs = 0;
    asynStatus status = asynSuccess;

    status = getServiceID (timRxNumber, addr, serviceName, &coreID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getServiceID, status=%d\n",
            driverName, functionName, status);
        goto get_service_id_err;
    }

    errs = snprintf(fullServiceName, fullServiceNameSize, "HALCS%d:DEVIO:%s%d",
            boardMap[timRxNumber].board, serviceName, coreID);
    if (errs < 0 || errs >= fullServiceNameSize){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error generating fullServiceName, errs=%d\n",
            driverName, functionName, errs);
        status = asynError;
        goto gen_full_service_name;
    }

gen_full_service_name:
get_service_id_err:
    return status;
}

/** Constructor for the drvTimRx class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] endpoint The device address string ]
 * */
drvTimRx::drvTimRx(const char *portName, const char *endpoint, int timRxNumber,
        int verbose, int timeout)
   : asynPortDriver(portName,
                    MAX_ADDR, /* maxAddr */
                    (int)NUM_PARAMS,
                    asynUInt32DigitalMask | asynFloat64Mask  | asynDrvUserMask,    /* Interface mask     */
                    asynUInt32DigitalMask | asynFloat64Mask ,                      /* Interrupt mask     */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver blocks it is multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    asynStatus status;
    const char *functionName = "drvTimRx";

    /* Create portName so we can create a new AsynUser later */
    timRxPortName = epicsStrDup(portName);

    this->endpoint = strdup(endpoint);
    if (this->endpoint == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvTimRx failure to copy endpoint\n",
                driverName, functionName);
        status = asynError;
        goto endpoint_dup_err;
    }

    if (timRxNumber < TIM_RX_NUMBER_MIN || timRxNumber > TIM_RX_NUMBER_MAX) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvTimRx invalid timRxNumber\n",
                driverName, functionName);
        status = asynError;
        goto invalid_timRx_number_err;
    }

    this->timRxNumber = timRxNumber;
    this->verbose = verbose;
    this->timeout = timeout;

    /* Create parameters */
    createParam(P_TimRxLinkStatusString, asynParamUInt32Digital,         &P_TimRxLinkStatus);
    createParam(P_TimRxRxEnStatusString, asynParamUInt32Digital,         &P_TimRxRxEnStatus);
    createParam(P_TimRxRefClkLockedString,
                                         asynParamUInt32Digital,         &P_TimRxRefClkLocked);

    /* Set the initial values of some parameters */
    setUIntDigitalParam(P_TimRxLinkStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRxEnStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRefClkLocked, 0, 0xFFFFFFFF);

    /* Do callbacks so higher layers see any changes. Call callbacks for every addr */
    for (int i = 0; i < MAX_ADDR; ++i) {
        callParamCallbacks(i);
    }

    /* TimRx Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    timRxHwFunc.emplace(P_TimRxLinkStatus,   timRxSetGetLinkStatusFunc);
    timRxHwFunc.emplace(P_TimRxRxEnStatus,   timRxRxEnStatusFunc);
    timRxHwFunc.emplace(P_TimRxRefClkLocked, timRxRefClkLockedFunc);

    lock();
    status = timRxClientConnect();
    unlock();

    /* If we correct connect for this first time, liclient
     * will ensure the reconnection to server if necessary, but we
     * must succeed here or we must abort completely */
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling timRxClientConnect, status=%d\n",
            driverName, functionName, status);
        exit(1);
    }

    epicsAtExit(exitHandlerC, this);

invalid_timRx_number_err:
    free (this->endpoint);
endpoint_dup_err:
    return;
}

/** Destructor for the drvTimRx class.
 */
drvTimRx::~drvTimRx()
{
    asynStatus status = asynSuccess;
    const char *functionName = "~drvTimRx";

    lock();
    status = timRxClientDisconnect();
    unlock();
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling timRxClientDisconnect, status=%d\n",
            driverName, functionName, status);
    }

    free (this->endpoint);
    this->endpoint = NULL;
    free (this->timRxPortName);
    this->timRxPortName = NULL;
}

asynStatus drvTimRx::connect(asynUser* pasynUser)
{
    return timRxClientConnect();
}

asynStatus drvTimRx::timRxClientConnect(void)
{
    asynStatus status = asynSuccess;
    const char *timRxLogFile = "stdout";
    const char *functionName = "timRxClientConnect";

    /* Connect TimRx */
    if (timRxClient == NULL) {
        timRxClient = halcs_client_new_time (endpoint, verbose, timRxLogFile, timeout);
        if (timRxClient == NULL) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s timRxClientConnect failure to create timRxClient instance\n",
                    driverName, functionName);
            status = asynError;
            goto create_halcs_client_err;
        }
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Tim Rx client connected\n",
        driverName, functionName);

    pasynManager->exceptionConnect(this->pasynUserSelf);

    return status;

create_halcs_client_err:
    return status;
}

asynStatus drvTimRx::disconnect(asynUser* pasynUser)
{
    return timRxClientDisconnect();
}

asynStatus drvTimRx::timRxClientDisconnect(void)
{
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s: calling timRxClientDisconnect\n",
            driverName);
    asynStatus status = asynSuccess;

    if (timRxClient != NULL) {
        halcs_client_destroy (&timRxClient);
    }

    pasynManager->exceptionDisconnect(this->pasynUserSelf);
    return status;
}

/********************************************************************/
/********************* Asyn overrided methods  **********************/
/********************************************************************/

/*
 * Asyn overrided methods that are called by higher layers
 */

/** Called when asyn clients call pasynUInt32Digital->write().
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus drvTimRx::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "writeUInt32Digital";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
        return status;
    }
    /* Set the parameter in the parameter library. */
    setUIntDigitalParam(function, value, mask);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParam32(function, mask, addr);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s, value=%d\n",
                driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynUInt32Digital->read().
 * For all parameters it gets the value in the parameter library..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[out] value Value to read. */
asynStatus drvTimRx::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *functionName = "readUInt32Digital";
    const char *paramName;

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get parameter, possibly from HW */
    status = getParam32(function, value, mask, addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);

    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvTimRx::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "writeFloat64";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%f",
                driverName, functionName, status, function, paramName, value);
        return status;
    }
    /* Set the parameter in the parameter library. */
    setDoubleParam(addr, function, value);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParamDouble(function, addr);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%f",
                driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s, value=%f\n",
                driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvTimRx::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "readFloat64";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get double param, possibly from HW */
    status = getParamDouble(function, value, addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);
    return status;
}

/********************************************************************/
/************ Function Mapping Overloaded Write functions ***********/
/********************************************************************/

asynStatus drvTimRx::doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(timRxClient, service, functionParam.argFloat64);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %f\n",
                driverName, functionName, service, functionParam.argFloat64);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
    return (asynStatus) status;
}

asynStatus drvTimRx::doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(timRxClient, service, functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, service, functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::executeHwWriteFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwWriteFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = timRxHwFunc.find (functionId);
    if (func == timRxHwFunc.end()) {
        /* This is not an error. Exit silently */
        status = asynSuccess;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d\n",
                driverName, functionName, functionId);
        goto get_reg_func_err;
    }

    /* Get service name from structure */
    funcService = func->second.getServiceName(*this);
    /* Create full service name*/
    status = getFullServiceName (this->timRxNumber, addr, funcService,
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Execute overloaded function for each function type we know of */
    status = func->second.executeHwWrite(*this, service, addr, functionParam);

get_reg_func_err:
get_service_err:
        return (asynStatus)status;
}

/********************************************************************/
/************ Function Mapping Overloaded Read functions ************/
/********************************************************************/

asynStatus drvTimRx::doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(timRxClient, service, &functionParam.argFloat64);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
    return (asynStatus) status;
}

asynStatus drvTimRx::doExecuteHwReadFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(timRxClient, service, &functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::executeHwReadFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwReadFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = timRxHwFunc.find (functionId);
    if (func == timRxHwFunc.end()) {
        /* We use disabled to indicate the function was not found on Hw mapping */
        status = asynDisabled;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d\n",
                driverName, functionName, functionId);
        goto get_reg_func_err;
    }

    /* Get service name from structure */
    funcService = func->second.getServiceName(*this);
    /* Create full service name*/
    status = getFullServiceName (this->timRxNumber, addr, funcService,
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Execute overloaded function for each function type we know of */
    status = func->second.executeHwRead(*this, service, addr, functionParam);

get_reg_func_err:
get_service_err:
        return (asynStatus)status;
}

/********************************************************************/
/*************** Generic 32-bit/Double Tim Rx Operations ***************/
/********************************************************************/

/*
* 32-bit/Double generic Tim Rx operations. These will map to real
* functions defined in the structures. e.g., functionsInt32_t
* and functionsFloat64_t
*/

asynStatus drvTimRx::setParam32(int functionId, epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParam32";

    status = getUIntDigitalParam(addr, functionId, &functionArgs.argUInt32, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

get_param_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::getParam32(int functionId, epicsUInt32 *param,
        epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParam32";

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(addr, functionId, param, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);
    if (status == asynSuccess) {
        /* Mask parameter according to the received mask */
        functionArgs.argUInt32 &= mask;
        *param = functionArgs.argUInt32;
    }
    /* We recover from asynDisabled just by retrieving
     * the parameter from the list */
    else if (status == asynDisabled){
        status = asynSuccess;
    }

get_param_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::setParamDouble(int functionId, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParamDouble";

    status = getDoubleParam(addr, functionId, &functionArgs.argFloat64);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: setParamDouble failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

get_param_err:
    return status;
}

asynStatus drvTimRx::getParamDouble(int functionId, epicsFloat64 *param, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParamDouble";

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(addr, functionId, param);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getParamDouble failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);
    if (status == asynSuccess) {
        *param = functionArgs.argFloat64;
    }
    /* We recover from asynDisabled just by retrieving
     * the parameter from the list */
    else if (status == asynDisabled){
        status = asynSuccess;
    }

get_param_err:
    return status;
}

/* Configuration routine.  Called directly, or from the iocsh function below */
extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvTimRx class.
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] endpoint The address device string */
    int drvTimRxConfigure(const char *portName, const char *endpoint,
            int timRxNumber, int verbose, int timeout)
    {
        new drvTimRx(portName, endpoint, timRxNumber, verbose, timeout);
        return(asynSuccess);
    }

    /* EPICS iocsh shell commands */
    static const iocshArg initArg0 = { "portName", iocshArgString};
    static const iocshArg initArg1 = { "endpoint", iocshArgString};
    static const iocshArg initArg2 = { "timRxNumber", iocshArgInt};
    static const iocshArg initArg3 = { "verbose", iocshArgInt};
    static const iocshArg initArg4 = { "timeout", iocshArgInt};
    static const iocshArg * const initArgs[] = {&initArg0,
        &initArg1,
        &initArg2,
        &initArg3,
        &initArg4};
    static const iocshFuncDef initFuncDef = {"drvTimRxConfigure",5,initArgs};
    static void initCallFunc(const iocshArgBuf *args)
    {
        drvTimRxConfigure(args[0].sval, args[1].sval, args[2].ival,
                args[3].ival, args[4].ival);
    }

    void drvTimRxRegister(void)
    {
        iocshRegister(&initFuncDef,initCallFunc);
    }

    epicsExportRegistrar(drvTimRxRegister);
}
