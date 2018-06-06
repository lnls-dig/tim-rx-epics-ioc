/*
 * drvTimRx.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Jan. 18, 2018
 */

#include "asynPortDriver.h"
#include <epicsExit.h>
#include <epicsMutex.h>
/* Third-party libraries */
#include <unordered_map>
#include <halcs_client.h>

// any implementation for non c++-17 compilers
#include "any.hpp"

using linb::any;
using linb::any_cast;
using linb::bad_any_cast;

#define ARRAY_SIZE(ARRAY)           (sizeof(ARRAY)/sizeof((ARRAY)[0]))

#define MAX_SLOTS                   12
#define MAX_TIM_RX_PER_SLOT         2
#define MAX_TIM_RXS                 (MAX_SLOTS*MAX_TIM_RX_PER_SLOT)

#define TIM_RX_NUMBER_MIN           1
#define TIM_RX_NUMBER_MAX           MAX_TIM_RXS

#define MAX_ADDR                    1

/* TIM_RX Mappping structure */
typedef struct {
    int board;
    int timRx;
} boardMap_t;

/* Write 64-bit float function pointer */
typedef halcs_client_err_e (*writeFloat64Fp)(halcs_client_t *self, char *service,
	double param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readFloat64Fp)(halcs_client_t *self, char *service,
	double *param);

/* TIM_RX command dispatch table */
typedef struct {
    const char *serviceName;
    writeFloat64Fp write;
    readFloat64Fp read;
} functionsFloat64_t;

/* Write 32-bit function pointer */
typedef halcs_client_err_e (*writeInt32Fp)(halcs_client_t *self, char *service,
	uint32_t param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readInt32Fp)(halcs_client_t *self, char *service,
	uint32_t *param);

/* TIM_RX command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32Fp write;
    readInt32Fp read;
} functionsInt32_t;

typedef struct {
    union {
        epicsUInt32 argUInt32;
        epicsFloat64 argFloat64;
    };
} functionsArgs_t;

/* Forward declaration as struct functionsAny_t needs it */
class drvTimRx;

/* Idea based on https://stackoverflow.com/questions/15102139/boostany-and-templates*/

/* Generic Function Structure for "any" function pointer */
struct functionsAny_t {
    template<typename T>
        functionsAny_t(T const& functionFp) :
            _functionFp(functionFp),
            _executeHwReadFunction(&functionsAny_t::executeHwReadFunction<T>),
            _executeHwWriteFunction(&functionsAny_t::executeHwWriteFunction<T>),
            _getServiceNameFromFunc(&functionsAny_t::getServiceNameFromFunc<T>) {}

    asynStatus executeHwRead(const drvTimRx& drvTimRx, char *service,
        int addr, functionsArgs_t &functionParam)
    {
        return (this->*_executeHwReadFunction)(drvTimRx, _functionFp,
                service, addr, functionParam);
    }

    asynStatus executeHwWrite(const drvTimRx& drvTimRx, char *service,
        int addr, functionsArgs_t &functionParam)
    {
        return (this->*_executeHwWriteFunction)(drvTimRx, _functionFp,
                service, addr, functionParam);
    }

    const char *getServiceName(const drvTimRx& drvTimRx)
    {
        return (this->*_getServiceNameFromFunc)(drvTimRx, _functionFp);
    }

private:
    any _functionFp;
    /* Read template function for Hw execution */
    typedef asynStatus (functionsAny_t::*executeHwReadFunctionFp)
        (const drvTimRx& drvTimRx, const any& functionFp,
         char *service, int addr, functionsArgs_t &functionParam);
    executeHwReadFunctionFp _executeHwReadFunction;
    /* Write template function for Hw execution */
    typedef asynStatus (functionsAny_t::*executeHwWriteFunctionFp)
        (const drvTimRx& drvTimRx, const any& functionFp,
         char *service, int addr, functionsArgs_t &functionParam);
    executeHwWriteFunctionFp _executeHwWriteFunction;
    /* Service name utilities */
    typedef const char * (functionsAny_t::*getServiceNameFromFuncFp)
        (const drvTimRx& drvTimRx, const any& functionFp) const;
    getServiceNameFromFuncFp _getServiceNameFromFunc;

    /* Read function for Hw execution */
    template<typename T>
    asynStatus executeHwReadFunction(const drvTimRx& drvTimRx,
            const any& functionFp, char *service, int addr,
            functionsArgs_t &functionParam);

    /* Write function for Hw execution */
    template<typename T>
    asynStatus executeHwWriteFunction(const drvTimRx& drvTimRx,
            const any& functionFp, char *service, int addr,
            functionsArgs_t &functionParam);

    /* Service name utilities */
    template<typename T>
    const char *getServiceNameFromFunc(const drvTimRx& drvTimRx,
            const any& functionFp) const;
};

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_TimRxLinkStatusString             "TIM_RX_LINK_STATUS"      /* asynUInt32Digital,  r/w */
#define P_TimRxRxEnStatusString             "TIM_RX_RX_EN_STATUS"     /* asynUInt32Digital,  r/w */
#define P_TimRxRefClkLockedString           "TIM_RX_REF_CLK_LOCKED"   /* asynUInt32Digital,  r/w */
#define P_TimRxEvtCode7String               "TIM_RX_EVT_CODE7"        /* asynUInt32Digital,  r/w */
#define P_TimRxEvtDelay7String              "TIM_RX_EVT_DELAY7"       /* asynUInt32Digital,  r/w */
#define P_TimRxEvtWidth7String              "TIM_RX_EVT_WIDTH7"       /* asynUInt32Digital,  r/w */

class drvTimRx : public asynPortDriver {
    public:
        drvTimRx(const char *portName, const char *endpoint,
                int timRxNumber, int verbose, int timeout);
        ~drvTimRx();

        /* These are the methods that we override from asynPortDriver */
        virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
                epicsUInt32 mask);
        virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
                epicsUInt32 mask);
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

        /* These methods are overwritten from asynPortDriver */
        virtual asynStatus connect(asynUser* pasynUser);
        virtual asynStatus disconnect(asynUser* pasynUser);

        /* Overloaded functions for extracting service name*/
        const char *doGetServiceNameFromFunc (functionsInt32_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functionsFloat64_t &func) const
        {
            return func.serviceName;
        }

        /* Overloaded function mappings called by functionsAny_t */
        asynStatus doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwWriteFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        asynStatus doExecuteHwReadFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwReadFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        /* General service name handling utilities */
        asynStatus getServiceID (int timRxNumber, int addr, const char *serviceName,
                int *serviceIDArg) const;
        asynStatus getFullServiceName (int timRxNumber, int addr, const char *serviceName,
                char *fullServiceName, int fullServiceNameSize) const;

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_TimRxLinkStatus;
#define FIRST_COMMAND P_TimRxLinkStatus
        int P_TimRxRxEnStatus;
        int P_TimRxRefClkLocked;
        int P_TimRxEvtCode7;
        int P_TimRxEvtDelay7;
        int P_TimRxEvtWidth7;
#define LAST_COMMAND P_TimRxEvtWidth7

    private:
        /* Our data */
        halcs_client_t *timRxClient;
        char *endpoint;
        int timRxNumber;
        int verbose;
        int timeout;
        char *timRxPortName;
        std::unordered_map<int, functionsAny_t> timRxHwFunc;

        /* Our private methods */

        /* Client connection management */
        asynStatus timRxClientConnect(void);
        asynStatus timRxClientDisconnect(void);

        /* General set/get hardware functions */
        asynStatus setParam32(int functionId, epicsUInt32 mask, int addr);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask, int addr);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)


/********************************************************************/
/*************** fucntionsAny_t template functions ******************/
/********************************************************************/

/* Read function for Hw execution */
template<typename T>
asynStatus functionsAny_t::executeHwReadFunction(const drvTimRx& drvTimRx,
        const any& functionFp, char *service,
    int addr, functionsArgs_t &functionParam)
{
    if(!any_cast<T>(functionFp).read) {
        return asynSuccess;
    }
    auto functionFpCast = any_cast<T>(functionFp);
    return drvTimRx.doExecuteHwReadFunction(functionFpCast, service, addr, functionParam);
}

/* Write function for Hw execution */
template<typename T>
asynStatus functionsAny_t::executeHwWriteFunction(const drvTimRx& drvTimRx,
        const any& functionFp, char *service,
    int addr, functionsArgs_t &functionParam)
{
    if(!any_cast<T>(functionFp).write) {
        return asynSuccess;
    }
    auto functionFpCast = any_cast<T>(functionFp);
    return drvTimRx.doExecuteHwWriteFunction(functionFpCast, service, addr, functionParam);
}

/* Service name utilities */
template<typename T>
const char *functionsAny_t::getServiceNameFromFunc(const drvTimRx& drvTimRx,
        const any& functionFp) const
{
    auto functionFpCast = any_cast<T>(functionFp);
    return drvTimRx.doGetServiceNameFromFunc(functionFpCast);
}
