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

/* Int32 functions mapping */
static const functionsAny_t timRxSetGetLinkStatusFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_link_status, afc_timing_get_link_status}};
static const functionsAny_t timRxSetGetRxenStatusFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rxen_status, afc_timing_get_rxen_status}};
static const functionsAny_t timRxSetGetRefClkLockedFunc =             {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_ref_clk_locked, afc_timing_get_ref_clk_locked}};
static const functionsAny_t timRxSetGetEvrenFunc =                    {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_evren, afc_timing_get_evren}};
static const functionsAny_t timRxSetGetAliveFunc =                    {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_alive, afc_timing_get_alive}};

static const functionsAny_t timRxSetGetAmcEnFunc =                    {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_en, halcs_get_afc_timing_amc_en}};
static const functionsAny_t timRxSetGetAmcPolFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_pol, halcs_get_afc_timing_amc_pol}};
static const functionsAny_t timRxSetGetAmcLogFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_log, halcs_get_afc_timing_amc_log}};
static const functionsAny_t timRxSetGetAmcItlFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_itl, halcs_get_afc_timing_amc_itl}};
static const functionsAny_t timRxSetGetAmcSrcFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_src, halcs_get_afc_timing_amc_src}};
static const functionsAny_t timRxSetGetAmcDirFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_dir, halcs_get_afc_timing_amc_dir}};
static const functionsAny_t timRxSetGetAmcPulsesFunc =                {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_pulses, halcs_get_afc_timing_amc_pulses}};
static const functionsAny_t timRxSetGetAmcEvtFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_evt, halcs_get_afc_timing_amc_evt}};
static const functionsAny_t timRxSetGetAmcDlyFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_dly, halcs_get_afc_timing_amc_dly}};
static const functionsAny_t timRxSetGetAmcWdtFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_amc_wdt, halcs_get_afc_timing_amc_wdt}};

static const functionsAny_t timRxSetGetFmc1EnFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_en, halcs_get_afc_timing_fmc1_en}};
static const functionsAny_t timRxSetGetFmc1PolFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_pol, halcs_get_afc_timing_fmc1_pol}};
static const functionsAny_t timRxSetGetFmc1LogFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_log, halcs_get_afc_timing_fmc1_log}};
static const functionsAny_t timRxSetGetFmc1ItlFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_itl, halcs_get_afc_timing_fmc1_itl}};
static const functionsAny_t timRxSetGetFmc1SrcFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_src, halcs_get_afc_timing_fmc1_src}};
static const functionsAny_t timRxSetGetFmc1DirFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_dir, halcs_get_afc_timing_fmc1_dir}};
static const functionsAny_t timRxSetGetFmc1PulsesFunc =               {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_pulses, halcs_get_afc_timing_fmc1_pulses}};
static const functionsAny_t timRxSetGetFmc1EvtFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_evt, halcs_get_afc_timing_fmc1_evt}};
static const functionsAny_t timRxSetGetFmc1DlyFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_dly, halcs_get_afc_timing_fmc1_dly}};
static const functionsAny_t timRxSetGetFmc1WdtFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc1_wdt, halcs_get_afc_timing_fmc1_wdt}};

static const functionsAny_t timRxSetGetFmc2EnFunc =                   {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_en, halcs_get_afc_timing_fmc2_en}};
static const functionsAny_t timRxSetGetFmc2PolFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_pol, halcs_get_afc_timing_fmc2_pol}};
static const functionsAny_t timRxSetGetFmc2LogFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_log, halcs_get_afc_timing_fmc2_log}};
static const functionsAny_t timRxSetGetFmc2ItlFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_itl, halcs_get_afc_timing_fmc2_itl}};
static const functionsAny_t timRxSetGetFmc2SrcFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_src, halcs_get_afc_timing_fmc2_src}};
static const functionsAny_t timRxSetGetFmc2DirFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_dir, halcs_get_afc_timing_fmc2_dir}};
static const functionsAny_t timRxSetGetFmc2PulsesFunc =               {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_pulses, halcs_get_afc_timing_fmc2_pulses}};
static const functionsAny_t timRxSetGetFmc2EvtFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_evt, halcs_get_afc_timing_fmc2_evt}};
static const functionsAny_t timRxSetGetFmc2DlyFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_dly, halcs_get_afc_timing_fmc2_dly}};
static const functionsAny_t timRxSetGetFmc2WdtFunc =                  {functionsInt32Chan_t{"LNLS_AFC_TIMING", halcs_set_afc_timing_fmc2_wdt, halcs_get_afc_timing_fmc2_wdt}};

static const functionsAny_t timRxSetGetRtmFreqKpFunc =                {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_freq_kp, afc_timing_get_rtm_freq_kp}};
static const functionsAny_t timRxSetGetRtmFreqKiFunc =                {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_freq_ki, afc_timing_get_rtm_freq_ki}};
static const functionsAny_t timRxSetGetRtmPhaseKpFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_kp, afc_timing_get_rtm_phase_kp}};
static const functionsAny_t timRxSetGetRtmPhaseKiFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_ki, afc_timing_get_rtm_phase_ki}};
static const functionsAny_t timRxSetGetRtmPhaseSetFunc =              {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_set, afc_timing_get_rtm_phase_set}};
static const functionsAny_t timRxSetGetRtmPhaseNavgFunc =             {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_navg, afc_timing_get_rtm_phase_navg}};
static const functionsAny_t timRxSetGetRtmPhaseDivExpFunc =           {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_div_exp, afc_timing_get_rtm_phase_div_exp}};
static const functionsAny_t timRxSetGetRtmRfreqHiFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_rfreq_hi, afc_timing_get_rtm_rfreq_hi}};
static const functionsAny_t timRxSetGetRtmRfreqLoFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_rfreq_lo, afc_timing_get_rtm_rfreq_lo}};
static const functionsAny_t timRxSetGetRtmN1Func =                    {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_n1, afc_timing_get_rtm_n1}};
static const functionsAny_t timRxSetGetRtmHsDivFunc =                 {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_hs_div, afc_timing_get_rtm_hs_div}};

static const functionsAny_t timRxSetGetAfcFreqKpFunc =                {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_freq_kp, afc_timing_get_afc_freq_kp}};
static const functionsAny_t timRxSetGetAfcFreqKiFunc =                {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_freq_ki, afc_timing_get_afc_freq_ki}};
static const functionsAny_t timRxSetGetAfcPhaseKpFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_kp, afc_timing_get_afc_phase_kp}};
static const functionsAny_t timRxSetGetAfcPhaseKiFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_ki, afc_timing_get_afc_phase_ki}};
static const functionsAny_t timRxSetGetAfcPhaseSetFunc =              {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_set, afc_timing_get_afc_phase_set}};
static const functionsAny_t timRxSetGetAfcPhaseNavgFunc =             {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_navg, afc_timing_get_afc_phase_navg}};
static const functionsAny_t timRxSetGetAfcPhaseDivExpFunc =           {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_div_exp, afc_timing_get_afc_phase_div_exp}};
static const functionsAny_t timRxSetGetAfcRfreqHiFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_rfreq_hi, afc_timing_get_afc_rfreq_hi}};
static const functionsAny_t timRxSetGetAfcRfreqLoFunc =               {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_rfreq_lo, afc_timing_get_afc_rfreq_lo}};
static const functionsAny_t timRxSetGetAfcN1Func =                    {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_n1, afc_timing_get_afc_n1}};
static const functionsAny_t timRxSetGetAfcHsDivFunc =                 {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_hs_div, afc_timing_get_afc_hs_div}};

static const char *driverName="drvTimRx";
void acqTask(void *drvPvt);

static void exitHandlerC(void *pPvt)
{
    drvTimRx *pdrvTimRx = (drvTimRx *)pPvt;
    pdrvTimRx->~drvTimRx();
}

asynStatus drvTimRx::getServiceChan (int timRxNumber, int addr, const char *serviceName,
        epicsUInt32 *chanArg) const
{
    static const char *functionName = "getServiceChan";
    asynStatus status = asynSuccess;
    epicsUInt32 chan = 0;

    chan = addr;

    *chanArg = chan;
    return status;
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
    createParam(P_TimRxLinkStatusString,   asynParamUInt32Digital,         &P_TimRxLinkStatus);
    createParam(P_TimRxRxenStatusString,   asynParamUInt32Digital,         &P_TimRxRxenStatus);
    createParam(P_TimRxRefClkLockedString,   asynParamUInt32Digital,         &P_TimRxRefClkLocked);
    createParam(P_TimRxEvrenString,   asynParamUInt32Digital,         &P_TimRxEvren);
    createParam(P_TimRxAliveString,   asynParamUInt32Digital,         &P_TimRxAlive);

    createParam(P_TimRxAmcEnString,   asynParamUInt32Digital,         &P_TimRxAmcEn);
    createParam(P_TimRxAmcPolString,   asynParamUInt32Digital,         &P_TimRxAmcPol);
    createParam(P_TimRxAmcLogString,   asynParamUInt32Digital,         &P_TimRxAmcLog);
    createParam(P_TimRxAmcItlString,   asynParamUInt32Digital,         &P_TimRxAmcItl);
    createParam(P_TimRxAmcSrcString,   asynParamUInt32Digital,         &P_TimRxAmcSrc);
    createParam(P_TimRxAmcDirString,   asynParamUInt32Digital,         &P_TimRxAmcDir);
    createParam(P_TimRxAmcPulsesString,   asynParamUInt32Digital,         &P_TimRxAmcPulses);
    createParam(P_TimRxAmcEvtString,   asynParamUInt32Digital,         &P_TimRxAmcEvt);
    createParam(P_TimRxAmcDlyString,   asynParamUInt32Digital,         &P_TimRxAmcDly);
    createParam(P_TimRxAmcWdtString,   asynParamUInt32Digital,         &P_TimRxAmcWdt);

    createParam(P_TimRxFmc1EnString,   asynParamUInt32Digital,         &P_TimRxFmc1En);
    createParam(P_TimRxFmc1PolString,   asynParamUInt32Digital,         &P_TimRxFmc1Pol);
    createParam(P_TimRxFmc1LogString,   asynParamUInt32Digital,         &P_TimRxFmc1Log);
    createParam(P_TimRxFmc1ItlString,   asynParamUInt32Digital,         &P_TimRxFmc1Itl);
    createParam(P_TimRxFmc1SrcString,   asynParamUInt32Digital,         &P_TimRxFmc1Src);
    createParam(P_TimRxFmc1DirString,   asynParamUInt32Digital,         &P_TimRxFmc1Dir);
    createParam(P_TimRxFmc1PulsesString,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses);
    createParam(P_TimRxFmc1EvtString,   asynParamUInt32Digital,         &P_TimRxFmc1Evt);
    createParam(P_TimRxFmc1DlyString,   asynParamUInt32Digital,         &P_TimRxFmc1Dly);
    createParam(P_TimRxFmc1WdtString,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt);

    createParam(P_TimRxFmc2EnString,   asynParamUInt32Digital,         &P_TimRxFmc2En);
    createParam(P_TimRxFmc2PolString,   asynParamUInt32Digital,         &P_TimRxFmc2Pol);
    createParam(P_TimRxFmc2LogString,   asynParamUInt32Digital,         &P_TimRxFmc2Log);
    createParam(P_TimRxFmc2ItlString,   asynParamUInt32Digital,         &P_TimRxFmc2Itl);
    createParam(P_TimRxFmc2SrcString,   asynParamUInt32Digital,         &P_TimRxFmc2Src);
    createParam(P_TimRxFmc2DirString,   asynParamUInt32Digital,         &P_TimRxFmc2Dir);
    createParam(P_TimRxFmc2PulsesString,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses);
    createParam(P_TimRxFmc2EvtString,   asynParamUInt32Digital,         &P_TimRxFmc2Evt);
    createParam(P_TimRxFmc2DlyString,   asynParamUInt32Digital,         &P_TimRxFmc2Dly);
    createParam(P_TimRxFmc2WdtString,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt);

    createParam(P_TimRxRtmFreqKpString,   asynParamUInt32Digital,         &P_TimRxRtmFreqKp);
    createParam(P_TimRxRtmFreqKiString,   asynParamUInt32Digital,         &P_TimRxRtmFreqKi);
    createParam(P_TimRxRtmPhaseKpString,   asynParamUInt32Digital,         &P_TimRxRtmPhaseKp);
    createParam(P_TimRxRtmPhaseKiString,   asynParamUInt32Digital,         &P_TimRxRtmPhaseKi);
    createParam(P_TimRxRtmPhaseSetString,   asynParamUInt32Digital,         &P_TimRxRtmPhaseSet);
    createParam(P_TimRxRtmPhaseNavgString,   asynParamUInt32Digital,         &P_TimRxRtmPhaseNavg);
    createParam(P_TimRxRtmPhaseDivExpString,   asynParamUInt32Digital,         &P_TimRxRtmPhaseDivExp);
    createParam(P_TimRxRtmRfreqHiString,   asynParamUInt32Digital,         &P_TimRxRtmRfreqHi);
    createParam(P_TimRxRtmRfreqLoString,   asynParamUInt32Digital,         &P_TimRxRtmRfreqLo);
    createParam(P_TimRxRtmN1String,   asynParamUInt32Digital,         &P_TimRxRtmN1);
    createParam(P_TimRxRtmHsDivString,   asynParamUInt32Digital,         &P_TimRxRtmHsDiv);
    createParam(P_TimRxRtmSi57xFreqString,   asynParamUInt32Digital,         &P_TimRxRtmSi57xFreq);

    createParam(P_TimRxAfcFreqKpString,   asynParamUInt32Digital,         &P_TimRxAfcFreqKp);
    createParam(P_TimRxAfcFreqKiString,   asynParamUInt32Digital,         &P_TimRxAfcFreqKi);
    createParam(P_TimRxAfcPhaseKpString,   asynParamUInt32Digital,         &P_TimRxAfcPhaseKp);
    createParam(P_TimRxAfcPhaseKiString,   asynParamUInt32Digital,         &P_TimRxAfcPhaseKi);
    createParam(P_TimRxAfcPhaseSetString,   asynParamUInt32Digital,         &P_TimRxAfcPhaseSet);
    createParam(P_TimRxAfcPhaseNavgString,   asynParamUInt32Digital,         &P_TimRxAfcPhaseNavg);
    createParam(P_TimRxAfcPhaseDivExpString,   asynParamUInt32Digital,         &P_TimRxAfcPhaseDivExp);
    createParam(P_TimRxAfcRfreqHiString,   asynParamUInt32Digital,         &P_TimRxAfcRfreqHi);
    createParam(P_TimRxAfcRfreqLoString,   asynParamUInt32Digital,         &P_TimRxAfcRfreqLo);
    createParam(P_TimRxAfcN1String,   asynParamUInt32Digital,         &P_TimRxAfcN1);
    createParam(P_TimRxAfcHsDivString,   asynParamUInt32Digital,         &P_TimRxAfcHsDiv);
    createParam(P_TimRxAfcSi57xFreqString,   asynParamUInt32Digital,         &P_TimRxAfcSi57xFreq);

    /* TimRx Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    timRxHwFunc.emplace(P_TimRxLinkStatus,    timRxSetGetLinkStatusFunc);
    timRxHwFunc.emplace(P_TimRxRxenStatus,    timRxSetGetRxenStatusFunc);
    timRxHwFunc.emplace(P_TimRxRefClkLocked,    timRxSetGetRefClkLockedFunc);
    timRxHwFunc.emplace(P_TimRxEvren,    timRxSetGetEvrenFunc);
    timRxHwFunc.emplace(P_TimRxAlive,    timRxSetGetAliveFunc);

    timRxHwFunc.emplace(P_TimRxAmcEn,    timRxSetGetAmcEnFunc);
    timRxHwFunc.emplace(P_TimRxAmcPol,    timRxSetGetAmcPolFunc);
    timRxHwFunc.emplace(P_TimRxAmcLog,    timRxSetGetAmcLogFunc);
    timRxHwFunc.emplace(P_TimRxAmcItl,    timRxSetGetAmcItlFunc);
    timRxHwFunc.emplace(P_TimRxAmcSrc,    timRxSetGetAmcSrcFunc);
    timRxHwFunc.emplace(P_TimRxAmcDir,    timRxSetGetAmcDirFunc);
    timRxHwFunc.emplace(P_TimRxAmcPulses,    timRxSetGetAmcPulsesFunc);
    timRxHwFunc.emplace(P_TimRxAmcEvt,    timRxSetGetAmcEvtFunc);
    timRxHwFunc.emplace(P_TimRxAmcDly,    timRxSetGetAmcDlyFunc);
    timRxHwFunc.emplace(P_TimRxAmcWdt,    timRxSetGetAmcWdtFunc);

    timRxHwFunc.emplace(P_TimRxFmc1En,    timRxSetGetFmc1EnFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Pol,    timRxSetGetFmc1PolFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Log,    timRxSetGetFmc1LogFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Itl,    timRxSetGetFmc1ItlFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Src,    timRxSetGetFmc1SrcFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Dir,    timRxSetGetFmc1DirFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses,    timRxSetGetFmc1PulsesFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Evt,    timRxSetGetFmc1EvtFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Dly,    timRxSetGetFmc1DlyFunc);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt,    timRxSetGetFmc1WdtFunc);

    timRxHwFunc.emplace(P_TimRxFmc2En,    timRxSetGetFmc2EnFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Pol,    timRxSetGetFmc2PolFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Log,    timRxSetGetFmc2LogFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Itl,    timRxSetGetFmc2ItlFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Src,    timRxSetGetFmc2SrcFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Dir,    timRxSetGetFmc2DirFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses,    timRxSetGetFmc2PulsesFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Evt,    timRxSetGetFmc2EvtFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Dly,    timRxSetGetFmc2DlyFunc);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt,    timRxSetGetFmc2WdtFunc);

    timRxHwFunc.emplace(P_TimRxRtmFreqKp,    timRxSetGetRtmFreqKpFunc);
    timRxHwFunc.emplace(P_TimRxRtmFreqKi,    timRxSetGetRtmFreqKiFunc);
    timRxHwFunc.emplace(P_TimRxRtmPhaseKp,    timRxSetGetRtmPhaseKpFunc);
    timRxHwFunc.emplace(P_TimRxRtmPhaseKi,    timRxSetGetRtmPhaseKiFunc);
    timRxHwFunc.emplace(P_TimRxRtmPhaseSet,    timRxSetGetRtmPhaseSetFunc);
    timRxHwFunc.emplace(P_TimRxRtmPhaseNavg,    timRxSetGetRtmPhaseNavgFunc);
    timRxHwFunc.emplace(P_TimRxRtmPhaseDivExp,    timRxSetGetRtmPhaseDivExpFunc);
    timRxHwFunc.emplace(P_TimRxRtmRfreqHi,    timRxSetGetRtmRfreqHiFunc);
    timRxHwFunc.emplace(P_TimRxRtmRfreqLo,    timRxSetGetRtmRfreqLoFunc);
    timRxHwFunc.emplace(P_TimRxRtmN1,    timRxSetGetRtmN1Func);
    timRxHwFunc.emplace(P_TimRxRtmHsDiv,    timRxSetGetRtmHsDivFunc);
    timRxHwFunc.emplace(P_TimRxAfcFreqKp,    timRxSetGetAfcFreqKpFunc);
    timRxHwFunc.emplace(P_TimRxAfcFreqKi,    timRxSetGetAfcFreqKiFunc);
    timRxHwFunc.emplace(P_TimRxAfcPhaseKp,    timRxSetGetAfcPhaseKpFunc);
    timRxHwFunc.emplace(P_TimRxAfcPhaseKi,    timRxSetGetAfcPhaseKiFunc);
    timRxHwFunc.emplace(P_TimRxAfcPhaseSet,    timRxSetGetAfcPhaseSetFunc);
    timRxHwFunc.emplace(P_TimRxAfcPhaseNavg,    timRxSetGetAfcPhaseNavgFunc);
    timRxHwFunc.emplace(P_TimRxAfcPhaseDivExp,    timRxSetGetAfcPhaseDivExpFunc);
    timRxHwFunc.emplace(P_TimRxAfcRfreqHi,    timRxSetGetAfcRfreqHiFunc);
    timRxHwFunc.emplace(P_TimRxAfcRfreqLo,    timRxSetGetAfcRfreqLoFunc);
    timRxHwFunc.emplace(P_TimRxAfcN1,    timRxSetGetAfcN1Func);
    timRxHwFunc.emplace(P_TimRxAfcHsDiv,    timRxSetGetAfcHsDivFunc);


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

    /* Set the initial values of some parameters */
    setUIntDigitalParam(P_TimRxLinkStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRxenStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRefClkLocked,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxEvren,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAlive,   0, 0xFFFFFFFF);

    for (int addr = 0; addr < MAX_AMC_TRIGGER_CH; ++addr) {
      setUIntDigitalParam(addr, P_TimRxAmcEn,       0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcPol,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcLog,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcItl,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcSrc,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcDir,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcPulses,   0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcEvt,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcDly,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxAmcWdt,      0, 0xFFFFFFFF);
    }

    for (int addr = 0; addr < MAX_FMC1_TRIGGER_CH; ++addr) {
      setUIntDigitalParam(addr, P_TimRxFmc1En,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Pol,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Log,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Itl,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Src,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Dir,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Pulses,  0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Evt,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Dly,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc1Wdt,     0, 0xFFFFFFFF);
    }

    for (int addr = 0; addr < MAX_FMC2_TRIGGER_CH; ++addr) {
      setUIntDigitalParam(addr, P_TimRxFmc2En,      0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Pol,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Log,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Itl,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Src,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Dir,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Pulses,  0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Evt,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Dly,     0, 0xFFFFFFFF);
      setUIntDigitalParam(addr, P_TimRxFmc2Wdt,     0, 0xFFFFFFFF);
    }

    setUIntDigitalParam(P_TimRxRtmFreqKp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmFreqKi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmPhaseKp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmPhaseKi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmPhaseSet,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmPhaseNavg,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmPhaseDivExp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmRfreqHi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmRfreqLo,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmN1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmHsDiv,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmSi57xFreq,   0, 0xFFFFFFFF);

    setUIntDigitalParam(P_TimRxAfcFreqKp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcFreqKi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcPhaseKp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcPhaseKi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcPhaseSet,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcPhaseNavg,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcPhaseDivExp,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcRfreqHi,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcRfreqLo,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcN1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcHsDiv,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcSi57xFreq,   0, 0xFFFFFFFF);

    /* Do callbacks so higher layers see any changes. Call callbacks for every addr */
    for (int i = 0; i < MAX_ADDR; ++i) {
        callParamCallbacks(i);
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

    if (function >= FIRST_COMMAND) {
        /* Set the parameter in the parameter library. */
        setUIntDigitalParam(addr, function, value, mask);
        /* Fetch the parameter string name for possible use in debugging */
        getParamName(function, &paramName);
    
        if (function == P_TimRxRtmSi57xFreq) {
            status = setRtmSi57xFreq(value, addr);
        }
        else if (function == P_TimRxAfcSi57xFreq) {
            status = setAfcSi57xFreq(value, addr);
        }
        else {
            /* Do operation on HW. Some functions do not set anything on hardware */
            status = setParam32(function, mask, addr);
        }
    }
    else {
        /* Call base class */
        status = asynPortDriver::writeUInt32Digital(pasynUser, value, mask);
    }

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

    if (function >= FIRST_COMMAND) {
        if (function == P_TimRxRtmSi57xFreq) {
            status = getRtmSi57xFreq(value, addr);
        }
        else if (function == P_TimRxAfcSi57xFreq) {
            status = getAfcSi57xFreq(value, addr);
        }
        else {
            /* Get parameter, possibly from HW */
            status = getParam32(function, value, mask, addr);
        }
    }
    else {
        /* Call base class */
        status = asynPortDriver::readUInt32Digital(pasynUser, value, mask);
    }

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

    if (function >= FIRST_COMMAND) {
        /* Set the parameter in the parameter library. */
        setDoubleParam(addr, function, value);
        /* Fetch the parameter string name for possible use in debugging */
        getParamName(function, &paramName);

        /* Do operation on HW. Some functions do not set anything on hardware */
        status = setParamDouble(function, addr);
    }
    else {
        /* Call base class */
        status = asynPortDriver::writeFloat64(pasynUser, value);
    }

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
    if (function >= FIRST_COMMAND) {
        status = getParamDouble(function, value, addr);
    }
    else {
        /* Call base class */
        status = asynPortDriver::readFloat64(pasynUser, value);
    }

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

asynStatus drvTimRx::doExecuteHwWriteFunction(functionsInt32Chan_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsInt32Chan_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    char serviceChanStr[SERVICE_NAME_SIZE];
    int serviceID = 0;
    epicsUInt32 serviceChan = addr;

    /* Execute registered function */
    err = func.write(timRxClient, service, serviceChan, functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, serviceChan,
                functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
get_service_id_err:
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

asynStatus drvTimRx::doExecuteHwReadFunction(functionsInt32Chan_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsInt32Chan_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    epicsUInt32 serviceChan = addr;

    /* Execute registered function */
    err = func.read(timRxClient, service, serviceChan, &functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %u\n",
                driverName, functionName, serviceChan);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
get_service_id_err:
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
    const char *paramName = NULL;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(addr, functionId, param, mask);
    if (status != asynSuccess) {
        getParamName(functionId, &paramName);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
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
    const char *paramName = NULL;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(addr, functionId, param);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getParamDouble failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
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

/********************************************************************/
/*********************** Misc BPM Operations ************************/
/********************************************************************/

/*
 * Miscellaneous functions that don't map easily
 * to our generic handlers get/setParam[32/Double]
 */

asynStatus drvTimRx::setSi57xFreq(epicsUInt32 value, uint32_t *n1, uint32_t *hs_div, uint32_t *ReqLo, uint32_t *ReqHi)
{
    int status = asynSuccess;
    const char* functionName = "setSi57xFreq";
    uint64_t fxtal = 114285000; // from Si57x datasheet
    uint64_t fdco_min = 4850000000; // from Si57x datasheet
    uint64_t fdco_max = 5670000000; // from Si57x datasheet
    uint64_t fdco = 0;
    uint64_t RFReq = 0;
    uint32_t n1_max_val = 128;
    uint32_t n1_min_val = 2;
    uint32_t n1_step = 2;
    const uint32_t hs_div_opt_size = 6;
    uint32_t hs_div_val[hs_div_opt_size] = {7, 5, 3, 2, 1, 0};
    uint32_t hs_div_opt[hs_div_opt_size] = {11, 9, 7, 6, 5, 4};
    
    for (uint32_t i = 0; i < hs_div_opt_size; ++i) {
        *hs_div = hs_div_val[i];
        for (*n1 = n1_min_val; *n1 <= n1_max_val; *n1=*n1+n1_step) {
            fdco = uint64_t(value)*uint64_t(hs_div_opt[i])*uint64_t(*n1);
            if ((fdco >= fdco_min) && (fdco <= fdco_max)) {
                break;
            }
        }
        if ((fdco >= fdco_min) && (fdco <= fdco_max))
            break;
    }
    
    *n1 = *n1 - 1;
    RFReq = uint64_t((double(fdco)/double(fxtal))*(1 << 28));
    *ReqHi = uint32_t(RFReq >> 20);
    *ReqLo = uint32_t(RFReq & 0xfffff);
    
    return (asynStatus)status;
}

asynStatus drvTimRx::setRtmSi57xFreq(epicsUInt32 value, int addr)
{
    int err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "setRtmSi57xFreq";
    epicsUInt32 RtmSi57xFreq = 0;
    
    uint32_t n1, hs_div, ReqLo, ReqHi;
    
    setSi57xFreq(value, &n1, &hs_div, &ReqLo, &ReqHi);
    
    /* Get correct service name*/
    status = getFullServiceName (this->timRxNumber, addr, "LNLS_AFC_TIMING",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    err = afc_timing_set_rtm_n1        (timRxClient, service, n1);
    err |= afc_timing_set_rtm_hs_div   (timRxClient, service, hs_div);
    err |= afc_timing_set_rtm_rfreq_lo (timRxClient, service, ReqLo);
    err |= afc_timing_set_rtm_rfreq_hi (timRxClient, service, ReqHi);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto set_AfcSi57xFreq_err;
    }

    afc_timing_get_rtm_n1       (timRxClient, service, &n1);
    afc_timing_get_rtm_hs_div   (timRxClient, service, &hs_div);
    afc_timing_get_rtm_rfreq_lo (timRxClient, service, &ReqLo);
    afc_timing_get_rtm_rfreq_hi (timRxClient, service, &ReqHi);

    setUIntDigitalParam(P_TimRxRtmRfreqHi,  ReqHi,  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmRfreqLo,  ReqLo,  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmN1,       n1,     0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRtmHsDiv,    hs_div, 0xFFFFFFFF);

set_AfcSi57xFreq_err:
get_service_err:
get_param_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::setAfcSi57xFreq(epicsUInt32 value, int addr)
{
    int err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "setAfcSi57xFreq";
    
    uint32_t n1, hs_div, ReqLo, ReqHi;
    
    setSi57xFreq(value, &n1, &hs_div, &ReqLo, &ReqHi);
    
    /* Get correct service name*/
    status = getFullServiceName (this->timRxNumber, addr, "LNLS_AFC_TIMING",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    err = afc_timing_set_afc_n1        (timRxClient, service, n1);
    err |= afc_timing_set_afc_hs_div   (timRxClient, service, hs_div);
    err |= afc_timing_set_afc_rfreq_lo (timRxClient, service, ReqLo);
    err |= afc_timing_set_afc_rfreq_hi (timRxClient, service, ReqHi);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto set_AfcSi57xFreq_err;
    }

    afc_timing_get_afc_n1       (timRxClient, service, &n1);
    afc_timing_get_afc_hs_div   (timRxClient, service, &hs_div);
    afc_timing_get_afc_rfreq_lo (timRxClient, service, &ReqLo);
    afc_timing_get_afc_rfreq_hi (timRxClient, service, &ReqHi);

    setUIntDigitalParam(P_TimRxAfcRfreqHi,  ReqHi,  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcRfreqLo,  ReqLo,  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcN1,       n1,     0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAfcHsDiv,    hs_div, 0xFFFFFFFF);

set_AfcSi57xFreq_err:
get_service_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::getSi57xFreq(epicsUInt32 *value, uint32_t n1, uint32_t hs_div, uint32_t ReqLo, uint32_t ReqHi)
{
    int status = asynSuccess;
    const char* functionName = "getSi57xFreq";
    double fxtal = 114285000; // from Si57x datasheet
    double RFReq = double((uint64_t(ReqHi)<< 20) + uint64_t(ReqLo))/double(1 << 28);
    double fdco = RFReq * fxtal;
    const uint32_t hs_div_opt_size = 6;
    uint32_t hs_div_val[hs_div_opt_size] = {7, 5, 3, 2, 1, 0};
    uint32_t hs_div_opt[hs_div_opt_size] = {11, 9, 7, 6, 5, 4};
    for (uint32_t i = 0; i < hs_div_opt_size; ++i) {
        if (hs_div == hs_div_val[i]) {
            *value = uint32_t(fdco/double((n1+1)*hs_div_opt[i])); 
            break;
        }
    }
    
    return (asynStatus)status;
}

asynStatus drvTimRx::getRtmSi57xFreq(epicsUInt32 *value, int addr)
{
    int err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "getRtmSi57xFreq";
    
    uint32_t n1, hs_div, ReqLo, ReqHi;
    
    /* Get correct service name*/
    status = getFullServiceName (this->timRxNumber, addr, "LNLS_AFC_TIMING",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    afc_timing_get_rtm_n1       (timRxClient, service, &n1);
    afc_timing_get_rtm_hs_div   (timRxClient, service, &hs_div);
    afc_timing_get_rtm_rfreq_lo (timRxClient, service, &ReqLo);
    afc_timing_get_rtm_rfreq_hi (timRxClient, service, &ReqHi);

    getSi57xFreq(value, n1, hs_div, ReqLo, ReqHi);

get_service_err:
    return (asynStatus)status;
}

asynStatus drvTimRx::getAfcSi57xFreq(epicsUInt32 *value, int addr)
{
    int err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "getAfcSi57xFreq";
    
    uint32_t n1, hs_div, ReqLo, ReqHi;
    
    /* Get correct service name*/
    status = getFullServiceName (this->timRxNumber, addr, "LNLS_AFC_TIMING",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    afc_timing_get_afc_n1       (timRxClient, service, &n1);
    afc_timing_get_afc_hs_div   (timRxClient, service, &hs_div);
    afc_timing_get_afc_rfreq_lo (timRxClient, service, &ReqLo);
    afc_timing_get_afc_rfreq_hi (timRxClient, service, &ReqHi);

    getSi57xFreq(value, n1, hs_div, ReqLo, ReqHi);

get_service_err:
    return (asynStatus)status;
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
