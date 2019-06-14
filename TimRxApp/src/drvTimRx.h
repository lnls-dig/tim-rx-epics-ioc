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

#define MAX_ADDR                    8

#define MAX_AMC_TRIGGER_CH          8
#define MAX_FMC1_TRIGGER_CH         5
#define MAX_FMC2_TRIGGER_CH         5

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

/* Write 32-bit function pointer with channel selection */
typedef halcs_client_err_e (*writeInt32ChanFp)(halcs_client_t *self, char *service,
    uint32_t chan, uint32_t param);
/* Read 32-bit function pointer with channel selection */
typedef halcs_client_err_e (*readInt32ChanFp)(halcs_client_t *self, char *service,
    uint32_t chan, uint32_t *param);

/* TIM_RX command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32ChanFp write;
    readInt32ChanFp read;
} functionsInt32Chan_t;

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
#define P_TimRxLinkStatusString         "TIM_RX_LINK_STATUS"      /* asynUInt32Digital,  r/w */
#define P_TimRxRxenStatusString         "TIM_RX_RXEN_STATUS"      /* asynUInt32Digital,  r/w */
#define P_TimRxRefClkLockedString       "TIM_RX_REF_CLK_LOCKED"      /* asynUInt32Digital,  r/w */
#define P_TimRxEvrenString              "TIM_RX_EVREN"      /* asynUInt32Digital,  r/w */
#define P_TimRxAliveString              "TIM_RX_ALIVE"      /* asynUInt32Digital,  r/w */

#define P_TimRxAmcEnString              "TIM_RX_AMC_EN"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPolString             "TIM_RX_AMC_POL"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLogString             "TIM_RX_AMC_LOG"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItlString             "TIM_RX_AMC_ITL"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrcString             "TIM_RX_AMC_SRC"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDirString             "TIM_RX_AMC_DIR"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulsesString          "TIM_RX_AMC_PULSES"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvtString             "TIM_RX_AMC_EVT"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDlyString             "TIM_RX_AMC_DLY"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdtString             "TIM_RX_AMC_WDT"      /* asynUInt32Digital,  r/w */

#define P_TimRxFmc1EnString             "TIM_RX_FMC1_EN"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1PolString            "TIM_RX_FMC1_POL"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1LogString            "TIM_RX_FMC1_LOG"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1ItlString            "TIM_RX_FMC1_ITL"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1SrcString            "TIM_RX_FMC1_SRC"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1DirString            "TIM_RX_FMC1_DIR"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1PulsesString         "TIM_RX_FMC1_PULSES"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1EvtString            "TIM_RX_FMC1_EVT"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1DlyString            "TIM_RX_FMC1_DLY"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1WdtString            "TIM_RX_FMC1_WDT"      /* asynUInt32Digital,  r/w */

#define P_TimRxFmc2EnString             "TIM_RX_FMC2_EN"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2PolString            "TIM_RX_FMC2_POL"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2LogString            "TIM_RX_FMC2_LOG"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2ItlString            "TIM_RX_FMC2_ITL"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2SrcString            "TIM_RX_FMC2_SRC"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2DirString            "TIM_RX_FMC2_DIR"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2PulsesString         "TIM_RX_FMC2_PULSES"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2EvtString            "TIM_RX_FMC2_EVT"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2DlyString            "TIM_RX_FMC2_DLY"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2WdtString            "TIM_RX_FMC2_WDT"      /* asynUInt32Digital,  r/w */

#define P_TimRxRtmFreqKpString          "TIM_RX_RTM_FREQ_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmFreqKiString          "TIM_RX_RTM_FREQ_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseKpString         "TIM_RX_RTM_PHASE_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseKiString         "TIM_RX_RTM_PHASE_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseSetString        "TIM_RX_RTM_PHASE_SET"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseNavgString       "TIM_RX_RTM_PHASE_NAVG"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseDivExpString     "TIM_RX_RTM_PHASE_DIV_EXP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmRfreqHiString         "TIM_RX_RTM_RFREQ_HI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmRfreqLoString         "TIM_RX_RTM_RFREQ_LO"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmN1String              "TIM_RX_RTM_N1"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmHsDivString           "TIM_RX_RTM_HS_DIV"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmSi57xFreqString       "TIM_RX_RTM_SI57XFREQ"      /* asynUInt32Digital,  r/w */

#define P_TimRxAfcFreqKpString          "TIM_RX_AFC_FREQ_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcFreqKiString          "TIM_RX_AFC_FREQ_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseKpString         "TIM_RX_AFC_PHASE_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseKiString         "TIM_RX_AFC_PHASE_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseSetString        "TIM_RX_AFC_PHASE_SET"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseNavgString       "TIM_RX_AFC_PHASE_NAVG"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseDivExpString     "TIM_RX_AFC_PHASE_DIV_EXP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcRfreqHiString         "TIM_RX_AFC_RFREQ_HI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcRfreqLoString         "TIM_RX_AFC_RFREQ_LO"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcN1String              "TIM_RX_AFC_N1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcHsDivString           "TIM_RX_AFC_HS_DIV"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcSi57xFreqString       "TIM_RX_AFC_SI57XFREQ"      /* asynUInt32Digital,  r/w */

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

        const char *doGetServiceNameFromFunc (functionsInt32Chan_t &func) const
        {
            return func.serviceName;
        }

        /* Overloaded function mappings called by functionsAny_t */
        asynStatus doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsInt32Chan_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwWriteFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        asynStatus doExecuteHwReadFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsInt32Chan_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwReadFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        /* General service name handling utilities */
        asynStatus getServiceChan (int timRxNumber, int addr, const char *serviceName,
                epicsUInt32 *chanArg) const;
        asynStatus getServiceID (int timRxNumber, int addr, const char *serviceName,
                int *serviceIDArg) const;
        asynStatus getFullServiceName (int timRxNumber, int addr, const char *serviceName,
                char *fullServiceName, int fullServiceNameSize) const;

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_TimRxLinkStatus;
#define FIRST_COMMAND P_TimRxLinkStatus
        int P_TimRxRxenStatus;
        int P_TimRxRefClkLocked;
        int P_TimRxEvren;
        int P_TimRxAlive;
        int P_TimRxAmcEn;
        int P_TimRxAmcPol;
        int P_TimRxAmcLog;
        int P_TimRxAmcItl;
        int P_TimRxAmcSrc;
        int P_TimRxAmcDir;
        int P_TimRxAmcPulses;
        int P_TimRxAmcEvt;
        int P_TimRxAmcDly;
        int P_TimRxAmcWdt;
        int P_TimRxFmc1En;
        int P_TimRxFmc1Pol;
        int P_TimRxFmc1Log;
        int P_TimRxFmc1Itl;
        int P_TimRxFmc1Src;
        int P_TimRxFmc1Dir;
        int P_TimRxFmc1Pulses;
        int P_TimRxFmc1Evt;
        int P_TimRxFmc1Dly;
        int P_TimRxFmc1Wdt;
        int P_TimRxFmc2En;
        int P_TimRxFmc2Pol;
        int P_TimRxFmc2Log;
        int P_TimRxFmc2Itl;
        int P_TimRxFmc2Src;
        int P_TimRxFmc2Dir;
        int P_TimRxFmc2Pulses;
        int P_TimRxFmc2Evt;
        int P_TimRxFmc2Dly;
        int P_TimRxFmc2Wdt;
        int P_TimRxRtmFreqKp;
        int P_TimRxRtmFreqKi;
        int P_TimRxRtmPhaseKp;
        int P_TimRxRtmPhaseKi;
        int P_TimRxRtmPhaseSet;
        int P_TimRxRtmPhaseNavg;
        int P_TimRxRtmPhaseDivExp;
        int P_TimRxRtmRfreqHi;
        int P_TimRxRtmRfreqLo;
        int P_TimRxRtmN1;
        int P_TimRxRtmHsDiv;
        int P_TimRxRtmSi57xFreq;
        int P_TimRxAfcFreqKp;
        int P_TimRxAfcFreqKi;
        int P_TimRxAfcPhaseKp;
        int P_TimRxAfcPhaseKi;
        int P_TimRxAfcPhaseSet;
        int P_TimRxAfcPhaseNavg;
        int P_TimRxAfcPhaseDivExp;
        int P_TimRxAfcRfreqHi;
        int P_TimRxAfcRfreqLo;
        int P_TimRxAfcN1;
        int P_TimRxAfcHsDiv;
        int P_TimRxAfcSi57xFreq;
#define LAST_COMMAND P_TimRxAfcSi57xFreq

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
        asynStatus timRxClientConnect(asynUser* pasynUser);
        asynStatus timRxClientDisconnect(asynUser* pasynUser);

        /* General set/get hardware functions */
        asynStatus setParamGeneric(int funcionId, int addr);
        asynStatus setParam32(int functionId, epicsUInt32 mask, int addr);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask, int addr);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);

        /* Specific hardware functions that need extra processing and don't
         * fit into the general set/get template */
        asynStatus setRtmSi57xFreq(epicsUInt32 value, int addr);
        asynStatus setAfcSi57xFreq(epicsUInt32 value, int addr);
        asynStatus setSi57xFreq(epicsUInt32 value, uint32_t *n1, uint32_t *hs_div,
                uint32_t *ReqLo, uint32_t *ReqHi);
        asynStatus getRtmSi57xFreq(epicsUInt32 *value, int addr);
        asynStatus getAfcSi57xFreq(epicsUInt32 *value, int addr);
        asynStatus getSi57xFreq(epicsUInt32 *value, uint32_t n1, uint32_t hs_div,
                uint32_t ReqLo, uint32_t ReqHi);

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
