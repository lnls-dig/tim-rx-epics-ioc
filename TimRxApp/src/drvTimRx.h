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
#define P_TimRxRxenStatusString             "TIM_RX_RXEN_STATUS"      /* asynUInt32Digital,  r/w */
#define P_TimRxRefClkLockedString             "TIM_RX_REF_CLK_LOCKED"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn0String             "TIM_RX_AMC_EN0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol0String             "TIM_RX_AMC_POL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog0String             "TIM_RX_AMC_LOG0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl0String             "TIM_RX_AMC_ITL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc0String             "TIM_RX_AMC_SRC0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir0String             "TIM_RX_AMC_DIR0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses0String             "TIM_RX_AMC_PULSES0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt0String             "TIM_RX_AMC_EVT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly0String             "TIM_RX_AMC_DLY0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt0String             "TIM_RX_AMC_WDT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn1String             "TIM_RX_AMC_EN1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol1String             "TIM_RX_AMC_POL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog1String             "TIM_RX_AMC_LOG1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl1String             "TIM_RX_AMC_ITL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc1String             "TIM_RX_AMC_SRC1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir1String             "TIM_RX_AMC_DIR1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses1String             "TIM_RX_AMC_PULSES1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt1String             "TIM_RX_AMC_EVT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly1String             "TIM_RX_AMC_DLY1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt1String             "TIM_RX_AMC_WDT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn2String             "TIM_RX_AMC_EN2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol2String             "TIM_RX_AMC_POL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog2String             "TIM_RX_AMC_LOG2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl2String             "TIM_RX_AMC_ITL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc2String             "TIM_RX_AMC_SRC2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir2String             "TIM_RX_AMC_DIR2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses2String             "TIM_RX_AMC_PULSES2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt2String             "TIM_RX_AMC_EVT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly2String             "TIM_RX_AMC_DLY2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt2String             "TIM_RX_AMC_WDT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn3String             "TIM_RX_AMC_EN3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol3String             "TIM_RX_AMC_POL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog3String             "TIM_RX_AMC_LOG3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl3String             "TIM_RX_AMC_ITL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc3String             "TIM_RX_AMC_SRC3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir3String             "TIM_RX_AMC_DIR3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses3String             "TIM_RX_AMC_PULSES3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt3String             "TIM_RX_AMC_EVT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly3String             "TIM_RX_AMC_DLY3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt3String             "TIM_RX_AMC_WDT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn4String             "TIM_RX_AMC_EN4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol4String             "TIM_RX_AMC_POL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog4String             "TIM_RX_AMC_LOG4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl4String             "TIM_RX_AMC_ITL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc4String             "TIM_RX_AMC_SRC4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir4String             "TIM_RX_AMC_DIR4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses4String             "TIM_RX_AMC_PULSES4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt4String             "TIM_RX_AMC_EVT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly4String             "TIM_RX_AMC_DLY4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt4String             "TIM_RX_AMC_WDT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn5String             "TIM_RX_AMC_EN5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol5String             "TIM_RX_AMC_POL5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog5String             "TIM_RX_AMC_LOG5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl5String             "TIM_RX_AMC_ITL5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc5String             "TIM_RX_AMC_SRC5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir5String             "TIM_RX_AMC_DIR5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses5String             "TIM_RX_AMC_PULSES5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt5String             "TIM_RX_AMC_EVT5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly5String             "TIM_RX_AMC_DLY5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt5String             "TIM_RX_AMC_WDT5"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn6String             "TIM_RX_AMC_EN6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol6String             "TIM_RX_AMC_POL6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog6String             "TIM_RX_AMC_LOG6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl6String             "TIM_RX_AMC_ITL6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc6String             "TIM_RX_AMC_SRC6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir6String             "TIM_RX_AMC_DIR6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses6String             "TIM_RX_AMC_PULSES6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt6String             "TIM_RX_AMC_EVT6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly6String             "TIM_RX_AMC_DLY6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt6String             "TIM_RX_AMC_WDT6"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEn7String             "TIM_RX_AMC_EN7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPol7String             "TIM_RX_AMC_POL7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcLog7String             "TIM_RX_AMC_LOG7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcItl7String             "TIM_RX_AMC_ITL7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcSrc7String             "TIM_RX_AMC_SRC7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDir7String             "TIM_RX_AMC_DIR7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcPulses7String             "TIM_RX_AMC_PULSES7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcEvt7String             "TIM_RX_AMC_EVT7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcDly7String             "TIM_RX_AMC_DLY7"      /* asynUInt32Digital,  r/w */
#define P_TimRxAmcWdt7String             "TIM_RX_AMC_WDT7"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1En0String             "TIM_RX_FMC1_EN0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pol0String             "TIM_RX_FMC1_POL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Log0String             "TIM_RX_FMC1_LOG0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Itl0String             "TIM_RX_FMC1_ITL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Src0String             "TIM_RX_FMC1_SRC0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pulses0String             "TIM_RX_FMC1_PULSES0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Evt0String             "TIM_RX_FMC1_EVT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Dly0String             "TIM_RX_FMC1_DLY0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Wdt0String             "TIM_RX_FMC1_WDT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1En1String             "TIM_RX_FMC1_EN1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pol1String             "TIM_RX_FMC1_POL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Log1String             "TIM_RX_FMC1_LOG1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Itl1String             "TIM_RX_FMC1_ITL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Src1String             "TIM_RX_FMC1_SRC1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pulses1String             "TIM_RX_FMC1_PULSES1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Evt1String             "TIM_RX_FMC1_EVT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Dly1String             "TIM_RX_FMC1_DLY1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Wdt1String             "TIM_RX_FMC1_WDT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1En2String             "TIM_RX_FMC1_EN2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pol2String             "TIM_RX_FMC1_POL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Log2String             "TIM_RX_FMC1_LOG2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Itl2String             "TIM_RX_FMC1_ITL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Src2String             "TIM_RX_FMC1_SRC2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pulses2String             "TIM_RX_FMC1_PULSES2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Evt2String             "TIM_RX_FMC1_EVT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Dly2String             "TIM_RX_FMC1_DLY2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Wdt2String             "TIM_RX_FMC1_WDT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1En3String             "TIM_RX_FMC1_EN3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pol3String             "TIM_RX_FMC1_POL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Log3String             "TIM_RX_FMC1_LOG3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Itl3String             "TIM_RX_FMC1_ITL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Src3String             "TIM_RX_FMC1_SRC3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pulses3String             "TIM_RX_FMC1_PULSES3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Evt3String             "TIM_RX_FMC1_EVT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Dly3String             "TIM_RX_FMC1_DLY3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Wdt3String             "TIM_RX_FMC1_WDT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1En4String             "TIM_RX_FMC1_EN4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pol4String             "TIM_RX_FMC1_POL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Log4String             "TIM_RX_FMC1_LOG4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Itl4String             "TIM_RX_FMC1_ITL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Src4String             "TIM_RX_FMC1_SRC4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Pulses4String             "TIM_RX_FMC1_PULSES4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Evt4String             "TIM_RX_FMC1_EVT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Dly4String             "TIM_RX_FMC1_DLY4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc1Wdt4String             "TIM_RX_FMC1_WDT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2En0String             "TIM_RX_FMC2_EN0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pol0String             "TIM_RX_FMC2_POL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Log0String             "TIM_RX_FMC2_LOG0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Itl0String             "TIM_RX_FMC2_ITL0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Src0String             "TIM_RX_FMC2_SRC0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pulses0String             "TIM_RX_FMC2_PULSES0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Evt0String             "TIM_RX_FMC2_EVT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Dly0String             "TIM_RX_FMC2_DLY0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Wdt0String             "TIM_RX_FMC2_WDT0"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2En1String             "TIM_RX_FMC2_EN1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pol1String             "TIM_RX_FMC2_POL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Log1String             "TIM_RX_FMC2_LOG1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Itl1String             "TIM_RX_FMC2_ITL1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Src1String             "TIM_RX_FMC2_SRC1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pulses1String             "TIM_RX_FMC2_PULSES1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Evt1String             "TIM_RX_FMC2_EVT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Dly1String             "TIM_RX_FMC2_DLY1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Wdt1String             "TIM_RX_FMC2_WDT1"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2En2String             "TIM_RX_FMC2_EN2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pol2String             "TIM_RX_FMC2_POL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Log2String             "TIM_RX_FMC2_LOG2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Itl2String             "TIM_RX_FMC2_ITL2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Src2String             "TIM_RX_FMC2_SRC2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pulses2String             "TIM_RX_FMC2_PULSES2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Evt2String             "TIM_RX_FMC2_EVT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Dly2String             "TIM_RX_FMC2_DLY2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Wdt2String             "TIM_RX_FMC2_WDT2"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2En3String             "TIM_RX_FMC2_EN3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pol3String             "TIM_RX_FMC2_POL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Log3String             "TIM_RX_FMC2_LOG3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Itl3String             "TIM_RX_FMC2_ITL3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Src3String             "TIM_RX_FMC2_SRC3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pulses3String             "TIM_RX_FMC2_PULSES3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Evt3String             "TIM_RX_FMC2_EVT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Dly3String             "TIM_RX_FMC2_DLY3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Wdt3String             "TIM_RX_FMC2_WDT3"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2En4String             "TIM_RX_FMC2_EN4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pol4String             "TIM_RX_FMC2_POL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Log4String             "TIM_RX_FMC2_LOG4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Itl4String             "TIM_RX_FMC2_ITL4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Src4String             "TIM_RX_FMC2_SRC4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Pulses4String             "TIM_RX_FMC2_PULSES4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Evt4String             "TIM_RX_FMC2_EVT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Dly4String             "TIM_RX_FMC2_DLY4"      /* asynUInt32Digital,  r/w */
#define P_TimRxFmc2Wdt4String             "TIM_RX_FMC2_WDT4"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmFreqKpString             "TIM_RX_RTM_FREQ_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmFreqKiString             "TIM_RX_RTM_FREQ_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseKpString             "TIM_RX_RTM_PHASE_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseKiString             "TIM_RX_RTM_PHASE_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseSetString             "TIM_RX_RTM_PHASE_SET"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseNavgString             "TIM_RX_RTM_PHASE_NAVG"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmPhaseDivExpString             "TIM_RX_RTM_PHASE_DIV_EXP"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmRfreqHiString             "TIM_RX_RTM_RFREQ_HI"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmRfreqLoString             "TIM_RX_RTM_RFREQ_LO"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmN1String             "TIM_RX_RTM_N1"      /* asynUInt32Digital,  r/w */
#define P_TimRxRtmHsDivString             "TIM_RX_RTM_HS_DIV"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcFreqKpString             "TIM_RX_AFC_FREQ_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcFreqKiString             "TIM_RX_AFC_FREQ_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseKpString             "TIM_RX_AFC_PHASE_KP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseKiString             "TIM_RX_AFC_PHASE_KI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseSetString             "TIM_RX_AFC_PHASE_SET"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseNavgString             "TIM_RX_AFC_PHASE_NAVG"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcPhaseDivExpString             "TIM_RX_AFC_PHASE_DIV_EXP"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcRfreqHiString             "TIM_RX_AFC_RFREQ_HI"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcRfreqLoString             "TIM_RX_AFC_RFREQ_LO"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcN1String             "TIM_RX_AFC_N1"      /* asynUInt32Digital,  r/w */
#define P_TimRxAfcHsDivString             "TIM_RX_AFC_HS_DIV"      /* asynUInt32Digital,  r/w */

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
        int P_TimRxRxenStatus;
        int P_TimRxRefClkLocked;
        int P_TimRxAmcEn0;
        int P_TimRxAmcPol0;
        int P_TimRxAmcLog0;
        int P_TimRxAmcItl0;
        int P_TimRxAmcSrc0;
        int P_TimRxAmcDir0;
        int P_TimRxAmcPulses0;
        int P_TimRxAmcEvt0;
        int P_TimRxAmcDly0;
        int P_TimRxAmcWdt0;
        int P_TimRxAmcEn1;
        int P_TimRxAmcPol1;
        int P_TimRxAmcLog1;
        int P_TimRxAmcItl1;
        int P_TimRxAmcSrc1;
        int P_TimRxAmcDir1;
        int P_TimRxAmcPulses1;
        int P_TimRxAmcEvt1;
        int P_TimRxAmcDly1;
        int P_TimRxAmcWdt1;
        int P_TimRxAmcEn2;
        int P_TimRxAmcPol2;
        int P_TimRxAmcLog2;
        int P_TimRxAmcItl2;
        int P_TimRxAmcSrc2;
        int P_TimRxAmcDir2;
        int P_TimRxAmcPulses2;
        int P_TimRxAmcEvt2;
        int P_TimRxAmcDly2;
        int P_TimRxAmcWdt2;
        int P_TimRxAmcEn3;
        int P_TimRxAmcPol3;
        int P_TimRxAmcLog3;
        int P_TimRxAmcItl3;
        int P_TimRxAmcSrc3;
        int P_TimRxAmcDir3;
        int P_TimRxAmcPulses3;
        int P_TimRxAmcEvt3;
        int P_TimRxAmcDly3;
        int P_TimRxAmcWdt3;
        int P_TimRxAmcEn4;
        int P_TimRxAmcPol4;
        int P_TimRxAmcLog4;
        int P_TimRxAmcItl4;
        int P_TimRxAmcSrc4;
        int P_TimRxAmcDir4;
        int P_TimRxAmcPulses4;
        int P_TimRxAmcEvt4;
        int P_TimRxAmcDly4;
        int P_TimRxAmcWdt4;
        int P_TimRxAmcEn5;
        int P_TimRxAmcPol5;
        int P_TimRxAmcLog5;
        int P_TimRxAmcItl5;
        int P_TimRxAmcSrc5;
        int P_TimRxAmcDir5;
        int P_TimRxAmcPulses5;
        int P_TimRxAmcEvt5;
        int P_TimRxAmcDly5;
        int P_TimRxAmcWdt5;
        int P_TimRxAmcEn6;
        int P_TimRxAmcPol6;
        int P_TimRxAmcLog6;
        int P_TimRxAmcItl6;
        int P_TimRxAmcSrc6;
        int P_TimRxAmcDir6;
        int P_TimRxAmcPulses6;
        int P_TimRxAmcEvt6;
        int P_TimRxAmcDly6;
        int P_TimRxAmcWdt6;
        int P_TimRxAmcEn7;
        int P_TimRxAmcPol7;
        int P_TimRxAmcLog7;
        int P_TimRxAmcItl7;
        int P_TimRxAmcSrc7;
        int P_TimRxAmcDir7;
        int P_TimRxAmcPulses7;
        int P_TimRxAmcEvt7;
        int P_TimRxAmcDly7;
        int P_TimRxAmcWdt7;
        int P_TimRxFmc1En0;
        int P_TimRxFmc1Pol0;
        int P_TimRxFmc1Log0;
        int P_TimRxFmc1Itl0;
        int P_TimRxFmc1Src0;
        int P_TimRxFmc1Pulses0;
        int P_TimRxFmc1Evt0;
        int P_TimRxFmc1Dly0;
        int P_TimRxFmc1Wdt0;
        int P_TimRxFmc1En1;
        int P_TimRxFmc1Pol1;
        int P_TimRxFmc1Log1;
        int P_TimRxFmc1Itl1;
        int P_TimRxFmc1Src1;
        int P_TimRxFmc1Pulses1;
        int P_TimRxFmc1Evt1;
        int P_TimRxFmc1Dly1;
        int P_TimRxFmc1Wdt1;
        int P_TimRxFmc1En2;
        int P_TimRxFmc1Pol2;
        int P_TimRxFmc1Log2;
        int P_TimRxFmc1Itl2;
        int P_TimRxFmc1Src2;
        int P_TimRxFmc1Pulses2;
        int P_TimRxFmc1Evt2;
        int P_TimRxFmc1Dly2;
        int P_TimRxFmc1Wdt2;
        int P_TimRxFmc1En3;
        int P_TimRxFmc1Pol3;
        int P_TimRxFmc1Log3;
        int P_TimRxFmc1Itl3;
        int P_TimRxFmc1Src3;
        int P_TimRxFmc1Pulses3;
        int P_TimRxFmc1Evt3;
        int P_TimRxFmc1Dly3;
        int P_TimRxFmc1Wdt3;
        int P_TimRxFmc1En4;
        int P_TimRxFmc1Pol4;
        int P_TimRxFmc1Log4;
        int P_TimRxFmc1Itl4;
        int P_TimRxFmc1Src4;
        int P_TimRxFmc1Pulses4;
        int P_TimRxFmc1Evt4;
        int P_TimRxFmc1Dly4;
        int P_TimRxFmc1Wdt4;
        int P_TimRxFmc2En0;
        int P_TimRxFmc2Pol0;
        int P_TimRxFmc2Log0;
        int P_TimRxFmc2Itl0;
        int P_TimRxFmc2Src0;
        int P_TimRxFmc2Pulses0;
        int P_TimRxFmc2Evt0;
        int P_TimRxFmc2Dly0;
        int P_TimRxFmc2Wdt0;
        int P_TimRxFmc2En1;
        int P_TimRxFmc2Pol1;
        int P_TimRxFmc2Log1;
        int P_TimRxFmc2Itl1;
        int P_TimRxFmc2Src1;
        int P_TimRxFmc2Pulses1;
        int P_TimRxFmc2Evt1;
        int P_TimRxFmc2Dly1;
        int P_TimRxFmc2Wdt1;
        int P_TimRxFmc2En2;
        int P_TimRxFmc2Pol2;
        int P_TimRxFmc2Log2;
        int P_TimRxFmc2Itl2;
        int P_TimRxFmc2Src2;
        int P_TimRxFmc2Pulses2;
        int P_TimRxFmc2Evt2;
        int P_TimRxFmc2Dly2;
        int P_TimRxFmc2Wdt2;
        int P_TimRxFmc2En3;
        int P_TimRxFmc2Pol3;
        int P_TimRxFmc2Log3;
        int P_TimRxFmc2Itl3;
        int P_TimRxFmc2Src3;
        int P_TimRxFmc2Pulses3;
        int P_TimRxFmc2Evt3;
        int P_TimRxFmc2Dly3;
        int P_TimRxFmc2Wdt3;
        int P_TimRxFmc2En4;
        int P_TimRxFmc2Pol4;
        int P_TimRxFmc2Log4;
        int P_TimRxFmc2Itl4;
        int P_TimRxFmc2Src4;
        int P_TimRxFmc2Pulses4;
        int P_TimRxFmc2Evt4;
        int P_TimRxFmc2Dly4;
        int P_TimRxFmc2Wdt4;
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
#define LAST_COMMAND P_TimRxAfcHsDiv

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
