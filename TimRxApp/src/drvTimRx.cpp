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
static const functionsAny_t timRxSetGetRxenStatusFunc = {functionsInt32_t{"LNLS_AFC_TIMING", NULL, afc_timing_get_rxen_status}};
static const functionsAny_t timRxSetGetRefClkLockedFunc = {functionsInt32_t{"LNLS_AFC_TIMING", NULL, afc_timing_get_ref_clk_locked}};
static const functionsAny_t timRxSetGetAmcEn0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en0, afc_timing_get_amc_en0}};
static const functionsAny_t timRxSetGetAmcPol0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol0, afc_timing_get_amc_pol0}};
static const functionsAny_t timRxSetGetAmcLog0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log0, afc_timing_get_amc_log0}};
static const functionsAny_t timRxSetGetAmcItl0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl0, afc_timing_get_amc_itl0}};
static const functionsAny_t timRxSetGetAmcSrc0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src0, afc_timing_get_amc_src0}};
static const functionsAny_t timRxSetGetAmcPulses0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses0, afc_timing_get_amc_pulses0}};
static const functionsAny_t timRxSetGetAmcEvt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt0, afc_timing_get_amc_evt0}};
static const functionsAny_t timRxSetGetAmcDly0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly0, afc_timing_get_amc_dly0}};
static const functionsAny_t timRxSetGetAmcWdt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt0, afc_timing_get_amc_wdt0}};
static const functionsAny_t timRxSetGetAmcEn1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en1, afc_timing_get_amc_en1}};
static const functionsAny_t timRxSetGetAmcPol1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol1, afc_timing_get_amc_pol1}};
static const functionsAny_t timRxSetGetAmcLog1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log1, afc_timing_get_amc_log1}};
static const functionsAny_t timRxSetGetAmcItl1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl1, afc_timing_get_amc_itl1}};
static const functionsAny_t timRxSetGetAmcSrc1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src1, afc_timing_get_amc_src1}};
static const functionsAny_t timRxSetGetAmcPulses1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses1, afc_timing_get_amc_pulses1}};
static const functionsAny_t timRxSetGetAmcEvt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt1, afc_timing_get_amc_evt1}};
static const functionsAny_t timRxSetGetAmcDly1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly1, afc_timing_get_amc_dly1}};
static const functionsAny_t timRxSetGetAmcWdt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt1, afc_timing_get_amc_wdt1}};
static const functionsAny_t timRxSetGetAmcEn2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en2, afc_timing_get_amc_en2}};
static const functionsAny_t timRxSetGetAmcPol2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol2, afc_timing_get_amc_pol2}};
static const functionsAny_t timRxSetGetAmcLog2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log2, afc_timing_get_amc_log2}};
static const functionsAny_t timRxSetGetAmcItl2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl2, afc_timing_get_amc_itl2}};
static const functionsAny_t timRxSetGetAmcSrc2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src2, afc_timing_get_amc_src2}};
static const functionsAny_t timRxSetGetAmcPulses2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses2, afc_timing_get_amc_pulses2}};
static const functionsAny_t timRxSetGetAmcEvt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt2, afc_timing_get_amc_evt2}};
static const functionsAny_t timRxSetGetAmcDly2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly2, afc_timing_get_amc_dly2}};
static const functionsAny_t timRxSetGetAmcWdt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt2, afc_timing_get_amc_wdt2}};
static const functionsAny_t timRxSetGetAmcEn3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en3, afc_timing_get_amc_en3}};
static const functionsAny_t timRxSetGetAmcPol3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol3, afc_timing_get_amc_pol3}};
static const functionsAny_t timRxSetGetAmcLog3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log3, afc_timing_get_amc_log3}};
static const functionsAny_t timRxSetGetAmcItl3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl3, afc_timing_get_amc_itl3}};
static const functionsAny_t timRxSetGetAmcSrc3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src3, afc_timing_get_amc_src3}};
static const functionsAny_t timRxSetGetAmcPulses3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses3, afc_timing_get_amc_pulses3}};
static const functionsAny_t timRxSetGetAmcEvt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt3, afc_timing_get_amc_evt3}};
static const functionsAny_t timRxSetGetAmcDly3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly3, afc_timing_get_amc_dly3}};
static const functionsAny_t timRxSetGetAmcWdt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt3, afc_timing_get_amc_wdt3}};
static const functionsAny_t timRxSetGetAmcEn4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en4, afc_timing_get_amc_en4}};
static const functionsAny_t timRxSetGetAmcPol4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol4, afc_timing_get_amc_pol4}};
static const functionsAny_t timRxSetGetAmcLog4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log4, afc_timing_get_amc_log4}};
static const functionsAny_t timRxSetGetAmcItl4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl4, afc_timing_get_amc_itl4}};
static const functionsAny_t timRxSetGetAmcSrc4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src4, afc_timing_get_amc_src4}};
static const functionsAny_t timRxSetGetAmcPulses4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses4, afc_timing_get_amc_pulses4}};
static const functionsAny_t timRxSetGetAmcEvt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt4, afc_timing_get_amc_evt4}};
static const functionsAny_t timRxSetGetAmcDly4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly4, afc_timing_get_amc_dly4}};
static const functionsAny_t timRxSetGetAmcWdt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt4, afc_timing_get_amc_wdt4}};
static const functionsAny_t timRxSetGetAmcEn5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en5, afc_timing_get_amc_en5}};
static const functionsAny_t timRxSetGetAmcPol5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol5, afc_timing_get_amc_pol5}};
static const functionsAny_t timRxSetGetAmcLog5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log5, afc_timing_get_amc_log5}};
static const functionsAny_t timRxSetGetAmcItl5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl5, afc_timing_get_amc_itl5}};
static const functionsAny_t timRxSetGetAmcSrc5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src5, afc_timing_get_amc_src5}};
static const functionsAny_t timRxSetGetAmcPulses5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses5, afc_timing_get_amc_pulses5}};
static const functionsAny_t timRxSetGetAmcEvt5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt5, afc_timing_get_amc_evt5}};
static const functionsAny_t timRxSetGetAmcDly5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly5, afc_timing_get_amc_dly5}};
static const functionsAny_t timRxSetGetAmcWdt5Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt5, afc_timing_get_amc_wdt5}};
static const functionsAny_t timRxSetGetAmcEn6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en6, afc_timing_get_amc_en6}};
static const functionsAny_t timRxSetGetAmcPol6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol6, afc_timing_get_amc_pol6}};
static const functionsAny_t timRxSetGetAmcLog6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log6, afc_timing_get_amc_log6}};
static const functionsAny_t timRxSetGetAmcItl6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl6, afc_timing_get_amc_itl6}};
static const functionsAny_t timRxSetGetAmcSrc6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src6, afc_timing_get_amc_src6}};
static const functionsAny_t timRxSetGetAmcPulses6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses6, afc_timing_get_amc_pulses6}};
static const functionsAny_t timRxSetGetAmcEvt6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt6, afc_timing_get_amc_evt6}};
static const functionsAny_t timRxSetGetAmcDly6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly6, afc_timing_get_amc_dly6}};
static const functionsAny_t timRxSetGetAmcWdt6Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt6, afc_timing_get_amc_wdt6}};
static const functionsAny_t timRxSetGetAmcEn7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_en7, afc_timing_get_amc_en7}};
static const functionsAny_t timRxSetGetAmcPol7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pol7, afc_timing_get_amc_pol7}};
static const functionsAny_t timRxSetGetAmcLog7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_log7, afc_timing_get_amc_log7}};
static const functionsAny_t timRxSetGetAmcItl7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_itl7, afc_timing_get_amc_itl7}};
static const functionsAny_t timRxSetGetAmcSrc7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_src7, afc_timing_get_amc_src7}};
static const functionsAny_t timRxSetGetAmcPulses7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_pulses7, afc_timing_get_amc_pulses7}};
static const functionsAny_t timRxSetGetAmcEvt7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_evt7, afc_timing_get_amc_evt7}};
static const functionsAny_t timRxSetGetAmcDly7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_dly7, afc_timing_get_amc_dly7}};
static const functionsAny_t timRxSetGetAmcWdt7Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_amc_wdt7, afc_timing_get_amc_wdt7}};
static const functionsAny_t timRxSetGetFmc1En0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_en0, afc_timing_get_fmc1_en0}};
static const functionsAny_t timRxSetGetFmc1Pol0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pol0, afc_timing_get_fmc1_pol0}};
static const functionsAny_t timRxSetGetFmc1Log0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_log0, afc_timing_get_fmc1_log0}};
static const functionsAny_t timRxSetGetFmc1Itl0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_itl0, afc_timing_get_fmc1_itl0}};
static const functionsAny_t timRxSetGetFmc1Src0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_src0, afc_timing_get_fmc1_src0}};
static const functionsAny_t timRxSetGetFmc1Pulses0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pulses0, afc_timing_get_fmc1_pulses0}};
static const functionsAny_t timRxSetGetFmc1Evt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_evt0, afc_timing_get_fmc1_evt0}};
static const functionsAny_t timRxSetGetFmc1Dly0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_dly0, afc_timing_get_fmc1_dly0}};
static const functionsAny_t timRxSetGetFmc1Wdt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_wdt0, afc_timing_get_fmc1_wdt0}};
static const functionsAny_t timRxSetGetFmc1En1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_en1, afc_timing_get_fmc1_en1}};
static const functionsAny_t timRxSetGetFmc1Pol1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pol1, afc_timing_get_fmc1_pol1}};
static const functionsAny_t timRxSetGetFmc1Log1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_log1, afc_timing_get_fmc1_log1}};
static const functionsAny_t timRxSetGetFmc1Itl1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_itl1, afc_timing_get_fmc1_itl1}};
static const functionsAny_t timRxSetGetFmc1Src1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_src1, afc_timing_get_fmc1_src1}};
static const functionsAny_t timRxSetGetFmc1Pulses1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pulses1, afc_timing_get_fmc1_pulses1}};
static const functionsAny_t timRxSetGetFmc1Evt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_evt1, afc_timing_get_fmc1_evt1}};
static const functionsAny_t timRxSetGetFmc1Dly1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_dly1, afc_timing_get_fmc1_dly1}};
static const functionsAny_t timRxSetGetFmc1Wdt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_wdt1, afc_timing_get_fmc1_wdt1}};
static const functionsAny_t timRxSetGetFmc1En2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_en2, afc_timing_get_fmc1_en2}};
static const functionsAny_t timRxSetGetFmc1Pol2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pol2, afc_timing_get_fmc1_pol2}};
static const functionsAny_t timRxSetGetFmc1Log2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_log2, afc_timing_get_fmc1_log2}};
static const functionsAny_t timRxSetGetFmc1Itl2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_itl2, afc_timing_get_fmc1_itl2}};
static const functionsAny_t timRxSetGetFmc1Src2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_src2, afc_timing_get_fmc1_src2}};
static const functionsAny_t timRxSetGetFmc1Pulses2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pulses2, afc_timing_get_fmc1_pulses2}};
static const functionsAny_t timRxSetGetFmc1Evt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_evt2, afc_timing_get_fmc1_evt2}};
static const functionsAny_t timRxSetGetFmc1Dly2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_dly2, afc_timing_get_fmc1_dly2}};
static const functionsAny_t timRxSetGetFmc1Wdt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_wdt2, afc_timing_get_fmc1_wdt2}};
static const functionsAny_t timRxSetGetFmc1En3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_en3, afc_timing_get_fmc1_en3}};
static const functionsAny_t timRxSetGetFmc1Pol3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pol3, afc_timing_get_fmc1_pol3}};
static const functionsAny_t timRxSetGetFmc1Log3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_log3, afc_timing_get_fmc1_log3}};
static const functionsAny_t timRxSetGetFmc1Itl3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_itl3, afc_timing_get_fmc1_itl3}};
static const functionsAny_t timRxSetGetFmc1Src3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_src3, afc_timing_get_fmc1_src3}};
static const functionsAny_t timRxSetGetFmc1Pulses3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pulses3, afc_timing_get_fmc1_pulses3}};
static const functionsAny_t timRxSetGetFmc1Evt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_evt3, afc_timing_get_fmc1_evt3}};
static const functionsAny_t timRxSetGetFmc1Dly3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_dly3, afc_timing_get_fmc1_dly3}};
static const functionsAny_t timRxSetGetFmc1Wdt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_wdt3, afc_timing_get_fmc1_wdt3}};
static const functionsAny_t timRxSetGetFmc1En4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_en4, afc_timing_get_fmc1_en4}};
static const functionsAny_t timRxSetGetFmc1Pol4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pol4, afc_timing_get_fmc1_pol4}};
static const functionsAny_t timRxSetGetFmc1Log4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_log4, afc_timing_get_fmc1_log4}};
static const functionsAny_t timRxSetGetFmc1Itl4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_itl4, afc_timing_get_fmc1_itl4}};
static const functionsAny_t timRxSetGetFmc1Src4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_src4, afc_timing_get_fmc1_src4}};
static const functionsAny_t timRxSetGetFmc1Pulses4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_pulses4, afc_timing_get_fmc1_pulses4}};
static const functionsAny_t timRxSetGetFmc1Evt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_evt4, afc_timing_get_fmc1_evt4}};
static const functionsAny_t timRxSetGetFmc1Dly4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_dly4, afc_timing_get_fmc1_dly4}};
static const functionsAny_t timRxSetGetFmc1Wdt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc1_wdt4, afc_timing_get_fmc1_wdt4}};
static const functionsAny_t timRxSetGetFmc2En0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_en0, afc_timing_get_fmc2_en0}};
static const functionsAny_t timRxSetGetFmc2Pol0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pol0, afc_timing_get_fmc2_pol0}};
static const functionsAny_t timRxSetGetFmc2Log0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_log0, afc_timing_get_fmc2_log0}};
static const functionsAny_t timRxSetGetFmc2Itl0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_itl0, afc_timing_get_fmc2_itl0}};
static const functionsAny_t timRxSetGetFmc2Src0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_src0, afc_timing_get_fmc2_src0}};
static const functionsAny_t timRxSetGetFmc2Pulses0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pulses0, afc_timing_get_fmc2_pulses0}};
static const functionsAny_t timRxSetGetFmc2Evt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_evt0, afc_timing_get_fmc2_evt0}};
static const functionsAny_t timRxSetGetFmc2Dly0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_dly0, afc_timing_get_fmc2_dly0}};
static const functionsAny_t timRxSetGetFmc2Wdt0Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_wdt0, afc_timing_get_fmc2_wdt0}};
static const functionsAny_t timRxSetGetFmc2En1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_en1, afc_timing_get_fmc2_en1}};
static const functionsAny_t timRxSetGetFmc2Pol1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pol1, afc_timing_get_fmc2_pol1}};
static const functionsAny_t timRxSetGetFmc2Log1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_log1, afc_timing_get_fmc2_log1}};
static const functionsAny_t timRxSetGetFmc2Itl1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_itl1, afc_timing_get_fmc2_itl1}};
static const functionsAny_t timRxSetGetFmc2Src1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_src1, afc_timing_get_fmc2_src1}};
static const functionsAny_t timRxSetGetFmc2Pulses1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pulses1, afc_timing_get_fmc2_pulses1}};
static const functionsAny_t timRxSetGetFmc2Evt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_evt1, afc_timing_get_fmc2_evt1}};
static const functionsAny_t timRxSetGetFmc2Dly1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_dly1, afc_timing_get_fmc2_dly1}};
static const functionsAny_t timRxSetGetFmc2Wdt1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_wdt1, afc_timing_get_fmc2_wdt1}};
static const functionsAny_t timRxSetGetFmc2En2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_en2, afc_timing_get_fmc2_en2}};
static const functionsAny_t timRxSetGetFmc2Pol2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pol2, afc_timing_get_fmc2_pol2}};
static const functionsAny_t timRxSetGetFmc2Log2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_log2, afc_timing_get_fmc2_log2}};
static const functionsAny_t timRxSetGetFmc2Itl2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_itl2, afc_timing_get_fmc2_itl2}};
static const functionsAny_t timRxSetGetFmc2Src2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_src2, afc_timing_get_fmc2_src2}};
static const functionsAny_t timRxSetGetFmc2Pulses2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pulses2, afc_timing_get_fmc2_pulses2}};
static const functionsAny_t timRxSetGetFmc2Evt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_evt2, afc_timing_get_fmc2_evt2}};
static const functionsAny_t timRxSetGetFmc2Dly2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_dly2, afc_timing_get_fmc2_dly2}};
static const functionsAny_t timRxSetGetFmc2Wdt2Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_wdt2, afc_timing_get_fmc2_wdt2}};
static const functionsAny_t timRxSetGetFmc2En3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_en3, afc_timing_get_fmc2_en3}};
static const functionsAny_t timRxSetGetFmc2Pol3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pol3, afc_timing_get_fmc2_pol3}};
static const functionsAny_t timRxSetGetFmc2Log3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_log3, afc_timing_get_fmc2_log3}};
static const functionsAny_t timRxSetGetFmc2Itl3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_itl3, afc_timing_get_fmc2_itl3}};
static const functionsAny_t timRxSetGetFmc2Src3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_src3, afc_timing_get_fmc2_src3}};
static const functionsAny_t timRxSetGetFmc2Pulses3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pulses3, afc_timing_get_fmc2_pulses3}};
static const functionsAny_t timRxSetGetFmc2Evt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_evt3, afc_timing_get_fmc2_evt3}};
static const functionsAny_t timRxSetGetFmc2Dly3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_dly3, afc_timing_get_fmc2_dly3}};
static const functionsAny_t timRxSetGetFmc2Wdt3Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_wdt3, afc_timing_get_fmc2_wdt3}};
static const functionsAny_t timRxSetGetFmc2En4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_en4, afc_timing_get_fmc2_en4}};
static const functionsAny_t timRxSetGetFmc2Pol4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pol4, afc_timing_get_fmc2_pol4}};
static const functionsAny_t timRxSetGetFmc2Log4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_log4, afc_timing_get_fmc2_log4}};
static const functionsAny_t timRxSetGetFmc2Itl4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_itl4, afc_timing_get_fmc2_itl4}};
static const functionsAny_t timRxSetGetFmc2Src4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_src4, afc_timing_get_fmc2_src4}};
static const functionsAny_t timRxSetGetFmc2Pulses4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_pulses4, afc_timing_get_fmc2_pulses4}};
static const functionsAny_t timRxSetGetFmc2Evt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_evt4, afc_timing_get_fmc2_evt4}};
static const functionsAny_t timRxSetGetFmc2Dly4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_dly4, afc_timing_get_fmc2_dly4}};
static const functionsAny_t timRxSetGetFmc2Wdt4Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_fmc2_wdt4, afc_timing_get_fmc2_wdt4}};
static const functionsAny_t timRxSetGetRtmFreqKpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_freq_kp, afc_timing_get_rtm_freq_kp}};
static const functionsAny_t timRxSetGetRtmFreqKiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_freq_ki, afc_timing_get_rtm_freq_ki}};
static const functionsAny_t timRxSetGetRtmPhaseKpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_kp, afc_timing_get_rtm_phase_kp}};
static const functionsAny_t timRxSetGetRtmPhaseKiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_ki, afc_timing_get_rtm_phase_ki}};
static const functionsAny_t timRxSetGetRtmPhaseSetFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_set, afc_timing_get_rtm_phase_set}};
static const functionsAny_t timRxSetGetRtmPhaseNavgFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_navg, afc_timing_get_rtm_phase_navg}};
static const functionsAny_t timRxSetGetRtmPhaseDivExpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_phase_div_exp, afc_timing_get_rtm_phase_div_exp}};
static const functionsAny_t timRxSetGetRtmRfreqHiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_rfreq_hi, afc_timing_get_rtm_rfreq_hi}};
static const functionsAny_t timRxSetGetRtmRfreqLoFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_rfreq_lo, afc_timing_get_rtm_rfreq_lo}};
static const functionsAny_t timRxSetGetRtmN1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_n1, afc_timing_get_rtm_n1}};
static const functionsAny_t timRxSetGetRtmHsDivFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_rtm_hs_div, afc_timing_get_rtm_hs_div}};
static const functionsAny_t timRxSetGetAfcFreqKpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_freq_kp, afc_timing_get_afc_freq_kp}};
static const functionsAny_t timRxSetGetAfcFreqKiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_freq_ki, afc_timing_get_afc_freq_ki}};
static const functionsAny_t timRxSetGetAfcPhaseKpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_kp, afc_timing_get_afc_phase_kp}};
static const functionsAny_t timRxSetGetAfcPhaseKiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_ki, afc_timing_get_afc_phase_ki}};
static const functionsAny_t timRxSetGetAfcPhaseSetFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_set, afc_timing_get_afc_phase_set}};
static const functionsAny_t timRxSetGetAfcPhaseNavgFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_navg, afc_timing_get_afc_phase_navg}};
static const functionsAny_t timRxSetGetAfcPhaseDivExpFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_phase_div_exp, afc_timing_get_afc_phase_div_exp}};
static const functionsAny_t timRxSetGetAfcRfreqHiFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_rfreq_hi, afc_timing_get_afc_rfreq_hi}};
static const functionsAny_t timRxSetGetAfcRfreqLoFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_rfreq_lo, afc_timing_get_afc_rfreq_lo}};
static const functionsAny_t timRxSetGetAfcN1Func = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_n1, afc_timing_get_afc_n1}};
static const functionsAny_t timRxSetGetAfcHsDivFunc = {functionsInt32_t{"LNLS_AFC_TIMING", afc_timing_set_afc_hs_div, afc_timing_get_afc_hs_div}};

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
    createParam(P_TimRxLinkStatusString,   asynParamUInt32Digital,         &P_TimRxLinkStatus);
    createParam(P_TimRxRxenStatusString,   asynParamUInt32Digital,         &P_TimRxRxenStatus);
    createParam(P_TimRxRefClkLockedString,   asynParamUInt32Digital,         &P_TimRxRefClkLocked);
    createParam(P_TimRxAmcEn0String,   asynParamUInt32Digital,         &P_TimRxAmcEn0);
    createParam(P_TimRxAmcPol0String,   asynParamUInt32Digital,         &P_TimRxAmcPol0);
    createParam(P_TimRxAmcLog0String,   asynParamUInt32Digital,         &P_TimRxAmcLog0);
    createParam(P_TimRxAmcItl0String,   asynParamUInt32Digital,         &P_TimRxAmcItl0);
    createParam(P_TimRxAmcSrc0String,   asynParamUInt32Digital,         &P_TimRxAmcSrc0);
    createParam(P_TimRxAmcPulses0String,   asynParamUInt32Digital,         &P_TimRxAmcPulses0);
    createParam(P_TimRxAmcEvt0String,   asynParamUInt32Digital,         &P_TimRxAmcEvt0);
    createParam(P_TimRxAmcDly0String,   asynParamUInt32Digital,         &P_TimRxAmcDly0);
    createParam(P_TimRxAmcWdt0String,   asynParamUInt32Digital,         &P_TimRxAmcWdt0);
    createParam(P_TimRxAmcEn1String,   asynParamUInt32Digital,         &P_TimRxAmcEn1);
    createParam(P_TimRxAmcPol1String,   asynParamUInt32Digital,         &P_TimRxAmcPol1);
    createParam(P_TimRxAmcLog1String,   asynParamUInt32Digital,         &P_TimRxAmcLog1);
    createParam(P_TimRxAmcItl1String,   asynParamUInt32Digital,         &P_TimRxAmcItl1);
    createParam(P_TimRxAmcSrc1String,   asynParamUInt32Digital,         &P_TimRxAmcSrc1);
    createParam(P_TimRxAmcPulses1String,   asynParamUInt32Digital,         &P_TimRxAmcPulses1);
    createParam(P_TimRxAmcEvt1String,   asynParamUInt32Digital,         &P_TimRxAmcEvt1);
    createParam(P_TimRxAmcDly1String,   asynParamUInt32Digital,         &P_TimRxAmcDly1);
    createParam(P_TimRxAmcWdt1String,   asynParamUInt32Digital,         &P_TimRxAmcWdt1);
    createParam(P_TimRxAmcEn2String,   asynParamUInt32Digital,         &P_TimRxAmcEn2);
    createParam(P_TimRxAmcPol2String,   asynParamUInt32Digital,         &P_TimRxAmcPol2);
    createParam(P_TimRxAmcLog2String,   asynParamUInt32Digital,         &P_TimRxAmcLog2);
    createParam(P_TimRxAmcItl2String,   asynParamUInt32Digital,         &P_TimRxAmcItl2);
    createParam(P_TimRxAmcSrc2String,   asynParamUInt32Digital,         &P_TimRxAmcSrc2);
    createParam(P_TimRxAmcPulses2String,   asynParamUInt32Digital,         &P_TimRxAmcPulses2);
    createParam(P_TimRxAmcEvt2String,   asynParamUInt32Digital,         &P_TimRxAmcEvt2);
    createParam(P_TimRxAmcDly2String,   asynParamUInt32Digital,         &P_TimRxAmcDly2);
    createParam(P_TimRxAmcWdt2String,   asynParamUInt32Digital,         &P_TimRxAmcWdt2);
    createParam(P_TimRxAmcEn3String,   asynParamUInt32Digital,         &P_TimRxAmcEn3);
    createParam(P_TimRxAmcPol3String,   asynParamUInt32Digital,         &P_TimRxAmcPol3);
    createParam(P_TimRxAmcLog3String,   asynParamUInt32Digital,         &P_TimRxAmcLog3);
    createParam(P_TimRxAmcItl3String,   asynParamUInt32Digital,         &P_TimRxAmcItl3);
    createParam(P_TimRxAmcSrc3String,   asynParamUInt32Digital,         &P_TimRxAmcSrc3);
    createParam(P_TimRxAmcPulses3String,   asynParamUInt32Digital,         &P_TimRxAmcPulses3);
    createParam(P_TimRxAmcEvt3String,   asynParamUInt32Digital,         &P_TimRxAmcEvt3);
    createParam(P_TimRxAmcDly3String,   asynParamUInt32Digital,         &P_TimRxAmcDly3);
    createParam(P_TimRxAmcWdt3String,   asynParamUInt32Digital,         &P_TimRxAmcWdt3);
    createParam(P_TimRxAmcEn4String,   asynParamUInt32Digital,         &P_TimRxAmcEn4);
    createParam(P_TimRxAmcPol4String,   asynParamUInt32Digital,         &P_TimRxAmcPol4);
    createParam(P_TimRxAmcLog4String,   asynParamUInt32Digital,         &P_TimRxAmcLog4);
    createParam(P_TimRxAmcItl4String,   asynParamUInt32Digital,         &P_TimRxAmcItl4);
    createParam(P_TimRxAmcSrc4String,   asynParamUInt32Digital,         &P_TimRxAmcSrc4);
    createParam(P_TimRxAmcPulses4String,   asynParamUInt32Digital,         &P_TimRxAmcPulses4);
    createParam(P_TimRxAmcEvt4String,   asynParamUInt32Digital,         &P_TimRxAmcEvt4);
    createParam(P_TimRxAmcDly4String,   asynParamUInt32Digital,         &P_TimRxAmcDly4);
    createParam(P_TimRxAmcWdt4String,   asynParamUInt32Digital,         &P_TimRxAmcWdt4);
    createParam(P_TimRxAmcEn5String,   asynParamUInt32Digital,         &P_TimRxAmcEn5);
    createParam(P_TimRxAmcPol5String,   asynParamUInt32Digital,         &P_TimRxAmcPol5);
    createParam(P_TimRxAmcLog5String,   asynParamUInt32Digital,         &P_TimRxAmcLog5);
    createParam(P_TimRxAmcItl5String,   asynParamUInt32Digital,         &P_TimRxAmcItl5);
    createParam(P_TimRxAmcSrc5String,   asynParamUInt32Digital,         &P_TimRxAmcSrc5);
    createParam(P_TimRxAmcPulses5String,   asynParamUInt32Digital,         &P_TimRxAmcPulses5);
    createParam(P_TimRxAmcEvt5String,   asynParamUInt32Digital,         &P_TimRxAmcEvt5);
    createParam(P_TimRxAmcDly5String,   asynParamUInt32Digital,         &P_TimRxAmcDly5);
    createParam(P_TimRxAmcWdt5String,   asynParamUInt32Digital,         &P_TimRxAmcWdt5);
    createParam(P_TimRxAmcEn6String,   asynParamUInt32Digital,         &P_TimRxAmcEn6);
    createParam(P_TimRxAmcPol6String,   asynParamUInt32Digital,         &P_TimRxAmcPol6);
    createParam(P_TimRxAmcLog6String,   asynParamUInt32Digital,         &P_TimRxAmcLog6);
    createParam(P_TimRxAmcItl6String,   asynParamUInt32Digital,         &P_TimRxAmcItl6);
    createParam(P_TimRxAmcSrc6String,   asynParamUInt32Digital,         &P_TimRxAmcSrc6);
    createParam(P_TimRxAmcPulses6String,   asynParamUInt32Digital,         &P_TimRxAmcPulses6);
    createParam(P_TimRxAmcEvt6String,   asynParamUInt32Digital,         &P_TimRxAmcEvt6);
    createParam(P_TimRxAmcDly6String,   asynParamUInt32Digital,         &P_TimRxAmcDly6);
    createParam(P_TimRxAmcWdt6String,   asynParamUInt32Digital,         &P_TimRxAmcWdt6);
    createParam(P_TimRxAmcEn7String,   asynParamUInt32Digital,         &P_TimRxAmcEn7);
    createParam(P_TimRxAmcPol7String,   asynParamUInt32Digital,         &P_TimRxAmcPol7);
    createParam(P_TimRxAmcLog7String,   asynParamUInt32Digital,         &P_TimRxAmcLog7);
    createParam(P_TimRxAmcItl7String,   asynParamUInt32Digital,         &P_TimRxAmcItl7);
    createParam(P_TimRxAmcSrc7String,   asynParamUInt32Digital,         &P_TimRxAmcSrc7);
    createParam(P_TimRxAmcPulses7String,   asynParamUInt32Digital,         &P_TimRxAmcPulses7);
    createParam(P_TimRxAmcEvt7String,   asynParamUInt32Digital,         &P_TimRxAmcEvt7);
    createParam(P_TimRxAmcDly7String,   asynParamUInt32Digital,         &P_TimRxAmcDly7);
    createParam(P_TimRxAmcWdt7String,   asynParamUInt32Digital,         &P_TimRxAmcWdt7);
    createParam(P_TimRxFmc1En0String,   asynParamUInt32Digital,         &P_TimRxFmc1En0);
    createParam(P_TimRxFmc1Pol0String,   asynParamUInt32Digital,         &P_TimRxFmc1Pol0);
    createParam(P_TimRxFmc1Log0String,   asynParamUInt32Digital,         &P_TimRxFmc1Log0);
    createParam(P_TimRxFmc1Itl0String,   asynParamUInt32Digital,         &P_TimRxFmc1Itl0);
    createParam(P_TimRxFmc1Src0String,   asynParamUInt32Digital,         &P_TimRxFmc1Src0);
    createParam(P_TimRxFmc1Pulses0String,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses0);
    createParam(P_TimRxFmc1Evt0String,   asynParamUInt32Digital,         &P_TimRxFmc1Evt0);
    createParam(P_TimRxFmc1Dly0String,   asynParamUInt32Digital,         &P_TimRxFmc1Dly0);
    createParam(P_TimRxFmc1Wdt0String,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt0);
    createParam(P_TimRxFmc1En1String,   asynParamUInt32Digital,         &P_TimRxFmc1En1);
    createParam(P_TimRxFmc1Pol1String,   asynParamUInt32Digital,         &P_TimRxFmc1Pol1);
    createParam(P_TimRxFmc1Log1String,   asynParamUInt32Digital,         &P_TimRxFmc1Log1);
    createParam(P_TimRxFmc1Itl1String,   asynParamUInt32Digital,         &P_TimRxFmc1Itl1);
    createParam(P_TimRxFmc1Src1String,   asynParamUInt32Digital,         &P_TimRxFmc1Src1);
    createParam(P_TimRxFmc1Pulses1String,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses1);
    createParam(P_TimRxFmc1Evt1String,   asynParamUInt32Digital,         &P_TimRxFmc1Evt1);
    createParam(P_TimRxFmc1Dly1String,   asynParamUInt32Digital,         &P_TimRxFmc1Dly1);
    createParam(P_TimRxFmc1Wdt1String,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt1);
    createParam(P_TimRxFmc1En2String,   asynParamUInt32Digital,         &P_TimRxFmc1En2);
    createParam(P_TimRxFmc1Pol2String,   asynParamUInt32Digital,         &P_TimRxFmc1Pol2);
    createParam(P_TimRxFmc1Log2String,   asynParamUInt32Digital,         &P_TimRxFmc1Log2);
    createParam(P_TimRxFmc1Itl2String,   asynParamUInt32Digital,         &P_TimRxFmc1Itl2);
    createParam(P_TimRxFmc1Src2String,   asynParamUInt32Digital,         &P_TimRxFmc1Src2);
    createParam(P_TimRxFmc1Pulses2String,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses2);
    createParam(P_TimRxFmc1Evt2String,   asynParamUInt32Digital,         &P_TimRxFmc1Evt2);
    createParam(P_TimRxFmc1Dly2String,   asynParamUInt32Digital,         &P_TimRxFmc1Dly2);
    createParam(P_TimRxFmc1Wdt2String,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt2);
    createParam(P_TimRxFmc1En3String,   asynParamUInt32Digital,         &P_TimRxFmc1En3);
    createParam(P_TimRxFmc1Pol3String,   asynParamUInt32Digital,         &P_TimRxFmc1Pol3);
    createParam(P_TimRxFmc1Log3String,   asynParamUInt32Digital,         &P_TimRxFmc1Log3);
    createParam(P_TimRxFmc1Itl3String,   asynParamUInt32Digital,         &P_TimRxFmc1Itl3);
    createParam(P_TimRxFmc1Src3String,   asynParamUInt32Digital,         &P_TimRxFmc1Src3);
    createParam(P_TimRxFmc1Pulses3String,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses3);
    createParam(P_TimRxFmc1Evt3String,   asynParamUInt32Digital,         &P_TimRxFmc1Evt3);
    createParam(P_TimRxFmc1Dly3String,   asynParamUInt32Digital,         &P_TimRxFmc1Dly3);
    createParam(P_TimRxFmc1Wdt3String,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt3);
    createParam(P_TimRxFmc1En4String,   asynParamUInt32Digital,         &P_TimRxFmc1En4);
    createParam(P_TimRxFmc1Pol4String,   asynParamUInt32Digital,         &P_TimRxFmc1Pol4);
    createParam(P_TimRxFmc1Log4String,   asynParamUInt32Digital,         &P_TimRxFmc1Log4);
    createParam(P_TimRxFmc1Itl4String,   asynParamUInt32Digital,         &P_TimRxFmc1Itl4);
    createParam(P_TimRxFmc1Src4String,   asynParamUInt32Digital,         &P_TimRxFmc1Src4);
    createParam(P_TimRxFmc1Pulses4String,   asynParamUInt32Digital,         &P_TimRxFmc1Pulses4);
    createParam(P_TimRxFmc1Evt4String,   asynParamUInt32Digital,         &P_TimRxFmc1Evt4);
    createParam(P_TimRxFmc1Dly4String,   asynParamUInt32Digital,         &P_TimRxFmc1Dly4);
    createParam(P_TimRxFmc1Wdt4String,   asynParamUInt32Digital,         &P_TimRxFmc1Wdt4);
    createParam(P_TimRxFmc2En0String,   asynParamUInt32Digital,         &P_TimRxFmc2En0);
    createParam(P_TimRxFmc2Pol0String,   asynParamUInt32Digital,         &P_TimRxFmc2Pol0);
    createParam(P_TimRxFmc2Log0String,   asynParamUInt32Digital,         &P_TimRxFmc2Log0);
    createParam(P_TimRxFmc2Itl0String,   asynParamUInt32Digital,         &P_TimRxFmc2Itl0);
    createParam(P_TimRxFmc2Src0String,   asynParamUInt32Digital,         &P_TimRxFmc2Src0);
    createParam(P_TimRxFmc2Pulses0String,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses0);
    createParam(P_TimRxFmc2Evt0String,   asynParamUInt32Digital,         &P_TimRxFmc2Evt0);
    createParam(P_TimRxFmc2Dly0String,   asynParamUInt32Digital,         &P_TimRxFmc2Dly0);
    createParam(P_TimRxFmc2Wdt0String,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt0);
    createParam(P_TimRxFmc2En1String,   asynParamUInt32Digital,         &P_TimRxFmc2En1);
    createParam(P_TimRxFmc2Pol1String,   asynParamUInt32Digital,         &P_TimRxFmc2Pol1);
    createParam(P_TimRxFmc2Log1String,   asynParamUInt32Digital,         &P_TimRxFmc2Log1);
    createParam(P_TimRxFmc2Itl1String,   asynParamUInt32Digital,         &P_TimRxFmc2Itl1);
    createParam(P_TimRxFmc2Src1String,   asynParamUInt32Digital,         &P_TimRxFmc2Src1);
    createParam(P_TimRxFmc2Pulses1String,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses1);
    createParam(P_TimRxFmc2Evt1String,   asynParamUInt32Digital,         &P_TimRxFmc2Evt1);
    createParam(P_TimRxFmc2Dly1String,   asynParamUInt32Digital,         &P_TimRxFmc2Dly1);
    createParam(P_TimRxFmc2Wdt1String,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt1);
    createParam(P_TimRxFmc2En2String,   asynParamUInt32Digital,         &P_TimRxFmc2En2);
    createParam(P_TimRxFmc2Pol2String,   asynParamUInt32Digital,         &P_TimRxFmc2Pol2);
    createParam(P_TimRxFmc2Log2String,   asynParamUInt32Digital,         &P_TimRxFmc2Log2);
    createParam(P_TimRxFmc2Itl2String,   asynParamUInt32Digital,         &P_TimRxFmc2Itl2);
    createParam(P_TimRxFmc2Src2String,   asynParamUInt32Digital,         &P_TimRxFmc2Src2);
    createParam(P_TimRxFmc2Pulses2String,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses2);
    createParam(P_TimRxFmc2Evt2String,   asynParamUInt32Digital,         &P_TimRxFmc2Evt2);
    createParam(P_TimRxFmc2Dly2String,   asynParamUInt32Digital,         &P_TimRxFmc2Dly2);
    createParam(P_TimRxFmc2Wdt2String,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt2);
    createParam(P_TimRxFmc2En3String,   asynParamUInt32Digital,         &P_TimRxFmc2En3);
    createParam(P_TimRxFmc2Pol3String,   asynParamUInt32Digital,         &P_TimRxFmc2Pol3);
    createParam(P_TimRxFmc2Log3String,   asynParamUInt32Digital,         &P_TimRxFmc2Log3);
    createParam(P_TimRxFmc2Itl3String,   asynParamUInt32Digital,         &P_TimRxFmc2Itl3);
    createParam(P_TimRxFmc2Src3String,   asynParamUInt32Digital,         &P_TimRxFmc2Src3);
    createParam(P_TimRxFmc2Pulses3String,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses3);
    createParam(P_TimRxFmc2Evt3String,   asynParamUInt32Digital,         &P_TimRxFmc2Evt3);
    createParam(P_TimRxFmc2Dly3String,   asynParamUInt32Digital,         &P_TimRxFmc2Dly3);
    createParam(P_TimRxFmc2Wdt3String,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt3);
    createParam(P_TimRxFmc2En4String,   asynParamUInt32Digital,         &P_TimRxFmc2En4);
    createParam(P_TimRxFmc2Pol4String,   asynParamUInt32Digital,         &P_TimRxFmc2Pol4);
    createParam(P_TimRxFmc2Log4String,   asynParamUInt32Digital,         &P_TimRxFmc2Log4);
    createParam(P_TimRxFmc2Itl4String,   asynParamUInt32Digital,         &P_TimRxFmc2Itl4);
    createParam(P_TimRxFmc2Src4String,   asynParamUInt32Digital,         &P_TimRxFmc2Src4);
    createParam(P_TimRxFmc2Pulses4String,   asynParamUInt32Digital,         &P_TimRxFmc2Pulses4);
    createParam(P_TimRxFmc2Evt4String,   asynParamUInt32Digital,         &P_TimRxFmc2Evt4);
    createParam(P_TimRxFmc2Dly4String,   asynParamUInt32Digital,         &P_TimRxFmc2Dly4);
    createParam(P_TimRxFmc2Wdt4String,   asynParamUInt32Digital,         &P_TimRxFmc2Wdt4);
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

    /* Set the initial values of some parameters */
    setUIntDigitalParam(P_TimRxLinkStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRxenStatus,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxRefClkLocked,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt5,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt6,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEn7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPol7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcLog7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcItl7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcSrc7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcPulses7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcEvt7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcDly7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxAmcWdt7,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1En0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pol0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Log0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Itl0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Src0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pulses0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Evt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Dly0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Wdt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1En1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pol1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Log1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Itl1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Src1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pulses1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Evt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Dly1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Wdt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1En2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pol2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Log2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Itl2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Src2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pulses2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Evt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Dly2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Wdt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1En3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pol3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Log3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Itl3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Src3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pulses3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Evt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Dly3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Wdt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1En4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pol4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Log4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Itl4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Src4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Pulses4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Evt4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Dly4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc1Wdt4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2En0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pol0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Log0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Itl0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Src0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pulses0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Evt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Dly0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Wdt0,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2En1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pol1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Log1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Itl1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Src1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pulses1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Evt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Dly1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Wdt1,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2En2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pol2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Log2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Itl2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Src2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pulses2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Evt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Dly2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Wdt2,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2En3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pol3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Log3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Itl3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Src3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pulses3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Evt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Dly3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Wdt3,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2En4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pol4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Log4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Itl4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Src4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Pulses4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Evt4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Dly4,   0, 0xFFFFFFFF);
    setUIntDigitalParam(P_TimRxFmc2Wdt4,   0, 0xFFFFFFFF);
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

    /* Do callbacks so higher layers see any changes. Call callbacks for every addr */
    for (int i = 0; i < MAX_ADDR; ++i) {
        callParamCallbacks(i);
    }

    /* TimRx Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    timRxHwFunc.emplace(P_TimRxLinkStatus,    timRxSetGetLinkStatusFunc);
    timRxHwFunc.emplace(P_TimRxRxenStatus,    timRxSetGetRxenStatusFunc);
    timRxHwFunc.emplace(P_TimRxRefClkLocked,    timRxSetGetRefClkLockedFunc);
    timRxHwFunc.emplace(P_TimRxAmcEn0,    timRxSetGetAmcEn0Func);
    timRxHwFunc.emplace(P_TimRxAmcPol0,    timRxSetGetAmcPol0Func);
    timRxHwFunc.emplace(P_TimRxAmcLog0,    timRxSetGetAmcLog0Func);
    timRxHwFunc.emplace(P_TimRxAmcItl0,    timRxSetGetAmcItl0Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc0,    timRxSetGetAmcSrc0Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses0,    timRxSetGetAmcPulses0Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt0,    timRxSetGetAmcEvt0Func);
    timRxHwFunc.emplace(P_TimRxAmcDly0,    timRxSetGetAmcDly0Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt0,    timRxSetGetAmcWdt0Func);
    timRxHwFunc.emplace(P_TimRxAmcEn1,    timRxSetGetAmcEn1Func);
    timRxHwFunc.emplace(P_TimRxAmcPol1,    timRxSetGetAmcPol1Func);
    timRxHwFunc.emplace(P_TimRxAmcLog1,    timRxSetGetAmcLog1Func);
    timRxHwFunc.emplace(P_TimRxAmcItl1,    timRxSetGetAmcItl1Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc1,    timRxSetGetAmcSrc1Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses1,    timRxSetGetAmcPulses1Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt1,    timRxSetGetAmcEvt1Func);
    timRxHwFunc.emplace(P_TimRxAmcDly1,    timRxSetGetAmcDly1Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt1,    timRxSetGetAmcWdt1Func);
    timRxHwFunc.emplace(P_TimRxAmcEn2,    timRxSetGetAmcEn2Func);
    timRxHwFunc.emplace(P_TimRxAmcPol2,    timRxSetGetAmcPol2Func);
    timRxHwFunc.emplace(P_TimRxAmcLog2,    timRxSetGetAmcLog2Func);
    timRxHwFunc.emplace(P_TimRxAmcItl2,    timRxSetGetAmcItl2Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc2,    timRxSetGetAmcSrc2Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses2,    timRxSetGetAmcPulses2Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt2,    timRxSetGetAmcEvt2Func);
    timRxHwFunc.emplace(P_TimRxAmcDly2,    timRxSetGetAmcDly2Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt2,    timRxSetGetAmcWdt2Func);
    timRxHwFunc.emplace(P_TimRxAmcEn3,    timRxSetGetAmcEn3Func);
    timRxHwFunc.emplace(P_TimRxAmcPol3,    timRxSetGetAmcPol3Func);
    timRxHwFunc.emplace(P_TimRxAmcLog3,    timRxSetGetAmcLog3Func);
    timRxHwFunc.emplace(P_TimRxAmcItl3,    timRxSetGetAmcItl3Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc3,    timRxSetGetAmcSrc3Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses3,    timRxSetGetAmcPulses3Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt3,    timRxSetGetAmcEvt3Func);
    timRxHwFunc.emplace(P_TimRxAmcDly3,    timRxSetGetAmcDly3Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt3,    timRxSetGetAmcWdt3Func);
    timRxHwFunc.emplace(P_TimRxAmcEn4,    timRxSetGetAmcEn4Func);
    timRxHwFunc.emplace(P_TimRxAmcPol4,    timRxSetGetAmcPol4Func);
    timRxHwFunc.emplace(P_TimRxAmcLog4,    timRxSetGetAmcLog4Func);
    timRxHwFunc.emplace(P_TimRxAmcItl4,    timRxSetGetAmcItl4Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc4,    timRxSetGetAmcSrc4Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses4,    timRxSetGetAmcPulses4Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt4,    timRxSetGetAmcEvt4Func);
    timRxHwFunc.emplace(P_TimRxAmcDly4,    timRxSetGetAmcDly4Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt4,    timRxSetGetAmcWdt4Func);
    timRxHwFunc.emplace(P_TimRxAmcEn5,    timRxSetGetAmcEn5Func);
    timRxHwFunc.emplace(P_TimRxAmcPol5,    timRxSetGetAmcPol5Func);
    timRxHwFunc.emplace(P_TimRxAmcLog5,    timRxSetGetAmcLog5Func);
    timRxHwFunc.emplace(P_TimRxAmcItl5,    timRxSetGetAmcItl5Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc5,    timRxSetGetAmcSrc5Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses5,    timRxSetGetAmcPulses5Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt5,    timRxSetGetAmcEvt5Func);
    timRxHwFunc.emplace(P_TimRxAmcDly5,    timRxSetGetAmcDly5Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt5,    timRxSetGetAmcWdt5Func);
    timRxHwFunc.emplace(P_TimRxAmcEn6,    timRxSetGetAmcEn6Func);
    timRxHwFunc.emplace(P_TimRxAmcPol6,    timRxSetGetAmcPol6Func);
    timRxHwFunc.emplace(P_TimRxAmcLog6,    timRxSetGetAmcLog6Func);
    timRxHwFunc.emplace(P_TimRxAmcItl6,    timRxSetGetAmcItl6Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc6,    timRxSetGetAmcSrc6Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses6,    timRxSetGetAmcPulses6Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt6,    timRxSetGetAmcEvt6Func);
    timRxHwFunc.emplace(P_TimRxAmcDly6,    timRxSetGetAmcDly6Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt6,    timRxSetGetAmcWdt6Func);
    timRxHwFunc.emplace(P_TimRxAmcEn7,    timRxSetGetAmcEn7Func);
    timRxHwFunc.emplace(P_TimRxAmcPol7,    timRxSetGetAmcPol7Func);
    timRxHwFunc.emplace(P_TimRxAmcLog7,    timRxSetGetAmcLog7Func);
    timRxHwFunc.emplace(P_TimRxAmcItl7,    timRxSetGetAmcItl7Func);
    timRxHwFunc.emplace(P_TimRxAmcSrc7,    timRxSetGetAmcSrc7Func);
    timRxHwFunc.emplace(P_TimRxAmcPulses7,    timRxSetGetAmcPulses7Func);
    timRxHwFunc.emplace(P_TimRxAmcEvt7,    timRxSetGetAmcEvt7Func);
    timRxHwFunc.emplace(P_TimRxAmcDly7,    timRxSetGetAmcDly7Func);
    timRxHwFunc.emplace(P_TimRxAmcWdt7,    timRxSetGetAmcWdt7Func);
    timRxHwFunc.emplace(P_TimRxFmc1En0,    timRxSetGetFmc1En0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pol0,    timRxSetGetFmc1Pol0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Log0,    timRxSetGetFmc1Log0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Itl0,    timRxSetGetFmc1Itl0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Src0,    timRxSetGetFmc1Src0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses0,    timRxSetGetFmc1Pulses0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Evt0,    timRxSetGetFmc1Evt0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Dly0,    timRxSetGetFmc1Dly0Func);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt0,    timRxSetGetFmc1Wdt0Func);
    timRxHwFunc.emplace(P_TimRxFmc1En1,    timRxSetGetFmc1En1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pol1,    timRxSetGetFmc1Pol1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Log1,    timRxSetGetFmc1Log1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Itl1,    timRxSetGetFmc1Itl1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Src1,    timRxSetGetFmc1Src1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses1,    timRxSetGetFmc1Pulses1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Evt1,    timRxSetGetFmc1Evt1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Dly1,    timRxSetGetFmc1Dly1Func);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt1,    timRxSetGetFmc1Wdt1Func);
    timRxHwFunc.emplace(P_TimRxFmc1En2,    timRxSetGetFmc1En2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pol2,    timRxSetGetFmc1Pol2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Log2,    timRxSetGetFmc1Log2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Itl2,    timRxSetGetFmc1Itl2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Src2,    timRxSetGetFmc1Src2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses2,    timRxSetGetFmc1Pulses2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Evt2,    timRxSetGetFmc1Evt2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Dly2,    timRxSetGetFmc1Dly2Func);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt2,    timRxSetGetFmc1Wdt2Func);
    timRxHwFunc.emplace(P_TimRxFmc1En3,    timRxSetGetFmc1En3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pol3,    timRxSetGetFmc1Pol3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Log3,    timRxSetGetFmc1Log3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Itl3,    timRxSetGetFmc1Itl3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Src3,    timRxSetGetFmc1Src3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses3,    timRxSetGetFmc1Pulses3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Evt3,    timRxSetGetFmc1Evt3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Dly3,    timRxSetGetFmc1Dly3Func);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt3,    timRxSetGetFmc1Wdt3Func);
    timRxHwFunc.emplace(P_TimRxFmc1En4,    timRxSetGetFmc1En4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pol4,    timRxSetGetFmc1Pol4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Log4,    timRxSetGetFmc1Log4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Itl4,    timRxSetGetFmc1Itl4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Src4,    timRxSetGetFmc1Src4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Pulses4,    timRxSetGetFmc1Pulses4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Evt4,    timRxSetGetFmc1Evt4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Dly4,    timRxSetGetFmc1Dly4Func);
    timRxHwFunc.emplace(P_TimRxFmc1Wdt4,    timRxSetGetFmc1Wdt4Func);
    timRxHwFunc.emplace(P_TimRxFmc2En0,    timRxSetGetFmc2En0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pol0,    timRxSetGetFmc2Pol0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Log0,    timRxSetGetFmc2Log0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Itl0,    timRxSetGetFmc2Itl0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Src0,    timRxSetGetFmc2Src0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses0,    timRxSetGetFmc2Pulses0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Evt0,    timRxSetGetFmc2Evt0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Dly0,    timRxSetGetFmc2Dly0Func);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt0,    timRxSetGetFmc2Wdt0Func);
    timRxHwFunc.emplace(P_TimRxFmc2En1,    timRxSetGetFmc2En1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pol1,    timRxSetGetFmc2Pol1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Log1,    timRxSetGetFmc2Log1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Itl1,    timRxSetGetFmc2Itl1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Src1,    timRxSetGetFmc2Src1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses1,    timRxSetGetFmc2Pulses1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Evt1,    timRxSetGetFmc2Evt1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Dly1,    timRxSetGetFmc2Dly1Func);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt1,    timRxSetGetFmc2Wdt1Func);
    timRxHwFunc.emplace(P_TimRxFmc2En2,    timRxSetGetFmc2En2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pol2,    timRxSetGetFmc2Pol2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Log2,    timRxSetGetFmc2Log2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Itl2,    timRxSetGetFmc2Itl2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Src2,    timRxSetGetFmc2Src2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses2,    timRxSetGetFmc2Pulses2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Evt2,    timRxSetGetFmc2Evt2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Dly2,    timRxSetGetFmc2Dly2Func);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt2,    timRxSetGetFmc2Wdt2Func);
    timRxHwFunc.emplace(P_TimRxFmc2En3,    timRxSetGetFmc2En3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pol3,    timRxSetGetFmc2Pol3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Log3,    timRxSetGetFmc2Log3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Itl3,    timRxSetGetFmc2Itl3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Src3,    timRxSetGetFmc2Src3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses3,    timRxSetGetFmc2Pulses3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Evt3,    timRxSetGetFmc2Evt3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Dly3,    timRxSetGetFmc2Dly3Func);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt3,    timRxSetGetFmc2Wdt3Func);
    timRxHwFunc.emplace(P_TimRxFmc2En4,    timRxSetGetFmc2En4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pol4,    timRxSetGetFmc2Pol4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Log4,    timRxSetGetFmc2Log4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Itl4,    timRxSetGetFmc2Itl4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Src4,    timRxSetGetFmc2Src4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Pulses4,    timRxSetGetFmc2Pulses4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Evt4,    timRxSetGetFmc2Evt4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Dly4,    timRxSetGetFmc2Dly4Func);
    timRxHwFunc.emplace(P_TimRxFmc2Wdt4,    timRxSetGetFmc2Wdt4Func);
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
