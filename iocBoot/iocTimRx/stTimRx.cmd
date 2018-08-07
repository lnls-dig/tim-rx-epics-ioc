< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")

< TimRx.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/TimRx.dbd")
TimRx_registerRecordDeviceDriver (pdbbase)

drvTimRxConfigure("$(TIM_RX_NAME)", "$(TIM_RX_ENDPOINT)", "$(TIM_RX_NUMBER)", "$(TIM_RX_VERBOSE)", "$(TIM_RX_TIMEOUT)")

## Load record instances
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxCfg.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")

dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC1, C=0, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC1, C=1, PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC1, C=2, PORT=$(PORT), ADDR=2, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC1, C=3, PORT=$(PORT), ADDR=3, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC1, C=4, PORT=$(PORT), ADDR=4, TIMEOUT=1")

dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC2, C=0, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC2, C=1, PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC2, C=2, PORT=$(PORT), ADDR=2, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC2, C=3, PORT=$(PORT), ADDR=3, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxFMCTrigCh.template", "P=${P}, R=${R}, S=FMC2, C=4, PORT=$(PORT), ADDR=4, TIMEOUT=1")

dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=0, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=1, PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=2, PORT=$(PORT), ADDR=2, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=3, PORT=$(PORT), ADDR=3, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=4, PORT=$(PORT), ADDR=4, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=5, PORT=$(PORT), ADDR=5, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=6, PORT=$(PORT), ADDR=6, TIMEOUT=1")
dbLoadRecords("${TOP}/TimRxApp/Db/TimRxAMCTrigCh.template", "P=${P}, R=${R}, S=AMC, C=7, PORT=$(PORT), ADDR=7, TIMEOUT=1")

dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}asyn,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")

< save_restore.cmd

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(TIM_RX_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(TIM_RX_NAME)",0,0xff)

iocInit()

< initTimRxCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
