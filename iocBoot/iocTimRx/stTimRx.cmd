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
