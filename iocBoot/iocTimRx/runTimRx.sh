#!/bin/sh

: ${EPICS_HOST_ARCH:?"Environment variable needs to be set"}

VALID_TIM_RX_NUMBER_STR="Valid values are between 0 and 23."

# Select endpoint.
TIM_RX_ENDPOINT=$1

if [ -z "$TIM_RX_ENDPOINT" ]; then
    echo "\"TIM_RX_ENDPOINT\" variable unset."
    exit 1
fi

# Select board in which we will work.
TIM_RX_NUMBER=$2

if [ -z "$TIM_RX_NUMBER" ]; then
    echo "\"TIM_RX_NUMBER\" variable unset. "$VALID_TIM_RX_NUMBERS_STR
    exit 1
fi

if [ "$TIM_RX_NUMBER" -lt 1 ] || [ "$TIM_RX_NUMBER" -gt 24 ]; then
    echo "Unsupported TimRx number. "$VALID_TIM_RX_NUMBERS_STR
    exit 1
fi

export TIM_RX_CURRENT_PV_AREA_PREFIX=${EPICS_PV_CRATE_PREFIX}_TIM_RX_${TIM_RX_NUMBER}_PV_AREA_PREFIX
export TIM_RX_CURRENT_PV_DEVICE_PREFIX=${EPICS_PV_CRATE_PREFIX}_TIM_RX_${TIM_RX_NUMBER}_PV_DEVICE_PREFIX
export EPICS_PV_AREA_PREFIX=${!TIM_RX_CURRENT_PV_AREA_PREFIX}
export EPICS_PV_DEVICE_PREFIX=${!TIM_RX_CURRENT_PV_DEVICE_PREFIX}

TIM_RX_ENDPOINT=${TIM_RX_ENDPOINT} TIM_RX_NUMBER=${TIM_RX_NUMBER} ../../bin/${EPICS_HOST_ARCH}/TimRx stTimRx.cmd
