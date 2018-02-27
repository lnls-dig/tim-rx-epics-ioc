#!/usr/bin/env bash

USER=$(whoami)
IP="localhost"
PREFIX=$(pwd)
DIR="tim-rx-epics-ioc-deploy"
DIST_NAME="tim-rx-epics-ioc"

./deploy.sh ${USER} ${IP} ${PREFIX}/${DIR}
makeself --bzip2  --notemp ${PREFIX}/${DIR} ${DIST_NAME}.bz2.run "LNLS Timing Receiver EPICS IOC Package" \
    make
