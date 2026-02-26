#!/bin/bash
set -euxo pipefail

LOGPATH="build/px4_sitl_default/rootfs/log"
LOGFOLDER="$(ls -t1 $LOGPATH | head -n 1)"
echo "LOGFOLDER is ${LOGFOLDER}"
LOGFILE="$(ls -t1 ${LOGPATH}/${LOGFOLDER} | head -n 1)"

echo "LOGFILE is ${LOGPATH}/${LOGFOLDER}/${LOGFILE}"
xdg-open "${LOGPATH}/${LOGFOLDER}/${LOGFILE}"