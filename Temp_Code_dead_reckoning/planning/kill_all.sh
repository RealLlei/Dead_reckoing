#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)
cd ${PROJECT_FOLDER}

killall -9 planning
# killall -9 localization
# killall -9 hmi
killall -9 ehp
# killall -9 control
# killall -9 soc_adas
