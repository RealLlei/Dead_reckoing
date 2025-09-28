#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)
cd ${PROJECT_FOLDER}

killall -9 planning
# killall -9 localization
# killall -9 hmi
killall -9 ehp
killall -9 dead_reckoning
killall -9 location
# killall -9 control
# killall -9 soc_adas

nohup bash run_planning.sh >/dev/null 2>&1 &
# sleep 3s
# nohup bash run_localization.sh >/dev/null 2>&1 &
# nohup bash run_hmi.sh >/dev/null 2>&1 &
nohup bash run_ehp.sh >/dev/null 2>&1 &
nohup bash run_dead_reckoning.sh >/dev/null 2>&1 &
# nohup bash run_control.sh >/dev/null 2>&1 &

# cd /opt/usr/ap/app/adas

# nohup bash run_adas.sh >/dev/null 2>&1 &

# function update_process_priority()
# {
#     process_name=$(ps -ef | grep $1 | grep -v grep | awk '{print $2}')
#     chrt -p -r -a 20 ${process_name};
# }
# sleep 2
# update_process_priority control
# sleep 1
# update_process_priority soc_adas
