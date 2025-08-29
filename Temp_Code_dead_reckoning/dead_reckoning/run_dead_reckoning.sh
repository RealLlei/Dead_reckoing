#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/app/1/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${PROJECT_FOLDER}/lib/:$LD_LIBRARY_PATH

export CM_CONFIG_FILE_PATH=manual_service/dead_reckoning/etc/locationProcess
export RT_DDS_URI=${CM_CONFIG_FILE_PATH}/dds.xml
chmod +x manual_service/dead_reckoning/bin/dead_reckoning
pmupload ./manual_service/dead_reckoning/bin/dead_reckoning --flagfile=conf/global_flagfile.txt --allocGroup=default_dm
