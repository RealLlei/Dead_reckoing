#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
if [ ! -d "${PROJECT_FOLDER}/data/log" ]; then
        mkdir "${PROJECT_FOLDER}/data/log"
fi
if [ ! -d "${PROJECT_FOLDER}/runtime_service/planning/etc/planningProcess" ]; then
        echo "!!error!! can not find cm conf in folder runtime_service/planning/etc/planningProcess,please check your lcoal release"
        return
fi
if [ ! -f runtime_service/planning/etc/planningProcess/network_binding.json ]; then
        echo "!!error!! can not find cm conf files in folder runtime_service/planning/etc/planningProcess,please check your lcoal release"
        return
fi
export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/app/1/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${PROJECT_FOLDER}/lib/:$LD_LIBRARY_PATH

dos2unix runtime_service/planning/conf/planning_config.yaml
export CM_CONFIG_FILE_PATH=runtime_service/planning/etc/planningProcess
export RT_DDS_URI=${CM_CONFIG_FILE_PATH}/dds.xml
chmod +x runtime_service/planning/bin/*
pmupload ./runtime_service/planning/bin/planning --flagfile=conf/planning/planning.conf --allocGroup=default_dm
