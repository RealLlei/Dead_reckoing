#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
if [ ! -d "${PROJECT_FOLDER}/runtime_service/control/etc/TLProcess_Control" ]; then
        echo"!!error!! can not find cm conf in folder runtime_service/control/etc/TLProcess_Control,please check your lcoal release"
        exit
fi
if [ ! -f runtime_service/control/etc/TLProcess_Control/network_binding.json ]; then
        echo "!!error!! can not find cm conf files in folder runtime_service/control/etc/TLProcess_Control,please check your lcoal release"
        exit
fi
export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/usr/app/1/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${PROJECT_FOLDER}/lib/:$LD_LIBRARY_PATH

dos2unix runtime_service/control/conf/control_config.yaml
export CM_CONFIG_FILE_PATH=runtime_service/control/etc/TLProcess_Control
export RT_DDS_URI=${CM_CONFIG_FILE_PATH}/dds.xml
chmod +x runtime_service/control/bin/control
pmupload ./runtime_service/control/bin/control --flagfile=conf/control/control.conf
