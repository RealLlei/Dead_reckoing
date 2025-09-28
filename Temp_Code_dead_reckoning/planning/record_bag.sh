#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
DAY=$(date "+%Y_%m_%d")
TIME=$(date "+%Y_%m_%d_%H_%M_%S")
HOUR=$(date "+%H_%M_%S")
function creat_primary_dir()
{
    if [ ! -d "${PROJECT_FOLDER}/bag" ]; then
        mkdir "${PROJECT_FOLDER}/bag"
    fi

    if [ ! -d "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}" ]; then
        mkdir "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}"
    fi

    if [ ! -d "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/all_${HOUR}" ]; then
        mkdir "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/all_${HOUR}"
    fi

    DIR="${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/all_${HOUR}"
}

function creat_secondary_dir()
{
    if [ ! -d "${PROJECT_FOLDER}/bag" ]; then
        mkdir "${PROJECT_FOLDER}/bag"
    fi

    if [ ! -d "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}" ]; then
        mkdir "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}"
    fi

    if [ ! -d "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/${TARGET}_${HOUR}" ]; then
        mkdir "${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/${TARGET}_${HOUR}"
    fi

    DIR="${PROJECT_FOLDER}/bag/${FUNCTION}_${DAY}/${TARGET}_${HOUR}"

}

function record_nnp_input()
{
    cd ${DIR}
    echo "-----------record_nnp_input-------------"
    echo "-----------bag dir:"${DIR}
    rtfbag record /TL/ServiceInterface/TLInterface_Chassis/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Chassis_DDS_1]\
 /TL/ServiceInterface/TLInterface_Mcu2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Mcu2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Aeb2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Aeb2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/RoadMark/Service/DDS/Provider/RoadMarkProviderInstance_LaneFusion_DDS_11]\
 /TL/ServiceInterface/TLInterface_Obj_Fusion/TLEvent[/TL/ModuleBase/PerceptionFusion/Service/DDS/Provider/PerFusionProviderInstance_ObjFusion_DDS_1]\
 /TL/ServiceInterface/TLInterface_Location/TLEvent[/TL/ModuleBase/NnpLocalization/Service/DDS/Provider/NnpLocalizationProviderInstance_Location_DDS_1]\
 /TL/ServiceInterface/HzSocMcuClientServiceInterface/MbdDebugDataEvent[/TL/Service/Provider/HzSocMcuAppRtfeventDdsProvidedServiceInstance]\
 /TL/ServiceInterface/HzSocMcuClientServiceInterface/MbdDebugDataEvent[/TL/Service/Provider/HzSocMcuAppRtfeventDdsProvidedServiceInstance]

}

function record_nnp_output()
{
    cd ${DIR}
    echo "-----------record_nnp_output-------------"
    echo "-----------bag dir:"${DIR}
    rtfbag record /TL/ServiceInterface/TLInterface_Planning/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoTrajectory_DDS_1]\
 /TL/ServiceInterface/TLInterface_PlanningDecisionInfo/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_PlanningDecision_DDS_1]\
 /TL/ServiceInterface/TLInterface_DebugPlan/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Debug_DDS_9202]\
 /TL/ServiceInterface/TLInterface_Ego2Mcu/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Ego2Mcu_DDS_1]\
 /TL/ServiceInterface/TLInterface_EgoHmi/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoHmi_DDS_1]

}

function record_nnp_all()
{
    cd ${DIR}
    echo "-----------record_nnp_all-------------"
    echo "-----------bag dir:"${DIR}
        rtfbag record /TL/ServiceInterface/TLInterface_Chassis/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Chassis_DDS_1]\
 /TL/ServiceInterface/TLInterface_Mcu2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Mcu2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Aeb2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Aeb2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/RoadMark/Service/DDS/Provider/RoadMarkProviderInstance_LaneFusion_DDS_11]\
 /TL/ServiceInterface/TLInterface_Obj_Fusion/TLEvent[/TL/ModuleBase/PerceptionFusion/Service/DDS/Provider/PerFusionProviderInstance_ObjFusion_DDS_1]\
 /TL/ServiceInterface/TLInterface_Location/TLEvent[/TL/ModuleBase/NnpLocalization/Service/DDS/Provider/NnpLocalizationProviderInstance_Location_DDS_1]\
 /TL/ServiceInterface/TLInterface_Planning/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoTrajectory_DDS_1]\
 /TL/ServiceInterface/TLInterface_PlanningDecisionInfo/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_PlanningDecision_DDS_1]\
 /TL/ServiceInterface/TLInterface_DebugPlan/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Debug_DDS_9202]\
 /TL/ServiceInterface/TLInterface_Ego2Mcu/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Ego2Mcu_DDS_1]\
 /TL/ServiceInterface/TLInterface_EgoHmi/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoHmi_DDS_1]\
 /TL/ServiceInterface/HzSocMcuClientServiceInterface/MbdDebugDataEvent[/TL/Service/Provider/HzSocMcuAppRtfeventDdsProvidedServiceInstance]

}

function record_avp_all()
{
 cd ${DIR}
 echo "-----------record_avp_all-----------"
 echo "-----------bag dir:"${DIR}
     rtfbag record /TL/ServiceInterface/TLInterface_Chassis/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Chassis_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/StateMachine/Service/DDS/Provider/StateMachineProviderInstance_StateMachine_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/StateMachine/Service/DDS/Provider/StateMachineProviderInstance_StateMachine_DDS_2]\
 /TL/ServiceInterface/TLInterface_Mcu2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Mcu2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Aeb2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Aeb2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Planning/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoTrajectory_DDS_1]\
 /TL/ServiceInterface/TLInterface_PlanningDecisionInfo/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_PlanningDecision_DDS_2]\
 /TL/ServiceInterface/TLInterface_DebugPlan/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Debug_DDS_9202]\
 /TL/ServiceInterface/TLInterface_Ego2Mcu/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Ego2Mcu_DDS_1]\
 /TL/ServiceInterface/TLInterface_EgoHmi/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoHmi_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_StateMachine_DDS_11]\
 /TL/ServiceInterface/TLInterface_Obj_Lidar/TLEvent[/TL/ModuleBase/uss_perception/Service/DDS/Provider/uss_perceptionProviderInstance_Lidar_DDS_100]\
 /TL/ServiceInterface/TLInterface_Obj_Camera/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_objcamera_DDS_104]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_hppLane_DDS_104]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_FreeSpace_DDS_104]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/uss_perception/Service/DDS/Provider/uss_perceptionProviderInstance_ParkingLot_DDS_100]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_ParkingLot_DDS_104]\
 /TL/ServiceInterface/TLInterface_Location/TLEvent[/TL/ModuleBase/parking_slam/Service/DDS/Provider/SLAMProviderInstance_Location_DDS_2]\
 /TL/ServiceInterface/TLInterface_Obj_Fusion/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_objfusion_DDS_11]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_hppLane_DDS_2]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_FreeSpace_DDS_2]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_ParkingLot_DDS_1]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/parking_slam/Service/DDS/Provider/SLAMProviderInstance_ParkingLot_DDS_106]\
 /TL/ServiceInterface/TLInterface_Uss/TLEvent[/TL/Service/DDS/Provider/ProviderInstance_Uss_DDS_1]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_79]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_80]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_81]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_82]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_h265_image_DDS_104]\
 /TL/ServiceInterface/TLInterface_ImuInfo/TLEvent[/TL/PlatformApplication/InsPvatbApplication/Service/Provider/ImuToAppProvidedDdsServiceInstance]\
 /TL/ServiceInterface/TLInterface_Obj_Lidar/TLEvent[/TL/ModuleBase/PerceptionLidar/Service/DDS/Provider/PerLidarProviderInstance_ObjLidar_DDS_10]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/PerceptionStateMachine/Service/DDS/Provider/PerceptionStateMachineProviderInstance_StateMachine_DDS_12]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/PerceptionLidar/Service/DDS/Provider/PerLidarProviderInstance_FreeSpace_DDS_10]\
 /TL/ServiceInterface/TLInterface_CanFdMsg/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_canfdmsg_DDS_102]\
 /TL/ServiceInterface/HzSocMcuAdasRtfClientServiceInterface/McuADASRtfEvent[/TL/Service/Provider/HzSocMcuAppAdasRecordRtfeventDdsProvidedServiceInstance]
}

function record_ntp_all()
{
 cd ${DIR}
 echo "-----------record_ntp_all-----------"
 echo "-----------bag dir:"${DIR}
     rtfbag record /TL/ServiceInterface/TLInterface_Chassis/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Chassis_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/StateMachine/Service/DDS/Provider/StateMachineProviderInstance_StateMachine_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/StateMachine/Service/DDS/Provider/StateMachineProviderInstance_StateMachine_DDS_2]\
 /TL/ServiceInterface/TLInterface_Mcu2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Mcu2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Aeb2Ego/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_Aeb2Ego_DDS_1]\
 /TL/ServiceInterface/TLInterface_Planning/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoTrajectory_DDS_1]\
 /TL/ServiceInterface/TLInterface_PlanningDecisionInfo/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_PlanningDecision_DDS_2]\
 /TL/ServiceInterface/TLInterface_DebugPlan/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Debug_DDS_9202]\
 /TL/ServiceInterface/TLInterface_Ego2Mcu/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_Ego2Mcu_DDS_1]\
 /TL/ServiceInterface/TLInterface_EgoHmi/TLEvent[/TL/ModuleBase/Planning/Service/DDS/Provider/PlanningProviderInstance_EgoHmi_DDS_1]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/Chassis/Service/DDS/Provider/ChassisProviderInstance_StateMachine_DDS_11]\
 /TL/ServiceInterface/TLInterface_Obj_Lidar/TLEvent[/TL/ModuleBase/uss_perception/Service/DDS/Provider/uss_perceptionProviderInstance_Lidar_DDS_100]\
 /TL/ServiceInterface/TLInterface_Obj_Camera/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_objcamera_DDS_104]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_hppLane_DDS_104]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_FreeSpace_DDS_104]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/uss_perception/Service/DDS/Provider/uss_perceptionProviderInstance_ParkingLot_DDS_100]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_ParkingLot_DDS_104]\
 /TL/ServiceInterface/TLInterface_Location/TLEvent[/TL/ModuleBase/parking_slam/Service/DDS/Provider/SLAMProviderInstance_Location_DDS_2]\
 /TL/ServiceInterface/TLInterface_Obj_Fusion/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_objfusion_DDS_11]\
 /TL/ServiceInterface/TLInterface_LaneLine/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_hppLane_DDS_2]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_FreeSpace_DDS_2]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_ParkingLot_DDS_1]\
 /TL/ServiceInterface/TLInterface_ParkingLot/TLEvent[/TL/ModuleBase/parking_slam/Service/DDS/Provider/SLAMProviderInstance_ParkingLot_DDS_106]\
 /TL/ServiceInterface/TLInterface_Uss/TLEvent[/TL/Service/DDS/Provider/ProviderInstance_Uss_DDS_1]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_79]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_80]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_81]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/HuaweiMDC/PlatformApplication/CameraVencApplication/CameraVencService/CameraServiceProvider/camera_venc_encoded_mbuf_82]\
 /HuaweiMDC/PlatformServiceInterface/CameraServiceInterfacePkg/CameraEncodedMbufServiceInterface/cameraEncodedMbufEvent[/TL/ModuleBase/fisheye_perception/Service/DDS/Provider/fisheye_perceptionProviderInstance_h265_image_DDS_104]\
 /TL/ServiceInterface/TLInterface_ImuInfo/TLEvent[/TL/PlatformApplication/InsPvatbApplication/Service/Provider/ImuToAppProvidedDdsServiceInstance]\
 /TL/ServiceInterface/TLInterface_Obj_Lidar/TLEvent[/TL/ModuleBase/PerceptionLidar/Service/DDS/Provider/PerLidarProviderInstance_ObjLidar_DDS_10]\
 /TL/ServiceInterface/TLInterface_StateMachine/TLEvent[/TL/ModuleBase/PerceptionStateMachine/Service/DDS/Provider/PerceptionStateMachineProviderInstance_StateMachine_DDS_12]\
 /TL/ServiceInterface/TLInterface_Freespace/TLEvent[/TL/ModuleBase/PerceptionLidar/Service/DDS/Provider/PerLidarProviderInstance_FreeSpace_DDS_10]\
 /TL/ServiceInterface/TLInterface_CanFdMsg/TLEvent[/TL/ModuleBase/parking_fusion/Service/DDS/Provider/obstacle_fusionProviderInstance_canfdmsg_DDS_102]\
 /TL/ServiceInterface/HzSocMcuAdasRtfClientServiceInterface/McuADASRtfEvent[/TL/Service/Provider/HzSocMcuAppAdasRecordRtfeventDdsProvidedServiceInstance]\
 /TL/ServiceInterface/TLInterface_PointCloud/TLEvent[/Service/DDS/Provider/LidarDdsProvidedServiceInstance_1]\
 /TL/ServiceInterface/TLInterface_PointCloud/TLEvent[/Service/DDS/Provider/LidarDdsProvidedServiceInstance_2]
}

function usage() {
    echo "
Usage:   $0 [Action]

Action:  Use \"bash record_bag.sh [FUNCTION] [INPUT OR OUTPUR]\"to record target event

Example: 
         $0 nnp (record nnp all input and output)
         $0 avp (record avp all input and output, without lidar cloud point)
         $0 ntp (record ntp all input and output, with lidar cloud point)
         $0 nnp in
         $0 nnp out
"
}
function main() {
if [ $# -eq 1 ]; then
    if [ "$1" == "nnp" ]; then
     FUNCTION="nnp"
    elif [ "$1" == "avp" ]; then
     FUNCTION="avp" 
    elif [ "$1" == "ntp" ]; then
     FUNCTION="ntp"
    fi
    creat_primary_dir && record_${FUNCTION}_all
fi

if [ $# -eq 2 ]; then 
    
    if [ "$1" == "nnp" ]; then
        FUNCTION="nnp"
    else
        usage && return 0
    fi
    if [ "$2" == "in" ]; then
        TARGET="input" 
    elif [ "$2" == "out" ]; then
        TARGET="output" 
    else
        usage && return 0
    fi
    creat_secondary_dir && record_${FUNCTION}_input
fi
if [ $# -eq 0 ]; then
    usage && return 1
fi
}
main "$@"

