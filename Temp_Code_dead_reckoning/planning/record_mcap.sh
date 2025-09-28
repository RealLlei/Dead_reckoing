#!/usr/bin/env bash
partition="."  # Specify the partition to check available space
# Get the available space of the partition (in GB)
available_space=$(df -BG "$partition" | awk 'NR==2 {print $4}' | sed 's/G//')
# Compare the available space with 1GB
if [[ $available_space -lt 1 ]]; then
    echo "Warning: Available space on partition $partition is less than 1GB!"
    exit 1
else
    echo "Available space on partition $partition is $available_space GB, ok."
fi
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
DAY=$(date "+%Y_%m_%d")
TIME=$(date "+%Y_%m_%d_%H_%M_%S")
HOUR=$(date "+%H_%M_%S")
FUNCTION="orin"
function creat_primary_dir()
{
    if [ ! -d "${PROJECT_FOLDER}/mcap" ]; then
        mkdir "${PROJECT_FOLDER}/mcap"
    fi

    if [ ! -d "${PROJECT_FOLDER}/mcap/${FUNCTION}_${DAY}" ]; then
        mkdir "${PROJECT_FOLDER}/mcap/${FUNCTION}_${DAY}"
    fi

    if [ ! -d "${PROJECT_FOLDER}/mcap/${FUNCTION}_${DAY}/all_${HOUR}" ]; then
        mkdir "${PROJECT_FOLDER}/mcap/${FUNCTION}_${DAY}/all_${HOUR}"
    fi

    DIR="${PROJECT_FOLDER}/mcap/${FUNCTION}_${DAY}/all_${HOUR}"
}

function record_avp_all()
{
 cd ${DIR}
 echo "-----------record avp all-----------"
 echo "-----------mcap dir:"${DIR}
 bag record \
    /localization/deadreckoning \
    /planning/ego_trajectory \
    /planning/prediction \
    /planning/routing \
    /planning/soc_to_mcu_pnc \
    /perception/parking/avm_slot_det \
    /perception/parking/avm_stitcher \
    /perception/parking/fisheye_freespace \
    /perception/parking/fisheye_obj \
    /perception/parking/freespace_2 \
    /perception/parking/obj_fusion_2 \
    /perception/parking/parkinglot2hmi_2 \
    /perception/parking/parkinglot_2 \
    /perception/parking/slam_location \
    /perception/parking/slam_slot \
    /perception/parking/state_machine \
    /perception/parking/uss_obj \
    /perception/fsd/onboard/detection \
    /perception/fsd/obj_fusion_1 \
    /soc/encoded_camera_11 \
    /soc/encoded_camera_10 \
    /soc/encoded_camera_9 \
    /soc/encoded_camera_8 \
    /soc/imuinsinfo \
    /soc/chassis \
    /soc/sm_to_hpp_perception \
    /soc/statemachine \
    /planning/control_data \
    /soc/sm_to_mcu \
    /soc/mcu2ego \
    /soc/apa2mcu_chassis \
    /soc/ussrawdata \
    /soc/mcu_to_soc_pnc \
    /perception/parking/slam_map \
    /perception/fsd/freespace_1 \
    /perception/parking/avm_segmentation \
    /soc/fsmcore_output_mod \
    /perception/parking/avpmap2hmi_2 \
    /perception/parking/obj_fusion2hmi_2 \
    /soc/mod_output_canfd 
}

function record_ntp_all()
{
 cd ${DIR}
 echo "-----------record ntp all-----------"
 echo "-----------mcap dir:"${DIR}
 bag record \
    /localization/deadreckoning \
    /planning/ego_trajectory \
    /planning/prediction \
    /planning/routing \
    /planning/soc_to_mcu_pnc \
    /perception/parking/avm_slot_det \
    /perception/parking/avm_stitcher \
    /perception/parking/fisheye_freespace \
    /perception/parking/fisheye_obj \
    /perception/parking/freespace_2 \
    /perception/parking/obj_fusion_2 \
    /perception/parking/parkinglot2hmi_2 \
    /perception/parking/parkinglot_2 \
    /perception/parking/slam_location \
    /perception/parking/slam_slot \
    /perception/parking/state_machine \
    /perception/parking/uss_obj \
    /perception/fsd/onboard/detection \
    /perception/fsd/obj_fusion_1 \
    /soc/encoded_camera_11 \
    /soc/encoded_camera_10 \
    /soc/encoded_camera_9 \
    /soc/encoded_camera_8 \
    /soc/imuinsinfo \
    /soc/chassis \
    /soc/sm_to_hpp_perception \
    /soc/statemachine \
    /planning/control_data \
    /soc/sm_to_mcu \
    /soc/mcu2ego \
    /soc/apa2mcu_chassis \
    /soc/ussrawdata \
    /soc/mcu_to_soc_pnc \
    /perception/parking/slam_map \
    /soc/rawpointcloud \
    /perception/fsd/freespace_1 \
    /perception/parking/avm_segmentation \
    /soc/fsmcore_output_mod \
    /perception/parking/avpmap2hmi_2 \
    /perception/parking/obj_fusion2hmi_2 \
    /soc/mod_output_canfd 
}

function record_nnp_all()
{
 cd ${DIR}
 echo "-----------record nnp all-----------"
 echo "-----------mcap dir:"${DIR}
 bag record /localization/fusionmap \
  /localization/local_map \
  /localization/location \
  /perception/fsd/obj_fusion_1 \
  /perception/fsd/transportelement_1 \
  /planning/ego_trajectory \
  /planning/prediction \
  /planning/routing \
  /planning/control_data \
  /planning/soc_to_mcu_pnc \
  /soc/chassis \
  /soc/mcu2ego \
  /soc/mcu_to_soc_pnc \
  /perception/fsd/environment/tlr_msg
}

 
function usage() {
    echo "
Usage:   $0 [Action]

Action:  Use \"bash record_mcap.sh [FUNCTION] [TOPIC]\"to record target event

Example: 
         $0 nnp (record nnp all input and output)
         $0 avp (record avp all input and output, without lidar cloud point)
         $0 ntp (record ntp all input and output, with lidar cloud point)
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
    else
     usage && return 1
    fi
    creat_primary_dir && record_${FUNCTION}_all
else
    usage && return 1
fi
}
main "$@"
