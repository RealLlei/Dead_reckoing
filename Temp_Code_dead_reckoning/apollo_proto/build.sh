#!/usr/bin/env bash

# Copyright (c) TL Technologies Co., Ltd. 2019-2020. All rights reserved.
set -e
START_TIME=$(date --date='0 days ago' "+%Y-%m-%d %H:%M:%S") && PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd) && PROJECT_NAME=${PROJECT_FOLDER##*/}
BUILD_TYPE=debug && DEBUG="debug" && RELEASE="release" && FAST="fast" && UNITTEST="UnitTest" && CLEAN="clean" && HELP="--help"
MDC_SYSROOT="/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/"

if [ "$JOBS" = "" ]; then
  JOBS=$(grep -c ^processor /proc/cpuinfo)
  JOBS=$((10#$JOBS / 2))
fi

export MAKEFLAGS="-j${JOBS} -l${JOBS}"

function usage() {
    echo "
Usage:   $0 [Action]

Action:  Use \"bash build.sh [BUILD_TYPE] [PLATFORM] (all)\" to specify BUILD_TYPE and PLATFORM
         all means compiling canbus and other modules.

Example:
         $0 clean
         $0 [debug/release/fast] [mdc/cyber/baidu]
         $0 [debug/release/fast] [mdc/cyber/baidu] all
    "
}

function determine_gpu_use_host() {
    USE_GPU_HOST=0
    # Check nvidia-driver and GPU device
    local nv_driver="nvidia-smi"
    if [ ! -x "$(command -v ${nv_driver})" ]; then
        echo "No nvidia-driver found. CPU will be used"
    elif [ -z "$(eval ${nv_driver})" ]; then
        echo "No GPU device found. CPU will be used."
    else
        USE_GPU_HOST=1
        echo "Nvidia Driver found.GPU will be used."
    fi
}

function prebuild() {
    determine_gpu_use_host
    delete_bin_lib
    clean
    cd "${PROJECT_FOLDER}"
    if [ ! -f CMakeLists.txt ]; then
        echo "CMakeLists.txt not exist in this folder"
        return 1
    fi
    if [ ! "${BUILD_TYPE}" ]; then
        BUILD_TYPE="release"
    fi
    echo -e "\033[1;32m BUILD_TYPE=${BUILD_TYPE} \033[0m"
    if [ ! -d "${PROJECT_FOLDER}/build" ]; then
        mkdir "${PROJECT_FOLDER}/build"
    fi
    cd "${PROJECT_FOLDER}/build" && echo "Start building project \"${PROJECT_NAME}\""
    cmake -DLLVM=${LLVM} -DPLATFORM=${PLATFORM} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DMDC_SDK=${MDC_SYSROOT} -DCOMPILE_ALL=${COMPILE_ALL} -DUSE_GPU_HOST=${USE_GPU_HOST} -DFOR_BAIDU_SIMULATION=${FOR_BAIDU_SIMULATION} ..
}

function build(){
    cd "${PROJECT_FOLDER}/build" && echo "Start building project \"${PROJECT_NAME}\""
    [ -f ${PROJECT_FOLDER}/build/Makefile ] || {
        echo "ERR: ${PROJECT_FOLDER}/build/Makefile is not exist. please input build type."
        usage
        exit 1
    }
    make install
    if [ $? -ne 0 ]; then
        echo -e "\033[0;31m Build ${PROJECT_NAME} Failed, please check error message in the log \033[0m"
        return 1
    fi
    finish_time=$(date --date='0 days ago' "+%Y-%m-%d %H:%M:%S")
    duration=$(($(($(date +%s -d "$finish_time") - $(date +%s -d "$START_TIME")))))
    if [ ! -f "libeuropa_common_proto.so" ]; then
        echo -e "\033[0;31m Build finished in $duration s, But can not find library libeuropa_common_proto.so in build folder, please check! \033[0m"
        return 1
    fi
    echo -e "\033[1;32m Build ${PROJECT_NAME} finished in $duration s with BUILD_TYPE=${BUILD_TYPE}. \033[0m"
    return 0
}

function clean() {
    local dirs=(
        "${PROJECT_FOLDER}/build"
        "${PROJECT_FOLDER}/${UNITTEST}")
    for dir in "${dirs[@]}"; do
        if [ -d "$dir" ]; then
            cd "$dir"
            if [ -d CMakeFiles ]; then
                rm -rf CMakeFiles && echo -e "\033[1;32m Remove $dir/CMakeFiles \033[0m"
            fi
            if [ -f CMakeCache.txt ]; then
                rm -rf CMakeCache.txt && echo -e "\033[1;32m Remove $dir/CMakeCache.txt \033[0m"
            fi
            if [ -f cmake_install.cmake ]; then
                rm -rf cmake_install.cmake && echo -e "\033[1;32m Remove $dir/cmake_install.cmake \033[0m"
            fi
            if [ -f Makefile ]; then
                rm -rf Makefile && echo -e "\033[1;32m Remove $dir/Makefile \033[0m"
            fi
        fi
    done
    return 0
}


function install_to_cyber(){
    OUTPUT_PATH=${PROJECT_FOLDER}/release/
    CYBER_PATH=${OUTPUT_PATH}/cyber/

    mkdir -p ${OUTPUT_PATH}/apollo_proto
    cd ${CYBER_PATH}
    cp lib/libeuropa_* ${OUTPUT_PATH}/apollo_proto

    cd ${PROJECT_FOLDER}
    cp -r `ls . | grep -v build |grep -v release | xargs` ./release/apollo_proto
    commitid=`git rev-parse --short HEAD`
    cd ${OUTPUT_PATH}
    tar czvf proto_${commitid}.tgz apollo_proto
    echo "Build success!"
}

function create_version_msg(){
    TARGET_PATH=${PROJECT_FOLDER}/release/mdc/europa
    VERSION_FILE="$TARGET_PATH/europa_version"
    if [ ! -f "VERSION_FILE" ]; then
        touch "$VERSION_FILE"
    else 
        echo "not need"
    fi
    commitid=`git rev-parse HEAD`
    DATE=`date '+%Y-%m-%d %H:%M:%S'`
    commitdata=`git log -n1`
    echo "COMPILE_TIME:${DATE}"> $VERSION_FILE
    echo "COMPILE_VERSION:${VERSION}" >>$VERSION_FILE
    # echo "COMMITID:${commitid}" >>$VERSION_FILE
    # echo "AUTHOR:$USER" >>$VERSION_FILE
    echo "--------------------------------" >>$VERSION_FILE
    echo "$commitdata" >>$VERSION_FILE
}

function delete_bin_lib(){
    OUTPUT_PATH=${PROJECT_FOLDER}/release
    if [ ${PLATFORM} == "MDC" ]; then
        MDC_INSTALL_PATH=${OUTPUT_PATH}/mdc
        if [ -d "${MDC_INSTALL_PATH}" ]; then
            rm -rf ${MDC_INSTALL_PATH}
        fi
    else
        CYBER_INSTALL_PATH=${OUTPUT_PATH}/cyber
        if [ -d "${CYBER_INSTALL_PATH}" ]; then
            cd ${CYBER_INSTALL_PATH} && rm -rf lib bin tools python/cyber_py3
        fi
    fi
    
}

function main() {
    git submodule init
    git submodule update
    # if [ $# -gt 1 ]; then
    #     usage && return 1
    # fi
    BUILD_TYPE="debug"
    PLATFORM="CYBER"
    FOR_BAIDU_SIMULATION=0
    COMPILE_ALL=0
    BUILD_FAST=0
    TARGET_PATH=${PROJECT_FOLDER}/release/cyber
    LLVM=0
    
    if [ $# -eq 1 ]; then
        case "$1" in
            "$CLEAN") clean && return 0 ;;
            "$HELP") usage && return 0 ;;
            *) usage && echo "no support" && return 1 ;;
        esac
    fi
    
    if [ $# -ge 2 ]; then
        if [ $# -eq 3 ] && [ "$3" == "all" ]; then
            COMPILE_ALL=1
        fi
        case "$1" in
            "$FAST") build && install_to_cyber && return 0;;
            "$DEBUG") BUILD_TYPE="debug" ;;
            "$RELEASE") BUILD_TYPE="release" ;;
            "$CLEAN") clean && return 0 ;;
            "$HELP") usage && return 0 ;;
            *) usage && echo "no support" && return 1 ;;
        esac
        case "$2" in
            baidu) FOR_BAIDU_SIMULATION=1 ;;
            mdc)  PLATFORM="MDC" && TARGET_PATH=${PROJECT_FOLDER}/release/mdc/europa ;;
            x86) PLATFORM="x86" ;;
            20dv) PLATFORM="20dv" ;;
        esac
        case "$3" in
            llvm) LLVM=1 && echo "" && echo "" && echo "USES LLVM, please make sure you installed clang-10 (sudo apt install clang-10)" && echo "" && echo "" ;;
        esac
	cd "${PROJECT_FOLDER}"
        [ ! -d ../third_party ] && cd .. && git clone -b master --single-branch https://code.TLauto.com:10443/algorithm/third_party.git
        cd "${PROJECT_FOLDER}"

        
        prebuild && build && install_to_cyber && return 0
    fi
    [ $# -eq 0 ] && usage && return 1
    return 0
}

cd "${PROJECT_FOLDER}"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PROJECT_FOLDER}/../third_party/x86/protobuf/lib/
main "$@"
exit $?
