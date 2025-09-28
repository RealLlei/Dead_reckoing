#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}

export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/usr/app/1/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${PROJECT_FOLDER}/lib/:$LD_LIBRARY_PATH

chmod +x bin/*
pmupload ./bin/topo_creator
