#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}

export LD_LIBRARY_PATH=/app/third_party:$LD_LIBRARY_PATH
chmod +x runtime_service/planning/bin/*
./runtime_service/planning/bin/planning --flagfile=conf/planning/planning.conf

