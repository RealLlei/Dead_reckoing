#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}
killall -9 ehp
killall -9 planning
chown -R mdc:mdc ./*
mount -o remount,rw /opt/usr/app/1
cp -r ./conf  /opt/app/1
cp -r ./data  /opt/app/1
cp -r ./lib  /opt/app/1
cp -r ./runtime_service  /opt/app/1
cp -r ./scripts  /opt/app/1
chown -R mdc:mdc /opt/app/1/runtime_service/ehp
chown -R mdc:mdc /opt/app/1/runtime_service/planning
chown -R mdc:mdc /opt/app/1/conf
chown -R mdc:mdc /opt/app/1/data
chown -R mdc:mdc /opt/app/1/lib
chown -R mdc:mdc /opt/app/1/scripts

