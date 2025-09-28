#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..
cd ${PROJECT_FOLDER}

killall -9  planning
if [ -f ./lib/libglobalproto.so ]; then
  rm ./lib/libglobalproto.so
fi
echo 'HZnvidia12#$' | sudo -S  mount -o remount,rw /app
chmod -R 755 /app/scripts/test/*
cp -r ./conf  /app
cp -r ./data  /app
cp -r ./lib  /app
cp -r ./runtime_service  /app
cp -r ./scripts  /app

# check after cp
function cmp_file() {
local file=$1
md5_todo=$(md5sum $file | awk '{print $1}')
md5_done=$(md5sum /app/$file|awk '{print $1}')
if [ "$md5_todo" != "$md5_done" ]; then
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
        md5sum $file
        md5sum /app/$file
        ((cp_failed_num++))
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
else
    echo $file "cp success."
fi
}

function check_cp_status(){

folder_array=('./conf' './data' './lib' './runtime_service' 'scripts')

for folder in ${folder_array[@]} ;do
        echo "*************** check start : " $folder " **************"
        file_array=$(find $folder -type f)
        for file in $file_array;do
                cmp_file $file
        done
        echo "*************** check end : " $folder  "**************"
done

if [[ $cp_failed_num -gt 0 ]]; then
    RED='\033[0;31m'
    echo -e $RED
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!! file cp failed num :"$cp_failed_num
    echo "planning program update failed. please contact pnc."
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
    echo "!!!!!!!!!!!!!*****************copy failed***************!!!!!!!!!!!!!"
else
    GREEN='\033[0;32m'
    echo -e ${GREEN}
    echo "ooooooooooooooooooooooo copy success ooooooooooooooooooooooooo"
    echo "planning program update success. please run: 'nos dbg reboot'."
    echo "ooooooooooooooooooooooo copy success ooooooooooooooooooooooooo"
fi

}

cp_failed_num=0
check_cp_status