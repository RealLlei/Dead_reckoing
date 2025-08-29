#set our env prior baidu clouds initialization
#export LD_LIBRARY_PATH=/apollo/lib:/apollo/cyber/lib
#export PYTHONPATH=/apollo/cyber/python/cyber_py3/:/apollo/cyber/python/:$PYTHONPATH
#set baidu clouds don't exist /mnt/simulation
# mkdir /mnt/simulation/
# mv /apollo/cyber/scripts/records/new_0_apollo_0.record /mnt/simulation/new_0_apollo_0.record
# mv /apollo/cyber/scripts/records/user_grading_result.json /mnt/simulation/user_grading_result.json
#local execute
python3 apollo_reports_manager.py --reportconf reports_conf.json  --local_record new_664d4a28b0023be0946bf002_apollo_0.record --local_map base_map.bin
#baidu clouds execute
#python3 /apollo/cyber/scripts/apollo_reports_manager.py --reportconf reports_conf.json 
# cp /mnt/simulation/user_grading_result.json /apollo/cyber/scripts/records/
cp /apollo/cyber/scripts/records/metrics_sys_log.syslog /var/log/local_result/log/
cd /mnt/simulation/
cp user_grading_result.json /var/log/local_result/log/ 
# cp *.record /var/log/local_result/log/
