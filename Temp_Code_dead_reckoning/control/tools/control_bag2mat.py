import time
import math
import os
import sys

sys.path.append("../")
from cyber_py3 import cyber
from cyber_py3 import record
from google.protobuf.descriptor_pb2 import FileDescriptorProto
from proto.canbus.chassis_detail_pb2 import ChassisDetail
from proto.localization.localization_pb2 import Localization
from proto.canbus.chassis_pb2 import Chassis
from proto.control.control_cmd_pb2 import ControlCommand
from proto.planning.planning_pb2 import ADCTrajectory
from scipy.io import savemat
import numpy as np

from cyber2mat_struct import control_cmd
from cyber2mat_struct import pose_calc_debug 
from cyber2mat_struct import traj_calc_debug 
from cyber2mat_struct import header  
from cyber2mat_struct import PlanningTest
from cyber2mat_struct import trajectory_point
from cyber2mat_struct import path_point
from cyber2mat_struct import localization
from cyber2mat_struct import ChassisTest
from cyber2mat_struct import CtrlOutputDebug
from cyber2mat_struct import ctrl_dec_debug

#RECORD_FILE = "/home/damu/Desktop/20220125111139.record.00011"
# scp caros@10.4.11.87:/home/caros/ad/tools/plot_central_line/cyber_log.txt ~/Desktop


def test_record_reader(reader_path):
    freader = record.RecordReader(reader_path)
    log_txt = open("cyber_log.txt","a+")
    log_txt.write("parse_the_file:" + str(reader_path) + "\n")
    print("begin to read file:" + str(reader_path))
    count = 1
    channel_name_list=["/TL/control",
                       "/TL/canbus/chassis",
                       "/TL/planning",
                       "/TL/localization/pose"]
    
    control_test = control_cmd()
    print(" control_test is a class ")
    planning_test = PlanningTest()
    chassis_test = ChassisTest()
    pose_test = localization()
    control_command = []

    control = []
    planning = []
    planning_trajectory = []
    chassis = []
    channel_name_not_in_list = []
     
    vehicle_heading_loc = 0
    is_print_flag = 0

    period_counter = 0
    is_header_time = 0
    start_time_record = 0
    control_list = []
    planning_list = []

    for channelname, msg, datatype, timestamp in freader.read_messages():
        #print("read [%d] msg" % count)
        #print("chnanel_name -> %s" % channelname)
        #print("msgtime -> %d" % timestamp)
        #print("msgtype -> %s" % datatype)
        #timestamp = timestamp /1e9
        if start_time_record == 0:
            start_time_record = timestamp
        if channelname ==channel_name_list[1]:

            chassis = Chassis() 
            chassis.ParseFromString(msg)
            chassis_test.update(chassis)
 
        if channelname ==channel_name_list[3]:
            Localization_ = Localization() 
            Localization_.ParseFromString(msg)
            pose_test.update(Localization_)
            vehicle_heading_loc = Localization_.pose.heading
        #  controlsavemat = [pose_calc_debug,traj_calc_debug,ctrl_dec_debug,lon_ctrl_debug, lat_ctrl_debug,ctrl_output_debug]
        
        if channelname ==channel_name_list[0] :
            control_command = ControlCommand() 
            control_command.ParseFromString(msg)  
            
            throttle = control_command.throttle
            brake = control_command.brake
            
            control_test.update(control_command)
            # control = [control_test.dict]
            # control_list.append(control)

        #  planning
        if channelname == channel_name_list[2]:
            planning_trajectory = ADCTrajectory()
            planning_trajectory.ParseFromString(msg)
            planning_test.update(planning_trajectory)
            # planning = [planning_test.dict]
            # planning_list.append(planning)
            
        #savemat("covert.mat", {'control_command':control_command,'planning_trajectory':planning_trajectory,'chassis':chassis})
        

        if channelname not in channel_name_list and channelname not in channel_name_not_in_list:
            print("\nthis channel is not in the list:" + channelname)
            #log_txt.write("this channel is not in the list:" + str(channelname) + '\n')
            channel_name_not_in_list.append(channelname)
        if is_print_flag == 1:
            if start_time_record != 0:
                log_txt.write("channel_time:%.3f time_stamp:%.3f   start_time:%.3f\n"%(timestamp - start_time_record,
                                                                                       timestamp,
                                                                                       start_time_record))
            # print("="*10 + "period_counter:%d"%(pecontrolriod_counter) + "="*10 )
            print('\r', "="*10 + "period_counter:{:d} ".format(period_counter) + "=" * 10 , end='', flush=True)
            period_counter += 1
            log_txt.write("\n\n"+"-"*40+"period_end"+"-"*40+"\n\n")
        count = count + 1
        if count > 1e400:
            break
        is_print_flag = 0
    log_txt.close()
    savemat("covert.mat", {'control':control_test.get_dict(),'planning':planning_test.get_dict(),'pose':pose_test.get_dict(),'chassis':chassis_test.get_dict()})
    # savemat("covert.mat", {'control':control_list,'planning':planning_test.get_dict(),'pose':pose_test.get_dict(),'chassis':chassis_test.get_dict()})
    #savemat("covert.mat", {'control':control})
    print()
# def get_all_the_files(record_log_address,fileName):
#     file_name_list = []
#     for path,dir_list,file_list in os.walk(record_log_address):
#         for file_name in file_list:
#             if fileName in file_name:
#                 #print(file_name)
#                 file_name_list.append(os.path.join(path,file_name))
#     file_name_list.sort()
#     return file_name_list

def get_all_the_files(casespath1,fileName):
    file_name_list = []
    for path,dir_list,file_list in os.walk(casespath1):
        for file_name in file_list:
            if fileName in file_name:
                #print(file_name)
                file_name_list.append(os.path.join(path,file_name))
    file_name_list.sort()
    return file_name_list

if __name__ == '__main__':
    os.system('clear')
    cyber.init()
    #record_log_address = "/media/caros/Ubuntu/record/"
    # record_log_address = "/home/wfl/Downloads/"
    # fileName = "20220125111139.record.00011"
    # record_log_address = "/home/wfl/Downloads/0126.14"
    # fileName = "20220126135237.record.00000"
    # record_log_address = "/home/wfl/Desktop/0225/test_duantoulu_0225_nomap"
    # fileName = "20220225111950.record.00000"
    # record_log_address = "/home/wfl/Desktop/0225/test_duantoulu_0225_nomap"
    # record_log_address = "/home/wfl/Desktop"
    record_log_address = "/home/wfl/"
    fileName = "20220225112327.record.00000"
    casespath1 = os.path.join(record_log_address, "Desktop","0225","test_duantoulu_0225_nomap")
    casespath2 = os.path.join(record_log_address, "Downloads")
    casespath3 = os.path.join(record_log_address, "3")
    fileNameList = get_all_the_files(casespath1,fileName)
    log_txt = open("cyber_log.txt","w")
    log_txt.close()
    data_mat = open("cyber2mat.mat","w")
    data_mat.close
    for file in fileNameList:
        test_record_reader(file)
    cyber.shutdown()
