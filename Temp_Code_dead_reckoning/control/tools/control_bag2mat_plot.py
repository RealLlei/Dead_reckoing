import time
import math
import os
import sys
import matplotlib.pyplot as plt

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
from cyber2mat_struct import lon_ctrl_debug
from cyber2mat_struct import header  
from cyber2mat_struct import PlanningTest
from cyber2mat_struct import trajectory_point
from cyber2mat_struct import path_point
from cyber2mat_struct import localization
from cyber2mat_struct import ChassisTest

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
            control = [control_test.dict]
            control_list.append(control)

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

    fig = plt.figure()
    fig,plt.subplots_adjust(left=0.05,right=0.95,top=0.95,bottom=0.05)
    ax1 = fig.add_subplot(3,3,1)
    ax2 = fig.add_subplot(3,3,2)
    ax3 = fig.add_subplot(3,3,3)
    ax4 = fig.add_subplot(3,3,4)
    ax5 = fig.add_subplot(3,3,5)
    ax6 = fig.add_subplot(3,3,6)
    ax7 = fig.add_subplot(3,3,7)
    ax8 = fig.add_subplot(3,3,8)
    ax9 = fig.add_subplot(3,3,9)


    #savemat("covert.mat", {'control':control})
    # position_x_list = pose_test.get_dict()['position_x']
    # y1 = pose_test.get_dict()['position_x']
    y1 = control_test.get_dict()['ctrldec_sysmode']
    # y2 = pose_test.get_dict()['linear_velocity_vrf_x']
    y2 = control_test.get_dict()['latictrl_fdbk_vxb']
    y3 = control_test.get_dict()['ctrldec_lat_sys_curvff']
    y4 = control_test.get_dict()['ctrldec_lat_sys_poserr']
    # y5 = control_test.get_dict()['lonctrl_sys_poserr']
    y5 = control_test.get_dict()['latictrl_tors_pure_yawerr']
    y6 = control_test.get_dict()['latictrl_tors_yawerr']
    y7 = control_test.get_dict()['latictrl_steer_steer_err']
    y8 = control_test.get_dict()['latictrl_fdbk_yaw']
    y9 = control_test.get_dict()['ctrldec_lat_sys_yawff']
    y10 = control_test.get_dict()['latictrl_fdbk_yawrate']
    y11 = control_test.get_dict()['lat_rate_yawratecmd_lmt']
    y12 = control_test.get_dict()['latictrl_fdbk_steer']
    y13 = control_test.get_dict()['lat_steer_output_steercmd']
    y14 = planning_test.get_dict()['driving_mode']

  
    # ax1.plot([array[0] for array in position_x_list],  [array[1] for array in position_x_list])
    ax1.plot([array[0] for array in y1],  [array[1] for array in y1],color='r',linestyle='-',label='ctrldec_sysmode')
    ax1.legend()
    ax1.grid()
    # print([array[0] for array in position_x_list])
    # print([array[1] for array in position_x_list])
    ax2.plot([array[0] for array in y2],  [array[1] for array in y2],color='r',linestyle='-',label='latictrl_fdbk_vxb')
    ax2.grid()
    ax2.legend()
    ax3.plot([array[0] for array in y3],  [array[1] for array in y3],color='b',linestyle='-',label='ctrldec_lat_sys_curvff')
    ax3.grid()
    ax3.legend()
    ax4.plot([array[0] for array in y4],  [array[1] for array in y4],color='b',linestyle='-',label='ctrldec_lat_sys_poserr')
    ax4.grid()
    ax4.legend()
    ax5.plot([array[0] for array in y5],  [array[1] for array in y5],color='r',linestyle='--',label='latictrl_tors_pure_yawerr')
    ax5.grid()
    ax5.legend()
    ax5.plot([array[0] for array in y6],  [array[1] for array in y6],color='b',linestyle='-',label='latictrl_tors_yawerr')
    ax5.legend()
    ax6.plot([array[0] for array in y7],  [array[1] for array in y7],color='b',linestyle='-',label='latictrl_steer_steer_err')
    ax6.legend()
    ax6.grid()
    ax7.plot([array[0] for array in y8],  [array[1] for array in y8],color='r',linestyle='--',label='latictrl_fdbk_heading')
    ax7.legend()
    ax7.grid()
    ax7.plot([array[0] for array in y9],  [array[1] for array in y9],color='b',linestyle='-',label='ctrldec_lat_sys_yawff')
    ax7.legend()
    ax8.plot([array[0] for array in y10],  [array[1] for array in y10],color='r',linestyle='--',label='latictrl_fdbk_yawrate')
    ax8.legend()
    ax8.grid()
    ax8.plot([array[0] for array in y11],  [array[1] for array in y11],color='b',linestyle='-',label='lat_rate_yawratecmd_lmt')
    ax8.legend()
    ax9.plot([array[0] for array in y12],  [array[1] for array in y12],color='r',linestyle='--',label='latictrl_fdbk_steer')
    ax9.legend()
    ax9.grid()
    ax9.plot([array[0] for array in y13],  [array[1] for array in y13],color='b',linestyle='-',label='lat_steer_output_steercmd')
    ax9.legend()
    ax9.plot([array[0] for array in y14],  [array[1] for array in y14],color='b',linestyle='-',label='driving_mode')
    ax9.legend()

    plt.show()





    
def get_all_the_files(record_log_address,fileName):
    file_name_list = []
    for path,dir_list,file_list in os.walk(record_log_address):
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
    record_log_address = "/home/wfl/Desktop/0225/test_duantoulu_0225_nomap"
    fileName = "20220225112327.record.00000"
    fileNameList = get_all_the_files(record_log_address,fileName)
    log_txt = open("cyber_log.txt","w")
    log_txt.close()
    data_mat = open("cyber2mat.mat","w")
    data_mat.close
    for file in fileNameList:
        test_record_reader(file)
    cyber.shutdown()
