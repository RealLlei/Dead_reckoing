from cmath import pi
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
# from proto.control.control_cmd_pb2 import ControlCommand
from proto.control.mbd_control_debug_pb2 import MbdDebugFromMCU
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
    channel_name_list=["/TL/mcu/control",
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
            control_command = MbdDebugFromMCU() 
            control_command.ParseFromString(msg)  
            
            # throttle = control_command.throttle
            # brake = control_command.brake
            
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
    savemat("covert032801.mat", {'control':control_test.get_dict(),'planning':planning_test.get_dict(),'pose':pose_test.get_dict(),'chassis':chassis_test.get_dict()})

    fig = plt.figure()
    fig,plt.subplots_adjust(left=0.05,right=0.95,top=0.95,bottom=0.05)
    ax1 = fig.add_subplot(3,3,1)
    ax2 = fig.add_subplot(3,3,2,sharex=ax1)
    ax3 = fig.add_subplot(3,3,3,sharex=ax1)
    ax4 = fig.add_subplot(3,3,4,sharex=ax1)
    ax5 = fig.add_subplot(3,3,5,sharex=ax1)
    ax6 = fig.add_subplot(3,3,6,sharex=ax1)
    ax7 = fig.add_subplot(3,3,7,sharex=ax1)
    ax8 = fig.add_subplot(3,3,8,sharex=ax1)
    ax9 = fig.add_subplot(3,3,9,sharex=ax1)


    #savemat("covert.mat", {'control':control})
    # position_x_list = pose_test.get_dict()['position_x']
    # y1 = pose_test.get_dict()['position_x']
    y1 = control_test.get_dict()['ctrldec_sysmode']
    # y2 = pose_test.get_dict()['linear_velocity_vrf_x']
    y2 = control_test.get_dict()['latictrl_fdbk_vxb']
    y3 = control_test.get_dict()['ctrldec_lat_sys_curvff']
    y4 = control_test.get_dict()['ctrldec_lat_sys_poserr']
    y5 = control_test.get_dict()['latictrl_tors_pure_yawerr']
    y6 = control_test.get_dict()['latictrl_tors_yawerr']
    # y7 = control_test.get_dict()['latictrl_steer_steer_err']
    y8 = control_test.get_dict()['latictrl_fdbk_yaw']
    y9 = control_test.get_dict()['ctrldec_lat_sys_yawff']
    y10 = control_test.get_dict()['latictrl_fdbk_yawrate']
    y11 = control_test.get_dict()['lat_rate_yawratecmd_lmt']
    y12 = control_test.get_dict()['latictrl_fdbk_steer']
    y13 = control_test.get_dict()['lat_steer_output_steercmd']
    y14 = planning_test.get_dict()['driving_mode']
    y15 = control_test.get_dict()['lonctrl_fdbk_vx']
    y16 = control_test.get_dict()['lonctrl_vel_velcmd_lmt']
    y17 = control_test.get_dict()['lonctrl_fdbk_ax']
    y18 = control_test.get_dict()['lon_thrust_acc_cmd_filter_lmt']
    y19 = control_test.get_dict()['lonctrl_analog_autput_throtcmd']
    # y20 = control_test.get_dict()['latictrl_tors_torsrateff']
    y20 = control_test.get_dict()['latictrl_rate_steerff']
    y21 = control_test.get_dict()['lat_rate_yawratecmd_lmt_filter']
    y22 = control_test.get_dict()['lonctrl_sys_poserr']
    y23 = control_test.get_dict()['latictrl_rate_pi_filter_steer']
    y24 = control_test.get_dict()['lat_rate_output_sw_steercmd']
    y27 = control_test.get_dict()['lon_pos_output_velcmd']
    y28 = control_test.get_dict()['lon_vel_output_accCmd_filter']
    y29 = control_test.get_dict()['lonctrl_vel_accff_filter']
    y30 = control_test.get_dict()['lon_vel_output_accCmd_filter']
    y31 = control_test.get_dict()['lat_steer_steercmd_lmt_filter']
    y32 = control_test.get_dict()['lonctrl_sys_velff']
    y33 = control_test.get_dict()['lonctrl_sys_accff']
    y34 = control_test.get_dict()['acc_cmd']
    y35 = control_test.get_dict()['lon_pos_poserr_filter']
    y36 = control_test.get_dict()['posedata_world_pos_y']
    y37 = control_test.get_dict()['trajcalc_lon_poserrcmd']
    y38 = control_test.get_dict()['tra_lon_startpoint_index']
    y39 = control_test.get_dict()['lonctrl_pos_pi_velcmd']
    y41 = control_test.get_dict()['gear_cmd']
    y42 = control_test.get_dict()['lat_tors_output_yawratecmd']
    y43 = control_test.get_dict()['lat_tors_pi_leadfilter_torsrate']
    y44 = control_test.get_dict()['latictrl_tors_pi_torsrate']
    y45 = control_test.get_dict()['latictrl_tors_torsrateff']
    y46 = control_test.get_dict()['lat_rate_output_front_steercmd']
    y47 = control_test.get_dict()['lat_output_front_steeroffset']
    y50 = control_test.get_dict()['latictrl_tors_pi_torsrate']
    y51 = control_test.get_dict()['lat_tors_pi_leadfilter_torsrate']
    y52 = control_test.get_dict()['trajdata_gearcmd']
    y53 = control_test.get_dict()['latictrl_fdbk_gear']
    y54 = control_test.get_dict()['lat_rate_yawratecmd_lmt_filter']
    y55 = control_test.get_dict()['lat_rate_pi_steer']
    y56 = control_test.get_dict()['ctrldec_sysmode']
    y57 = control_test.get_dict()['lonctrl_modecmd']
    y58 = control_test.get_dict()['latctrl_modecmd']
    y59 = control_test.get_dict()['lonctrl_fdbk_pitch']
    y60 = control_test.get_dict()['lonctrl_vel_output_accCmd']
    y61 = control_test.get_dict()['lonctrl_vel_accff_filter']
    y62 = control_test.get_dict()['lonctrl_sys_accff']
    y63 = control_test.get_dict()['lonctrl_vel_pi_acc_cmd']
    y65 = control_test.get_dict()['posedata_world_pos_x']
    y66 = control_test.get_dict()['posedata_world_pos_y']
    y67 = control_test.get_dict()['trajcalc_lat_match_pointx']
    y68 = control_test.get_dict()['trajcalc_lat_match_pointy']
    y69 = control_test.get_dict()['trajcalc_lat_headingcmd']
    y74 = control_test.get_dict()['latictrl_offset_pi_torscmd']
    y75 = control_test.get_dict()['ctrldec_lat_resetflag']
    y76 = control_test.get_dict()['latictrl_steer_steercmd_filter']
    y77 = control_test.get_dict()['ctrldec_lat_api_steercmd']
    y78 = control_test.get_dict()['ctrldec_lat_resetflag']
    y79 = control_test.get_dict()['tra_lat_startpoint_index']
    y80 = control_test.get_dict()['tra_lon_startpoint_index']
    y81 = control_test.get_dict()['tr_latpre_startpoint_index']
    y82 = control_test.get_dict()['replaning_flag']
    y83 = control_test.get_dict()['trajdata_timestamp']
    y84 = control_test.get_dict()['lat_rate_output_sw_steercmd']
    y85 = control_test.get_dict()['lat_steer_steercmd_lmt_filter']
    y86 = control_test.get_dict()['ctrldec_ctrl_err']
    # print("line----------------173",type(y20))
    # print("line----------------173",y11)
    # print('trajdata_time:',y83)

    y25 = [[i[0],i[1]] for i in y20]
    y26 = [[j[0],j[1]] for j in y23]
    
    lat_poserr = []
    for i in range(len(y4)):
        if y56[i][1]==9 and y52[i][1]!=3:
            # tmp = i
        #     t1.append(tmp)
        # for m in t1:
            tmp = y4[i][1]
            lat_poserr.append(tmp)

            index = max_data = 0
            for i, data in enumerate(lat_poserr):
                if i == 0:
                  max_data = abs(data)
                # 如果下一个值大于当前最大值，那下一个值就是最大值，并记录其下表
                if abs(data) > abs(max_data):
                  max_data = data
                  index = i
            # print('i:',index)
    print('max_lat_poserr:',lat_poserr[index])

    lon_poserr = []
    for i in range(len(y22)):
        if y56[i][1]==9 and y52[i][1]!=3:
            tmp1 = y22[i][1]
            lon_poserr.append(tmp1)
            index = max_data = 0
            for i, data in enumerate(lon_poserr):
                if i == 0:
                  max_data = abs(data)
                if abs(data) > abs(max_data):
                  max_data = data
                  index = i
    print('max_lon_poserr:',lon_poserr[index])

    pure_yawerr = []
    for i in range(len(y5)):
        if y56[i][1]==9 and y52[i][1]!=3:
            tmp2 = y5[i][1]
            pure_yawerr.append(tmp2)
            index = max_data = 0
            for i, data in enumerate(pure_yawerr):
                if i == 0:
                  max_data = abs(data)
                if abs(data) > abs(max_data):
                  max_data = data
                  index = i
    print('max_pure_yawerr:',pure_yawerr[index])

    # for i in y25:
    #     print(i)

    # y15 =[-i for i in y11]
    y40 = []
    for i in range(len(y12)):
        tmp = y15[i][1] * y12[i][1] / 16.5 / 2.77
        y40.append([y12[i][0],tmp])

    y64 = []
    for i in range(len(y34)):
        if y34[i][1]>0:
            y64.append([y34[i][0],y34[i][1]/1.5*3])
        else:
            y64.append([y34[i][0],y34[i][1]/5*3])
    a=[]
    b=[]
    c=[]
    d=[]
    e=[]
    f=[]
    t=[]
    # acccmd = []
    # for i in range(len(y64)):
    #     if y56[i][1]==9 and y52[i][1]!=3:
    #         tmp3 = y64[i][1]
    #         acccmd.append(tmp3)
    #         index = max_data = 0
    #         for i, data in enumerate(acccmd):
    #             if i == 0:
    #               max_data = abs(data)
    #             if abs(data) >= abs(max_data):
    #               max_data = data
    #               index = i
    #     a.append(index)
    # print('a:',a) 
    # print('end_lat_poserr:',y4[index-1][1])
    # acccmd = []
    # for i in range(len(y64)):
    #     if y56[i][1]==9:
    #         t.append(i)
    # # print('t:',max(t))

    ##AVP最终停车横向跟踪偏差
    for i in range(len(y64)):
        if y56[i][1]==9 and y52[i][1]==3:
            a.append(i)
    # print('a:',a)
    for index in a:
        # print('y4:',y4[index-1][1])
        if abs(y4[index-1][1])<=0.2 and abs(y4[index-1][1])!=0:
            b.append(index)
    # print('b:',b)
    # tmp4 = min(b)-1
    # # print('tmp4:',tmp4)
    # print('end_lat_poserr:',y4[tmp4][1])

    ##AVP最终停车纵向跟踪偏差
    for i in range(len(y22)):
        if y56[i][1]==9 and y52[i][1]==3:
            c.append(i)
    # print('c:',c)
    for index in c:
        # print('y5:',y5[index-1][1])
        if abs(y22[index-1][1])<=3 and abs(y22[index-1][1])!=0:
            d.append(index)
    # tmp5 = min(d)-1
    # print('end_lon_poserr:',y22[tmp5][1])

    ##AVP最终停车角度跟踪偏差
    # for i in range(len(y5)):
    #     if y56[i][1]==9 and y52[i][1]==3:
    #         e.append(i)
    # for index in e:
    #     print('y5:',y5[index-1][1])
    #     if abs(y5[index-1][1])<=1 and abs(y5[index-1][1])!=0:
    #         f.append(index)
    # tmp6 = min(f)-1
    # print('end_pure_poserr:',y5[tmp6-1][1])
    
    ##NTP最终停车跟踪偏差
    # acccmd = []
    # for i in range(len(y64)):
    #         # print('b:',y64[index][1])
    #     if abs(y64[i][1])>= 2:
    #         b.append(i)
    #         temp=min(b)
    # print('end_lat_poserr:',y4[temp-1][1])

    ##AVP泊车把数
    # temp1=0
    # for i in range(len(y52)):
    #     if y56[i][1]==9 and y52[i][1]!=3:
    #         if y52[i+1][1]!=y52[i][1]:
    #             temp1=temp1+1
    # print('泊车完成把数：',temp1)

            

    y70 = []
    y71 = []
    y72 = []
    y73 = []
    for i in range(len(y65)):
        y70.append([y65[i][0],(y67[i][1]- y65[i][1])*math.sin(y69[i][1]/57.3)])
        y71.append([y65[i][0],(y68[i][1]- y66[i][1])*math.cos(y69[i][1]/57.3)])
        y72.append([y65[i][0],y71[i][1]-y70[i][1]])
        y73.append([y65[i][1],y66[i][1]])
  
    # ax1.plot([array[0] for array in position_x_list],  [array[1] for array in position_x_list])
    # ax1.plot([array[0] for array in y38],  [array[1] for array in y38],color='r',linestyle='-',label='tra_lon_startpoint_index')
    # ax1.legend()
    ax1.plot([array[0] for array in y22],  [array[1] for array in y22],color='b',linestyle='-',label='lonctrl_sys_poserr')
    ax1.grid()
    ax1.legend()
    ax1.plot([array[0] for array in y4],  [array[1] for array in y4],color='r',linestyle='-',label='ctrldec_lat_sys_poserr')
    ax1.legend()
    ax1.plot([array[0] for array in y82],  [array[1] for array in y82],color='y',linestyle='-',label='replaning_flag')
    ax1.legend()
    # ax1.plot([array[0] for array in y73],  [array[1] for array in y73],color='g',linestyle='-',label='pose')
    # ax1.legend()
    # ax1.plot([array[0] for array in y72],  [array[1] for array in y72],color='g',linestyle='-',label='Cal_lat_sys_poserr')
    # ax1.legend()
    
    # ax1.plot([array[0] for array in y36],  [array[1] for array in y36],color='g',linestyle='-',label='posedata_world_pos_y')
    # ax1.legend()
    # print([array[0] for array in position_x_list])
    # print([array[1] for array in position_x_list])
    ax2.plot([array[0] for array in y15],  [array[1] for array in y15],color='g',linestyle='-',label='lonctrl_fdbk_vx')
    ax2.grid()
    ax2.legend()
    ax2.plot([array[0] for array in y32],  [array[1] for array in y32],color='r',linestyle='--',label='lonctrl_sys_velff')
    ax2.legend()
    # ax2.plot([array[0] for array in y28],  [array[1] for array in y28],color='b',linestyle='-.',label='lon_vel_output_accCmd_filter')
    # ax2.legend()
    # ax2.plot([array[0] for array in y16],  [array[1] for array in y16],linestyle='-',label='lonctrl_vel_velcmd_lmt')
    # ax2.legend()


    #ax3.plot([array[0] for array in y3],  [array[1] for array in y3],color='b',linestyle='-',label='ctrldec_lat_sys_curvff')
    #ax3.grid()
    #ax3.legend()
    ax3.plot([array[0] for array in y17],  [array[1] for array in y17],color='b',linestyle='-',label='lonctrl_fdbk_ax')
    ax3.grid()
    ax3.legend()
    
    T = []
    ACCff = []
    for i in range(len(y62)):
        T.append(y62[i][0])
        ACCff.append(y62[i][1])

    ax3.plot([array[0] for array in y62],  [array[1] for array in y62],color='r',linestyle='--',label='lonctrl_sys_accff')
    ax3.legend()
    # plt.scatter(T,ACCff)
    # plt.show
    # ax3.legend()
    ax3.plot([array[0] for array in y64],  [array[1] for array in y64],color='y',linestyle='-',label='acc_cmd')
    ax3.legend()
    ax3.plot([array[0] for array in y63],  [array[1] for array in y63],color='g',linestyle='-',label='alonctrl_vel_pi_acc_cmd')
    ax3.legend()
    # ax3.plot([array[0] for array in y60],  [array[1] for array in y60],color='y',linestyle='-',label='lonctrl_vel_output_accCmd')
    # ax3.legend()
    # ax3.plot([array[0] for array in y61],  [array[1] for array in y61],color='g',linestyle='-',label='lonctrl_vel_accff_filter')
    # ax3.legend()
    # ax3.plot([array[0] for array in y19],  [array[1] for array in y19],linestyle='-.',label='lonctrl_analog_autput_throtcmd')
    # ax3.legend()


    # ax4.plot([array[0] for array in y4],  [array[1] for array in y4],color='b',linestyle='-',label='ctrldec_lat_sys_poserr')
    # ax4.grid()
    # ax4.legend()
    ax4.plot([array[0] for array in y5],  [array[1] for array in y5],color='r',linestyle='--',label='latictrl_tors_pure_yawerr')
    ax4.legend()
    ax4.grid()
    ax4.plot([array[0] for array in y6],  [array[1] for array in y6],linestyle='-',label='latictrl_tors_yawerr')
    ax4.legend()
    # ax4.set_ylim([-20,20])
    # ax4.plot([array[0] for array in y50],  [array[1] for array in y50],color='r',linestyle='-',label='latictrl_tors_pi_torsrate')
    # ax4.legend()
    #ax6.plot([array[0] for array in y7],  [array[1] for array in y7],color='b',linestyle='-',label='latictrl_steer_steer_err')
    #ax6.legend()
    #ax6.grid()
    ax5.plot([array[0] for array in y8],  [array[1] for array in y8],color='r',linestyle='--',label='latictrl_fdbk_heading')
    ax5.legend()
    ax5.grid()
    ax5.plot([array[0] for array in y9],  [array[1] for array in y9],color='b',linestyle='-',label='ctrldec_lat_sys_yawff')
    ax5.legend()
    # ax8.plot([array[0] for array in y10],  [array[1] for array in y10],color='r',linestyle='--',label='latictrl_fdbk_yawrate')
    # ax8.legend()
    # ax8.grid()
    # ax8.plot([array[0] for array in y11],  [array[1] for array in y11],color='b',linestyle='-',label='lat_rate_yawratecmd_lmt')
    # ax8.legend()
    # ax6.plot([array[0] for array in y74],  [array[1] for array in y74],color='g',linestyle='-.',label='latictrl_offset_pi_torscmd')
    # ax6.legend()
    # ax6.plot([array[0] for array in y45],  [array[1] for array in y45],color='y',linestyle='-',label='latictrl_tors_torsrateff')
    # ax6.legend()
    ax6.plot([array[0] for array in y10],  [array[1] for array in y10],color='b',linestyle='-',label='latictrl_fdbk_yawrate')
    ax6.legend()
    ax6.grid()
    # ax6.plot([array[0] for array in y40],  [array[1] for array in y40],color='b',linestyle='-.',label='kinematic_yawrate')
    # ax6.legend()
    ax6.plot([array[0] for array in y11],  [array[1] for array in y11],linestyle='-',label='lat_rate_yawratecmd_lmt')
    ax6.legend()
    # ax6.plot([array[0] for array in y51],  [array[1] for array in y51],linestyle='--',label='lat_tors_pi_leadfilter_torsrate')
    # ax6.legend()
    ax6.plot([array[0] for array in y54],  [array[1] for array in y54],color='r',linestyle='-',label='lat_rate_yawratecmd_lmt_filter')
    ax6.legend()
    # ax6.plot([array[0] for array in y50],  [array[1] for array in y50],color='g',linestyle='-',label='latictrl_tors_pi_torsrate')
    # ax6.legend()
    # ax6.plot([array[0] for array in y75],  [array[1] for array in y75],color='b',linestyle='-',label='ctrldec_lat_resetflag')
    # ax6.legend()
    # ax6.plot([array[0] for array in y55],  [array[1] for array in y55],color='y',linestyle='-',label='lat_rate_pi_steer')
    # ax6.legend()
    # ax6.plot([array[0] for array in y46],  [array[1] for array in y46],linestyle='-',label='latictrl_rate_output_front_steercmd')
    # ax6.legend()
    # ax6.plot([array[0] for array in y47],  [array[1] for array in y47],linestyle='-',label='latictrl_rate_output_front_steercmd_offset')
    # ax6.legend()
    # ax6.plot([array[0] for array in y43],  [array[1] for array in y43],linestyle='--',label='latictrl_tors_pi_torsrate')
    # ax6.legend()
    # ax6.plot([array[0] for array in y45],  [array[1] for array in y45],linestyle='--',label='latictrl_tors_torsrateff')
    # ax6.legend()

    # ax8.plot([array[0] for array in y15],  [array[1] for array in y15],color='b',linestyle='-',label='lat_rate_yawratecmd_lmt')
    # ax8.legend()
    ax7.plot([array[0] for array in y12],  [array[1] for array in y12],color='r',linestyle='-',label='latictrl_fdbk_steer')
    ax7.legend()
    ax7.grid()
    ax7.plot([array[0] for array in y13],  [array[1] for array in y13],color='b',linestyle='-',label='latictrl_steer_output_steercmd')
    ax7.legend()
    # ax7.plot([array[0] for array in y20],  [array[1] for array in y20],color='g',linestyle='-',label='latictrl_rate_steerff')
    # ax7.legend()
    ax7.plot([array[0] for array in y85],  [array[1] for array in y85],color='r',linestyle='-.',label='latictrl_steer_steercmd_lmt_filter')
    ax7.legend()
    # ax7.plot([array[0] for array in y84],  [array[1] for array in y84],color='g',linestyle='-',label='lat_rate_output_sw_steercmd')
    # ax7.legend()
    ax7.plot([array[0] for array in y76],  [array[1] for array in y76],color='y',linestyle='-',label='latictrl_steer_steercmd_filter')
    ax7.legend()
    ax7.plot([array[0] for array in y77],  [array[1] for array in y77],color='g',linestyle='--',label='ctrldec_lat_api_steercmd')
    ax7.legend()
    # ax7.plot([array[0] for array in y26],  [array[1] for array in y26],color='r',linestyle='-',label='latictrl_rate_pi_filter_steer')
    # ax7.legend()
    # ax9.plot([array[0] for array in y14],  [array[1] for array in y14],color='b',linestyle='-',label='driving_mode')
    # ax9.legend()
    ax8.plot([array[0] for array in y3],  [array[1] for array in y3],color='b',linestyle='-',label='ctrldec_lat_sys_curvff')
    ax8.grid()
    ax8.legend()
    # ax9.plot([array[0] for array in y79],  [array[1] for array in y79],color='y',linestyle='-',label='tra_lat_startpoint_index')
    # ax9.legend()
    # ax8.plot([array[0] for array in y80],  [array[1] for array in y80],color='g',linestyle='-',label='tra_lon_startpoint_index')
    # ax8.legend()
    # ax9.plot([array[0] for array in y81],  [array[1] for array in y81],color='r',linestyle='-',label='tra_latpre_startpoint_index')
    # ax9.legend()

    # ax9.plot([array[0] for array in y11],  [array[1] for array in y11],color='b',linestyle='-',label='lat_rate_yawratecmd_lmt')
    # ax9.legend()
    # ax9.grid()
    # ax9.plot([array[0] for array in y20],  [array[1] for array in y20],linestyle='-',label='latictrl_tors_torsrateff')
    # ax9.legend()
    ax9.plot([array[0] for array in y41],  [array[1] for array in y41],color='r',linestyle='-',label='gear_cmd')
    ax9.grid()
    ax9.legend()
    ax9.plot([array[0] for array in y52],  [array[1] for array in y52],color='y',linestyle='-',label='trajdata_gearcmd')
    ax9.legend()
    ax9.plot([array[0] for array in y53],  [array[1] for array in y53],color='b',linestyle='--',label='fdbk_gear')
    ax9.legend()
    # ax9.plot([array[0] for array in y52],  [array[1] for array in y52],color='y',linestyle='-',label='trajdata_gearcmd')
    # ax9.legend()
    # ax9.plot([array[0] for array in y53],  [array[1] for array in y53],color='b',linestyle='--',label='fdbk_gear')
    # ax9.legend()
    # ax9.plot([array[0] for array in y57],  [array[1] for array in y57],color='g',linestyle='-',label='lonctrl_modecmd')
    # ax9.legend()
    ax9.plot([array[0] for array in y58],  [array[1] for array in y58],color='r',linestyle='--',label='latctrl_modecmd')
    ax9.legend()
    ax9.plot([array[0] for array in y56],  [array[1] for array in y56],color='g',linestyle='-',label='ctrldec_sysmode')
    ax9.legend()
    ax9.plot([array[0] for array in y78],  [array[1] for array in y78],color='g',linestyle='-.',label='ctrldec_lat_resetflag')
    ax9.legend()
    # ax9.plot([array[0] for array in y59],  [array[1] for array in y59],color='g',linestyle='-',label='pitch')
    # ax9.legend()
    # ax9.plot([array[0] for array in y78],  [array[1] for array in y78],color='g',linestyle='-.',label='ctrldec_lat_resetflag')
    # ax9.legend()
    # ax1.plot([array[0] for array in y82],  [array[1] for array in y82],color='y',linestyle='-',label='replaning_flag')
    # ax1.legend()
    # ax1.plot([array[0] for array in y86],  [array[1] for array in y86],color='y',linestyle='-.',label='ctrldec_ctrl_err')
    # ax1.legend()
    plt.show()





    
def get_all_the_files(record_log_address,fileName):
    file_name_list = []
    for path,dir_list,file_list in os.walk(record_log_address):
        for file_name in file_list:
            if fileName in file_name:
                file_name_list.append(os.path.join(path,file_name))
    file_name_list.sort()
    return file_name_list

# def get_all_the_files(record_log_address):
#     file_name_list = []
#     for path,dir_list,file_list in os.walk(record_log_address):
#         for file_name in file_list:
#             file_name_list.append(os.path.join(path,file_name))
#     file_name_list.sort()
#     return file_name_list

if __name__ == '__main__':
    os.system('clear')
    cyber.init()
    #record_log_address = "/media/caros/Ubuntu/record/"
    # record_log_address = "/home/wfl/Downloads/"
    # fileName = "20220125111139.record.00011"
    # record_log_address = "/home/wfl/Desktop/0225/test_duantoulu_0225_nomap"
    # record_log_address = "/home/wfl/Downloads/AVP/0310/0310_control"
    record_log_address = sys.argv[1]
    
    fileName = sys.argv[2]
    fileNameList = get_all_the_files(record_log_address,fileName)
    # fileNameList = get_all_the_files(record_log_address)
    log_txt = open("cyber_log.txt","w")
    log_txt.close()
    data_mat = open("cyber2mat.mat","w")
    data_mat.close
    for file in fileNameList:
        test_record_reader(file)
    cyber.shutdown()
