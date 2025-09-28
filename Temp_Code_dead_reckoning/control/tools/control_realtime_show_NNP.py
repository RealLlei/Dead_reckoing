from operator import length_hint
from matplotlib import lines
import localization
from map_trajectory import Map
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import planning
from planning import planning_pb2
from proto.planning.planning_pb2 import ADCTrajectory
from proto.control.control_cmd_pb2 import ControlCommand
from proto.localization.localization_pb2 import Localization
from proto.map.navigation_pb2 import MapMsg
from google.protobuf.descriptor_pb2 import FileDescriptorProto
from cyber_py3 import record
from cyber_py3 import cyber
import time
import sys
import threading
from proto.canbus.chassis_detail_pb2 import ChassisDetail

global localization_point
sys.path.append("../")

OBSkeepInterval = 300  # ms
Kepsinon = 0.00001


class Obstacle:
    def __init__(self, id, width, x, y, vx, vy, time) -> None:
        self.id = id
        self.width = width
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.update_time = time
        x_left = x - width/2
        x_right = x + width/2
        self.poly = [[x_left, x_right], [y, y]]
        


class DataManager:
    def __init__(self, ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9) -> None:
        self.localization_point = Localization()
        self.map_manager = Map()
        self.traj_x = []
        self.traj_y = []
        self.laness = []
        self.control_dict = dict()
        self.planning_dict = dict ()
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3
        self.ax4 = ax4
        self.ax5 = ax5
        self.ax6 = ax6
        self.ax7 = ax7
        self.ax8 = ax8
        self.ax9 = ax9
        self.start_time = None
        self.need_update_plot = True


        self.line1, = self.ax1.plot([], [], color='r',linestyle='-',label='ctrldec_sysmode')
        self.ax1.legend()
        self.ax1.grid()
        # self.line14, = self.ax1.plot([], [], color='r',linestyle='-',label='isReplanning')
        # self.ax1.legend()
        self.line2, = self.ax2.plot([], [], linestyle='-',label='latictrl_fdbk_vxb')
        self.ax2.legend()
        self.ax2.grid()
        self.line3, = self.ax3.plot([], [], linestyle='-',label='ctrldec_lat_sys_curvff')
        self.ax3.legend()
        self.ax3.grid()
        self.line4, = self.ax4.plot([], [], linestyle='-',label='ctrldec_lat_sys_poserr')
        self.ax4.legend()
        self.ax4.grid()
        self.line5, = self.ax5.plot([], [], linestyle='--',label='latictrl_tors_pure_yawerr')
        self.ax5.legend()
        self.ax5.grid()
        self.line6, = self.ax5.plot([], [], linestyle='-',label='latictrl_tors_yawerr')
        self.ax5.legend()
        self.line7, = self.ax6.plot([], [], linestyle='-',label='latictrl_steer_steer_err')
        self.ax6.legend()
        self.ax6.grid()
        self.line8, = self.ax7.plot([], [], linestyle='--',label='latictrl_fdbk_heading')
        self.ax7.legend()
        self.ax7.grid()
        self.line9, = self.ax7.plot([], [], linestyle='-',label='ctrldec_lat_sys_yawff')
        self.ax7.legend()
        self.line10, = self.ax8.plot([], [], linestyle='--',label='latictrl_fdbk_yawrate')
        self.ax8.legend()
        self.ax8.grid()
        self.line11, = self.ax8.plot([], [], linestyle='-',label='latictrl_rate_yawratecmd_lmt')
        self.ax8.legend()
        self.line12, = self.ax9.plot([], [],color='r', linestyle='--',label='latictrl_fdbk_steer')
        self.ax9.legend()
        self.ax9.grid()
        self.line13, = self.ax9.plot([], [], color='b',linestyle='-',label='latictrl_steer_output_steercmd')
        self.ax9.legend()
        #line, = ax.plot(self.traj_x, self.traj_y, linestyle='--',
        #                color='red', markersize=2)  # "r--"
        #self.laness.append(line)

        #for p in range(1, 4):
        #    line, = ax.plot(self.traj_x, self.traj_y, linestyle='-', color='blue',
        #                    markersize=1)  # "b-"
        #    self.laness.append(line)
        #    line, = ax.plot(self.traj_x, self.traj_y,  linestyle='-',
        #                    color='blue', markersize=1)  # "b-"
        #    self.laness.append(line)
        #    line, = ax.plot(self.traj_x, self.traj_y,  linestyle=':',
        #                    color='green', markersize=1)  # "g:"
        #    self.laness.append(line)

        #for p in range(10, 30):  # appen obs line
        #    line, = ax.plot([], [], 'bo-', markersize=2)
        #    self.laness.append(line)

        self.r_lock = threading.RLock()
        #self.obs_dict = {}

    def update(self) -> None:
        self.r_lock.acquire()
        # self.ax1.clear()
        # self.ax2.clear()
        # self.ax3.clear()
        # self.ax4.clear()
        # self.ax5.clear()
        # self.ax6.clear()
        # self.ax7.clear()
        # self.ax8.clear()
        # self.ax9.clear()
        x = []
        y1 = []
        y2 = []
        y3 = []
        y4 = []
        y5 = []
        y6 = []
        y7 = []
        y8 = []
        y9 = []
        y10 = []
        y11 = []
        y12 = []
        y13 = []
        # y14 = []

        self.current_time = []
        x1 = []
        for t,control_pb in self.control_dict.items():
            x.append(t)
            if self.start_time == None:
                self.start_time = x[0]
                # print("pppppp: " + str(x[0]))
            self.current_time = [t - self.start_time for t in x]
            x1 = self.current_time

            mbd_debug = control_pb.mbd_debug
        
            y1.append(mbd_debug.ctrl_dec_debug.ctrldec_sysmode)
            y2.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_vxb)
            y3.append(mbd_debug.ctrl_dec_debug.ctrldec_lat_sys_curvff)
            y4.append(mbd_debug.ctrl_dec_debug.ctrldec_lat_sys_poserr)
            y5.append(mbd_debug.lat_ctrl_debug.latictrl_tors_pure_yawerr)
            y6.append(mbd_debug.lat_ctrl_debug.latictrl_tors_yawerr)
            y7.append(mbd_debug.lat_ctrl_debug.latictrl_steer_steer_err)
            y8.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_heading)
            y9.append(mbd_debug.ctrl_dec_debug.ctrldec_lat_sys_yawff)
            y10.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_yawrate)
            y11.append(mbd_debug.lat_ctrl_debug.latictrl_rate_yawratecmd_lmt)
            y12.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_steer)
            y13.append(mbd_debug.lat_ctrl_debug.latictrl_steer_output_steercmd)
        # for t,planning_pb in self.planning_dict.items():
        #     y14.append(planning_pb.is_replan)

            # y2.append(mbd_debug.ctrl_dec_debug.ctrldec_lat_sys_poserr)
            # y3.append(mbd_debug.ctrl_dec_debug.ctrldec_lat_sys_yawff)
            # y4.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_heading)
            # y5.append(mbd_debug.lat_ctrl_debug.latictrl_tors_yawerr)
            # y6.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_yawrate)
            # y7.append(mbd_debug.lat_ctrl_debug.latictrl_tors_output_yawratecmd)
            # y8.append(mbd_debug.lat_ctrl_debug.latictrl_fdbk_steer)
            # y9.append(mbd_debug.lat_ctrl_debug.latictrl_steer_output_steercmd)
            # y10.append(mbd_debug.lat_ctrl_debug.latictrl_steer_steertorque_cmd)


        if len(x1) > 0:
            # print(time.time(),"plot start")
            self.ax1.set_xlim(x1[0], x1[0] + 5)
            self.ax2.set_xlim(x1[0], x1[0] + 5)
            self.ax2.set_ylim(min(y2) - 1, max(y2) +1)

            self.ax3.set_xlim(x1[0], x1[0] + 5)
            self.ax3.set_ylim(min(y3)-0.1, max(y3) +0.1)

            self.ax4.set_xlim(x1[0], x1[0] + 5)
            self.ax4.set_ylim(min(y3) - 1, min(y3) +1)

            self.ax5.set_xlim(x1[0], x1[0] + 5)
            self.ax5.set_ylim(min(y5) - 1, max(y5) +1)

            self.ax6.set_xlim(x1[0], x1[0] + 5)
            self.ax6.set_ylim(min(y7) - 1, max(y7) +1)

            self.ax7.set_xlim(x1[0], x1[0] + 5)
            self.ax7.set_ylim(min(y8) - 1, max(y8) +1)

            self.ax8.set_xlim(x1[0], x1[0] + 5)
            self.ax8.set_ylim(min(y10) - 1, max(y10) +1)

            self.ax9.set_xlim(x1[0], x1[0] + 5)
            self.ax9.set_ylim(min(y12) - 10, max(y12) +10)
            self.line1.set_data(x1, y1)
            self.line2.set_data(x1, y2)
            self.line3.set_data(x1, y3)
            self.line4.set_data(x1, y4)
            self.line5.set_data(x1, y5)
            self.line6.set_data(x1, y6)
            self.line7.set_data(x1, y7)
            self.line8.set_data(x1, y8)
            self.line9.set_data(x1, y9)
            self.line10.set_data(x1, y10)
            self.line11.set_data(x1, y11)
            self.line12.set_data(x1, y12)
            self.line13.set_data(x1, y13)
            # self.line14.set_data(x1,y14)

        # self.ax1.plot(x1,y1,color='r',linestyle='-',label='ctrldec_sysmode')
        # self.ax1.legend()
        # self.ax1.grid()
        # self.ax2.plot(x1,y2,color='b',linestyle='--',label='lat_sys_poserr')
        # self.ax2.legend()
        # self.ax3.plot(x1,y3,color='b',linestyle='--',label='yawff')
        # self.ax3.legend()
        # self.ax2.grid()
        # self.ax3.plot(x1,y4,label='heading')
        # self.ax3.legend()
        # self.ax3.grid()
        # self.ax4.plot(x1,y5,label='latictrl_tors_yawerr')
        # self.ax4.grid()
        # self.ax4.legend()
        # self.ax5.plot(x1,y6,label='latictrl_fdbk_yawrate')
        # self.ax5.legend()
        # self.ax5.grid()
        # self.ax6.plot(x1,y7,label='latictrl_tors_output_yawratecmd')
        # self.ax6.legend()
        # self.ax6.grid()
        # self.ax7.plot(x1,y8,label='latictrl_fdbk_steer')
        # self.ax7.legend()
        # self.ax7.grid()
        # self.ax8.plot(x1,y9,label='latictrl_steer_output_steercmd')
        # self.ax8.legend()
        # self.ax8.grid()
        # self.ax9.plot(x,y10,label='latictrl_steer_steertorque_cmd')
        # self.ax9.legend()
        # self.ax9.grid()

        # self.need_update_plot = False
        self.r_lock.release()

        # # remove old obstalce
        # for k in list(self.obs_dict.keys()):
        #     now = time.time()
        #     interval = now - self.obs_dict[k].update_time
        #     if(interval * 1000 > OBSkeepInterval):
        #         del self.obs_dict[k]

        # # draw_map
        # lane_ids = []
        # # self.map_manager.draw_lanes(ax, lane_ids, self.laness)

        # # draw trajectory

        # self.laness[0].set_xdata(self.traj_y)
        # self.laness[0].set_ydata(self.traj_x)


        # # draw obstacle
        # for p in range(10, 25):
        #     self.laness[p].set_data([], [])
        # for obs in self.obs_dict.values():
        #     self.laness[obs.id+10].set_data(obs.poly[0], obs.poly[1])
        # self.r_lock.release()

    def updateControl(self, control_pb) -> None:
        try:
            timestamp = control_pb.mbd_debug.traj_calc_debug.trajcalc_globaltime_timestamp
            self.control_dict[timestamp] = control_pb

            if(len(self.control_dict)>50):
                self.control_dict.pop(list(self.control_dict.keys())[0])

            self.need_update_plot = True

            # line, = ax.plot(traj_y, traj_x, "r--", markersize=2)
        except NameError:
            time.sleep(0.1)
        

    def updateTrajectory(self, planning_pb) -> None:
        self.localization_point = planning_pb.debug.planning_data.init_point.path_point
        try:
            # ti = planning_pb.header.data_stamp
            # self.planning_dict[ti] = planning_pb
            # if(len(self.planning_dict)>50):
            #     self.planning_dict.pop(list(self.planning_dict.keys())[0])
            

            self.need_update_plot = True

            self.traj_x = []
            self.traj_y = []
            # for p in planning_pb.trajectory_point:
            #     x = p.path_point.x - self.localization_point.x
            #     y = p.path_point.y - self.localization_point.y
            #     sin_theta = math.sin(self.localization_point.theta)
            #     cos_theta = math.cos(self.localization_point.theta)
            #     self.traj_x.append(x*cos_theta + y*sin_theta)
            #     self.traj_y.append(-(-x*sin_theta + y*cos_theta))
            # line, = ax.plot(traj_y, traj_x, "r--", markersize=2)
        except NameError:
            time.sleep(1)

    def updateMap(self, navimap_pb) -> None:
        self.map_manager.update(navimap_pb)

    def updateObstacle(self, obstacle):
        self.obs_dict[obstacle.id] = obstacle


def callback_plot(i):
    if data_manager.need_update_plot == True:
        data_manager.update()   

def callback_control(control_pb):
    data_manager.r_lock.acquire()
    data_manager.updateControl(control_pb)
    data_manager.r_lock.release()


def callback_planning(planning_pb):
    data_manager.r_lock.acquire()
    data_manager.updateTrajectory(planning_pb)
    data_manager.r_lock.release()


def callback_hdmap(navimap_pb):
    data_manager.r_lock.acquire()
    data_manager.updateMap(navimap_pb)
    data_manager.r_lock.release()


def callback_chassis(chassis_detail_pb):
    id = chassis_detail_pb.neta.vis_obs_msg_1_675.vis_obs_id_01
    x = chassis_detail_pb.neta.vis_obs_msg_3_677.vis_obs_lat_pos_01
    y = chassis_detail_pb.neta.vis_obs_msg_3_677.vis_obs_long_pos_01
    vx = chassis_detail_pb.neta.vis_obs_msg_3_677.vis_obs_lat_vel_01
    vy = chassis_detail_pb.neta.vis_obs_msg_3_677.vis_obs_long_vel_01
    width = chassis_detail_pb.neta.vis_obs_msg_3_677.vis_obs_width_01
    now = time.time()
    if(abs(x) >= Kepsinon and abs(y) >= Kepsinon):
        data_manager.r_lock.acquire()
        data_manager.updateObstacle(Obstacle(id, width, x, y, vx, vy, now))
        data_manager.r_lock.release()
    return


if __name__ == "__main__":
    global data_manager

    #fig = plt.figure(figsize=(8, 8))
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

    data_manager = DataManager(ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9)
    # ax.set_aspect(1)
    #plt.xlim(-10, 10)
    #plt.y1lim(-5, 100)

    ax1.set_ylim(-5, 2)
    # ax2.set_xlim(0, 50)
    # ax2.set_ylim(-1, 25)
    # ax3.set_xlim(0, 50)
    # ax3.set_ylim(-0.001, 0.001)
    # ax4.set_xlim(0, 50)
    # ax4.set_ylim(-1, 1.5)
    # ax5.set_xlim(0, 50)
    ax5.set_ylim(-2, 2)
    # ax6.set_xlim(0, 50)
    ax6.set_ylim(-5, 6)
    # ax7.set_xlim(0, 50)
    ax7.set_ylim(-7, 0)
    # ax8.set_xlim(0, 50)
    ax8.set_ylim(-2, 2)
    # ax9.set_xlim(0, 50)
    ax9.set_ylim(-10,15)




    # plt.get_current_fig_manager().full_screen_toggle()

    cyber.init()
    navigation_sub = cyber.Node("stat_map")
    navigation_sub.create_reader('/TL/navigation_hdmap',
                                 MapMsg, callback_hdmap)

    # planning_sub = cyber.Node("stat_planning")
    # planning_sub.create_reader('/TL/planning',
    #                           ADCTrajectory, callback_planning)
    control_sub = cyber.Node("stat_control")
    control_sub.create_reader('/TL/control',
                               ControlCommand, callback_control)

    # eq3_sub = cyber.Node("stat_eq3")
    # eq3_sub.create_reader(
    #     '/TL/canbus/chassis_detail', ChassisDetail, callback_chassis)

    anim = animation.FuncAnimation(fig, callback_plot, interval=100)
    plt.show()
    # while not cyber.is_shutdown():
    #     plt.clf
    #     plt.pause(0.1)
    #     plt.ioff()
    # time.sleep(5)
