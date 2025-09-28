
from xml.sax.handler import property_interning_dict
import numpy as np

class control_cmd:

    dict ={}
    def __init__(self):
        self.name = 'control'     # 名称
        self.clear()
    def clear(self):
        self.header = []
        self.ctrl_req_mode = []
        self.throttle = []
        self.brake = []
        self.steering_rate = []
        self.steering_target = []
        self.parking_brake = []
        self.speed = []
        self.acceleration = []
        self.reset_model = []
        self.engine_on_off = []
        self.trajectory_fraction = []
        self.gear_location = []
        self.signal = []
        self.latency_stats = []
        self.pad_msg = []
        self.engage_advice = []
        self.is_in_safe_mode = []
        self.torque = []
        self.steering_angle_rate = []
        self.steering_angle = []
        self.steering_torque = []
        self.left_turn = []
        self.right_turn = []
        self.high_beam = []
        self.low_beam = []
        self.horn = []
        self.turnsignal = []
        self.MbdDebugFromMCU = []         
        self.ctrl_dec_debug = []         
        self.lon_ctrl_debug = []
        self.lat_ctrl_debug = []
        self.ctrl_output_debug = []
        self.brake_cmd = []
        self.throttle_cmd = []
        self.acc_cmd = []
        self.gear_enable = []
        self.gear_cmd = []
        self.emerg_enable = []
        self.steer_cmd = []
        self.steer_torque_cmd = []
        self.posecalc_posedata_timestamp = []
        self.posecalc_inputdata_valid = []
        self.posedata_world_pos_x = []
        self.posedata_world_pos_y = []
        self.posedata_vrf_vel_x = []
        self.posecalc_vrf_vel_y = []
        self.posecalc_world_vel_x = []
        self.posecalc_world_vel_y = []
        self.posecalc_world_acc_x = []
        self.posecalc_world_acc_y = []
        self.posecalc_world_heading = []
        self.posecalc_world_pitch = []
        self.posecalc_timedelay = []
        self.posecalc_enable = []
        self.global_time_timestamp_sec = []
        self.global_time_timestamp_nec = []
        self.global_time_gettime_enable = []



        self.trajCalc_trajdata_replaning_flag = []
        self.trajcalc_trajdata_estop = []
        self.trajcalc_trajdata_gearcmd = []
        self.trajcalc_inputdata_valid = []
        self.trajcalc_trajdata_timestamp = []
        self.trajcalc_globaltime_timestamp = []
        self.trajcalc_trajdata_pointtime_check = []
        self.trajcalc_trajdata_timecheck = []
        self.trajcalc_enable = []
        self.trajcalc_lon_startpoint_index = []
        self.trajcalc_lon_startpoint_time = []
        self.trajcalc_lon_linear_ratio = []
        self.trajcalc_lon_match_pointx = []
        self.trajcalc_lon_match_pointy = []
        self.trajcalc_lon_poserrcmd = []
        self.trajcalc_lon_headingcmd = []
        self.trajcalc_lon_velcmd = []
        self.trajcalc_lon_acc_cmd = []
        self.trajcalc_lon_curvcmd = []
        self.trajcalc_lonpre_startpoint_index = []
        self.trajcalc_lonpre_startpoint_time = []
        self.trajcalc_lonpre_linear_ratio = []
        self.trajcalc_lonpre_match_pointx = []
        self.trajcalc_lonpre_match_pointy = []
        self.trajcalc_lonpre_poserrcmd = []
        self.trajcalc_lonpre_headingcmd = []
        self.trajcalc_lonpre_velcmd = []
        self.trajcalc_lonpre_acc_cmd = []
        self.trajcalc_lonpre_curvrmd = []
        self.trajcalc_posedata_posex = []
        self.trajcalc_PoseData_posey = []
        self.trajcalc_lat_Dot_fstpoint2veh = []
        self.trajcalc_lat_Dot_endpoint2veh = []
        self.trajcalc_lat_matchpoint_vase = []
        self.trajcalc_lat_startpoint_index = []
        self.trajcalc_lat_linear_ratio = []
        self.trajcalc_lat_match_pointx = []
        self.trajcalc_lat_match_pointy = []
        self.trajcalc_lat_poserrcmd = []
        self.trajcalc_lat_headingcmd = []
        self.trajcalc_lat_velcmd = []
        self.trajcalc_lat_acc_cmd = []
        self.trajcalc_lat_curvcmd = []
        self.trajcalc_posedata_preposex = []
        self.trajcalc_posedata_preposey = []
        self.trajcalc_latpre_dot_fstpoint2veh = []
        self.trajcalc_latpre_dot_endpoint2veh = []
        self.trajcalc_latpre_matchpoint_vase = []
        self.trajcalc_latpre_startpoint_index = []
        self.trajcalc_latpre_linear_ratio = []
        self.trajcalc_latpre_match_pointx = []
        self.trajcalc_latpre_match_pointy = []
        self.trajcalc_latpre_poserrcmd = []
        self.trajcalc_latpre_headingcmd = []
        self.trajcalc_latpre_velcmd = []
        self.trajcalc_latpre_acc_cmd = []
        self.trajcalc_latpre_curvcmd = []

        self.ctrldec_sysmode = []
        self.ctrldec_req_auto = []
        self.ctrldec_automode_is_estop = []
        self.ctrldec_is_gear_change_req = []
        self.ctrldec_is_stop_steer_ctrl = []
        self.ctrldec_is_vehicle_standstill = []
        self.ctrldec_is_steer_set_ok = []
        self.ctrldec_lat_sysmodecmd = []
        self.ctrldec_lat_resetflag = []
        self.ctrldec_lat_sys_poserr = []
        self.ctrldec_lat_sys_yawff = []
        self.ctrldec_lat_sys_velff = []
        self.ctrldec_lat_sys_curvff = []
        self.ctrldec_lat_api_poscmd = []
        self.ctrldec_lat_api_yawcmd = []
        self.ctrldec_lat_api_curvcmd = []
        self.ctrldec_lat_api_steercmd = []
        self.ctrldec_lon_sysmodecmd = []
        self.ctrldec_lon_resetflag = []
        self.ctrldec_lon_sys_poserr = []
        self.ctrldec_lon_sys_velff = []
        self.ctrldec_lon_sys_accff = []
        self.ctrldec_lon_sys_gearcmd = []
        self.ctrldec_lon_sys_gear_ena = []
        self.ctrldec_lon_sys_brk_emerg = []
        self.ctrldec_lon_api_poscmd = []
        self.ctrldec_lon_api_velff = []
        self.ctrldec_lon_api_acc_cmd = []
        self.ctrldec_lon_api_thrcmd = []
        self.ctrldec_lon_api_brkcmd = []
        self.ctrldec_ctrl_err = []

        self.lonctrl_modecmd = []
        self.lonctrl_resetflag = []
        self.lonctrl_sys_poserr = []
        self.lonctrl_sys_velff = []
        self.lonctrl_sys_accff = []
        self.lonctrl_sys_gearcmd = []
        self.lonctrl_sys_gearena = []
        self.lonctrl_sys_brkemerg = []
        self.lonctrl_api_poscmd = []
        self.lonctrl_api_velcmd = []
        self.lonctrl_api_acccmd = []
        self.lonctrl_api_thrcmd = []
        self.lonctrl_api_brkcmd = []
        self.lonctrl_fdbk_vx = []
        self.lonctrl_fdbk_ax = []
        self.lonctrl_fdbk_pitch = []
        self.lonctrl_fdbk_gear = []
        self.lonctrl_fdbk_reverse = []
        self.lonctrl_pos_dyn = []
        self.lonctrl_posctrl_p = []
        self.lonctrl_pos_poserr_filter = []
        self.lonctrl_pos_pi_velcmd = []
        self.lonctrl_pos_output_velcmd = []
        self.lonctrl_vel_dyn = []
        self.lonctrl_vel_velcmd_lmt = []
        self.lonctrl_vel_vel_err = []
        self.lonctrl_velctrl_p = []
        self.lonctrl_velctrl_i = []
        self.lonctrl_vel_pi_acc_cmd = []
        self.lonctrl_vel_pi_acccmd_filter = []
        self.lonctrl_vel_accpitch = []
        self.lonctrl_vel_accdamper = []
        self.lonctrl_vel_accff_filter = []
        self.lonctrl_vel_output_accCmd = []
        self.lonctrl_vel_output_accCmd_filter = []
        self.lonctrl_thrust_thr_dyn = []
        self.lonctrl_thrust_thr_accerr = []
        self.lonctrl_thrust_brk_dyn = []
        self.lonctrl_thrust_brk_accerr = []
        self.lonctrl_thrust_fdbk_ax_filter = []
        self.lonctrl_thrust_thr_acc_cmd_filter = []
        self.lonctrl_thrust_brk_acc_cmd_filter = []
        self.lonctrl_thrustctrl_thr_p = []
        self.lonctrl_thrustctrl_thr_i = []
        self.lonctrl_thrustctrl_brk_p = []
        self.lonctrl_thrustctrl_brk_i = []
        self.lonctrl_thrust_pi_thr_acc_cmd = []
        self.lonctrl_thrust_pi_brk_acc_cmd = []
        self.lonctrl_thrust_acc_cmd_filter_lmt = []
        self.lonctrl_thrust_acctothr_gain = []
        self.lonctrl_thrust_throut_throcmd = []
        self.lonctrl_thrust_acctobrk_gain = []
        self.lonctrl_thrust_brkout_brkcmd = []
        self.lonctrl_analog_autput_throtcmd = []
        self.lonctrl_analog_autput_brkcmd = []
        
        self.latctrl_modecmd = []
        self.latctrl_resetflag = []
        self.latctrl_sys_poserr = []
        self.latctrl_sys_yawff = []
        self.latctrl_sys_velff = []
        self.latctrl_sys_curvff = []
        self.latctrl_api_poscmd = []
        self.latctrl_api_yawcmd = []
        self.latctrl_api_curvcmd = []
        self.latctrl_api_steercmd = []
        self.latictrl_fdbk_vxb = []
        self.latictrl_fdbk_heading = []
        self.latictrl_fdbk_yawrate = []
        self.latictrl_fdbk_steer = []
        self.latictrl_fdbk_gear = []
        self.latictrl_fdbk_rvsflag = []
        self.latictrl_offset_dyn = []
        self.latictrl_offsetctrl_i = []
        self.latictrl_offsetctrl_p = []
        self.latictrl_offset_offseterr = []
        self.latictrl_offset_pi_torscmd = []
        self.latictrl_offset_torsrateffcmd = []
        self.latictrl_offset_output_yawcmd = []
        self.latictrl_tors_dyn = []
        self.latictrl_tors_yawerr = []
        self.latictrl_yawctrl_p = []
        self.latictrl_yawctrl_i = []
        self.latictrl_tors_pi_torsrate = []
        self.latictrl_tors_pi_leadfilter_torsrate = []
        self.latictrl_tors_torsrateff = []
        self.latictrl_tors_output_yawratecmd = []
        self.latictrl_rate_dyn = []
        self.latictrl_rate_p = []
        self.latictrl_rate_i = []
        self.latictrl_rate_yawratecmd_lmt = []
        self.latictrl_rate_filter_yawratecmd_lmt = []
        self.latictrl_rate_pi_steer = []
        self.latictrl_rate_pi_filter_steer = []
        self.latictrl_rate_steerff = []
        self.latictrl_rate_output_front_steercmd = []
        self.latictrl_rate_output_sw_steercmd = []
        self.latictrl_steer_steercmd_filter = []
        self.latictrl_steer_max_steerrate_value = []
        self.latictrl_steer_steercmd_lmt_filter = []
        self.latictrl_steer_output_steercmd = []
        self.latictrl_steer_steertorque_initial = []
        self.latictrl_steer_steertorque_cmd = []
        self.latictrl_tors_pure_yawerr = []
        self.latictrl_rate_output_front_steercmd_offset = []
        # self.latictrl_steer_steer_err = []
        self.latictrl_yaw_curve_compsate = []
        self.latictrl_rate_reference_model = []
        self.latictrl_rate_mrac_cmd = []
        self.latictrl_rate_eso_cmd = []
        self.latictrl_rate_steer_offset = []
        self.latictrl_rate_ramp_estimate = []
        self.latictrl_error = []










      
        self.header = header3()
        self.start_time = None
        self.time = []
        #print(" control_cmd init succesfully ")
        
        self.pose_calc_debug = pose_calc_debug()
        self.traj_calc_debug = traj_calc_debug()
        self.ctrl_dec_debug = ctrl_dec_debug()
        self.lon_ctrl_debug = lon_ctrl_debug()
        self.lat_ctrl_debug = lat_ctrl_debug()
        self.ctrl_output_debug = CtrlOutputDebug()
        self.header = header3()

    def update(self,MbdDebugFromMCU):
        # self.clear()
        self.ctrl_output_debug.clear()
        self.traj_calc_debug.clear()
        self.pose_calc_debug.clear()
        self.ctrl_dec_debug.clear()
        self.lon_ctrl_debug.clear()
        self.lat_ctrl_debug.clear()
        # for p in MbdDebugFromMCU.MbdDebugFromMCU:
        #     self.ctrl_output_debug.update(p)
        self.ctrl_output_debug.update(MbdDebugFromMCU)
        self.traj_calc_debug.update(MbdDebugFromMCU)
        self.pose_calc_debug.update(MbdDebugFromMCU)
        self.ctrl_dec_debug.update(MbdDebugFromMCU)
        self.lon_ctrl_debug.update(MbdDebugFromMCU)
        self.lat_ctrl_debug.update(MbdDebugFromMCU)
        self.header.update(MbdDebugFromMCU)
        if self.start_time is None:
            self.start_time = self.header.data_stamp
        current_time = self.header.data_stamp - self.start_time
        # self.ctrl_req_mode = MbdDebugFromMCU.ctrl_req_mode
        # self.throttle = MbdDebugFromMCU.throttle
        # self.brake = MbdDebugFromMCU.brake
        # self.steering_rate = MbdDebugFromMCU.steering_rate
        # self.steering_target = MbdDebugFromMCU.steering_target
        # self.parking_brake = MbdDebugFromMCU.parking_brake
        self.time.append([current_time,self.header.data_stamp])
        self.brake_cmd.append([current_time] + self.ctrl_output_debug.brake_cmd)
        self.throttle_cmd.append([current_time] + self.ctrl_output_debug.throttle_cmd)
        self.acc_cmd.append([current_time] + self.ctrl_output_debug.acc_cmd)
        self.gear_enable.append([current_time] + self.ctrl_output_debug.gear_enable)
        self.gear_cmd.append([current_time] + self.ctrl_output_debug.gear_cmd)
        self.emerg_enable.append([current_time] + self.ctrl_output_debug.emerg_enable)
        self.steer_cmd.append([current_time] + self.ctrl_output_debug.steer_cmd)
        self.steer_torque_cmd.append([current_time] + self.ctrl_output_debug.steer_torque_cmd)

        self.posecalc_posedata_timestamp.append([current_time] + self.pose_calc_debug.posecalc_posedata_timestamp)
        self.posecalc_inputdata_valid.append([current_time] + self.pose_calc_debug.posecalc_inputdata_valid)
        self.posedata_world_pos_x.append([current_time] + self.pose_calc_debug.posedata_world_pos_x)
        self.posedata_world_pos_y.append([current_time] + self.pose_calc_debug.posedata_world_pos_y)
        self.posedata_vrf_vel_x.append([current_time] + self.pose_calc_debug.posedata_vrf_vel_x)
        self.posecalc_vrf_vel_y.append([current_time] + self.pose_calc_debug.posecalc_vrf_vel_y)
        self.posecalc_world_vel_x.append([current_time] + self.pose_calc_debug.posecalc_world_vel_x)
        self.posecalc_world_vel_y.append([current_time] + self.pose_calc_debug.posecalc_world_vel_y)
        self.posecalc_world_acc_x.append([current_time] + self.pose_calc_debug.posecalc_world_acc_x)
        self.posecalc_world_acc_y.append([current_time] + self.pose_calc_debug.posecalc_world_acc_y)
        self.posecalc_world_heading.append([current_time] + self.pose_calc_debug.posecalc_world_heading)
        self.posecalc_world_pitch.append([current_time] + self.pose_calc_debug.posecalc_world_pitch)
        self.posecalc_timedelay.append([current_time] + self.pose_calc_debug.posecalc_timedelay)
        self.posecalc_enable.append([current_time] + self.pose_calc_debug.posecalc_enable)
        self.global_time_timestamp_sec.append([current_time] + self.pose_calc_debug.global_time_timestamp_sec)
        self.global_time_timestamp_nec.append([current_time] + self.pose_calc_debug.global_time_timestamp_nec)
        self.global_time_gettime_enable.append([current_time] + self.pose_calc_debug.global_time_gettime_enable)

        self.trajCalc_trajdata_replaning_flag.append([current_time] + self.traj_calc_debug.trajCalc_trajdata_replaning_flag)
        self.trajcalc_trajdata_estop.append([current_time] + self.traj_calc_debug.trajcalc_trajdata_estop)
        self.trajcalc_trajdata_gearcmd.append([current_time] + self.traj_calc_debug.trajcalc_trajdata_gearcmd)
        self.trajcalc_inputdata_valid.append([current_time] + self.traj_calc_debug.trajcalc_inputdata_valid)
        self.trajcalc_trajdata_timestamp.append([current_time] + self.traj_calc_debug.trajcalc_trajdata_timestamp)
        self.trajcalc_globaltime_timestamp.append([current_time] + self.traj_calc_debug.trajcalc_globaltime_timestamp)
        self.trajcalc_trajdata_pointtime_check.append([current_time] + self.traj_calc_debug.trajcalc_trajdata_pointtime_check)
        self.trajcalc_trajdata_timecheck.append([current_time] + self.traj_calc_debug.trajcalc_trajdata_timecheck)
        self.trajcalc_enable.append([current_time] + self.traj_calc_debug.trajcalc_enable)
        self.trajcalc_lon_startpoint_index.append([current_time] + self.traj_calc_debug.trajcalc_lon_startpoint_index)
        self.trajcalc_lon_startpoint_time.append([current_time] + self.traj_calc_debug.trajcalc_lon_startpoint_time)
        self.trajcalc_lon_linear_ratio.append([current_time] + self.traj_calc_debug.trajcalc_lon_linear_ratio)
        self.trajcalc_lon_match_pointx.append([current_time] + self.traj_calc_debug.trajcalc_lon_match_pointx)
        self.trajcalc_lon_match_pointy.append([current_time] + self.traj_calc_debug.trajcalc_lon_match_pointy)
        self.trajcalc_lon_poserrcmd.append([current_time] + self.traj_calc_debug.trajcalc_lon_poserrcmd)
        self.trajcalc_lon_headingcmd.append([current_time] + self.traj_calc_debug.trajcalc_lon_headingcmd)
        self.trajcalc_lon_velcmd.append([current_time] + self.traj_calc_debug.trajcalc_lon_velcmd)
        self.trajcalc_lon_acc_cmd.append([current_time] + self.traj_calc_debug.trajcalc_lon_acc_cmd)
        self.trajcalc_lon_curvcmd.append([current_time] + self.traj_calc_debug.trajcalc_lon_curvcmd)
        self.trajcalc_lonpre_startpoint_index.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_startpoint_index)
        self.trajcalc_lonpre_startpoint_time.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_startpoint_time)
        self.trajcalc_lonpre_linear_ratio.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_linear_ratio)
        self.trajcalc_lonpre_match_pointx.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_match_pointx)
        self.trajcalc_lonpre_match_pointy.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_match_pointy)
        self.trajcalc_lonpre_poserrcmd.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_poserrcmd)
        self.trajcalc_lonpre_headingcmd.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_headingcmd)
        self.trajcalc_lonpre_velcmd.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_velcmd)
        self.trajcalc_lonpre_acc_cmd.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_acc_cmd)
        self.trajcalc_lonpre_curvrmd.append([current_time] + self.traj_calc_debug.trajcalc_lonpre_curvrmd)
        self.trajcalc_posedata_posex.append([current_time] + self.traj_calc_debug.trajcalc_posedata_posex)
        self.trajcalc_PoseData_posey.append([current_time] + self.traj_calc_debug.trajcalc_PoseData_posey)
        self.trajcalc_lat_Dot_fstpoint2veh.append([current_time] + self.traj_calc_debug.trajcalc_lat_Dot_fstpoint2veh)
        self.trajcalc_lat_Dot_endpoint2veh.append([current_time] + self.traj_calc_debug.trajcalc_lat_Dot_endpoint2veh)
        self.trajcalc_lat_matchpoint_vase.append([current_time] + self.traj_calc_debug.trajcalc_lat_matchpoint_vase)
        self.trajcalc_lat_startpoint_index.append([current_time] + self.traj_calc_debug.trajcalc_lat_startpoint_index)
        self.trajcalc_lat_linear_ratio.append([current_time] + self.traj_calc_debug.trajcalc_lat_linear_ratio)
        self.trajcalc_lat_match_pointx.append([current_time] + self.traj_calc_debug.trajcalc_lat_match_pointx)
        self.trajcalc_lat_match_pointy.append([current_time] + self.traj_calc_debug.trajcalc_lat_match_pointy)
        self.trajcalc_lat_poserrcmd.append([current_time] + self.traj_calc_debug.trajcalc_lat_poserrcmd)
        self.trajcalc_lat_headingcmd.append([current_time] + self.traj_calc_debug.trajcalc_lat_headingcmd)
        self.trajcalc_lat_velcmd.append([current_time] + self.traj_calc_debug.trajcalc_lat_velcmd)
        self.trajcalc_lat_acc_cmd.append([current_time] + self.traj_calc_debug.trajcalc_lat_acc_cmd)
        self.trajcalc_lat_curvcmd.append([current_time] + self.traj_calc_debug.trajcalc_lat_curvcmd)
        self.trajcalc_posedata_preposex.append([current_time] + self.traj_calc_debug.trajcalc_posedata_preposex)
        self.trajcalc_posedata_preposey.append([current_time] + self.traj_calc_debug.trajcalc_posedata_preposey)
        self.trajcalc_latpre_dot_fstpoint2veh.append([current_time] + self.traj_calc_debug.trajcalc_latpre_dot_fstpoint2veh)
        self.trajcalc_latpre_dot_endpoint2veh.append([current_time] + self.traj_calc_debug.trajcalc_latpre_dot_endpoint2veh)
        self.trajcalc_latpre_matchpoint_vase.append([current_time] + self.traj_calc_debug.trajcalc_latpre_matchpoint_vase)
        self.trajcalc_latpre_startpoint_index.append([current_time] + self.traj_calc_debug.trajcalc_latpre_startpoint_index)
        self.trajcalc_latpre_linear_ratio.append([current_time] + self.traj_calc_debug.trajcalc_latpre_linear_ratio)
        self.trajcalc_latpre_match_pointx.append([current_time] + self.traj_calc_debug.trajcalc_latpre_match_pointx)
        self.trajcalc_latpre_match_pointy.append([current_time] + self.traj_calc_debug.trajcalc_latpre_match_pointy)
        self.trajcalc_latpre_poserrcmd.append([current_time] + self.traj_calc_debug.trajcalc_latpre_poserrcmd)
        self.trajcalc_latpre_headingcmd.append([current_time] + self.traj_calc_debug.trajcalc_latpre_headingcmd)
        self.trajcalc_latpre_velcmd.append([current_time] + self.traj_calc_debug.trajcalc_latpre_velcmd)
        self.trajcalc_latpre_acc_cmd.append([current_time] + self.traj_calc_debug.trajcalc_latpre_acc_cmd)
        self.trajcalc_latpre_curvcmd.append([current_time] + self.traj_calc_debug.trajcalc_latpre_curvcmd)
        
        self.ctrldec_sysmode.append([current_time] + self.ctrl_dec_debug.ctrldec_sysmode)
        self.ctrldec_req_auto.append([current_time] + self.ctrl_dec_debug.ctrldec_req_auto)
        self.ctrldec_automode_is_estop.append([current_time] + self.ctrl_dec_debug.ctrldec_automode_is_estop)
        self.ctrldec_is_gear_change_req.append([current_time] + self.ctrl_dec_debug.ctrldec_is_gear_change_req)
        self.ctrldec_is_stop_steer_ctrl.append([current_time] + self.ctrl_dec_debug.ctrldec_is_stop_steer_ctrl)
        self.ctrldec_is_vehicle_standstill.append([current_time] + self.ctrl_dec_debug.ctrldec_is_vehicle_standstill)
        self.ctrldec_is_steer_set_ok.append([current_time] + self.ctrl_dec_debug.ctrldec_is_steer_set_ok)
        self.ctrldec_lat_sysmodecmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_sysmodecmd)
        self.ctrldec_lat_resetflag.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_resetflag)
        self.ctrldec_lat_sys_poserr.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_sys_poserr)
        self.ctrldec_lat_sys_yawff.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_sys_yawff)
        self.ctrldec_lat_sys_velff.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_sys_velff)
        self.ctrldec_lat_sys_curvff.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_sys_curvff)
        self.ctrldec_lat_api_poscmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_api_poscmd)
        self.ctrldec_lat_api_yawcmd.append([current_time] +self.ctrl_dec_debug.ctrldec_lat_api_yawcmd)
        self.ctrldec_lat_api_curvcmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_api_curvcmd)
        self.ctrldec_lat_api_steercmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lat_api_steercmd)
        self.ctrldec_lon_sysmodecmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sysmodecmd)
        self.ctrldec_lon_resetflag.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_resetflag)
        self.ctrldec_lon_sys_poserr.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_poserr)
        self.ctrldec_lon_sys_velff.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_velff)
        self.ctrldec_lon_sys_accff.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_accff)
        self.ctrldec_lon_sys_gearcmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_gearcmd)
        self.ctrldec_lon_sys_gear_ena.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_gear_ena)
        self.ctrldec_lon_sys_brk_emerg.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_sys_brk_emerg)
        self.ctrldec_lon_api_poscmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_api_poscmd)
        self.ctrldec_lon_api_velff.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_api_velff)
        self.ctrldec_lon_api_acc_cmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_api_acc_cmd)
        self.ctrldec_lon_api_thrcmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_api_thrcmd)
        self.ctrldec_lon_api_brkcmd.append([current_time] + self.ctrl_dec_debug.ctrldec_lon_api_brkcmd)
        self.ctrldec_ctrl_err.append([current_time] + self.ctrl_dec_debug.ctrldec_ctrl_err)


        self.lonctrl_modecmd.append([current_time] + self.lon_ctrl_debug.lonctrl_modecmd)
        self.lonctrl_resetflag.append([current_time] + self.lon_ctrl_debug.lonctrl_resetflag)
        self.lonctrl_sys_poserr.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_poserr)
        self.lonctrl_sys_velff.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_velff)
        self.lonctrl_sys_accff.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_accff)
        self.lonctrl_sys_gearcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_gearcmd)
        self.lonctrl_sys_gearena.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_gearena)
        self.lonctrl_sys_brkemerg.append([current_time] + self.lon_ctrl_debug.lonctrl_sys_brkemerg)
        self.lonctrl_api_poscmd.append([current_time] + self.lon_ctrl_debug.lonctrl_api_poscmd)
        self.lonctrl_api_velcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_api_velcmd)
        self.lonctrl_api_acccmd.append([current_time] + self.lon_ctrl_debug.lonctrl_api_acccmd)
        self.lonctrl_api_thrcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_api_thrcmd)
        self.lonctrl_api_brkcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_api_brkcmd)
        self.lonctrl_fdbk_vx.append([current_time] + self.lon_ctrl_debug.lonctrl_fdbk_vx)
        self.lonctrl_fdbk_ax.append([current_time] + self.lon_ctrl_debug.lonctrl_fdbk_ax)
        self.lonctrl_fdbk_pitch.append([current_time] + self.lon_ctrl_debug.lonctrl_fdbk_pitch)
        self.lonctrl_fdbk_gear.append([current_time] + self.lon_ctrl_debug.lonctrl_fdbk_gear)
        self.lonctrl_fdbk_reverse.append([current_time] + self.lon_ctrl_debug.lonctrl_fdbk_reverse)
        self.lonctrl_pos_dyn.append([current_time] + self.lon_ctrl_debug.lonctrl_pos_dyn)
        self.lonctrl_posctrl_p.append([current_time] + self.lon_ctrl_debug.lonctrl_posctrl_p)
        self.lonctrl_pos_poserr_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_pos_poserr_filter)
        self.lonctrl_pos_pi_velcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_pos_pi_velcmd)
        self.lonctrl_pos_output_velcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_pos_output_velcmd)
        self.lonctrl_vel_dyn.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_dyn)
        self.lonctrl_vel_velcmd_lmt.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_velcmd_lmt)
        self.lonctrl_vel_vel_err.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_vel_err)
        self.lonctrl_velctrl_p.append([current_time] + self.lon_ctrl_debug.lonctrl_velctrl_p)
        self.lonctrl_velctrl_i.append([current_time] + self.lon_ctrl_debug.lonctrl_velctrl_i)
        self.lonctrl_vel_pi_acc_cmd.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_pi_acc_cmd)
        self.lonctrl_vel_pi_acccmd_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_pi_acccmd_filter)
        self.lonctrl_vel_accpitch.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_accpitch)
        self.lonctrl_vel_accdamper.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_accdamper)
        self.lonctrl_vel_accff_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_accff_filter)
        self.lonctrl_vel_output_accCmd.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_output_accCmd)
        self.lonctrl_vel_output_accCmd_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_vel_output_accCmd_filter)
        self.lonctrl_thrust_thr_dyn.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_thr_dyn)
        self.lonctrl_thrust_thr_accerr.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_thr_accerr)
        self.lonctrl_thrust_brk_dyn.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_brk_dyn)
        self.lonctrl_thrust_brk_accerr.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_brk_accerr)
        self.lonctrl_thrust_fdbk_ax_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_fdbk_ax_filter)
        self.lonctrl_thrust_thr_acc_cmd_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_thr_acc_cmd_filter)
        self.lonctrl_thrust_brk_acc_cmd_filter.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_brk_acc_cmd_filter)
        self.lonctrl_thrustctrl_thr_p.append([current_time] + self.lon_ctrl_debug.lonctrl_thrustctrl_thr_p)
        self.lonctrl_thrustctrl_thr_i.append([current_time] + self.lon_ctrl_debug.lonctrl_thrustctrl_thr_i)
        self.lonctrl_thrustctrl_brk_p.append([current_time] + self.lon_ctrl_debug.lonctrl_thrustctrl_brk_p)
        self.lonctrl_thrustctrl_brk_i.append([current_time] + self.lon_ctrl_debug.lonctrl_thrustctrl_brk_i)
        self.lonctrl_thrust_pi_thr_acc_cmd.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_pi_thr_acc_cmd)
        self.lonctrl_thrust_pi_brk_acc_cmd.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_pi_brk_acc_cmd)
        self.lonctrl_thrust_acc_cmd_filter_lmt.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_acc_cmd_filter_lmt)
        self.lonctrl_thrust_acctothr_gain.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_acctothr_gain)
        self.lonctrl_thrust_throut_throcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_throut_throcmd)
        self.lonctrl_thrust_acctobrk_gain.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_acctobrk_gain)
        self.lonctrl_thrust_brkout_brkcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_thrust_brkout_brkcmd)
        self.lonctrl_analog_autput_throtcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_analog_autput_throtcmd)
        self.lonctrl_analog_autput_brkcmd.append([current_time] + self.lon_ctrl_debug.lonctrl_analog_autput_brkcmd)


        self.latctrl_modecmd.append([current_time] + self.lat_ctrl_debug.latctrl_modecmd)
        self.latctrl_resetflag.append([current_time] + self.lat_ctrl_debug.latctrl_resetflag)
        self.latctrl_sys_poserr.append([current_time] + self.lat_ctrl_debug.latctrl_sys_poserr)
        self.latctrl_sys_yawff.append([current_time] + self.lat_ctrl_debug.latctrl_sys_yawff)
        self.latctrl_sys_velff.append([current_time] + self.lat_ctrl_debug.latctrl_sys_velff)
        self.latctrl_sys_curvff.append([current_time] + self.lat_ctrl_debug.latctrl_sys_curvff)
        self.latctrl_api_poscmd.append([current_time] + self.lat_ctrl_debug.latctrl_api_yawcmd)
        self.latctrl_api_yawcmd.append([current_time] + self.lat_ctrl_debug.latctrl_api_yawcmd)
        self.latctrl_api_curvcmd.append([current_time] + self.lat_ctrl_debug.latctrl_api_curvcmd)
        self.latctrl_api_steercmd.append([current_time] + self.lat_ctrl_debug.latctrl_api_steercmd)
        self.latictrl_fdbk_vxb.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_vxb)
        self.latictrl_fdbk_heading.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_heading)
        self.latictrl_fdbk_yawrate.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_yawrate)
        self.latictrl_fdbk_steer.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_steer)
        self.latictrl_fdbk_gear.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_gear)
        self.latictrl_fdbk_rvsflag.append([current_time] + self.lat_ctrl_debug.latictrl_fdbk_rvsflag)
        self.latictrl_offset_dyn.append([current_time] + self.lat_ctrl_debug.latictrl_offset_dyn)
        self.latictrl_offsetctrl_i.append([current_time] + self.lat_ctrl_debug.latictrl_offsetctrl_i)
        self.latictrl_offsetctrl_p.append([current_time] + self.lat_ctrl_debug.latictrl_offsetctrl_p)
        self.latictrl_offset_offseterr.append([current_time] + self.lat_ctrl_debug.latictrl_offset_offseterr)
        self.latictrl_offset_pi_torscmd.append([current_time] + self.lat_ctrl_debug.latictrl_offset_pi_torscmd)
        self.latictrl_offset_torsrateffcmd.append([current_time] + self.lat_ctrl_debug.latictrl_offset_torsrateffcmd)
        self.latictrl_offset_output_yawcmd.append([current_time] + self.lat_ctrl_debug.latictrl_offset_output_yawcmd)
        self.latictrl_tors_dyn.append([current_time] + self.lat_ctrl_debug.latictrl_tors_dyn)
        self.latictrl_tors_yawerr.append([current_time] + self.lat_ctrl_debug.latictrl_tors_yawerr)
        self.latictrl_yawctrl_p.append([current_time] + self.lat_ctrl_debug.latictrl_yawctrl_p)
        self.latictrl_yawctrl_i.append([current_time] + self.lat_ctrl_debug.latictrl_yawctrl_i)
        self.latictrl_tors_pi_torsrate.append([current_time] + self.lat_ctrl_debug.latictrl_tors_pi_torsrate)
        self.latictrl_tors_pi_leadfilter_torsrate.append([current_time] + self.lat_ctrl_debug.latictrl_tors_pi_leadfilter_torsrate)
        self.latictrl_tors_torsrateff.append([current_time] + self.lat_ctrl_debug.latictrl_tors_torsrateff)
        self.latictrl_tors_output_yawratecmd.append([current_time] + self.lat_ctrl_debug.latictrl_tors_output_yawratecmd)
        self.latictrl_rate_dyn.append([current_time] + self.lat_ctrl_debug.latictrl_rate_dyn)
        self.latictrl_rate_p.append([current_time] + self.lat_ctrl_debug.latictrl_rate_p)
        self.latictrl_rate_i.append([current_time] + self.lat_ctrl_debug.latictrl_rate_i)
        self.latictrl_rate_yawratecmd_lmt.append([current_time] + self.lat_ctrl_debug.latictrl_rate_yawratecmd_lmt)
        self.latictrl_rate_filter_yawratecmd_lmt.append([current_time] + self.lat_ctrl_debug.latictrl_rate_filter_yawratecmd_lmt)
        self.latictrl_rate_pi_steer.append([current_time] + self.lat_ctrl_debug.latictrl_rate_pi_steer)
        self.latictrl_rate_pi_filter_steer.append([current_time] + self.lat_ctrl_debug.latictrl_rate_pi_filter_steer)
        self.latictrl_rate_steerff.append([current_time] + self.lat_ctrl_debug.latictrl_rate_steerff)
        self.latictrl_rate_output_front_steercmd.append([current_time] + self.lat_ctrl_debug.latictrl_rate_output_front_steercmd)
        self.latictrl_rate_output_sw_steercmd.append([current_time] + self.lat_ctrl_debug.latictrl_rate_output_sw_steercmd)
        self.latictrl_steer_steercmd_filter.append([current_time] + self.lat_ctrl_debug.latictrl_steer_steercmd_filter)
        self.latictrl_steer_max_steerrate_value.append([current_time] + self.lat_ctrl_debug.latictrl_steer_max_steerrate_value)
        self.latictrl_steer_steercmd_lmt_filter.append([current_time] + self.lat_ctrl_debug.latictrl_steer_steercmd_lmt_filter)
        self.latictrl_steer_output_steercmd.append([current_time] + self.lat_ctrl_debug.latictrl_steer_output_steercmd)
        self.latictrl_steer_steertorque_initial.append([current_time] + self.lat_ctrl_debug.latictrl_steer_steertorque_initial)
        self.latictrl_steer_steertorque_cmd.append([current_time] + self.lat_ctrl_debug.latictrl_steer_steertorque_cmd)
        self.latictrl_tors_pure_yawerr.append([current_time] + self.lat_ctrl_debug.latictrl_tors_pure_yawerr)
        self.latictrl_rate_output_front_steercmd_offset.append([current_time] + self.lat_ctrl_debug.latictrl_rate_output_front_steercmd_offset)
        # self.latictrl_steer_steer_err.append([current_time] + self.lat_ctrl_debug.latictrl_steer_steer_err)
        self.latictrl_yaw_curve_compsate.append([current_time] + self.lat_ctrl_debug.latictrl_yaw_curve_compsate)
        self.latictrl_rate_reference_model.append([current_time] + self.lat_ctrl_debug.latictrl_rate_reference_model)
        self.latictrl_rate_mrac_cmd.append([current_time] + self.lat_ctrl_debug.latictrl_rate_mrac_cmd)
        self.latictrl_rate_eso_cmd.append([current_time] + self.lat_ctrl_debug.latictrl_rate_eso_cmd)
        self.latictrl_rate_steer_offset.append([current_time] + self.lat_ctrl_debug.latictrl_rate_steer_offset)
        self.latictrl_rate_ramp_estimate.append([current_time] + self.lat_ctrl_debug.latictrl_rate_ramp_estimate)
        self.latictrl_error.append([current_time] + self.lat_ctrl_debug.latictrl_error)


        #print(" the value assign to brake is  :" , MbdDebugFromMCU.brake)
        #print(" update sccesfully, brake is  :" , self.brake)
        #header.update(self,MbdDebugFromMCU)
        self.pose_calc_debug.update(MbdDebugFromMCU)
        self.traj_calc_debug.update(MbdDebugFromMCU)
        self.lon_ctrl_debug.update(MbdDebugFromMCU)
        # self.ctrl_dec_debug.update(MbdDebugFromMCU.MbdDebugFromMCU)
        # self.lat_ctrl_debug.update(MbdDebugFromMCU.MbdDebugFromMCU)
        self.ctrl_output_debug.update(MbdDebugFromMCU)
        self.mbd_debug_dict = {'pose_calc_debug':self.pose_calc_debug.dict,\
                               'traj_calc_debug':self.traj_calc_debug.dict,\
                               'CtrlOutputDebug':self.ctrl_output_debug.dict}
        #print(" the self.mbd_debug_dict  :" ,self.mbd_debug_dict)
        # self.dict= {'header':self.header.dict,\
        #             'ctrl_req_mode':self.ctrl_req_mode,\
        #             'throttle':self.throttle,\
        #             'brake':self.brake,\
        #             'MbdDebugFromMCU':self.mbd_debug_dict}
        # self.dict= {
        #             'ctrl_req_mode':self.ctrl_req_mode,\
        #             'throttle':self.throttle,\
        #             'brake':self.brake,\
        #             'MbdDebugFromMCU':self.mbd_debug_dict}
        # print(" the control command is  :" ,self.dict)
    def get_dict(self):
        return {
                'time':self.time,
                'brake_cmd':self.brake_cmd,
                'throttle_cmd':self.throttle_cmd,
                'acc_cmd':self.acc_cmd,
                'gear_enable':self.gear_enable,
                'gear_cmd':self.gear_cmd,
                'emerg_enable':self.emerg_enable,
                'steer_cmd':self.steer_cmd,
                'steer_torque_cmd':self.steer_torque_cmd,
                'posedata_timestamp':self.posecalc_posedata_timestamp,
                'posecalc_valid':self.posecalc_inputdata_valid,
                'posedata_world_pos_x':self.posedata_world_pos_x,
                'posedata_world_pos_y':self.posedata_world_pos_y,
                'posedata_vrf_vel_x':self.posedata_vrf_vel_x,
                'posecalc_vrf_vel_y':self.posecalc_vrf_vel_y,
                'posecalc_world_vel_x':self.posecalc_world_vel_x,
                'posecalc_world_vel_y':self.posecalc_world_vel_y,
                'posecalc_world_acc_x':self.posecalc_world_acc_x,
                'posecalc_world_acc_y':self.posecalc_world_acc_y,
                'posecalc_world_heading':self.posecalc_world_heading,
                'posecalc_world_pitch':self.posecalc_world_pitch,
                'posecalc_timedelay':self.posecalc_timedelay,
                'posecalc_enable':self.posecalc_enable,
                'global_time_timestamp_sec':self.global_time_timestamp_sec,
                'global_time_timestamp_nec':self.global_time_timestamp_nec,
                'global_time_gettime_enable':self.global_time_gettime_enable,
                'replaning_flag':self.trajCalc_trajdata_replaning_flag,
                'trajdata_estop':self.trajcalc_trajdata_estop,
                'trajdata_gearcmd':self.trajcalc_trajdata_gearcmd,
                'tr_inputdata_valid':self.trajcalc_inputdata_valid,
                'trajdata_timestamp':self.trajcalc_trajdata_timestamp,
                'tra_globaltime_timestamp':self.trajcalc_globaltime_timestamp,
                'trajdata_pointtime_check':self.trajcalc_trajdata_pointtime_check,
                'trajdata_timecheck':self.trajcalc_trajdata_timecheck,
                'trajcalc_enable':self.trajcalc_enable,
                'tra_lon_startpoint_index':self.trajcalc_lon_startpoint_index,
                'tra_lon_startpoint_time':self.trajcalc_lon_startpoint_time,
                'trajcalc_lon_linear_ratio':self.trajcalc_lon_linear_ratio,
                'trajcalc_lon_match_pointx':self.trajcalc_lon_match_pointx,
                'trajcalc_lon_match_pointy':self.trajcalc_lon_match_pointy,
                'trajcalc_lon_poserrcmd':self.trajcalc_lon_poserrcmd,
                'trajcalc_lon_headingcmd':self.trajcalc_lon_headingcmd,
                'trajcalc_lon_velcmd':self.trajcalc_lon_velcmd,
                'trajcalc_lon_acc_cmd':self.trajcalc_lon_acc_cmd,
                'trajcalc_lon_curvcmd':self.trajcalc_lon_curvcmd,
                'tr_lonpre_startpoint_index':self.trajcalc_lonpre_startpoint_index,
                'tra_lonpre_startpoint_time':self.trajcalc_lonpre_startpoint_time,
                'tra_lonpre_linear_ratio':self.trajcalc_lonpre_linear_ratio,
                'tra_lonpre_match_pointx':self.trajcalc_lonpre_match_pointx,
                'tra_lonpre_match_pointy':self.trajcalc_lonpre_match_pointy,
                'tra_lonpre_poserrcmd':self.trajcalc_lonpre_poserrcmd,
                'tra_lonpre_headingcmd':self.trajcalc_lonpre_headingcmd,
                'tra_lonpre_velcmd':self.trajcalc_lonpre_velcmd,
                'tra_lonpre_acc_cmd':self.trajcalc_lonpre_acc_cmd,
                'tra_lonpre_curvrmd':self.trajcalc_lonpre_curvrmd,
                'trajcalc_posedata_posex':self.trajcalc_posedata_posex,
                'trajcalc_PoseData_posey':self.trajcalc_PoseData_posey,
                'tr_lat_Dot_fstpoint2veh':self.trajcalc_lat_Dot_fstpoint2veh,
                'tr_lat_Dot_endpoint2veh':self.trajcalc_lat_Dot_endpoint2veh,
                'tra_lat_matchpoint_vase':self.trajcalc_lat_matchpoint_vase,
                'tra_lat_startpoint_index':self.trajcalc_lat_startpoint_index,
                'trajcalc_lat_linear_ratio':self.trajcalc_lat_linear_ratio,
                'trajcalc_lat_match_pointx':self.trajcalc_lat_match_pointx,
                'trajcalc_lat_match_pointy':self.trajcalc_lat_match_pointy,
                'trajcalc_lat_poserrcmd':self.trajcalc_lat_poserrcmd,
                'trajcalc_lat_headingcmd':self.trajcalc_lat_headingcmd,
                'trajcalc_lat_velcmd':self.trajcalc_lat_velcmd,
                'trajcalc_lat_acc_cmd':self.trajcalc_lat_acc_cmd,
                'trajcalc_lat_curvcmd':self.trajcalc_lat_curvcmd,
                'trajcalc_posedata_preposex':self.trajcalc_posedata_preposex,
                'trajcalc_posedata_preposey':self.trajcalc_posedata_preposey,
                'tr_latpre_dot_fstpoint2veh':self.trajcalc_latpre_dot_fstpoint2veh,
                'tr_latpre_dot_endpoint2veh':self.trajcalc_latpre_dot_endpoint2veh,
                'tr_latpre_matchpoint_vase':self.trajcalc_latpre_matchpoint_vase,
                'tr_latpre_startpoint_index':self.trajcalc_latpre_startpoint_index,
                'tra_latpre_linear_ratio':self.trajcalc_latpre_linear_ratio,
                'tra_latpre_match_pointx':self.trajcalc_latpre_match_pointx,
                'tra_latpre_match_pointy':self.trajcalc_latpre_match_pointy,
                'tra_latpre_poserrcmd':self.trajcalc_latpre_poserrcmd,
                'trajdata_estop':self.trajcalc_latpre_headingcmd,
                'tra_latpre_velcmd':self.trajcalc_latpre_velcmd,
                'tra_latpre_acc_cmd':self.trajcalc_latpre_acc_cmd,
                'tra_latpre_curvcmd':self.trajcalc_latpre_curvcmd,
                'ctrldec_sysmode':self.ctrldec_sysmode,
                'ctrldec_req_auto':self.ctrldec_req_auto,
                'ctrldec_automode_is_estop':self.ctrldec_automode_is_estop,
                'ctrldec_is_gear_change_req':self.ctrldec_is_gear_change_req,
                'ctrldec_is_stop_steer_ctrl':self.ctrldec_is_stop_steer_ctrl,
                'ctrldec_is_vehicle_standstill':self.ctrldec_is_vehicle_standstill,
                'ctrldec_is_steer_set_ok':self.ctrldec_is_steer_set_ok,
                'ctrldec_lat_sysmodecmd':self.ctrldec_lat_sysmodecmd,
                'ctrldec_lat_resetflag':self.ctrldec_lat_resetflag,
                'ctrldec_lat_sys_poserr':self.ctrldec_lat_sys_poserr,
                'ctrldec_lat_sys_yawff':self.ctrldec_lat_sys_yawff,
                'ctrldec_lat_sys_velff':self.ctrldec_lat_sys_velff,
                'ctrldec_lat_sys_curvff':self.ctrldec_lat_sys_curvff,
                'ctrldec_lat_api_poscmd':self.ctrldec_lat_api_poscmd,
                'ctrldec_lat_api_yawcmd':self.ctrldec_lat_api_yawcmd,
                'ctrldec_lat_api_curvcmd':self.ctrldec_lat_api_curvcmd,
                'ctrldec_lat_api_steercmd':self.ctrldec_lat_api_steercmd,
                'ctrldec_lon_sysmodecmd':self.ctrldec_lon_sysmodecmd,
                'ctrldec_lon_resetflag':self.ctrldec_lon_resetflag,
                'ctrldec_lon_sys_poserr':self.ctrldec_lon_sys_poserr,
                'ctrldec_lon_sys_velff':self.ctrldec_lon_sys_velff,
                'ctrldec_lon_sys_accff':self.ctrldec_lon_sys_accff,
                'ctrldec_lon_sys_gearcmd':self.ctrldec_lon_sys_gearcmd,
                'ctrldec_lon_sys_gear_ena':self.ctrldec_lon_sys_gear_ena,
                'ctrldec_lon_sys_brk_emerg':self.ctrldec_lon_sys_brk_emerg,
                'ctrldec_lon_api_poscmd':self.ctrldec_lon_api_poscmd,
                'ctrldec_lon_api_velff':self.ctrldec_lon_api_velff,
                'ctrldec_lon_api_acc_cmd':self.ctrldec_lon_api_acc_cmd,
                'ctrldec_lon_api_thrcmd':self.ctrldec_lon_api_thrcmd,
                'ctrldec_lon_api_brkcmd':self.ctrldec_lon_api_brkcmd,
                'ctrldec_ctrl_err':self.ctrldec_ctrl_err,
                'lonctrl_modecmd':self.lonctrl_modecmd,
                'lonctrl_resetflag':self.lonctrl_resetflag,
                'lonctrl_sys_poserr':self.lonctrl_sys_poserr,
                'lonctrl_sys_velff':self.lonctrl_sys_velff,
                'lonctrl_sys_accff':self.lonctrl_sys_accff,
                'lonctrl_sys_gearcmd':self.lonctrl_sys_gearcmd,
                'lonctrl_sys_gearena':self.lonctrl_sys_gearena,
                'lonctrl_sys_brkemerg':self.lonctrl_sys_brkemerg,
                'lonctrl_api_poscmd':self.lonctrl_api_poscmd,
                'lonctrl_api_velcmd':self.lonctrl_api_velcmd,
                'lonctrl_api_acccmd':self.lonctrl_api_acccmd,
                'lonctrl_api_thrcmd':self.lonctrl_api_thrcmd,
                'lonctrl_api_brkcmd':self.lonctrl_api_brkcmd,
                'lonctrl_fdbk_vx':self.lonctrl_fdbk_vx,
                'lonctrl_fdbk_ax':self.lonctrl_fdbk_ax,
                'lonctrl_fdbk_pitch':self.lonctrl_fdbk_pitch,
                'lonctrl_fdbk_gear':self.lonctrl_fdbk_gear,
                'lonctrl_fdbk_reverse':self.lonctrl_fdbk_reverse,
                'lonctrl_pos_dyn':self.lonctrl_pos_dyn,
                'lonctrl_posctrl_p':self.lonctrl_posctrl_p,
                'lon_pos_poserr_filter':self.lonctrl_pos_poserr_filter,
                'lonctrl_pos_pi_velcmd':self.lonctrl_pos_pi_velcmd,
                'lon_pos_output_velcmd':self.lonctrl_pos_output_velcmd,
                'lonctrl_vel_dyn':self.lonctrl_vel_dyn,
                'lonctrl_vel_velcmd_lmt':self.lonctrl_vel_velcmd_lmt,
                'lonctrl_vel_vel_err':self.lonctrl_vel_vel_err,
                'lonctrl_velctrl_p':self.lonctrl_velctrl_p,
                'lonctrl_velctrl_i':self.lonctrl_velctrl_i,
                'lonctrl_vel_pi_acc_cmd':self.lonctrl_vel_pi_acc_cmd,
                'lon_vel_pi_acccmd_filter':self.lonctrl_vel_pi_acccmd_filter,
                'lonctrl_vel_accpitch':self.lonctrl_vel_accpitch,
                'lonctrl_vel_accdamper':self.lonctrl_vel_accdamper,
                'lonctrl_vel_accff_filter':self.lonctrl_vel_accff_filter,
                'lonctrl_vel_output_accCmd':self.lonctrl_vel_output_accCmd,
                'lon_vel_output_accCmd_filter':self.lonctrl_vel_output_accCmd_filter,
                'lonctrl_thrust_thr_dyn':self.lonctrl_thrust_thr_dyn,
                'lonctrl_thrust_thr_accerr':self.lonctrl_thrust_thr_accerr,
                'lonctrl_thrust_brk_dyn':self.lonctrl_thrust_brk_dyn,
                'lonctrl_thrust_brk_accerr':self.lonctrl_thrust_brk_accerr,
                'lon_thrust_fdbk_ax_filter':self.lonctrl_thrust_fdbk_ax_filter,
                'lon_thrust_thr_acc_cmd_filter':self.lonctrl_thrust_thr_acc_cmd_filter,
                'lon_thrust_brk_acc_cmd_filter':self.lonctrl_thrust_brk_acc_cmd_filter,
                'lonctrl_thrustctrl_thr_p':self.lonctrl_thrustctrl_thr_p,
                'lonctrl_thrustctrl_thr_i':self.lonctrl_thrustctrl_thr_i,
                'lonctrl_thrustctrl_brk_p':self.lonctrl_thrustctrl_brk_p,
                'lonctrl_thrustctrl_brk_i':self.lonctrl_thrustctrl_brk_i,
                'lon_thrust_pi_thr_acc_cmd':self.lonctrl_thrust_pi_thr_acc_cmd,
                'lon_thrust_pi_brk_acc_cmd':self.lonctrl_thrust_pi_brk_acc_cmd,
                'lon_thrust_acc_cmd_filter_lmt':self.lonctrl_thrust_acc_cmd_filter_lmt,
                'lon_thrust_acctothr_gain':self.lonctrl_thrust_acctothr_gain,
                'lon_thrust_throut_throcmd':self.lonctrl_thrust_throut_throcmd,
                'lon_thrust_acctobrk_gain':self.lonctrl_thrust_acctobrk_gain,
                'lon_thrust_brkout_brkcmd':self.lonctrl_thrust_brkout_brkcmd,
                'lonctrl_analog_autput_throtcmd':self.lonctrl_analog_autput_throtcmd,
                'lonctrl_analog_autput_brkcmd':self.lonctrl_analog_autput_brkcmd,
                'latctrl_modecmd':self.latctrl_modecmd,
                'latctrl_resetflag':self.latctrl_resetflag,
                'latctrl_sys_poserr':self.latctrl_sys_poserr,
                'latctrl_sys_yawff':self.latctrl_sys_yawff,
                'latctrl_sys_velff':self.latctrl_sys_velff,
                'latctrl_sys_curvff':self.latctrl_sys_curvff,
                'latctrl_api_poscmd':self.latctrl_api_poscmd,
                'latctrl_api_yawcmd':self.latctrl_api_yawcmd,
                'latctrl_api_curvcmd':self.latctrl_api_curvcmd,
                'latctrl_api_steercmd':self.latctrl_api_steercmd,
                'latictrl_fdbk_vxb':self.latictrl_fdbk_vxb,
                'latictrl_fdbk_yaw':self.latictrl_fdbk_heading,
                'latictrl_fdbk_yawrate':self.latictrl_fdbk_yawrate,
                'latictrl_fdbk_steer':self.latictrl_fdbk_steer,
                'latictrl_fdbk_gear':self.latictrl_fdbk_gear,
                'latictrl_fdbk_rvsflag':self.latictrl_fdbk_rvsflag,
                'latictrl_offset_dyn':self.latictrl_offset_dyn,
                'latictrl_offsetctrl_i':self.latictrl_offsetctrl_i,
                'latictrl_offsetctrl_p':self.latictrl_offsetctrl_p,
                'latictrl_offset_offseterr':self.latictrl_offset_offseterr,
                'latictrl_offset_pi_torscmd':self.latictrl_offset_pi_torscmd,
                'latictrl_offset_torsrateffcmd':self.latictrl_offset_torsrateffcmd,
                'latictrl_offset_output_yawcmd':self.latictrl_offset_output_yawcmd,
                'latictrl_tors_dyn':self.latictrl_tors_dyn,
                'latictrl_tors_yawerr':self.latictrl_tors_yawerr,
                'latictrl_yawctrl_p':self.latictrl_yawctrl_p,
                'latictrl_yawctrl_i':self.latictrl_yawctrl_i,
                'latictrl_tors_pi_torsrate':self.latictrl_tors_pi_torsrate,
                'lat_tors_pi_leadfilter_torsrate':self.latictrl_tors_pi_leadfilter_torsrate,
                'latictrl_tors_torsrateff':self.latictrl_tors_torsrateff,
                'lat_tors_output_yawratecmd':self.latictrl_tors_output_yawratecmd,
                'latictrl_rate_dyn':self.latictrl_rate_dyn,
                'latictrl_rate_p':self.latictrl_rate_p,
                'latictrl_rate_i':self.latictrl_rate_i,
                'lat_rate_yawratecmd_lmt':self.latictrl_rate_yawratecmd_lmt,
                'lat_rate_yawratecmd_lmt_filter':self.latictrl_rate_filter_yawratecmd_lmt,
                'lat_rate_pi_steer':self.latictrl_rate_pi_steer,
                'latictrl_rate_pi_filter_steer':self.latictrl_rate_pi_filter_steer,
                'latictrl_rate_steerff':self.latictrl_rate_steerff,
                'lat_rate_output_front_steercmd':self.latictrl_rate_output_front_steercmd,
                'lat_rate_output_sw_steercmd':self.latictrl_rate_output_sw_steercmd,
                'latictrl_steer_steercmd_filter':self.latictrl_steer_steercmd_filter,
                'lat_steer_max_steerrate_value':self.latictrl_steer_max_steerrate_value,
                'lat_steer_steercmd_lmt_filter':self.latictrl_steer_steercmd_lmt_filter,
                'lat_steer_output_steercmd':self.latictrl_steer_output_steercmd,
                'lat_steer_steertorque_initial':self.latictrl_steer_steertorque_initial,
                'lat_steer_steertorque_cmd':self.latictrl_steer_steertorque_cmd,
                'latictrl_tors_pure_yawerr':self.latictrl_tors_pure_yawerr,
                'lat_output_front_steeroffset':self.latictrl_rate_output_front_steercmd_offset,
                # 'latictrl_steer_steer_err':self.latictrl_steer_steer_err,
                'latictrl_yaw_curve_compsate':self.latictrl_yaw_curve_compsate,
                'latictrl_rate_reference_model':self.latictrl_rate_reference_model,
                'latictrl_rate_mrac_cmd':self.latictrl_rate_mrac_cmd,
                'latictrl_rate_eso_cmd':self.latictrl_rate_eso_cmd,
                'latictrl_rate_steer_offset':self.latictrl_rate_steer_offset,
                'latictrl_rate_ramp_estimate':self.latictrl_rate_ramp_estimate,
                'latictrl_error':self.latictrl_error}
    

class pose_calc_debug:
    dict = {}
    def __init__(self):
        self.name = 'pose_calc_debug'     # 名称
        
        self.posecalc_posedata_timestamp = []
        self.posecalc_inputdata_valid = []
        self.posedata_world_pos_x = []
        self.posedata_world_pos_y = []
        self.posecalc_world_vel_x = []
        self.posecalc_world_vel_y = []
        self.posedata_vrf_vel_x = []
        self.posecalc_vrf_vel_y = []
        self.posecalc_world_acc_x = []
        self.posecalc_world_acc_y = []
        self.posecalc_world_heading = []
        self.posecalc_world_pitch = []
        self.posecalc_timedelay = []
        self.posecalc_enable = []
        self.global_time_timestamp_sec = []
        self.global_time_timestamp_nec = []
        self.global_time_gettime_enable = []
        #print(" pose_calc_debug init succesfully ")
    
    def update(self,MbdDebugFromMCU):
        self.posecalc_posedata_timestamp.append(MbdDebugFromMCU.pose_calc_debug.posecalc_posedata_timestamp)
        self.posecalc_inputdata_valid.append(MbdDebugFromMCU.pose_calc_debug.posecalc_inputdata_valid)
        self.posedata_world_pos_x.append(MbdDebugFromMCU.pose_calc_debug.posedata_world_pos_x)
        self.posedata_world_pos_y.append(MbdDebugFromMCU.pose_calc_debug.posedata_world_pos_y)
        self.posedata_vrf_vel_x.append(MbdDebugFromMCU.pose_calc_debug.posedata_vrf_vel_x)
        self.posecalc_vrf_vel_y.append(MbdDebugFromMCU.pose_calc_debug.posecalc_vrf_vel_y)
        self.posecalc_world_vel_x.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_vel_x)
        self.posecalc_world_vel_y.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_vel_y)
        self.posecalc_world_acc_x.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_acc_x)
        self.posecalc_world_acc_y.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_acc_y)
        self.posecalc_world_heading.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_heading)
        self.posecalc_world_pitch.append(MbdDebugFromMCU.pose_calc_debug.posecalc_world_pitch)
        self.posecalc_timedelay.append(MbdDebugFromMCU.pose_calc_debug.posecalc_timedelay)
        self.posecalc_enable.append(MbdDebugFromMCU.pose_calc_debug.posecalc_enable)
        # self.global_time_timestamp_sec.append(MbdDebugFromMCU.pose_calc_debug.global_time_timestamp_sec)
        # self.global_time_timestamp_nec.append(MbdDebugFromMCU.pose_calc_debug.global_time_timestamp_nec)
        # self.global_time_gettime_enable.append(MbdDebugFromMCU.pose_calc_debug.global_time_gettime_enable)

        self.dict = {
                'posedata_timestamp':self.posecalc_posedata_timestamp,
                'inputdata_valid':self.posecalc_inputdata_valid,
                'world_pos_x':self.posedata_world_pos_x,
                'world_pos_y':self.posedata_world_pos_y,
                'vrf_vel_x':self.posedata_vrf_vel_x,
                'vrf_vel_y:':self.posecalc_vrf_vel_y,
                'world_vel_x':self.posecalc_world_vel_x,
                'world_vel_y':self.posecalc_world_vel_y,
                'world_acc_x':self.posecalc_world_acc_x,
                'world_acc_y':self.posecalc_world_acc_y,
                'global_time_timestamp_sec':self.global_time_timestamp_sec,
                'global_time_timestamp_nec':self.global_time_timestamp_nec,
                'global_time_gettime_enable':self.global_time_gettime_enable}
        
        #print(" the value assign to pose_calc_debug is  :" ,self.list)
    def clear(self):
          self.posecalc_posedata_timestamp.clear()
          self.posecalc_inputdata_valid.clear()
          self.posedata_world_pos_x.clear()
          self.posedata_world_pos_y.clear()
          self.posedata_vrf_vel_x.clear()
          self.posecalc_vrf_vel_y.clear()
          self.posecalc_world_vel_x.clear()
          self.posecalc_world_vel_y.clear()
          self.posecalc_world_acc_x.clear()
          self.posecalc_world_acc_y.clear()
          self.posecalc_world_heading.clear()
          self.posecalc_world_pitch.clear()
          self.posecalc_timedelay.clear()
          self.posecalc_enable.clear()
          self.global_time_timestamp_sec.clear()
          self.global_time_timestamp_nec.clear()
          self.global_time_gettime_enable.clear()




class traj_calc_debug:
    dict = {}
    def __init__(self):
        self.name = 'traj_calc_debug'     # 名称
        
        self.trajCalc_trajdata_replaning_flag = []
        self.trajcalc_trajdata_estop = []
        self.trajcalc_trajdata_gearcmd = []
        self.trajcalc_inputdata_valid = []
        self.trajcalc_trajdata_timestamp = []
        self.trajcalc_globaltime_timestamp = []
        self.trajcalc_trajdata_pointtime_check = []
        self.trajcalc_trajdata_timecheck = []
        self.trajcalc_enable = []
        self.trajcalc_lon_startpoint_index = []
        self.trajcalc_lon_startpoint_time = []
        self.trajcalc_lon_linear_ratio = []
        self.trajcalc_lon_match_pointx = []
        self.trajcalc_lon_match_pointy = []
        self.trajcalc_lon_poserrcmd = []
        self.trajcalc_lon_headingcmd = []
        self.trajcalc_lon_velcmd = []
        self.trajcalc_lon_acc_cmd = []
        self.trajcalc_lon_curvcmd = []
        self.trajcalc_lonpre_startpoint_index = []
        self.trajcalc_lonpre_startpoint_time = []
        self.trajcalc_lonpre_linear_ratio = []
        self.trajcalc_lonpre_match_pointx = []
        self.trajcalc_lonpre_match_pointy = []
        self.trajcalc_lonpre_poserrcmd = []
        self.trajcalc_lonpre_headingcmd = []
        self.trajcalc_lon_velcmd = []
        self.trajcalc_lon_acc_cmd = []
        self.trajcalc_lon_curvcmd = []
        self.trajcalc_lonpre_startpoint_index = []
        self.trajcalc_lonpre_startpoint_time = []
        self.trajcalc_lonpre_linear_ratio = []
        self.trajcalc_lonpre_match_pointx = []
        self.trajcalc_lonpre_match_pointy = []
        self.trajcalc_lonpre_poserrcmd = []
        self.trajcalc_lonpre_headingcmd = []
        self.trajcalc_lonpre_velcmd = []
        self.trajcalc_lonpre_acc_cmd = []
        self.trajcalc_lonpre_curvrmd = []
        self.trajcalc_posedata_posex = []
        self.trajcalc_PoseData_posey = []
        self.trajcalc_lat_Dot_fstpoint2veh = []
        self.trajcalc_lat_Dot_endpoint2veh = []
        self.trajcalc_lat_matchpoint_vase = []
        self.trajcalc_lat_startpoint_index = []
        self.trajcalc_lat_linear_ratio = []
        self.trajcalc_lat_match_pointx = []
        self.trajcalc_lat_match_pointy = []
        self.trajcalc_lat_poserrcmd = []
        self.trajcalc_lat_headingcmd = []
        self.trajcalc_lat_velcmd = []
        self.trajcalc_lat_acc_cmd = []
        self.trajcalc_lat_curvcmd = []
        self.trajcalc_posedata_preposex = []
        self.trajcalc_posedata_preposey = []
        self.trajcalc_latpre_dot_fstpoint2veh = []
        self.trajcalc_latpre_dot_endpoint2veh = []
        self.trajcalc_latpre_matchpoint_vase = []
        self.trajcalc_latpre_startpoint_index = []
        self.trajcalc_latpre_linear_ratio = []
        self.trajcalc_latpre_match_pointx = []
        self.trajcalc_latpre_match_pointy = []
        self.trajcalc_latpre_poserrcmd = []
        self.trajcalc_latpre_headingcmd = []
        self.trajcalc_latpre_velcmd = []
        self.trajcalc_latpre_acc_cmd = []
        self.trajcalc_latpre_curvcmd = []
    
        #print(" traj_calc_debug init succesfully ")
    
    def update(self,MbdDebugFromMCU):
        self.trajCalc_trajdata_replaning_flag.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_replaning_flag)
        self.trajcalc_trajdata_estop.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_estop)
        self.trajcalc_trajdata_gearcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_gearcmd)
        self.trajcalc_inputdata_valid.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_inputdata_valid)
        self.trajcalc_trajdata_timestamp.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_timestamp)
        self.trajcalc_globaltime_timestamp.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_globaltime_timestamp)
        self.trajcalc_trajdata_pointtime_check.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_pointtime_check)
        self.trajcalc_trajdata_timecheck.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_trajdata_timecheck)
        self.trajcalc_enable.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_enable)
        self.trajcalc_lon_startpoint_index.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_startpoint_index)
        # self.trajcalc_lon_startpoint_time.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_startpoint_time)
        self.trajcalc_lon_linear_ratio.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_linear_ratio)
        # self.trajcalc_lon_match_pointx.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_match_pointx)
        # self.trajcalc_lon_match_pointy.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_match_pointy)
        self.trajcalc_lon_poserrcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_poserrcmd)
        self.trajcalc_lon_headingcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_headingcmd)
        self.trajcalc_lon_velcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_velcmd)
        self.trajcalc_lon_acc_cmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_acc_cmd)
        self.trajcalc_lon_curvcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lon_curvcmd)
        self.trajcalc_lonpre_startpoint_index.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_startpoint_index)
        # self.trajcalc_lonpre_startpoint_time.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_startpoint_time)
        self.trajcalc_lonpre_linear_ratio.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_linear_ratio)
        # self.trajcalc_lonpre_match_pointx.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_match_pointx)
        # self.trajcalc_lonpre_match_pointy.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_match_pointy)
        self.trajcalc_lonpre_poserrcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_poserrcmd)
        self.trajcalc_lonpre_headingcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_headingcmd)
        self.trajcalc_lonpre_velcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_velcmd)
        self.trajcalc_lonpre_acc_cmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_acc_cmd)
        self.trajcalc_lonpre_curvrmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lonpre_curvrmd)
        self.trajcalc_posedata_posex.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_posedata_posex)
        self.trajcalc_PoseData_posey.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_posedata_posey)
        # self.trajcalc_lat_Dot_fstpoint2veh.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_Dot_fstpoint2veh)
        # self.trajcalc_lat_Dot_endpoint2veh.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_Dot_endpoint2veh)
        # self.trajcalc_lat_matchpoint_vase.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_matchpoint_vase)
        self.trajcalc_lat_startpoint_index.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_startpoint_index)
        self.trajcalc_lat_linear_ratio.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_linear_ratio)
        self.trajcalc_lat_match_pointx.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_match_pointx)
        self.trajcalc_lat_match_pointy.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_match_pointy)
        self.trajcalc_lat_poserrcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_poserrcmd)
        self.trajcalc_lat_headingcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_headingcmd)
        self.trajcalc_lat_velcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_velcmd)
        self.trajcalc_lat_acc_cmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_acc_cmd)
        self.trajcalc_lat_curvcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_lat_curvcmd)
        self.trajcalc_posedata_preposex.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_posedata_preposex)
        self.trajcalc_posedata_preposey.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_posedata_preposey)
        # self.trajcalc_latpre_dot_fstpoint2veh.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_dot_fstpoint2veh)
        # self.trajcalc_latpre_dot_endpoint2veh.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_dot_endpoint2veh)
        # self.trajcalc_latpre_matchpoint_vase.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_matchpoint_vase)
        self.trajcalc_latpre_startpoint_index.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_startpoint_index)
        self.trajcalc_latpre_linear_ratio.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_linear_ratio)
        self.trajcalc_latpre_match_pointx.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_match_pointx)
        self.trajcalc_latpre_match_pointy.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_match_pointy)
        self.trajcalc_latpre_poserrcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_poserrcmd)
        self.trajcalc_latpre_headingcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_headingcmd)
        self.trajcalc_latpre_velcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_velcmd)
        self.trajcalc_latpre_acc_cmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_acc_cmd)
        self.trajcalc_latpre_curvcmd.append(MbdDebugFromMCU.traj_calc_debug.trajcalc_latpre_curvcmd)

        #print(" the value assign to traj_calc_debug is  :" ,self.trajCalc_trajdata_replaning_flag)
        self.dict = {'replaning_flag':self.trajCalc_trajdata_replaning_flag,
                                'trajdata_estop':self.trajcalc_trajdata_estop,
                                'gearcmd':self.trajcalc_trajdata_gearcmd,
                                'trajcalc_valid':self.trajcalc_inputdata_valid,
                                'trajdata_timestamp':self.trajcalc_trajdata_timestamp,
                                'globaltime_timestamp':self.trajcalc_globaltime_timestamp,
                                'pointtime_check':self.trajcalc_trajdata_pointtime_check,
                                'trajdata_timecheck':self.trajcalc_trajdata_timecheck,
                                'trajcalc_enable':self.trajcalc_enable,
                                'lon_startpoint_index':self.trajcalc_lon_startpoint_index,
                                'lon_startpoint_time':self.trajcalc_lon_startpoint_time,
                                'lon_linear_ratio':self.trajcalc_lon_linear_ratio,
                                'lon_match_pointx':self.trajcalc_lon_match_pointx,
                                'lon_match_pointy':self.trajcalc_lon_match_pointy,
                                'tr_lon_poserrcmd':self.trajcalc_lon_poserrcmd,
                                'tr_lon_headingcmd':self.trajcalc_lon_headingcmd,
                                'tr_lon_velcmd':self.trajcalc_lon_velcmd,
                                'tr_lon_acc_cmd':self.trajcalc_lon_acc_cmd,
                                'tr_lon_curvcmd':self.trajcalc_lon_curvcmd,
                                'tr_lonpre_start_index':self.trajcalc_lonpre_startpoint_index,
                                'tr_lonpre_start_time':self.trajcalc_lonpre_startpoint_time,
                                'tr_lonpre_linearratio':self.trajcalc_lonpre_linear_ratio,
                                'tr_lonpre_match_pointx':self.trajcalc_lonpre_match_pointx,
                                'tr_lonpre_match_pointy':self.trajcalc_lonpre_match_pointy,
                                'tr_lonpre_poserrcmd':self.trajcalc_lonpre_poserrcmd,
                                'tr_lonpre_headingcmd':self.trajcalc_lonpre_headingcmd,
                                'tr_lonpre_velcmd':self.trajcalc_lonpre_velcmd,
                                'tr_lonpre_acc_cmd':self.trajcalc_lonpre_acc_cmd,
                                'tr_lonpre_curvrmd':self.trajcalc_lonpre_curvrmd,
                                'tr_posedata_posex':self.trajcalc_posedata_posex,
                                'tr_PoseData_posey':self.trajcalc_PoseData_posey,
                                'tr_lat_Dot_fstpoint2veh':self.trajcalc_lat_Dot_fstpoint2veh,
                                'tr_lat_Dot_endpoint2veh':self.trajcalc_lat_Dot_endpoint2veh,
                                'tr_lat_matchpoint_vase':self.trajcalc_lat_matchpoint_vase,
                                'tr_lat_startpoint_index':self.trajcalc_lat_startpoint_index,
                                'tr_lat_linear_ratio':self.trajcalc_lat_linear_ratio,
                                'tr_lat_match_pointx':self.trajcalc_lat_match_pointx,
                                'tr_lat_match_pointy':self.trajcalc_lat_match_pointy,
                                'tr_lat_poserrcmd':self.trajcalc_lat_poserrcmd,
                                'tr_lat_headingcmd':self.trajcalc_lat_headingcmd,
                                'tr_lat_velcmd':self.trajcalc_lat_velcmd,
                                'tr_lat_acc_cmd':self.trajcalc_lat_acc_cmd,
                                'tr_lat_curvcmd':self.trajcalc_lat_curvcmd,
                                'tr_posedata_preposex':self.trajcalc_posedata_preposex,
                                'tr_posedata_preposey':self.trajcalc_posedata_preposey,
                                'tra_latpre_dot_fstpoint2veh':self.trajcalc_latpre_dot_fstpoint2veh,
                                'tr_latpre_dot_endpoint2veh':self.trajcalc_latpre_dot_endpoint2veh,
                                'tr_latpre_matchpoint_vase':self.trajcalc_latpre_matchpoint_vase,
                                'tr_latpre_startpoint_index':self.trajcalc_latpre_startpoint_index,
                                'tr_latpre_linear_ratio':self.trajcalc_latpre_linear_ratio,
                                'tr_latpre_match_pointx':self.trajcalc_latpre_match_pointx,
                                'tr_latpre_match_pointy':self.trajcalc_latpre_match_pointy,
                                'tr_latpre_poserrcmd':self.trajcalc_latpre_poserrcmd,
                                'tr_latpre_headingcmd':self.trajcalc_latpre_headingcmd,
                                'tr_latpre_velcmd':self.trajcalc_latpre_velcmd,
                                'tr_latpre_acc_cmd':self.trajcalc_latpre_acc_cmd,
                                'tr_latpre_curvcmd':self.trajcalc_latpre_curvcmd}
    def clear(self):
          self.trajCalc_trajdata_replaning_flag.clear()
          self.trajcalc_trajdata_estop.clear()
          self.trajcalc_trajdata_gearcmd.clear()
          self.trajcalc_inputdata_valid.clear()
          self.trajcalc_trajdata_timestamp.clear()
          self.trajcalc_globaltime_timestamp.clear()
          self.trajcalc_trajdata_pointtime_check.clear()
          self.trajcalc_trajdata_timecheck.clear()
          self.trajcalc_enable.clear()
          self.trajcalc_lon_startpoint_index.clear()
          self.trajcalc_lon_startpoint_time.clear()
          self.trajcalc_lon_linear_ratio.clear()
          self.trajcalc_lon_match_pointx.clear()
          self.trajcalc_lon_match_pointy.clear()
          self.trajcalc_lon_poserrcmd.clear()
          self.trajcalc_lon_headingcmd.clear()
          self.trajcalc_lon_velcmd.clear()
          self.trajcalc_lon_acc_cmd.clear()
          self.trajcalc_lon_curvcmd.clear()
          self.trajcalc_lonpre_startpoint_index.clear()
          self.trajcalc_lonpre_startpoint_time.clear()
          self.trajcalc_lonpre_linear_ratio.clear()
          self.trajcalc_lonpre_match_pointx.clear()
          self.trajcalc_lonpre_match_pointy.clear()
          self.trajcalc_lonpre_poserrcmd.clear()
          self.trajcalc_lonpre_headingcmd.clear()
          self.trajcalc_lonpre_velcmd.clear()
          self.trajcalc_lonpre_acc_cmd.clear()
          self.trajcalc_lonpre_curvrmd.clear()
          self.trajcalc_posedata_posex.clear()
          self.trajcalc_PoseData_posey.clear()
          self.trajcalc_lat_Dot_fstpoint2veh.clear()
          self.trajcalc_lat_Dot_endpoint2veh.clear()
          self.trajcalc_lat_matchpoint_vase.clear()
          self.trajcalc_lat_startpoint_index.clear()
          self.trajcalc_lat_linear_ratio.clear()
          self.trajcalc_lat_match_pointx.clear()
          self.trajcalc_lat_match_pointy.clear()
          self.trajcalc_lat_poserrcmd.clear()
          self.trajcalc_lat_headingcmd.clear()
          self.trajcalc_lat_velcmd.clear()
          self.trajcalc_lat_acc_cmd.clear()
          self.trajcalc_lat_curvcmd.clear()
          self.trajcalc_posedata_preposex.clear()
          self.trajcalc_posedata_preposey.clear()
          self.trajcalc_latpre_dot_fstpoint2veh.clear()
          self.trajcalc_latpre_dot_endpoint2veh.clear()
          self.trajcalc_latpre_matchpoint_vase.clear()
          self.trajcalc_latpre_startpoint_index.clear()
          self.trajcalc_latpre_linear_ratio.clear()
          self.trajcalc_latpre_match_pointx.clear()
          self.trajcalc_latpre_match_pointy.clear()
          self.trajcalc_latpre_poserrcmd.clear()
          self.trajcalc_latpre_headingcmd.clear()
          self.trajcalc_latpre_velcmd.clear()
          self.trajcalc_latpre_acc_cmd.clear()
          self.trajcalc_latpre_curvcmd.clear()

class ctrl_dec_debug:
    dict = {}
    def __init__(self):
        self.name = 'ctrl_dec_debug'     # 名称
        self.ctrldec_sysmode = []
        self.ctrldec_req_auto = []
        self.ctrldec_automode_is_estop = []
        self.ctrldec_is_gear_change_req = []
        self.ctrldec_is_stop_steer_ctrl = []
        self.ctrldec_is_vehicle_standstill = []
        self.ctrldec_is_steer_set_ok = []
        self.ctrldec_lat_sysmodecmd = []
        self.ctrldec_lat_resetflag = []
        self.ctrldec_lat_sys_poserr = []
        self.ctrldec_lat_sys_yawff = []
        self.ctrldec_lat_sys_velff = []
        self.ctrldec_lat_sys_curvff = []
        self.ctrldec_lat_api_poscmd = []
        self.ctrldec_lat_api_yawcmd = []
        self.ctrldec_lat_api_curvcmd = []
        self.ctrldec_lat_api_steercmd = []
        self.ctrldec_lon_sysmodecmd = []
        self.ctrldec_lon_resetflag = []
        self.ctrldec_lon_sys_poserr = []
        self.ctrldec_lon_sys_velff = []
        self.ctrldec_lon_sys_accff = []
        self.ctrldec_lon_sys_gearcmd = []
        self.ctrldec_lon_sys_gear_ena = []
        self.ctrldec_lon_sys_brk_emerg = []
        self.ctrldec_lon_api_poscmd = []
        self.ctrldec_lon_api_velff = []
        self.ctrldec_lon_api_acc_cmd = []
        self.ctrldec_lon_api_thrcmd = []
        self.ctrldec_lon_api_brkcmd = []
        self.ctrldec_ctrl_err = []

    def update(self,MbdDebugFromMCU):
        self.ctrldec_sysmode.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_sysmode)
        self.ctrldec_req_auto.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_req_auto)
        self.ctrldec_automode_is_estop.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_automode_is_estop)
        self.ctrldec_is_gear_change_req.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_is_gear_change_req)
        self.ctrldec_is_stop_steer_ctrl.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_is_stop_steer_ctrl)
        self.ctrldec_is_vehicle_standstill.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_is_vehicle_standstill)
        self.ctrldec_is_steer_set_ok.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_is_steer_set_ok)
        self.ctrldec_lat_sysmodecmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_sysmodecmd)
        self.ctrldec_lat_resetflag.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_resetflag)
        self.ctrldec_lat_sys_poserr.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_sys_poserr)
        self.ctrldec_lat_sys_yawff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_sys_yawff)
        self.ctrldec_lat_sys_velff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_sys_velff)
        self.ctrldec_lat_sys_curvff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_sys_curvff)
        self.ctrldec_lat_api_poscmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_api_poscmd)
        self.ctrldec_lat_api_yawcmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_api_yawcmd)
        self.ctrldec_lat_api_curvcmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_api_curvcmd)
        self.ctrldec_lat_api_steercmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lat_api_steercmd)
        self.ctrldec_lon_sysmodecmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sysmodecmd)
        self.ctrldec_lon_resetflag.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_resetflag)
        self.ctrldec_lon_sys_poserr.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_poserr)
        self.ctrldec_lon_sys_velff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_velff)
        self.ctrldec_lon_sys_accff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_accff)
        self.ctrldec_lon_sys_gearcmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_gearcmd)
        self.ctrldec_lon_sys_gear_ena.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_gear_ena)
        self.ctrldec_lon_sys_brk_emerg.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_sys_brk_emerg)
        self.ctrldec_lon_api_poscmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_api_poscmd)
        self.ctrldec_lon_api_velff.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_api_velff)
        self.ctrldec_lon_api_acc_cmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_api_acc_cmd)
        self.ctrldec_lon_api_thrcmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_api_thrcmd)
        self.ctrldec_lon_api_brkcmd.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_lon_api_brkcmd)
        self.ctrldec_ctrl_err.append(MbdDebugFromMCU.ctrl_dec_debug.ctrldec_ctrl_err)
        
        self.dict = {
                'ctrldec_sysmode':self.ctrldec_sysmode,
                'ctrldec_req_auto':self.ctrldec_req_auto,
                'ctrldec_automode_is_estop':self.ctrldec_automode_is_estop,
                'ctrldec_is_gear_change_req':self.ctrldec_is_gear_change_req,
                'ctrldec_is_stop_steer_ctrl':self.ctrldec_is_stop_steer_ctrl,
                'ctrldec_is_vehicle_standstill':self.ctrldec_is_vehicle_standstill,
                'ctrldec_is_steer_set_ok':self.ctrldec_is_steer_set_ok,
                'ctrldec_lat_sysmodecmd':self.ctrldec_lat_sysmodecmd,
                'ctrldec_lat_resetflag':self.ctrldec_lat_resetflag,
                'ctrldec_lat_sys_poserr':self.ctrldec_lat_sys_poserr,
                'ctrldec_lat_sys_yawff':self.ctrldec_lat_sys_yawff,
                'ctrldec_lat_sys_velff':self.ctrldec_lat_sys_velff,
                'ctrldec_lat_sys_curvff':self.ctrldec_lat_sys_curvff,
                'ctrldec_lat_api_poscmd':self.ctrldec_lat_api_poscmd,
                'ctrldec_lat_api_yawcmd':self.ctrldec_lat_api_yawcmd,
                'ctrldec_lat_api_curvcmd':self.ctrldec_lat_api_curvcmd,
                'ctrldec_lat_api_steercmd':self.ctrldec_lat_api_steercmd,
                'ctrldec_lon_sysmodecmd':self.ctrldec_lon_sysmodecmd,
                'ctrldec_lon_resetflag':self.ctrldec_lon_resetflag,
                'ctrldec_lon_sys_poserr':self.ctrldec_lon_sys_poserr,
                'ctrldec_lon_sys_velff':self.ctrldec_lon_sys_velff,
                'ctrldec_lon_sys_accff':self.ctrldec_lon_sys_accff,
                'ctrldec_lon_sys_gearcmd':self.ctrldec_lon_sys_gearcmd,
                'ctrldec_lon_sys_gear_ena':self.ctrldec_lon_sys_gear_ena,
                'ctrldec_lon_sys_brk_emerg':self.ctrldec_lon_sys_brk_emerg,
                'ctrldec_lon_api_poscmd':self.ctrldec_lon_api_poscmd,
                'ctrldec_lon_api_velff':self.ctrldec_lon_api_velff,
                'ctrldec_lon_api_acc_cmd':self.ctrldec_lon_api_acc_cmd,
                'ctrldec_lon_api_thrcmd':self.ctrldec_lon_api_thrcmd,
                'ctrldec_lon_api_brkcmd':self.ctrldec_lon_api_brkcmd,
                'ctrldec_ctrl_err':self.ctrldec_ctrl_err}

    def clear(self):
        self.ctrldec_sysmode.clear()
        self.ctrldec_req_auto.clear()
        self.ctrldec_automode_is_estop.clear()
        self.ctrldec_is_gear_change_req.clear()
        self.ctrldec_is_stop_steer_ctrl.clear()
        self.ctrldec_is_vehicle_standstill.clear()
        self.ctrldec_is_steer_set_ok.clear()
        self.ctrldec_lat_sysmodecmd.clear()
        self.ctrldec_lat_resetflag.clear()
        self.ctrldec_lat_sys_poserr.clear()
        self.ctrldec_lat_sys_yawff.clear()
        self.ctrldec_lat_sys_velff.clear()
        self.ctrldec_lat_sys_curvff.clear()
        self.ctrldec_lat_api_poscmd.clear()
        self.ctrldec_lat_api_yawcmd.clear()
        self.ctrldec_lat_api_curvcmd.clear()
        self.ctrldec_lat_api_steercmd.clear()
        self.ctrldec_lon_sysmodecmd.clear()
        self.ctrldec_lon_resetflag.clear()
        self.ctrldec_lon_sys_poserr.clear()
        self.ctrldec_lon_sys_velff.clear()
        self.ctrldec_lon_sys_accff.clear()
        self.ctrldec_lon_sys_gearcmd.clear()
        self.ctrldec_lon_sys_gear_ena.clear()
        self.ctrldec_lon_sys_brk_emerg.clear()
        self.ctrldec_lon_api_poscmd.clear()
        self.ctrldec_lon_api_velff.clear()
        self.ctrldec_lon_api_acc_cmd.clear()
        self.ctrldec_lon_api_thrcmd.clear()
        self.ctrldec_lon_api_brkcmd.clear()
        self.ctrldec_ctrl_err.clear()


class lon_ctrl_debug:
    dict = {}
    def __init__(self):
        self.name = 'lon_ctrl_debug'     # 名称
        self.lonctrl_modecmd = []
        self.lonctrl_resetflag = []
        self.lonctrl_sys_poserr = []
        self.lonctrl_sys_velff = []
        self.lonctrl_sys_accff = []
        self.lonctrl_sys_gearcmd = []
        self.lonctrl_sys_gearena = []
        self.lonctrl_sys_brkemerg = []
        self.lonctrl_api_poscmd = []
        self.lonctrl_api_velcmd = []
        self.lonctrl_api_acccmd = []
        self.lonctrl_api_thrcmd = []
        self.lonctrl_api_brkcmd = []
        self.lonctrl_fdbk_vx = []
        self.lonctrl_fdbk_ax = []
        self.lonctrl_fdbk_pitch = []
        self.lonctrl_fdbk_gear = []
        self.lonctrl_fdbk_reverse = []
        self.lonctrl_pos_dyn = []
        self.lonctrl_posctrl_p = []
        self.lonctrl_pos_poserr_filter = []
        self.lonctrl_pos_pi_velcmd = []
        self.lonctrl_pos_output_velcmd = []
        self.lonctrl_vel_dyn = []
        self.lonctrl_vel_velcmd_lmt = []
        self.lonctrl_vel_vel_err = []
        self.lonctrl_velctrl_p = []
        self.lonctrl_velctrl_i = []
        self.lonctrl_vel_pi_acc_cmd = []
        self.lonctrl_vel_pi_acccmd_filter = []
        self.lonctrl_vel_accpitch = []
        self.lonctrl_vel_accdamper = []
        self.lonctrl_vel_accff_filter = []
        self.lonctrl_vel_output_accCmd = []
        self.lonctrl_vel_output_accCmd_filter = []
        self.lonctrl_thrust_thr_dyn = []
        self.lonctrl_thrust_thr_accerr = []
        self.lonctrl_thrust_brk_dyn = []
        self.lonctrl_thrust_brk_accerr = []
        self.lonctrl_thrust_fdbk_ax_filter = []
        self.lonctrl_thrust_thr_acc_cmd_filter = []
        self.lonctrl_thrust_brk_acc_cmd_filter = []
        self.lonctrl_thrustctrl_thr_p = []
        self.lonctrl_thrustctrl_thr_i = []
        self.lonctrl_thrustctrl_brk_p = []
        self.lonctrl_thrustctrl_brk_i = []
        self.lonctrl_thrust_pi_thr_acc_cmd = []
        self.lonctrl_thrust_pi_brk_acc_cmd = []
        self.lonctrl_thrust_acc_cmd_filter_lmt = []
        self.lonctrl_thrust_acctothr_gain = []
        self.lonctrl_thrust_throut_throcmd = []
        self.lonctrl_thrust_acctobrk_gain = []
        self.lonctrl_thrust_brkout_brkcmd = []
        self.lonctrl_analog_autput_throtcmd = []
        self.lonctrl_analog_autput_brkcmd = []

    def update(self,MbdDebugFromMCU):
        self.lonctrl_modecmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_modecmd)
        self.lonctrl_resetflag.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_resetflag)
        self.lonctrl_sys_poserr.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_poserr)
        self.lonctrl_sys_velff.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_velff)
        self.lonctrl_sys_accff.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_accff)
        self.lonctrl_sys_gearcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_gearcmd)
        self.lonctrl_sys_gearena.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_gearena)
        self.lonctrl_sys_brkemerg.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_sys_brkemerg)
        self.lonctrl_api_poscmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_api_poscmd)
        self.lonctrl_api_velcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_api_velcmd)
        self.lonctrl_api_acccmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_api_acccmd)
        self.lonctrl_api_thrcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_api_thrcmd)
        self.lonctrl_api_brkcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_api_brkcmd)
        self.lonctrl_fdbk_vx.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_fdbk_vx)
        self.lonctrl_fdbk_ax.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_fdbk_ax)
        self.lonctrl_fdbk_pitch.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_fdbk_pitch)
        self.lonctrl_fdbk_gear.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_fdbk_gear)
        self.lonctrl_fdbk_reverse.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_fdbk_reverse)
        self.lonctrl_pos_dyn.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_pos_dyn)
        self.lonctrl_posctrl_p.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_posctrl_p)
        self.lonctrl_pos_poserr_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_pos_poserr_filter)
        self.lonctrl_pos_pi_velcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_pos_pi_velcmd)
        self.lonctrl_pos_output_velcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_pos_output_velcmd)
        self.lonctrl_vel_dyn.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_dyn)
        self.lonctrl_vel_velcmd_lmt.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_velcmd_lmt)
        self.lonctrl_vel_vel_err.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_vel_err)
        self.lonctrl_velctrl_p.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_velctrl_p)
        self.lonctrl_velctrl_i.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_velctrl_i)
        self.lonctrl_vel_pi_acc_cmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_pi_acc_cmd)
        self.lonctrl_vel_pi_acccmd_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_pi_acccmd_filter)
        self.lonctrl_vel_accpitch.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_accpitch)
        self.lonctrl_vel_accdamper.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_accdamper)
        self.lonctrl_vel_accff_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_accff_filter)
        self.lonctrl_vel_output_accCmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_output_acccmd)
        self.lonctrl_vel_output_accCmd_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_vel_output_acccmd_filter)
        self.lonctrl_thrust_thr_dyn.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_thr_dyn)
        self.lonctrl_thrust_thr_accerr.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_thr_accerr)
        self.lonctrl_thrust_brk_dyn.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_brk_dyn)
        self.lonctrl_thrust_brk_accerr.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_brk_accerr)
        self.lonctrl_thrust_fdbk_ax_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_fdbk_ax_filter)
        self.lonctrl_thrust_thr_acc_cmd_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_thr_acc_cmd_filter)
        self.lonctrl_thrust_brk_acc_cmd_filter.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_brk_acc_cmd_filter)
        self.lonctrl_thrustctrl_thr_p.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrustctrl_thr_p)
        self.lonctrl_thrustctrl_thr_i.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrustctrl_thr_i)
        self.lonctrl_thrustctrl_brk_p.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrustctrl_brk_p)
        self.lonctrl_thrustctrl_brk_i.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrustctrl_brk_i)
        self.lonctrl_thrust_pi_thr_acc_cmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_pi_thr_acc_cmd)
        self.lonctrl_thrust_pi_brk_acc_cmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_pi_brk_acc_cmd)
        self.lonctrl_thrust_acc_cmd_filter_lmt.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_acc_cmd_filter_lmt)
        self.lonctrl_thrust_acctothr_gain.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_acctothr_gain)
        self.lonctrl_thrust_throut_throcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_throut_throcmd)
        self.lonctrl_thrust_acctobrk_gain.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_acctobrk_gain)
        self.lonctrl_thrust_brkout_brkcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_thrust_brkout_brkcmd)
        self.lonctrl_analog_autput_throtcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_analog_autput_throtcmd)
        self.lonctrl_analog_autput_brkcmd.append(MbdDebugFromMCU.lon_ctrl_debug.lonctrl_analog_autput_brkcmd)

        self.dict = {
                'lonctrl_modecmd':self.lonctrl_modecmd,
                'lonctrl_resetflag':self.lonctrl_resetflag,
                'lonctrl_sys_poserr':self.lonctrl_sys_poserr,
                'lonctrl_sys_velff':self.lonctrl_sys_velff,
                'lonctrl_sys_accff':self.lonctrl_sys_accff,
                'lonctrl_sys_gearcmd':self.lonctrl_sys_gearcmd,
                'lonctrl_sys_gearena':self.lonctrl_sys_gearena,
                'lonctrl_sys_brkemerg':self.lonctrl_sys_brkemerg,
                'lonctrl_api_poscmd':self.lonctrl_api_poscmd,
                'lonctrl_api_velcmd':self.lonctrl_api_velcmd,
                'lonctrl_api_acccmd':self.lonctrl_api_acccmd,
                'lonctrl_api_thrcmd':self.lonctrl_api_thrcmd,
                'lonctrl_api_brkcmd':self.lonctrl_api_brkcmd,
                'lonctrl_fdbk_vx':self.lonctrl_fdbk_vx,
                'lonctrl_fdbk_ax':self.lonctrl_fdbk_ax,
                'lonctrl_fdbk_pitch':self.lonctrl_fdbk_pitch,
                'lonctrl_fdbk_gear':self.lonctrl_fdbk_gear,
                'lonctrl_fdbk_reverse':self.lonctrl_fdbk_reverse,
                'lonctrl_pos_dyn':self.lonctrl_pos_dyn,
                'lonctrl_posctrl_p':self.lonctrl_posctrl_p,
                'lon_pos_poserr_filter':self.lonctrl_pos_poserr_filter,
                'lon_pos_pi_velcmd':self.lonctrl_pos_pi_velcmd,
                'lon_pos_output_velcmd':self.lonctrl_pos_output_velcmd,
                'lon_vel_dyn':self.lonctrl_vel_dyn,
                'lon_vel_velcmd_lmt':self.lonctrl_vel_velcmd_lmt,
                'lon_vel_vel_err':self.lonctrl_vel_vel_err,
                'lon_velctrl_p':self.lonctrl_velctrl_p,
                'lonctrl_velctrl_i':self.lonctrl_velctrl_i,
                'lon_vel_pi_acc_cmd':self.lonctrl_vel_pi_acc_cmd,
                'lon_vel_pi_acccmd_filter':self.lonctrl_vel_pi_acccmd_filter,
                'lon_vel_accpitch':self.lonctrl_vel_accpitch,
                'lon_vel_accdamper':self.lonctrl_vel_accdamper,
                'lon_vel_accff_filter':self.lonctrl_vel_accff_filter,
                'lon_vel_output_accCmd':self.lonctrl_vel_output_accCmd,
                'lon_vel_output_accCmd_filter':self.lonctrl_vel_output_accCmd_filter,
                'lonctrl_thrust_thr_dyn':self.lonctrl_thrust_thr_dyn,
                'lon_thrust_thr_accerr':self.lonctrl_thrust_thr_accerr,
                'lon_thrust_brk_dyn':self.lonctrl_thrust_brk_dyn,
                'lonctrl_thrust_brk_accerr':self.lonctrl_thrust_brk_accerr,
                'lon_thrust_fdbk_ax_filter':self.lonctrl_thrust_fdbk_ax_filter,
                'lon_thrust_thr_acc_cmd_filter':self.lonctrl_thrust_thr_acc_cmd_filter,
                'lon_thrust_brk_acc_cmd_filter':self.lonctrl_thrust_brk_acc_cmd_filter,
                'lon_thrustctrl_thr_p':self.lonctrl_thrustctrl_thr_p,
                'lon_thrustctrl_thr_i':self.lonctrl_thrustctrl_thr_i,
                'lonctrl_thrustctrl_brk_p':self.lonctrl_thrustctrl_brk_p,
                'lonctrl_thrustctrl_brk_i':self.lonctrl_thrustctrl_brk_i,
                'lon_thrust_pi_thr_acc_cmd':self.lonctrl_thrust_pi_thr_acc_cmd,
                'lon_thrust_pi_brk_acc_cmd':self.lonctrl_thrust_pi_brk_acc_cmd,
                'lon_thrust_acc_cmd_filter_lmt':self.lonctrl_thrust_acc_cmd_filter_lmt,
                'lon_thrust_acctothr_gain':self.lonctrl_thrust_acctothr_gain,
                'lon_thrust_throut_throcmd':self.lonctrl_thrust_throut_throcmd,
                'lon_thrust_acctobrk_gain':self.lonctrl_thrust_acctobrk_gain,
                'lon_thrust_brkout_brkcmd':self.lonctrl_thrust_brkout_brkcmd,
                'lon_analog_autput_throtcmd':self.lonctrl_analog_autput_throtcmd,
                'lon_analog_autput_brkcmd':self.lonctrl_analog_autput_brkcmd}

    def clear(self):
          self.lonctrl_modecmd.clear()
          self.lonctrl_resetflag.clear()
          self.lonctrl_sys_poserr.clear()
          self.lonctrl_sys_velff.clear()
          self.lonctrl_sys_accff.clear()
          self.lonctrl_sys_gearcmd.clear()
          self.lonctrl_sys_gearena.clear()
          self.lonctrl_sys_brkemerg.clear()
          self.lonctrl_api_poscmd.clear()
          self.lonctrl_api_velcmd.clear()
          self.lonctrl_api_acccmd.clear()
          self.lonctrl_api_thrcmd.clear()
          self.lonctrl_api_brkcmd.clear()
          self.lonctrl_fdbk_vx.clear()
          self.lonctrl_fdbk_ax.clear()
          self.lonctrl_fdbk_pitch.clear()
          self.lonctrl_fdbk_gear.clear()
          self.lonctrl_fdbk_reverse.clear()
          self.lonctrl_pos_dyn.clear()
          self.lonctrl_posctrl_p.clear()
          self.lonctrl_pos_poserr_filter.clear()
          self.lonctrl_pos_pi_velcmd.clear()
          self.lonctrl_pos_output_velcmd.clear()
          self.lonctrl_vel_dyn.clear()
          self.lonctrl_vel_velcmd_lmt.clear()
          self.lonctrl_vel_vel_err.clear()
          self.lonctrl_velctrl_p.clear()
          self.lonctrl_velctrl_i.clear()
          self.lonctrl_vel_pi_acc_cmd.clear()
          self.lonctrl_vel_pi_acccmd_filter.clear()
          self.lonctrl_vel_accpitch.clear()
          self.lonctrl_vel_accdamper.clear()
          self.lonctrl_vel_accff_filter.clear()
          self.lonctrl_vel_output_accCmd.clear()
          self.lonctrl_vel_output_accCmd_filter.clear()
          self.lonctrl_thrust_thr_dyn.clear()
          self.lonctrl_thrust_thr_accerr.clear()
          self.lonctrl_thrust_brk_dyn.clear()
          self.lonctrl_thrust_brk_accerr.clear()
          self.lonctrl_thrust_fdbk_ax_filter.clear()
          self.lonctrl_thrust_thr_acc_cmd_filter.clear()
          self.lonctrl_thrust_brk_acc_cmd_filter.clear()
          self.lonctrl_thrustctrl_thr_p.clear()
          self.lonctrl_thrustctrl_thr_i.clear()
          self.lonctrl_thrustctrl_brk_p.clear()
          self.lonctrl_thrustctrl_brk_i.clear()
          self.lonctrl_thrust_pi_thr_acc_cmd.clear()
          self.lonctrl_thrust_pi_brk_acc_cmd.clear()
          self.lonctrl_thrust_acc_cmd_filter_lmt.clear()
          self.lonctrl_thrust_acctothr_gain.clear()
          self.lonctrl_thrust_throut_throcmd.clear()
          self.lonctrl_thrust_acctobrk_gain.clear()
          self.lonctrl_thrust_brkout_brkcmd.clear()
          self.lonctrl_analog_autput_throtcmd.clear()
          self.lonctrl_analog_autput_brkcmd.clear()

class lat_ctrl_debug:
    dict = {}
    def __init__(self):
        self.name = 'lat_ctrl_debug'     # 名称
        self.latctrl_modecmd = []
        self.latctrl_resetflag = []
        self.latctrl_sys_poserr = []
        self.latctrl_sys_yawff = []
        self.latctrl_sys_velff = []
        self.latctrl_sys_curvff = []
        self.latctrl_api_poscmd = []
        self.latctrl_api_yawcmd = []
        self.latctrl_api_curvcmd = []
        self.latctrl_api_steercmd = []
        self.latictrl_fdbk_vxb = []
        self.latictrl_fdbk_heading = []
        self.latictrl_fdbk_yawrate = []
        self.latictrl_fdbk_steer = []
        self.latictrl_fdbk_gear = []
        self.latictrl_fdbk_rvsflag = []
        self.latictrl_offset_dyn = []
        self.latictrl_offsetctrl_i = []
        self.latictrl_offsetctrl_p = []
        self.latictrl_offset_offseterr = []
        self.latictrl_offset_pi_torscmd = []
        self.latictrl_offset_torsrateffcmd = []
        self.latictrl_offset_output_yawcmd = []
        self.latictrl_tors_dyn = []
        self.latictrl_tors_yawerr = []
        self.latictrl_yawctrl_p = []
        self.latictrl_yawctrl_i = []
        self.latictrl_tors_pi_torsrate = []
        self.latictrl_tors_pi_leadfilter_torsrate = []
        self.latictrl_tors_torsrateff = []
        self.latictrl_tors_output_yawratecmd = []
        self.latictrl_rate_dyn = []
        self.latictrl_rate_p = []
        self.latictrl_rate_i = []
        self.latictrl_rate_yawratecmd_lmt = []
        self.latictrl_rate_filter_yawratecmd_lmt = []
        self.latictrl_rate_pi_steer = []
        self.latictrl_rate_pi_filter_steer = []
        self.latictrl_rate_steerff = []
        self.latictrl_rate_output_front_steercmd = []
        self.latictrl_rate_output_sw_steercmd = []
        self.latictrl_steer_steercmd_filter = []
        self.latictrl_steer_max_steerrate_value = []
        self.latictrl_steer_steercmd_lmt_filter = []
        self.latictrl_steer_output_steercmd = []
        self.latictrl_steer_steertorque_initial = []
        self.latictrl_steer_steertorque_cmd = []
        self.latictrl_tors_pure_yawerr = []
        self.latictrl_rate_output_front_steercmd_offset = []
        # self.latictrl_steer_steer_err = []
        self.latictrl_yaw_curve_compsate = []
        self.latictrl_rate_reference_model = []
        self.latictrl_rate_mrac_cmd = []
        self.latictrl_rate_eso_cmd = []
        self.latictrl_rate_steer_offset = []
        self.latictrl_rate_ramp_estimate = []
        self.latictrl_error = []

    def update(self,MbdDebugFromMCU):
        self.latctrl_modecmd.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_modecmd)
        self.latctrl_resetflag.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_resetflag)
        self.latctrl_sys_poserr.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_sys_poserr)
        self.latctrl_sys_yawff.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_sys_yawff)
        self.latctrl_sys_velff.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_sys_velff)
        self.latctrl_sys_curvff.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_sys_curvff)
        self.latctrl_api_poscmd.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_api_poscmd)
        self.latctrl_api_yawcmd.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_api_yawcmd)
        self.latctrl_api_curvcmd.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_api_curvcmd)
        self.latctrl_api_steercmd.append(MbdDebugFromMCU.lat_ctrl_debug.latctrl_api_steercmd)
        self.latictrl_fdbk_vxb.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_vxb)
        self.latictrl_fdbk_heading.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_heading)
        self.latictrl_fdbk_yawrate.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_yawrate)
        self.latictrl_fdbk_steer.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_steer)
        self.latictrl_fdbk_gear.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_gear)
        self.latictrl_fdbk_rvsflag.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_fdbk_rvsflag)
        self.latictrl_offset_dyn.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offset_dyn)
        self.latictrl_offsetctrl_i.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offsetctrl_i)
        self.latictrl_offsetctrl_p.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offsetctrl_p)
        self.latictrl_offset_offseterr.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offset_offseterr)
        self.latictrl_offset_pi_torscmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offset_pi_torscmd)
        self.latictrl_offset_torsrateffcmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offset_torsrateffcmd)
        self.latictrl_offset_output_yawcmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_offset_output_yawcmd)
        self.latictrl_tors_dyn.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_dyn)
        self.latictrl_tors_yawerr.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_yawerr)
        self.latictrl_yawctrl_p.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_yawctrl_p)
        self.latictrl_yawctrl_i.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_yawctrl_i)
        self.latictrl_tors_pi_torsrate.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_pi_torsrate)
        self.latictrl_tors_pi_leadfilter_torsrate.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_pi_leadfilter_torsrate)
        self.latictrl_tors_torsrateff.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_torsrateff)
        self.latictrl_tors_output_yawratecmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_output_yawratecmd)
        self.latictrl_rate_dyn.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_dyn)
        self.latictrl_rate_p.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_p)
        self.latictrl_rate_i.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_i)
        self.latictrl_rate_yawratecmd_lmt.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_yawratecmd_lmt)
        self.latictrl_rate_filter_yawratecmd_lmt.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_filter_yawratecmd_lmt)
        self.latictrl_rate_pi_steer.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_pi_steer)
        self.latictrl_rate_pi_filter_steer.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_pi_filter_steer)
        self.latictrl_rate_steerff.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_steerff)
        self.latictrl_rate_output_front_steercmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_output_front_steercmd)
        self.latictrl_rate_output_sw_steercmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_output_sw_steercmd)
        self.latictrl_steer_steercmd_filter.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_steercmd_filter)
        self.latictrl_steer_max_steerrate_value.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_max_steerrate_value)
        self.latictrl_steer_steercmd_lmt_filter.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_steercmd_lmt_filter)
        self.latictrl_steer_output_steercmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_output_steercmd)
        # self.latictrl_steer_steertorque_initial.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_steertorque_initial)
        # self.latictrl_steer_steertorque_cmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_steertorque_cmd)
        self.latictrl_tors_pure_yawerr.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_tors_pure_yawerr)
        self.latictrl_rate_output_front_steercmd_offset.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_output_front_steercmd_offset)
        # self.latictrl_steer_steer_err.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_steer_steer_err)
        self.latictrl_yaw_curve_compsate.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_yaw_curve_compsate)
        self.latictrl_rate_reference_model.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_reference_model)
        self.latictrl_rate_mrac_cmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_mrac_cmd)
        self.latictrl_rate_eso_cmd.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_eso_cmd)
        self.latictrl_rate_steer_offset.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_steer_offset)
        self.latictrl_rate_ramp_estimate.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_rate_ramp_estimate)
        self.latictrl_error.append(MbdDebugFromMCU.lat_ctrl_debug.latictrl_error)

        self.dict = {
                'latctrl_modecmd':self.latctrl_modecmd,
                'latctrl_resetflag':self.latctrl_resetflag,
                'latctrl_sys_poserr':self.latctrl_sys_poserr,
                'latctrl_sys_yawff':self.latctrl_sys_yawff,
                'latctrl_sys_velff':self.latctrl_sys_velff,
                'latctrl_sys_curvff':self.latctrl_sys_curvff,
                'latctrl_api_poscmd':self.latctrl_api_poscmd,
                'latctrl_api_yawcmd':self.latctrl_api_yawcmd,
                'latctrl_api_curvcmd':self.latctrl_api_curvcmd,
                'latctrl_api_steercmd':self.latctrl_api_steercmd,
                'latictrl_fdbk_vxb':self.latictrl_fdbk_vxb,
                'latictrl_fdbk_yaw':self.latictrl_fdbk_heading,
                'latictrl_fdbk_yawrate':self.latictrl_fdbk_yawrate,
                'latictrl_fdbk_steer':self.latictrl_fdbk_steer,
                'latictrl_fdbk_gear':self.latictrl_fdbk_gear,
                'latictrl_fdbk_rvsflag':self.latictrl_fdbk_rvsflag,
                'latictrl_offset_dyn':self.latictrl_offset_dyn,
                'latictrl_offsetctrl_i':self.latictrl_offsetctrl_i,
                'latictrl_offsetctrl_p':self.latictrl_offsetctrl_p,
                'lat_offset_offseterr':self.latictrl_offset_offseterr,
                'lat_offset_pi_torscmd':self.latictrl_offset_pi_torscmd,
                'lat_offset_torsrateffcmd':self.latictrl_offset_torsrateffcmd,
                'lat_offset_output_yawcmd':self.latictrl_offset_output_yawcmd,
                'latictrl_tors_dyn':self.latictrl_tors_dyn,
                'latictrl_tors_yawerr':self.latictrl_tors_yawerr,
                'latictrl_yawctrl_p':self.latictrl_yawctrl_p,
                'latictrl_yawctrl_i':self.latictrl_yawctrl_i,
                'latictrl_tors_pi_torsrate':self.latictrl_tors_pi_torsrate,
                'lat_pi_torsrate_by_leadfilter':self.latictrl_tors_pi_leadfilter_torsrate,
                'lat_tors_torsrateff':self.latictrl_tors_torsrateff,
                'lat_tors_output_yawratecmd':self.latictrl_tors_output_yawratecmd,
                'latictrl_rate_dyn':self.latictrl_rate_dyn,
                'latictrl_rate_p':self.latictrl_rate_p,
                'latictrl_rate_i':self.latictrl_rate_i,
                'lat_rate_yawratecmd_lmt':self.latictrl_rate_yawratecmd_lmt,
                'lat_rate_yawratecmd_lmt_filter':self.latictrl_rate_filter_yawratecmd_lmt,
                'latictrl_rate_pi_steer':self.latictrl_rate_pi_steer,
                'lat_rate_pi_steer_filter':self.latictrl_rate_pi_filter_steer,
                'latictrl_rate_steerff':self.latictrl_rate_steerff,
                'lat_rate_output_front_steercmd':self.latictrl_rate_output_front_steercmd,
                'lat_rate_output_sw_steercmd':self.latictrl_rate_output_sw_steercmd,
                'lat_steer_steercmd_filter':self.latictrl_steer_steercmd_filter,
                'lat_steer_max_steerrate_value':self.latictrl_steer_max_steerrate_value,
                'lat_steer_steercmd_lmt_filter':self.latictrl_steer_steercmd_lmt_filter,
                'lat_steer_output_steercmd':self.latictrl_steer_output_steercmd,
                'lat_steer_steertorque_initial':self.latictrl_steer_steertorque_initial,
                'latictrl_steer_steertorque_cmd':self.latictrl_steer_steertorque_cmd,
                'latictrl_tors_pure_yawerr':self.latictrl_tors_pure_yawerr,
                'lat_output_front_steeroffset':self.latictrl_rate_output_front_steercmd_offset,
                # 'latictrl_steer_steer_err':self.latictrl_steer_steer_err,
                'latictrl_yaw_curve_compsate':self.latictrl_yaw_curve_compsate,
                'latictrl_rate_reference_model':self.latictrl_rate_reference_model,
                'latictrl_rate_mrac_cmd':self.latictrl_rate_mrac_cmd,
                'latictrl_rate_eso_cmd':self.latictrl_rate_eso_cmd,
                'latictrl_rate_steer_offset':self.latictrl_rate_steer_offset,
                'latictrl_rate_ramp_estimate':self.latictrl_rate_ramp_estimate,
                'latictrl_error':self.latictrl_error}

    def clear(self):
          self.latctrl_modecmd.clear()
          self.latctrl_resetflag.clear()
          self.latctrl_sys_poserr.clear()
          self.latctrl_sys_yawff.clear()
          self.latctrl_sys_velff.clear()
          self.latctrl_sys_curvff.clear()
          self.latctrl_api_poscmd.clear()
          self.latctrl_api_yawcmd.clear()
          self.latctrl_api_curvcmd.clear()
          self.latctrl_api_steercmd.clear()
          self.latictrl_fdbk_vxb.clear()
          self.latictrl_fdbk_heading.clear()
          self.latictrl_fdbk_yawrate.clear()
          self.latictrl_fdbk_steer.clear()
          self.latictrl_fdbk_gear.clear()
          self.latictrl_fdbk_rvsflag.clear()
          self.latictrl_offset_dyn.clear()
          self.latictrl_offsetctrl_i.clear()
          self.latictrl_offsetctrl_p.clear()
          self.latictrl_offset_offseterr.clear()
          self.latictrl_offset_pi_torscmd.clear()
          self.latictrl_offset_torsrateffcmd.clear()
          self.latictrl_offset_output_yawcmd.clear() 
          self.latictrl_tors_dyn.clear()
          self.latictrl_tors_yawerr.clear()
          self.latictrl_yawctrl_p.clear()
          self.latictrl_yawctrl_i.clear()
          self.latictrl_tors_pi_torsrate.clear()
          self.latictrl_tors_pi_leadfilter_torsrate.clear()
          self.latictrl_tors_torsrateff.clear()
          self.latictrl_tors_output_yawratecmd.clear()
          self.latictrl_rate_dyn.clear()
          self.latictrl_rate_p.clear()
          self.latictrl_rate_i.clear()
          self.latictrl_rate_yawratecmd_lmt.clear()
          self.latictrl_rate_filter_yawratecmd_lmt.clear()
          self.latictrl_rate_pi_steer.clear()
          self.latictrl_rate_pi_filter_steer.clear()
          self.latictrl_rate_steerff.clear()
          self.latictrl_rate_output_front_steercmd.clear()
          self.latictrl_rate_output_sw_steercmd.clear()
          self.latictrl_steer_steercmd_filter.clear()
          self.latictrl_steer_max_steerrate_value.clear()
          self.latictrl_steer_steercmd_lmt_filter.clear()
          self.latictrl_steer_output_steercmd.clear()
          self.latictrl_steer_steertorque_initial.clear()
          self.latictrl_steer_steertorque_cmd.clear()
          self.latictrl_tors_pure_yawerr.clear()
          self.latictrl_rate_output_front_steercmd_offset.clear()
        #   self.latictrl_steer_steer_err.clear()
          self.latictrl_yaw_curve_compsate.clear()
          self.latictrl_rate_reference_model.clear()
          self.latictrl_rate_mrac_cmd.clear()
          self.latictrl_rate_eso_cmd.clear()
          self.latictrl_rate_steer_offset.clear()
          self.latictrl_rate_ramp_estimate.clear()
          self.latictrl_error.clear()
                         
        
class CtrlOutputDebug:
    dict = {}
    def __init__(self):
        self.name = 'CtrlOutputDebug'     # 名称
        self.brake_cmd = []
        self.throttle_cmd = []
        self.acc_cmd = []
        self.gear_enable = []
        self.gear_cmd = []
        self.emerg_enable = []
        self.steer_cmd = []
        self.steer_torque_cmd = []
    def update(self,MbdDebugFromMCU):
        self.brake_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_brake_cmd)
        self.throttle_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_throttle_cmd)
        self.acc_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_acc_cmd)
        self.gear_enable.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_gear_enable)
        self.gear_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_gear_cmd)
        self.emerg_enable.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_emerg_enable)
        self.steer_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_steer_cmd)
        self.steer_torque_cmd.append(MbdDebugFromMCU.ctrl_output_debug.ctrlout_steer_torque_cmd)
     
        self.dict = {
                'brake_cmd':self.brake_cmd,
                'throttle_cmd':self.throttle_cmd,
                'acc_cmd':self.acc_cmd,
                'gear_enable':self.gear_enable,
                'gear_cmd':self.gear_cmd,
                'emerg_enable:':self.emerg_enable,
                'steer_cmd':self.steer_cmd,
                'steer_torque_cmd':self.steer_torque_cmd}
    def clear(self):
          self.brake_cmd.clear()
          self.throttle_cmd.clear()
          self.acc_cmd.clear()
          self.gear_enable.clear()
          self.gear_cmd.clear()
          self.emerg_enable.clear()
          self.steer_cmd.clear()
          self.steer_torque_cmd.clear()

# class header:
#     dict = []
#     def __init__(self):
#         self.name = 'header'     # 名称
       
#         self.data_stamp = []
#         self.module_name = []
 
#         #print(" header init succesfullu ")
    
#     def update(self,MbdDebugFromMCU):
#         self.data_stamp = MbdDebugFromMCU.header.data_stamp
#         header.dict = {'data_stamp':self.data_stamp}


# class PlanningTest:


#     def __init__(self):
#         self.name = 'planning'     # 名称
#         self.dict = {}
#         self.clear()

#     def clear(self):
#         self.header = []
#         self.total_path_length = []
#         self.total_path_time = []
#         self.trajectory_point = []
#         self.estop = []
#         self.is_replan = []
#         self.gear = []
#         self.x = []
#         self.y = []
#         self.z = []
#         self.theta = []
#         self.kappa = []
#         self.s = []
#         self.dkappa = []
#         self.ddkappa = []
#         self.lane_id = []
#         self.x_derivative = []
#         self.y_derivative = []
#         self.v = []
#         self.a = []
#         self.relative_time = []
#         self.da = []
#         self.steer = []


        #print(" pose_calc_debug init succesfully ")
    
    # def update(self,ADCTrajectory):
    #     self.clear()
    #     try:
    #         self.total_path_length.append(ADCTrajectory.total_path_length)
    #         self.total_path_time.append(ADCTrajectory.total_path_time)
    #         self.estop.append(ADCTrajectory.estop.is_estop)
    #         self.is_replan.append(ADCTrajectory.is_replan)
    #         self.gear.append(ADCTrajectory.gear)
    #         #self.header.append(ADCTrajectory.header)
    #         for p in ADCTrajectory.trajectory_point:
    #             self.x.append(p.path_point.x)
    #             self.y.append(p.path_point.y) 
    #             self.z.append(p.path_point.z)
    #             self.theta.append(p.path_point.theta)
    #             self.kappa.append(p.path_point.kappa)
    #             self.s.append(p.path_point.s)
    #             self.dkappa.append(p.path_point.dkappa)
    #             self.ddkappa.append(p.path_point.ddkappa)
    #             self.lane_id.append(p.path_point.lane_id)
    #             self.x_derivative.append(p.path_point.x_derivative)
    #             self.y_derivative.append(p.path_point.y_derivative)
    #             self.v.append(p.v)
    #             self.a.append(p.a)
    #             self.relative_time.append(p.relative_time)
    #             self.da.append(p.da)
    #             self.steer.append(p.steer)

    #         self.dict = {
    #             #'header':self.header,
    #             'path_length':self.total_path_length,
    #             'path_time':self.total_path_length,
    #             'estop':self.estop,
    #             'is_replan':self.is_replan,
    #             'gear':self.gear,
    #             'x':self.x,
    #             'y':self.y,
    #             'z':self.z,
    #             'theta':self.theta,
    #             'kappa':self.kappa,
    #             's':self.s,
    #             'dkappa':self.dkappa,
    #             'ddkappa':self.ddkappa,
    #             'lane_id':self.lane_id,
    #             'x_derivative':self.x_derivative,
    #             #'y_derivative':self.y_derivative,
    #             'v':self.v,
    #             'a':self.a,
    #             'relative_time':self.relative_time,
    #             'da':self.da,
    #             'steer':self.steer}
            
    #     except NameError:
    #         print("error")
        
        #print(" the value assign to pose_calc_debug is  :" ,self.list)
# class PlanningTest:

#     dict = {}
#     def __init__(self):
#         self.name = 'planning'     # 名称
#         self.clear()
#     def clear(self):
#         self.header = []
#         self.total_path_length = []
#         self.total_path_time = []
#         self.trajectory_point = []
#         self.estop = []
#         self.is_replan = []
#         self.gear = []

#         self.trajectory_point = trajectory_point()
#         self.header = header()


#     def update(self,ADCTrajectory):
#             self.clear()
#             header.update(self,ADCTrajectory)
#             self.total_path_length.append(ADCTrajectory.total_path_length)
#             self.total_path_time.append(ADCTrajectory.total_path_time)
#             self.estop.append(ADCTrajectory.estop.is_estop)
#             self.is_replan.append(ADCTrajectory.is_replan)
#             self.gear.append(ADCTrajectory.gear)
#             for p in ADCTrajectory.trajectory_point:
#                 self.trajectory_point.update(p)
            
#             self.trajectory_point_dict = {'trajectory_point':self.trajectory_point.dict}
            
#             self.dict = {'header':self.header.dict,
#                 'path_length':self.total_path_length,
#                 'path_time':self.total_path_time,
#                 'estop':self.estop,
#                 'is_replan':self.is_replan,
#                 'gear':self.gear,
#                 'trajectory_point':self.trajectory_point_dict}
    

# class trajectory_point:
    
#     def __init__(self):
#         self.name = 'trajectory_point'     # 名称
#         self.path_point = []
#         self.v = []
#         self.a = []
#         self.relative_time = []
#         self.da = []
#         self.steer = []
#         self.path_point = path_point()
#         self.dict = {}
#     def update(self,trajectory_point):
#         #self.path_point.append(ADCTrajectory.trajectory_point.path_point)
#         self.v.append(trajectory_point.v)
#         self.a.append(trajectory_point.a)
#         self.relative_time.append(trajectory_point.relative_time)
#         self.da.append(trajectory_point.da)
#         self.steer.append(trajectory_point.steer)
#         self.path_point.update(trajectory_point.path_point)
        
#         self.path_point_dict = {'path_point':self.path_point.path_point_dict}
#         self.dict = {
#             'path_point':self.path_point_dict,
#             'v':self.v,
#             'a':self.a,
#             'relative_time':self.relative_time,
#             'da':self.da,
#             'steer':self.steer}

# class path_point:
    
#     def __init__(self):
#         path_point_dict=dict()
#         self.name = 'path_point'     # 名称
#         self.x = []
#         self.y = []
#         self.z = []
#         self.theta = []
#         self.kappa = []
#         self.s = []
#         self.dkappa = []
#         self.ddkappa = []
#         self.lane_id = []
#         self.x_derivative = []
#         self.path_point_dict = {}
#     def update(self,path_point):
#         self.x.append(path_point.x)
#         self.y.append(path_point.y)
#         self.z.append(path_point.z)
#         self.theta.append(path_point.theta)ddkappa)
#         self.x_derivative.append(path_point.x_derivative)

#         self.path_point_dict = {
#                 'x':self.x,
#                 'y':self.y,
#                 'z':self.z,
#                 'theta':self.theta,
#                 'kappa':self.kappa,
#                 's':self.s,
#                 'dkappa':self.dkappa,
#                 'ddkappa': self.ddkappa,
#                 'lane_id':self.lane_id,
#                 'x_derivative':self.x_derivative}

# class header:
#     dict = []
#     def __init__(self):
#         self.name = 'header'     # 名称
       
#         self.data_stamp = []
#         self.module_name = []
 
#         #print(" header init succesfullu ")
    
#     def update(self,ADCTrajectory):
#         self.data_stamp = ADCTrajectory.header.data_stamp
#         header.dict = {'data_stamp':self.data_stamp}
class PlanningTest:

    dict = {}
    def __init__(self):
        self.name = 'planning'     # 名称
        self.clear()
    def clear(self):
        self.header = []
        self.total_path_length = []
        self.total_path_time = []
        self.trajectory_point = []
        self.estop = []
        self.is_replan = []
        self.gear = []

        self.trajectory_point = trajectory_point()
        self.trajectory_point_v = []
        self.trajectory_point_a = []
        self.relative_time = []
        self.da = []
        self.steer = []
        self.path_point_x = []
        self.path_point_y = []
        self.path_point_z = []
        self.theta = []
        self.kappa = []
        self.s = []
       
        self.header = header()
        self.start_time = None
        self.time = []
        self.driving_mode = []

    def update(self,ADCTrajectory):
            # self.clear()
            self.trajectory_point.clear() 
            self.header.update(ADCTrajectory)
            if self.start_time is None:
                self.start_time = self.header.data_stamp
            current_time = self.header.data_stamp - self.start_time
            self.time.append([current_time,self.header.data_stamp]) 
            self.total_path_length.append([current_time, ADCTrajectory.total_path_length])
            self.total_path_time.append([current_time, ADCTrajectory.total_path_time])
            self.estop.append([current_time, ADCTrajectory.estop.is_estop])
            self.is_replan.append([current_time, ADCTrajectory.is_replan])
            self.gear.append([current_time, ADCTrajectory.gear])
            self.driving_mode.append([current_time, ADCTrajectory.driving_mode])

            for p in ADCTrajectory.trajectory_point:
                self.trajectory_point.update(p)
            # self.trajectory_point.v.insert(0, current_time)
            # # print(current_time)
            # self.trajectory_point_v.append([list.copy(self.trajectory_point.v)])
            # self.trajectory_point_dict = {'trajectory_point':self.trajectory_point.dict}
#             # temp = [1.0]
#             # for v in self.trajectory_point.v:
#             #     temp.append(v)
            self.trajectory_point_v.append([current_time] + self.trajectory_point.v)
            self.trajectory_point_a.append([current_time] + self.trajectory_point.a)
            self.relative_time.append([current_time] + self.trajectory_point.relative_time)
            self.da.append([current_time] + self.trajectory_point.da)
            self.steer.append([current_time] + self.trajectory_point.steer)
            self.path_point_x.append([current_time] + self.trajectory_point.path_point.path_point_dict['x'])
            self.path_point_y.append([current_time] + self.trajectory_point.path_point.path_point_dict['y'])
            self.path_point_z.append([current_time] + self.trajectory_point.path_point.path_point_dict['z'])
            self.theta.append([current_time] + self.trajectory_point.path_point.path_point_dict['theta'])
            self.kappa.append([current_time] + self.trajectory_point.path_point.path_point_dict['kappa'])
            self.s.append([current_time] + self.trajectory_point.path_point.path_point_dict['s'])
            
    # def get_dict(self):
    #     return {'header':self.header.dict,
    #             'time':self.current_time,
    #             'path_length':self.total_path_length,
    #             'path_time':self.total_path_time,
    #             'estop':self.estop,
    #             'is_replan':self.is_replan,
    #             'gear':self.gear,
    #             'trajectory_point_v':self.trajectory_point_v,
    #             'trajectory_point_a':self.trajectory_point_a,
    #             'relative_time':self.relative_time,
    #             'da':self.da,
    #             'steer':self.steer,
    #             'path_point_x': self.path_point_x,
    #             'path_point_y':self.path_point_y,
    #             'path_point_z':self.path_point_z,
    #             'theta':self.theta,
    #             'kappa':self.kappa,
    #             's':self.s}
                # 'trajectory_point':self.trajectory_point_dict}
    def get_dict(self):
        return {
                'time':self.time,
                'path_length':self.total_path_length,
                'path_time':self.total_path_time,
                'estop':self.estop,
                'is_replan':self.is_replan,
                'gear':self.gear,
                'trajectory_point_v':self.trajectory_point_v,
                'trajectory_point_a':self.trajectory_point_a,
                'relative_time':self.relative_time,
                'da':self.da,
                'steer':self.steer,
                'path_point_x': self.path_point_x,
                'path_point_y':self.path_point_y,
                'path_point_z':self.path_point_z,
                'theta':self.theta,
                'kappa':self.kappa,
                's':self.s,
                'driving_mode':self.driving_mode}
    

class trajectory_point:
    
    def __init__(self):
        self.name = 'trajectory_point'     # 名称
        self.v = []
        self.a = []
        self.relative_time = []
        self.da = []
        self.steer = []
        self.path_point = path_point()
        self.dict = {}
    def update(self,trajectory_point):
        #self.path_point.append(ADCTrajectory.trajectory_point.path_point)
        self.v.append(trajectory_point.v)
        self.a.append(trajectory_point.a)
        self.relative_time.append(trajectory_point.relative_time)
        self.da.append(trajectory_point.da)
        self.steer.append(trajectory_point.steer)
        self.path_point.update(trajectory_point.path_point)
        
        self.path_point_dict = {'path_point':self.path_point.path_point_dict}
        self.dict = {
            'path_point':self.path_point_dict,
            'v':self.v,
            'a':self.a,
            'relative_time':self.relative_time,
            'da':self.da,
            'steer':self.steer}

    def clear(self):
          self.v.clear()
          self.a.clear()
          self.relative_time.clear()
          self.da.clear()
          self.steer.clear()
          self.path_point.clear()

class path_point:
    
    def __init__(self):
        path_point_dict=dict()
        self.name = 'path_point'     # 名称
        self.x = []
        self.y = []
        self.z = []
        self.theta = []
        self.kappa = []
        self.s = []
        self.dkappa = []
        self.ddkappa = []
        self.lane_id = []
        self.x_derivative = []
        self.path_point_dict = {}
    def update(self,path_point):
        self.x.append(path_point.x)
        self.y.append(path_point.y)
        self.z.append(path_point.z)
        self.theta.append(path_point.theta)
        self.s.append(path_point.s)
        self.kappa.append(path_point.kappa)
        self.dkappa.append(path_point.dkappa)
        self.ddkappa.append(path_point.ddkappa)
        self.lane_id.append(path_point.lane_id)
        self.x_derivative.append(path_point.x_derivative)

        self.path_point_dict = {
                'x':self.x,
                'y':self.y,
                'z':self.z,
                'theta':self.theta,
                'kappa':self.kappa,
                's':self.s,
                'dkappa':self.dkappa,
                'ddkappa': self.ddkappa,
                'lane_id':self.lane_id,
                'x_derivative':self.x_derivative}

    def clear(self):
        self.x.clear()
        self.y.clear()
        self.z.clear()
        self.theta.clear()
        self.kappa.clear()
        self.s.clear()

class header:
    dict = []
    def __init__(self):
        self.name = 'header'     # 名称
       
        self.data_stamp = []
        self.module_name = []
 
        #print(" header init succesfullu ")
    
    def update(self,ADCTrajectory):
        self.data_stamp = ADCTrajectory.header.data_stamp
        header.dict = {'data_stamp':self.data_stamp}

class header1:
    dict = []
    def __init__(self):
        self.name = 'header'     # 名称
       
        self.data_stamp = []
        self.module_name = []
 
        #print(" header init succesfullu ")
    
    def update(self,Localization):
        self.data_stamp = Localization.header.data_stamp
        header1.dict = {'data_stamp':self.data_stamp}

class header2:
    dict = []
    def __init__(self):
        self.name = 'header'     # 名称
       
        self.data_stamp = []
        self.module_name = []
 
        #print(" header init succesfullu ")
    
    def update(self,Chassis):
        self.data_stamp = Chassis.header.data_stamp
        header2.dict = {'data_stamp':self.data_stamp}

class header3:
    dict = []
    def __init__(self):
        self.name = 'header'     # 名称
       
        self.data_stamp = []
        self.module_name = []
 
        #print(" header init succesfullu ")
    
    def update(self,Chassis):
        self.data_stamp = Chassis.header.data_stamp
        header3.dict = {'data_stamp':self.data_stamp}

class localization:
    list = []
    def __init__(self):
        self.name = 'pose'     # 名称
        self.position_x, self.position_y, self.position_z = [], [], []
        self.orientation_qx, self.orientation_qy, self.orientation_qz, self.orientation_qw = [], [], [], []
        self.linear_velocity_x ,self.linear_velocity_y, self.linear_velocity_z = [], [], []
        self.linear_acceleration_x, self.linear_acceleration_y,self.linear_acceleration_z = [], [], []
        self.angular_velocity_x = []
        self.angular_velocity_y = []
        self.angular_velocity_z = []
        self.heading = []
        self.linear_velocity_vrf_x = []
        self.linear_velocity_vrf_y = []
        self.linear_velocity_vrf_z = []
        self.linear_acceleration_vrf_x = []
        self.linear_acceleration_vrf_y = []
        self.linear_acceleration_vrf_z = []
        self.angular_velocity_vrf_x = []
        self.angular_velocity_vrf_y = []
        self.angular_velocity_vrf_z = []         
        self.euler_angles_x = []
        self.euler_angles_y = []
        self.euler_angles_z = []         
        self.wgs_x = []
        self.wgs_y = []
        self.wgs_z = []
        
        self.header = header1()
        self.start_time = None
        self.time = []
        
    def update(self,Localization):
        self.header.update(Localization)
        if self.start_time is None:
                self.start_time = self.header.data_stamp
        current_time = self.header.data_stamp - self.start_time
        self.time.append([current_time,self.header.data_stamp])
        self.position_x.append([current_time,Localization.pose.position.x])
        self.position_y.append([current_time,Localization.pose.position.y])
        self.position_z.append([current_time,Localization.pose.position.z])
        self.orientation_qx.append([current_time,Localization.pose.quaternion.qx])
        self.orientation_qy.append([current_time,Localization.pose.quaternion.qy])
        self.orientation_qz.append([current_time,Localization.pose.quaternion.qz])
        self.orientation_qw.append([current_time,Localization.pose.quaternion.qw])
        self.linear_velocity_x.append([current_time,Localization.pose.linear_velocity.x])
        self.linear_velocity_y.append([current_time,Localization.pose.linear_velocity.y])
        self.linear_velocity_z.append([current_time,Localization.pose.linear_velocity.z])
        self.linear_acceleration_x.append([current_time,Localization.pose.linear_acceleration.x])
        self.linear_acceleration_y.append([current_time,Localization.pose.linear_acceleration.y])
        self.linear_acceleration_z.append([current_time,Localization.pose.linear_acceleration.z])
        self.angular_velocity_x.append([current_time,Localization.pose.angular_velocity.x])
        self.angular_velocity_y.append([current_time,Localization.pose.angular_velocity.y])
        self.angular_velocity_z.append([current_time,Localization.pose.angular_velocity.z])
        self.heading.append([current_time,Localization.pose.heading])
        self.linear_velocity_vrf_x.append([current_time,Localization.pose.linear_velocity_vrf.x])
        self.linear_velocity_vrf_y.append([current_time,Localization.pose.linear_velocity_vrf.y])
        self.linear_velocity_vrf_z.append([current_time,Localization.pose.linear_velocity_vrf.z])
        self.linear_acceleration_vrf_x.append([current_time,Localization.pose.linear_acceleration_vrf.x])
        self.linear_acceleration_vrf_y.append([current_time,Localization.pose.linear_acceleration_vrf.y])
        self.linear_acceleration_vrf_z.append([current_time,Localization.pose.linear_acceleration_vrf.z])
        self.angular_velocity_vrf_x.append([current_time,Localization.pose.angular_velocity_vrf.x])
        self.angular_velocity_vrf_y.append([current_time,Localization.pose.angular_velocity_vrf.y])
        self.angular_velocity_vrf_z.append([current_time,Localization.pose.angular_velocity_vrf.z])
        self.euler_angles_x.append([current_time,Localization.pose.euler_angles.x])
        self.euler_angles_y.append([current_time,Localization.pose.euler_angles.y])
        self.euler_angles_z.append([current_time,Localization.pose.euler_angles.z])
        self.wgs_x.append([current_time,Localization.pose.wgs.x])
        self.wgs_y.append([current_time,Localization.pose.wgs.y])
        self.wgs_z.append([current_time,Localization.pose.wgs.z])

    def get_dict(self):
        return {
                'time':self.time,
                'position_x':self.position_x,
                'position_y':self.position_y,
                'position_z':self.position_z,
                'orientation_qx':self.orientation_qx,
                'orientation_qy':self.orientation_qy,
                'orientation_qz':self.orientation_qz,
                'orientation_qw':self.orientation_qw,
                'linear_velocity_x':self.linear_velocity_x,
                'linear_velocity_y':self.linear_velocity_y,
                'linear_velocity_z':self.linear_velocity_z,
                'linear_acceleration_x':self.linear_acceleration_x,
                'linear_acceleration_y':self.linear_acceleration_y,
                'linear_acceleration_z':self.linear_acceleration_z,
                'angular_velocity_x':self.angular_velocity_x,
                'angular_velocity_y':self.angular_velocity_y,
                'angular_velocity_z':self.angular_velocity_z,
                'heading':self.heading,
                'linear_velocity_vrf_x':self.linear_velocity_vrf_x,
                'linear_velocity_vrf_y':self.linear_velocity_vrf_y,
                'inear_velocity_vrf_z':self.linear_velocity_vrf_z,
                'linear_acceleration_vrf_x':self.linear_acceleration_vrf_x,
                'linear_acceleration_vrf_y':self.linear_acceleration_vrf_y,
                'linear_acceleration_vrf_z':self.linear_acceleration_vrf_z,
                'angular_velocity_vrf_x':self.angular_velocity_vrf_x,
                'angular_velocity_vrf_y':self.angular_velocity_vrf_y,
                'angular_velocity_vrf_z':self.angular_velocity_vrf_z,
                'euler_angles_x':self.euler_angles_x,
                'euler_angles_y':self.euler_angles_y,
                'euler_angles_z':self.euler_angles_z,
                'wgs_x':self.wgs_x,
                'wgs_y':self.wgs_y,
                'wgs_z':self.wgs_z}

class ChassisTest:
    list = []
    def __init__(self):
        self.name = 'chassis'
        self.DrivingMode = []
        self.ErrorCode = []
        self.GearPosition = []
        self.GearPosition = []
        self.engine_started = []
        self.engine_rpm = []
        self.speed_mps = []
        self.odometer_m = []
        self.fuel_range_m = []
        self.throttle_percentage = []
        self.brake_percentage = []
        self.steering_percentage = []
        self.steering_torque_nm = []
        self.parking_brake = []
        self.high_beam_signal = []
        self.low_beam_signal = []
        self.left_turn_signal = []
        self.right_turn_signal = []
        self.horn = []
        self.wiper = []
        self.disengage_status = []
        self.driving_mode = []
        self.error_code = []
        self.gear_location = []
        self.steering_timestamp = []
        self.chassis_error_mask = []
        self.battery_soc_percentage = []
        self.yaw_rate = []
        self.steering_angle = []
        self.steering_control_torque_nm = []
        
        self.header = header2()
        self.start_time = None
        self.time = []


    def update(self,Chassis):
        self.header.update(Chassis)
        if self.start_time is None:
                self.start_time = self.header.data_stamp
        current_time = self.header.data_stamp - self.start_time
        self.time.append([current_time,self.header.data_stamp])
        self.engine_started.append([current_time,Chassis.engine_started])
        self.engine_rpm.append([current_time,Chassis.engine_rpm])
        self.speed_mps.append([current_time,Chassis.speed_mps])
        self.odometer_m.append([current_time,Chassis.odometer_m])
        self.fuel_range_m.append([current_time,Chassis.fuel_range_m])
        self.throttle_percentage.append([current_time,Chassis.throttle_percentage])
        self.brake_percentage.append([current_time,Chassis.brake_percentage])
        self.steering_percentage.append([current_time,Chassis.steering_percentage])
        self.steering_torque_nm.append([current_time,Chassis.steering_torque_nm])
        self.parking_brake.append([current_time,Chassis.parking_brake])
        self.high_beam_signal.append([current_time,Chassis.high_beam_signal])
        self.low_beam_signal.append([current_time,Chassis.low_beam_signal])
        self.left_turn_signal.append([current_time,Chassis.left_turn_signal])
        self.right_turn_signal.append([current_time,Chassis.right_turn_signal])
        self.horn.append([current_time,Chassis.horn])
        self.wiper.append([current_time,Chassis.wiper])
        self.disengage_status.append([current_time,Chassis.disengage_status])
        self.driving_mode.append([current_time,(int)(Chassis.driving_mode)])
        self.error_code.append([current_time,(int)(Chassis.error_code)])
        self.gear_location.append([current_time,(int)(Chassis.gear_location)])
        self.steering_timestamp.append([current_time,Chassis.steering_timestamp])
        self.chassis_error_mask.append([current_time,Chassis.chassis_error_mask])
        self.battery_soc_percentage.append([current_time,Chassis.battery_soc_percentage])
        self.yaw_rate.append([current_time,Chassis.yaw_rate])
        self.steering_angle.append([current_time,Chassis.steering_angle])
        self.steering_control_torque_nm.append([current_time,Chassis.steering_control_torque_nm])

    def get_dict(self):
        return {
                'time':self.time,
                'engine_started':self.engine_started,
                'engine_rpm':self.engine_rpm,
                'speed':self.speed_mps,
                'odometer_m':self.odometer_m,
                'fuel_range_m':self.fuel_range_m,
                'throttle_percentage':self.throttle_percentage,
                'brake_percentage':self.brake_percentage,
                'steering_percentage':self.steering_percentage,
                'steering_torque_nm':self.steering_torque_nm,
                'parking_brake':self.parking_brake,
                'high_beam_signal':self.high_beam_signal,
                'low_beam_signal':self.low_beam_signal,
                'left_turn_signal':self.left_turn_signal,
                'right_turn_signal':self.right_turn_signal,
                'horn':self.horn,
                'wiper':self.wiper,
                'disengage_status':self.disengage_status,
                'driving_mode':self.driving_mode,
                'error_code':self.error_code,
                'gear_location':self.gear_location,
                'steering_timestamp':self.steering_timestamp,
                'chassis_error_mask':self.chassis_error_mask,
                'battery_soc_percentage':self.battery_soc_percentage,
                'yaw_rate':self.yaw_rate,
                'steering_angle':self.steering_angle,
                'steering_control_torque_nm':self.steering_control_torque_nm}