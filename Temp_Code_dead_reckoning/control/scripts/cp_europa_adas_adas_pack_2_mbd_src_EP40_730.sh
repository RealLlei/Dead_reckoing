#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..

RELEASE_PATH=$1
ADAS_PACK_PATH=$2
DESTINATION_PATH=$3



function cp_release_code(){
        cd ${RELEASE_PATH}/code/
        cp ChassisInfoCalc.* \
           CtrlDecisionData.c \
           Ctrl_Diagnose.* \
           Ctrl_PoseCalc.* \
           Diagnose_Table.* \
           EStOl.* \
           TLCtrl.* \
           TLCtrlData.* \
           LatiCtrlCalData.* \
           PoseCalc_Data.* \
           rtmodel.h ${DESTINATION_PATH}
}


function cp_release_com(){
        cd ${RELEASE_PATH}/com/
        cp AdasApp_EnumDef.h \
           adas_signal_def.* \
           adas_sys_mode_if.h \
           Adas_version_lable.h \
           ads_com.h \
           C2F_DistCal.* \
           Cart2Frnt.* \
           ctrl_common.* \
           RollingCntr_if.h \
           rte_enable_calib.h \
           rte_enable_calib_if.h \
           rtwtypes.h \
           SetAdas_SafetyMode.c \
           WdgMon_define.h \
           mcu_fault_report.* ${DESTINATION_PATH}
    
}


function cp_release_shared(){
        cd ${RELEASE_PATH}/shared/
        cp AdasApp_ConstDef.h \
           averageFunc_pWZeqTVf.* \
           BINARYSEARCH_real32_T.* \
           binsearch_u32df.* \
           binsearch_u32f.* \
           ButterWorthFilterCoef.h \
           Constant.h \
           CtrlConstant.h \
           CtrlDecisionData.h \
           CtrlDecision_Table.h \
           Ctrl_Type.h \
           DiagnoseParameters.h \
           Diagnose_Table.h \
           TLCtrlData.h \
           TLCtrl_Table.h \
           intrp1d_fu32fl_pw.* \
           LatiCtrlCalData.h \
           LatiCtrl_Table.h \
           look1_iflf_bingxpw.* \
           look1_iflf_binlcapw.* \
           look1_iflf_binlcpw.* \
           look1_iflf_binlxpw.* \
           look2_iflf_binlcapw.* \
           look2_iflf_binlcpw.* \
           look2_iflf_binlxpw.* \
           LookUp_real32_T_real32_T.* \
           LookUp_real_Treal32_T_real32_T.* \
           mldivide_zNRhXhcH.* \
           mod_u1Qzif9N.* \
           NumMin_WMCFCks4.* \
           plook_u32dfd_binc.* \
           plook_u32ff_binx.* \
           polyfit_0WNnu9k3.* \
           polyfit_LI1T31Ng.* \
           PoseCalc_struct_def.h \
           qrpf_0St1GiqK.* \
           qrpf_pPGXeM2T.* \
           rt_defines.h \
           rt_hypotf.* \
           rt_mrdivide_U1f3x2_U2f2x2_Yf3x2.* \
           rt_mrdivide_U1f4x3_U2f3x3_Yf4x3.* \
           rtwtypes.h \
           valueMax_15vug7y0.* \
           valueMin_F5c9sF2u.* \
           xnrm2_fEleR32B.* \
           xnrm2_SUJ9K2Q9.* \
           xzgetrf_IDUA3ACD.* \
           zero_crossing_types.h ${DESTINATION_PATH}
   
    
}

function cp_adas_pack(){
        cd ${ADAS_PACK_PATH}
        cp AppIntg_Pack/app/APP_ADF/ADF_Adas_Cfg.* \
        AppIntg_Pack/lib/dependency/memcfg/Arithmetic_Memmap.h \
        AppIntg_Pack/lib/dependency/memcfg/APPL_MemMap.* \
        AppIntg_Pack/lib/dependency/memcfg/AEB_MemMap.h \
        AppIntg_Pack/lib/dependency/INTF_IfCore1.h \
        AppIntg_Pack/lib/dependency/PlatformTypes.h \
        AppIntg_Pack/app/Adas/shared/binsearch_u32df.* \
        AppIntg_Pack/app/Adas/shared/Constant.h \
        AppIntg_Pack/app/Adas/shared/Ctrl_Type.h \
        AppIntg_Pack/app/Adas/shared/LatiCtrlCalData.h \
        AppIntg_Pack/app/Adas/shared/look1_iflf_binlxpw.* \
        AppIntg_Pack/app/Adas/shared/plook_u32dfd_binc.* \
        AppIntg_Pack/app/Adas/shared/PoseCalc_struct_def.h \
        AppIntg_Pack/app/APP_ADF/ADF_Adas_If.* \
        AppIntg_Pack/app/AEB_FCW/solver_zc.h \
        AppIntg_Pack/app/Adas/code/TrajCalcCalData.c ${DESTINATION_PATH}
}

function main(){
        echo "Please input release path, adas-pack path, and mbd_src path !!!"
        case "$4" in
                file) cp_release_code && cp_release_com && cp_release_shared && cp_adas_pack ;;
                *) cp_release_code && cp_release_com && cp_release_shared && cp_adas_pack ;;
        esac

        echo "cp C code success !!!"

        if [ ! -f "${DESTINATION_PATH}/CMakeLists.txt" ]; then
                echo "CAUTION: CMakeLists.txt does not exist in mbd_control/mbd_src. cp, waiting..."
                cp ${PROJECT_FOLDER}/control/scripts/CMakeLists.txt ${DESTINATION_PATH}
        fi
        echo "cp CMakeLists.txt success !!!"
    
}

main "$@"