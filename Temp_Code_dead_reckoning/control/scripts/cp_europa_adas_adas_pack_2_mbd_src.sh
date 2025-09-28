#!/usr/bin/env bash
PROJECT_FOLDER=$(cd "$(dirname ""$0"")"; pwd)/../..

RELEASE_PATH=$1
ADAS_PACK_PATH=$2
DESTINATION_PATH=$3



function cp_release_code(){
        cd ${RELEASE_PATH}/code/
        cp TLCtrl.* \
           LatiCtrlData.* \
           LongiCtrlData.* \
           LatiCtrlCalData.* \
           LongiCtrlActData.* \
           rtmodel.h ${DESTINATION_PATH}
}


function cp_release_com(){
        cd ${RELEASE_PATH}/com/
        cp AdasApp_EnumDef.h \
           adas_signal_def.* \
           Adas_version_lable.h \
           ads_com.h \
           C2F_DistCal.* \
           Cart2Frnt.* \
           mcu_fault_report.* ${DESTINATION_PATH}
    
}


function cp_release_shared(){
        cd ${RELEASE_PATH}/shared/
        cp rt_mrdivide_U1f3x2_U2f2x2_Yf3x2.* \
           rt_mrdivide_U1f4x3_U2f3x3_Yf4x3.* \
           AdasApp_ConstDef.h \
           averageFunc_pWZeqTVf.* \
           BINARYSEARCH_real32_T.* \
           binsearch_u32df.* \
           binsearch_u32f.* \
           ButterWorthFilterCoef.h \
           Constant.h \
           Ctrl_Type.h \
           CtrlConstant.h \
           intrp1d_fu32fl_pw.* \
           LatiCtrl_Table.h \
           LatiCtrlCalData.h \
           LatiCtrlData.h \
           LongiCtrl_Table.h \
           LongiCtrlAct_Table.h \
           LongiCtrlActData.h \
           LongiCtrlData.h \
           look1_iflf_bingxpw.* \
           look1_iflf_binlcapw.* \
           look1_iflf_binlcpw.* \
           look1_iflf_binlxpw.* \
           look2_iflf_binlcapw.* \
           look2_iflf_binlcpw.* \
           look2_iflf_binlxpw.* \
           LookUp_real_Treal32_T_real32_T.* \
           LookUp_real32_T_real32_T.* \
           mldivide_zNRhXhcH.* \
           mod_u1Qzif9N.* \
           NumMin_WMCFCks4.* \
           plook_u32dfd_binc.* \
           plook_u32ff_binx.* \
           polyfit_0WNnu9k3.* \
           polyfit_LI1T31Ng.* \
           qrpf_0St1GiqK.* \
           qrpf_pPGXeM2T.* \
           rt_defines.h \
           rt_hypotf.* \
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
        AppIntg_Pack/lib/dependency/INTF_IfCore1.h \
        AppIntg_Pack/lib/dependency/PlatformTypes.h \
        AppIntg_Pack/app/Adas/shared/polyfit_JPIL9moA.* \
        AppIntg_Pack/app/Adas/shared/polyfit_kjy2NzM3.* \
        AppIntg_Pack/app/Adas/shared/qrpf_i1WVVbnh.* \
        AppIntg_Pack/app/Adas/shared/qrpf_ohZNTKsN.* \
        AppIntg_Pack/app/Adas/shared/rt_mrdivide_U1d3x2_U2d2x2_Yd3x2.* \
        AppIntg_Pack/app/Adas/shared/rt_mrdivide_U1d4x3_U2d3x3_Yd4x3.* \
        AppIntg_Pack/app/AEB_FCW/solver_zc.h \
        AppIntg_Pack/app/Adas/code/TrajCalcCalData.c \
        AppIntg_Pack/app/Adas/shared/TrajCalcCalData.h \
        AppIntg_Pack/app/Adas/shared/xnrm2_hN3ev20W.* \
        AppIntg_Pack/app/Adas/shared/const_params.* \
        AppIntg_Pack/app/Adas/shared/xnrm2_X0nQz2lz.* ${DESTINATION_PATH}
}

function cp_release_all(){
        cd ${RELEASE_PATH}/
        cp code/*.h code/*.c com/*.h com/*.c shared/*.h shared/*.c ${DESTINATION_PATH}

}

function cp_adas_pack_all(){
        cd ${ADAS_PACK_PATH}
        cp AppIntg_Pack/app/APP_ADF/*.h \
        AppIntg_Pack/app/APP_ADF/*.c \
        AppIntg_Pack/lib/dependency/*.h \
        AppIntg_Pack/lib/dependency/*.c \
        AppIntg_Pack/app/Adas/shared/*.h \
        AppIntg_Pack/app/Adas/shared/*.c \
        AppIntg_Pack/app/AEB_FCW/*.h \
        AppIntg_Pack/app/AEB_FCW/*.c \
        AppIntg_Pack/app/Adas/code/*.h \
        AppIntg_Pack/app/Adas/code/*.c \
        AppIntg_Pack/app/Adas/shared/*.h \
        AppIntg_Pack/app/Adas/shared/*.c ${DESTINATION_PATH}
}

function cp_cmakelist_WdgM_2_mbd_src(){
        cd ${ADAS_PACK_PATH}

}

function main(){
        echo "Please input release path, adas-pack path, and mbd_src path !!!"
        case "$4" in
                all) cp_release_all && cp_adas_pack_all;;
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