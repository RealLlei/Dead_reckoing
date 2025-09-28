#ifndef LBS_OSE_H
#define LBS_OSE_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "lbs_ose_calculation.h"
#include "lbs_ose_ext.h"

/*****************************************************************************
        TYPEDEFS : INPUT   PARAMETER    OUTPUT    DEBUG
*****************************************************************************/

/*****************************************************************************
        VARIABLES
*****************************************************************************/

/*****************************************************************************
        FUNCTION PROTOTYPES
*****************************************************************************/
void OSE_PreProcess(const OSEInReq_t* reqPorts, const OSEParam_t* params,
                    OSEGlobal_t* OSEGlobal);

void OSE_MainProcess(const OSEInReq_t* reqPorts, const OSEParam_t* params,
                     OSEGlobal_t* pOSEGlobal);

void OSE_PostProcess(OSEOutPro_t* proPorts, OSEDebug_t* debug,
                     OSEGlobal_t* pOSEGlobal);

#ifdef __cplusplus
}
#endif

#endif
