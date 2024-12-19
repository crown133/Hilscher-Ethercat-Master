/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_Slave_Diag.h 116433 2023-10-11 11:36:30Z SBormann $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef __STRUCT_ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T__
#define __STRUCT_ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T__

#include <Hil_Compiler.h>

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_PACK_1(ECMIF_SLAVE_DIAG)
#endif

/*! Slave diagnostic information
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_Ttag
{
  uint32_t ulStationAddress; /*!< Station address */
  uint32_t ulAutoIncAddress; /*!< Known auto-inc address */
  uint32_t ulCurrentState; /*!< Current state */
  uint32_t ulLastError; /*!< Last error */
  uint8_t  szSlaveName[80]; /*!< Name of slave from configuration */
  uint32_t fEmergencyReported; /*!< Is an emergency reported */
} ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T;

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_UNPACK_1(ECMIF_SLAVE_DIAG)
#endif

#endif
