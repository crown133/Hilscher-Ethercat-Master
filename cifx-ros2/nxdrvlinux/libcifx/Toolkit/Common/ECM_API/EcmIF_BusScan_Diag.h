/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_BusScan_Diag.h 116433 2023-10-11 11:36:30Z SBormann $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef __ECM_BUS_SCAN_INFO_T__
#define __ECM_BUS_SCAN_INFO_T__

#include <stdint.h>
#include <Hil_Compiler.h>

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_PACK_1(ECMIF_BUSSCAN_DIAG)
#endif

/* this structure is deliberately compatible with ECM v3.x */

/*! Bus Scan Info
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_BUS_SCAN_INFO_Ttag
{
  uint32_t ulVendorId; /*!< Vendor id */
  uint32_t ulProductCode; /*!< Product code */
  uint32_t ulRevisionNumber; /*!< Revision number */
  uint32_t ulSerialNumber; /*!< Serial number */
  uint32_t ulPortState; /*!< Port state */
} ETHERCAT_MASTER_BUS_SCAN_INFO_T;

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_UNPACK_1(ECMIF_BUSSCAN_DIAG)
#endif

#endif
