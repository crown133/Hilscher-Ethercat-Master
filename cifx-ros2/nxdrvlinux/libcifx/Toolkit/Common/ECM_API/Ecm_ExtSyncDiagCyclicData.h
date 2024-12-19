/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: Ecm_ExtSyncDiagCyclicData.h 116445 2023-10-11 12:06:45Z SBormann $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef ECM_EXTSYNCDIAGCYCLICDATA_H
#define ECM_EXTSYNCDIAGCYCLICDATA_H

#include <Hil_Compiler.h>

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_PACK_1(ECM_EXTSYNCDIAGCYCLICDATA)
#endif

/*! Ext Sync Cyclic Diagnostic Data */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_EXT_SYNC_DIAG_CYCLIC_DATA_Ttag
{
  uint32_t ulExtSyncInfoFlags; /*!< Status flags of ExtSync logic */
  uint16_t usExtSyncStationAddress; /*!< Fixed station address of slave providing ExtSync data via PDO 0x10F4. */
  uint16_t usControlledStationAddress; /*!< Fixed station address of DC reference clock */
  uint64_t ullDcToExtTimeOffsetNs; /*!< Time difference between this network and the other network providing the ExtSync base time.
  * Relationship of timestamps: DcToExtTimeOffsetNs = ExternalTimestampNs - InternalTimestampNs */
  uint32_t ulDcExtErrorDiffNsSignMag; /*!< Sign magnitude field to show the actual deviation between external synchronization reference time and own DcSysTime */
  uint32_t ulExtSyncUpdateCount; /*!< Counter incrementing every time when ExtSync data is processed */
} ECM_EXT_SYNC_DIAG_CYLIC_DATA_T;

/*! Definition of ulExtSyncInfoFlags
 */
typedef enum ECM_EXT_SYNC_INFO_FLAGS_Etag
{
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_SYNC_MODE_SLAVE = 0x00000001, /*!< The master is using ExtSync reference provided by other network */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_IS_64BIT = 0x00000004, /*!< ExtSync timestamp size
  * If not set, the timestamp has 32 bit.
  * If set, the timestamp has 64 bit */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_NOT_CONNECTED = 0x00000010, /*!< ExtSync device is not connected when this bit is set */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_SYNC_MODE_MASTER = 0x00000020, /*!< The master is providing ExtSync to other network */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONNECTED_AS_SLAVE = 0x00008000, /*!< Ext Sync Device is connected as slave */ /* result of !(External Device Not Connected) && (Sync Mode.Bit 1) */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 0x00FF0000, /*!< Used internally */
  SRT_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 16, /*!< Used internally */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_DC_TO_EXT_OFFSET_VALID = 0x20000000, /*!< ullDcToExtTimeOffsetNs is valid */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_ACTIVE = 0x40000000, /*!< The master is actively using External synchronization on device */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONFIGURED = 0x80000000 /*!< Ext Sync device has been configured */
} ECM_EXT_SYNC_INFO_FLAGS_E;

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_UNPACK_1(ECM_EXTSYNCDIAGCYCLICDATA)
#endif

#endif
