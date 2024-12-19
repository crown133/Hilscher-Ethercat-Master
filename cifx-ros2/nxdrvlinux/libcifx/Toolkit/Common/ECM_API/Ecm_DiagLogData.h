/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: Ecm_DiagLogData.h 116445 2023-10-11 12:06:45Z SBormann $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef _ECM_DIAGLOGDATA_H
#define _ECM_DIAGLOGDATA_H

/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_PACK_1(ECM_DIAGLOGDATA)
#endif

/******************************************************************************
 * Diagnostic Entry Structure
 */

/*! Diagnostic Entry Structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_HEADER_Ttag
{
  uint16_t usEntryType; /*!< Diagnostic entry type */
  uint64_t ullTimestampNs; /*!< Timestamp in ns */
} ECM_DIAG_ENTRY_HEADER_T;

/*! Diagnostic Entry Types
 *
 * Entry Type: 0x0000 is reserved for no data
 */
typedef enum ECM_DIAG_ENTRY_TYPE_Etag
{
  VAL_ECM_DIAG_ENTRY_TYPE_NEW_STATE                        = 0x0001, /*!< New State reached */
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_ON                           = 0x0002, /*!< Bus On */
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_OFF                          = 0x0003, /*!< Bus Off */
  VAL_ECM_DIAG_ENTRY_TYPE_CHANNEL_INIT                     = 0x0004, /*!< Channel Init */
  VAL_ECM_DIAG_ENTRY_TYPE_DPM_WATCHDOG                     = 0x0005, /*!< DPM Watchdog */
  VAL_ECM_DIAG_ENTRY_TYPE_TOPOLOGY_CHANGED                 = 0x0006, /*!< Topology Changed */
  VAL_ECM_DIAG_ENTRY_TYPE_INTERNAL_ERROR                   = 0x0007, /*!< Internal Error */
  VAL_ECM_DIAG_ENTRY_TYPE_ALL_SLAVES_LOST                  = 0x0008, /*!< All Slaves Lost */
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_SCAN_REQUESTED               = 0x0009, /*!< Bus Scan Requested */
  VAL_ECM_DIAG_ENTRY_TYPE_IDENTITY_MISMATCH                = 0x000A, /*!< Identity Mismatch */
  VAL_ECM_DIAG_ENTRY_TYPE_COE_INITCMD_FAILED               = 0x000B, /*!< CoE InitCmd failed */
  VAL_ECM_DIAG_ENTRY_TYPE_SOE_INITCMD_FAILED               = 0x000C, /*!< SoE InitCmd failed */
  VAL_ECM_DIAG_ENTRY_TYPE_EOE_INITCMD_FAILED               = 0x000D, /*!< EoE InitCmd failed */
  VAL_ECM_DIAG_ENTRY_TYPE_AOE_INITCMD_FAILED               = 0x000E, /*!< AoE InitCmd failed */
  VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_WARNING              = 0x000F, /*!< Reg InitCmd warning */
  VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_FAILED               = 0x0010, /*!< Reg InitCmd Failed */
  VAL_ECM_DIAG_ENTRY_TYPE_ALCONTROL_FAILED                 = 0x0011, /*!< ALControl failed */
  VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_ECAT_FAILED        = 0x0012, /*!< SII Assign To ECAT failed */
  VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_PDI_FAILED         = 0x0013, /*!< SII Assign to PDI failed */
  VAL_ECM_DIAG_ENTRY_TYPE_SII_READ_REQUEST_FAILED          = 0x0014, /*!< SII Read Request failed */
  VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_WARNING                    = 0x0015, /*!< Slave Warning */
  VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_ERROR                      = 0x0016, /*!< Slave Error */
  VAL_ECM_DIAG_ENTRY_TYPE_VOE_INITCMD_FAILED               = 0x0017, /*!< VoE InitCmd failed */
} ECM_DIAG_ENTRY_TYPE_E;

/*! Data for New State entry
 * used by type:
 * - VAL_ECM_DIAG_ENTRY_TYPE_NEW_STATE
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_NEW_STATE_Ttag
{
  uint8_t bState; /*!< State */
} ECM_DIAG_ENTRY_NEW_STATE_T;


/*! Data for Internal Error
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_INTERNAL_ERROR
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_INTERNAL_ERROR_Ttag
{
  uint32_t ulFunctionId; /*!< Function id */
  uint32_t ulErrorCode; /*!< Error code */
} ECM_DIAG_ENTRY_INTERNAL_ERROR_T;


/*! Data for Identity Mismatch
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_IDENTITY_MISMATCH
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_IDENTITY_MISMATCH_Ttag
{
  uint16_t usTopologyPosition; /*!< Topology position */
  uint16_t usCompareFlags; /*!< Compare flags */
  uint32_t ulExpectedVendorId; /*!< Expected Vendor Id */
  uint32_t ulExpectedProductCode; /*!< Expected Product Code */
  uint32_t ulExpectedRevisionNo; /*!< Expected Revision Number */
  uint32_t ulExpectedSerialNo; /*!< Expected Serial Number */
  uint32_t ulFoundVendorId; /*!< Found Vendor Id */
  uint32_t ulFoundProductCode; /*!< Found Product Code */
  uint32_t ulFoundRevisionNo; /*!< Found Revision Number */
  uint32_t ulFoundSerialNo; /*!< Found Serial Number */
} ECM_DIAG_ENTRY_IDENTITY_MISMATCH_T;

/*! Identity Compare Flags
 *
 */
typedef enum ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_FLAGS_Etag
{
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_VENDOR_ID = 0x0001, /*!< Identity mismatch on Vendor Id  */
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_PRODUCT_CODE = 0x0002, /*!< Identity mismatch on Product Code */
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_REVISION_NO = 0x0004, /*!< Identity mismatch on Revision Number */
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_SERIAL_NO = 0x0008, /*!< Identity mismatch on Serial Number */
} ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_FLAGS_E;


/*! Diag Entry for CoE InitCmd Failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_COE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_COE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usIndex; /*!< Object index */
  uint8_t bSubIndex; /*!< Sub index */
  uint8_t bAction; /*!< CoE InitCmd Action */
  uint16_t fIsCompleteAccess; /*!< != 0 if complete access */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_COE_INITCMD_FAILED_T;

/*! Diag Entry for SoE InitCmd Failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SOE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bDriveNo; /*!< Drive number */
  uint16_t usIDN; /*!< IDN number */
  uint8_t bElements; /*!< SoE element flags */
  uint8_t bAction; /*!< SoE InitCmd Action */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_T;

/*! Diag Entry for VoE InitCmd failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_VOE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_T;

/*! Diag Entry for Reg InitCmd Warning/Failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_WARNING
 * - VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_REG_INITCMD_INFO_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bCmd; /*!< Telegram command */
  uint16_t usAdo; /*!< Register offset */
  uint16_t usLength; /*!< Access length */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_REG_INITCMD_INFO_T;

/*! Diag Entry for ALControl failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_ALCONTROL_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_ALCONTROL_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bTargetState; /*!< Requested target state */
  uint16_t usAlStatusCode; /*!< ALStatus code */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_ALCONTROL_FAILED_T;

/*! Diag Entry for SII Assign to
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_ECAT_FAILED
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_PDI_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_T;

/*!< Diag Entry for SII Read Request failed
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_READ_REQUEST_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SII_REQUEST_FAILED_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulSiiWordOffset; /*!< SII word offset */
  uint32_t ulResult; /*!< Error code */
} ECM_DIAG_ENTRY_SII_REQUEST_FAILED_T;

/*! Diag Entry for Slave Warning
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_WARNING
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SLAVE_WARNING_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulWarningType; /*!< Warning type */
  uint32_t ulWarningParam; /*!< value depends on ulWarningType */
} ECM_DIAG_ENTRY_SLAVE_WARNING_T;

/*!< Slave Warning Types */
typedef enum ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_Etag
{
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_ADVERTISED_64BIT_DC_NOT_WORKING = 0x00000001, /*!< Slave advertises 64Bit DC but is only 32bit */
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_ADVERTISED_DC_NOT_WORKING = 0x00000002, /*!< Slave advertises DC but is not working */
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_SLAVE_DID_NOT_ACCEPT_EOE_SET_IP_PARAMS = 0x00000003, /*!< ulWarningParam contains ECM_ERROR_CODE_E */
} ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_E;

/*! Diag Entry for Slave Error
 * used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_ERROR
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SLAVE_ERROR_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulErrorType; /*!< Error type */
  uint32_t ulErrorParam; /* value depends on ulErrorType */
} ECM_DIAG_ENTRY_SLAVE_ERROR_T;

/*!< Slave Error Types */
typedef enum ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_Etag
{
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_SYNC_NOT_POSSIBLE_WITHOUT_WORKING_DC = 0x00000001, /*!< Sync not possible without working DC */
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_SYNC_LATCHING_ERROR = 0x00000002, /*!< Latching error, ulErrorParam contains ECM_ERROR_CODE_E */
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_STATE_CHANGE_ENCOUNTERED = 0x00000003, /*!< State change encountered, ulErrorParam contains ECM_ERROR_CODE_E */
} ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_E;

/*! Diag Entry Data */
typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_DIAG_ENTRY_DATA_Ttag
{
  ECM_DIAG_ENTRY_NEW_STATE_T tNewState; /*!< New state */
  ECM_DIAG_ENTRY_INTERNAL_ERROR_T tInternalError; /*!< Internal error */
  ECM_DIAG_ENTRY_IDENTITY_MISMATCH_T tIdentityMismatch; /*!< Identity mismatch */
  ECM_DIAG_ENTRY_COE_INITCMD_FAILED_T tCoEInitCmdFailed; /*!< CoE InitCmd failed */
  ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_T tSoEInitCmdFailed; /*!< SoE InitCmd failed */
  ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_T tVoEInitCmdFailed; /*!< VoE InitCmd failed */
  ECM_DIAG_ENTRY_REG_INITCMD_INFO_T tRegInitCmdInfo; /*!< Reg InitCmd info */
  ECM_DIAG_ENTRY_ALCONTROL_FAILED_T tAlControlFailed; /*!< AlControl failed */
  ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_T tSiiAssignFailed; /*!< SII Assign failed */
  ECM_DIAG_ENTRY_SII_REQUEST_FAILED_T tSiiRequestFailed; /*!< SII Request failed */
  ECM_DIAG_ENTRY_SLAVE_WARNING_T tSlaveWarning; /*!< Slave warning */
  ECM_DIAG_ENTRY_SLAVE_ERROR_T tSlaveError; /*!< Slave error */
} ECM_DIAG_ENTRY_DATA_T;

/*! Diag Entry */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_Ttag
{
  ECM_DIAG_ENTRY_HEADER_T tHead; /*!< Diag entry header */
  ECM_DIAG_ENTRY_DATA_T tData; /*!< Diag entry data */
} ECM_DIAG_ENTRY_T;

/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_UNPACK_1(ECM_DIAGLOGDATA)
#endif

#endif
