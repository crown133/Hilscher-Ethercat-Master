/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_Public.h 120087 2024-04-30 13:18:24Z SBormann $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef ECM_IF_PUBLIC_H_INCLUDED
#define ECM_IF_PUBLIC_H_INCLUDED

#include <stdint.h>
#include <Hil_Packet.h>

#include "Ecm_DiagLogData.h"

/***************************************************************************************/

#define ECM_IF_PROCESS_QUEUE_NAME       "QUE_ECM_IF"


/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_PACK_1(ECM_IF_PUBLIC)
#endif

// ECM_IF_COMMAND_START                         = 0x00009E00,

/******************************************************************************/
/*! Physical slave address
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_ADDR_PHYS_Ttag
{
  uint16_t usAdp; /*!< Station address */
  uint16_t usAdo; /*!< Register address */
} ECM_IF_SLAVE_ADDR_PHYS_T;

/*! Slave addressing
 *
 * It is used to describe both physical addressing and logical addressing.
 */
typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_SLAVE_ADDR_Ttag
{
  ECM_IF_SLAVE_ADDR_PHYS_T tPhys; /*!< Physical addressing */
  uint32_t ulLogAddress; /*!< Logical addressing */
} ECM_IF_SLAVE_ADDR_T;

/******************************************************************************/
/* ulMasterStatusFlags */
/*! Master Status Flags
 *
 */
enum ECM_IF_MASTER_STATUS_FLAGS_Etag
{
  MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_LOST = 0x00000001, /*!< At least one mandatory slave lost */
  MSK_ECM_IF_MASTER_STATUS_FLAGS_DC_XRMW_STOPPED = 0x00000002, /*!< DC ARMW/FRMW is stopped */
  MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_NOT_IN_OP = 0x00000004, /*!< signaled when Master in OP and required slave is not in OP */
  MSK_ECM_IF_MASTER_STATUS_FLAGS_DC_RESYNC_RETRIGGER_ACTIVE = 0x00000008, /*!< DC Resync Retrigger active */
  MSK_ECM_IF_MASTER_STATUS_FLAGS_DC_SETUP_NOT_IN_SYNC = 0x00000010, /*!< DC Setup not in sync (should never stay for longer time) */
};

/******************************************************************************/
/* bCmd */
/*! EtherCAT telegram command
 *
 */
typedef enum ECM_IF_CMD_TYPE_Etag
{
  ECM_IF_COMMAND_NOP = 0,  /*!< NOP command */
  ECM_IF_COMMAND_APRD = 1, /*!< APRD command */
  ECM_IF_COMMAND_APWR = 2, /*!< APWR command */
  ECM_IF_COMMAND_APRW = 3, /*!< APRW command */
  ECM_IF_COMMAND_FPRD = 4, /*!< FPRD command */
  ECM_IF_COMMAND_FPWR = 5, /*!< FPWR command */
  ECM_IF_COMMAND_FPRW = 6, /*!< FPRW command */
  ECM_IF_COMMAND_BRD = 7,  /*!< BRD command */
  ECM_IF_COMMAND_BWR = 8,  /*!< BWR command */
  ECM_IF_COMMAND_BRW = 9,  /*!< BRW command */
  ECM_IF_COMMAND_LRD = 10, /*!< LRD command */
  ECM_IF_COMMAND_LWR = 11, /*!< LWR command */
  ECM_IF_COMMAND_LRW = 12, /*!< LRW command */
  ECM_IF_COMMAND_ARMW = 13,/*!< ARMW command */
  ECM_IF_COMMAND_FRMW = 14 /*!< FRMW command */
} ECM_IF_CMD_TYPE_E;

/**************************************************************************************************

#######   #####    #####         #   #####   ###  ###           #      #####    #####   #######   #####    #####
#        #     #  #     #       #   #     #   #    #           # #    #     #  #     #  #        #     #  #     #
#        #        #            #    #         #    #          #   #   #        #        #        #        #
#####     #####   #           #      #####    #    #         #     #  #        #        #####     #####    #####
#              #  #          #            #   #    #         #######  #        #        #              #        #
#        #     #  #     #   #       #     #   #    #         #     #  #     #  #     #  #        #     #  #     #
#######   #####    #####   #         #####   ###  ###        #     #   #####    #####   #######   #####    #####

 */

#define ECM_IF_CMD_READ_REGS_REQ 0x9E70 /*!< Read Register Request */
#define ECM_IF_CMD_READ_REGS_CNF 0x9E71 /*!< Read Register Confirmation */

#define ECM_IF_CMD_WRITE_REGS_REQ 0x9E72 /*!< Write Register Request */
#define ECM_IF_CMD_WRITE_REGS_CNF 0x9E73 /*!< Write Register Confirmation */

#define ECM_IF_CMD_READ_SII_REQ 0x9E80 /*!< Read SII Request */
#define ECM_IF_CMD_READ_SII_CNF 0x9E81 /*!< Read SII Confirmation */

#define ECM_IF_CMD_WRITE_SII_REQ 0x9E82 /*!< Write SII Request */
#define ECM_IF_CMD_WRITE_SII_CNF 0x9E83 /*!< Write SII Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_READ_REGS_REQ/ECM_IF_CMD_READ_REGS_CNF
 */

/* request packet */
/*! Read Register Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usPhysAddr; /*!< Physical address */
  uint16_t usPhysLength; /*!< Physical length */
} ECM_IF_READ_REGS_REQ_DATA_T;

/*! Read Register Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_REGS_REQ, ECM_IF_CMD_READ_REGS_REQ>>
 * \param ulLen 6
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_REGS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_REGS_REQ_T;


/* confirmation packet */
/*! Read Register Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usPhysAddr; /*!< Physical address */
  uint16_t usPhysLength; /*!< Physical length */
  uint8_t abData[1024]; /*!< Read data */
} ECM_IF_READ_REGS_CNF_DATA_T;

/*! Read Register Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_REGS_CNF, ECM_IF_CMD_READ_REGS_CNF>>
 * \param ulLen 6+usPhysLength
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_REGS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_REGS_CNF_T;


/* packet union */
typedef union ECM_IF_READ_REGS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_REGS_REQ_T tReq;
  ECM_IF_READ_REGS_CNF_T tCnf;
} ECM_IF_READ_REGS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_WRITE_REGS_REQ/ECM_IF_CMD_WRITE_REGS_CNF
 */

/* request packet */
/** Write Register Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usPhysAddr; /*!< Physical address */
  uint16_t usPhysLength; /*!< Physical length */
  uint8_t abData[1024]; /*! Write data */
} ECM_IF_WRITE_REGS_REQ_DATA_T;

/*! Write Register Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_WRITE_REGS_REQ, ECM_IF_CMD_WRITE_REGS_REQ>>
 * \param ulLen 6+usPhysLength
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_WRITE_REGS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_WRITE_REGS_REQ_T;


/* confirmation packet */
/** Write Register Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usPhysAddr; /*!< Physical address */
  uint16_t usPhysLength; /*!< Physical length */
} ECM_IF_WRITE_REGS_CNF_DATA_T;

/*! Write Register Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_WRITE_REGS_CNF, ECM_IF_CMD_WRITE_REGS_CNF>>
 * \param ulLen 6
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_WRITE_REGS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_WRITE_REGS_CNF_T;


/* packet union */
typedef union ECM_IF_WRITE_REGS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_REGS_REQ_T tReq;
  ECM_IF_WRITE_REGS_CNF_T tCnf;
} ECM_IF_WRITE_REGS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_READ_SII_REQ/ECM_IF_CMD_READ_SII_CNF
 */

/* request packet */
/*! Read SII Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulSiiWordOffset; /*!< SII word offset */
  uint32_t ulSiiByteLength; /*!< access length dividable by 2 */
} ECM_IF_READ_SII_REQ_DATA_T;

/*! Read SII Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_SII_REQ, ECM_IF_CMD_READ_SII_REQ>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_SII_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_SII_REQ_T;


/* confirmation packet */
/*! Read SII Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulSiiWordOffset; /*!< SII word offset */
  uint32_t ulSiiByteLength; /*!< access length dividable by 2 */
  uint8_t abData[1024]; /*!< Read data */
} ECM_IF_READ_SII_CNF_DATA_T;

/*! Read SII Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_SII_CNF, ECM_IF_CMD_READ_SII_CNF>>
 * \param ulLen 10+ulSiiByteLength
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_SII_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_SII_CNF_T;


/* packet union */
typedef union ECM_IF_READ_SII_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_SII_REQ_T tReq;
  ECM_IF_READ_SII_CNF_T tCnf;
} ECM_IF_READ_SII_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_WRITE_SII_REQ/ECM_IF_CMD_WRITE_SII_CNF
 */

/* request packet */
/*! Write SII Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulSiiWordOffset; /*!< SII word offset */
  uint32_t ulReserved; /*!< reserved */ /* kept free for specific use in confirmation */
  uint8_t abData[1024]; /*!< Write data. actual length is defined by ulLen - 8 */
} ECM_IF_WRITE_SII_REQ_DATA_T;

/*! Write SII Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_WRITE_SII_REQ, ECM_IF_CMD_WRITE_SII_REQ>>
 * \param ulLen 10+actual byte length in abData
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_WRITE_SII_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_WRITE_SII_REQ_T;


/* confirmation packet */
/*! Write SII Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulSiiWordOffset; /*!< SII word offset */
  uint32_t ulWrittenByteLength; /*!< actual written byte length */
} ECM_IF_WRITE_SII_CNF_DATA_T;

/*! Write SII Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_WRITE_SII_CNF, ECM_IF_CMD_WRITE_SII_CNF>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_WRITE_SII_CNF_DATA_T tData; /*!< Packet data */
}  ECM_IF_WRITE_SII_CNF_T;


/* packet union */
typedef union ECM_IF_WRITE_SII_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_SII_REQ_T tReq;
  ECM_IF_WRITE_SII_CNF_T tCnf;
} ECM_IF_WRITE_SII_PCK_T;


/**********************************************************************************************************************

 #####   #######     #     #######  #######         #####   #######  #     #  #######  ######   #######  #
#     #     #       # #       #     #              #     #  #     #  ##    #     #     #     #  #     #  #
#           #      #   #      #     #              #        #     #  # #   #     #     #     #  #     #  #
 #####      #     #     #     #     #####          #        #     #  #  #  #     #     ######   #     #  #
      #     #     #######     #     #              #        #     #  #   # #     #     #   #    #     #  #
#     #     #     #     #     #     #              #     #  #     #  #    ##     #     #    #   #     #  #
 #####      #     #     #     #     #######         #####   #######  #     #     #     #     #  #######  #######

 */

/*! State Enumeration
 *
 */
enum EMC_IF_STATE_Etag
{
  ECM_IF_STATE_BUSOFF = 0, /*!< Bus off */
  ECM_IF_STATE_INIT = 1, /*!< INIT */
  ECM_IF_STATE_PREOP = 2, /*!< PREOP */
  ECM_IF_STATE_BOOT = 3, /*!< BOOT */
  ECM_IF_STATE_SAFEOP = 4, /*!< SAFEOP */
  ECM_IF_STATE_OP = 8, /*!< OP */

  /* Master States */
  ECM_IF_STATE_LEAVE_OP = 0x18, /*!< Master is leaving OP state */
  ECM_IF_STATE_MANUAL_MODE = 0x1C, /*!< Manual Mode */
  ECM_IF_STATE_BUSSCAN_COMPLETE_NO_PREOP = 0x1D, /*!< Bus Scan Complete. PREOP not used */
  ECM_IF_STATE_BUSSCAN = 0x1E, /*!< Bus Scan in progress */
  ECM_IF_STATE_BUSSCAN_COMPLETE = 0x1F, /*!< Bus Scan Complete. PREOP accessible */

  /* Slave Error States */
  ECM_IF_STATE_NOT_CONNECTED = 0x00, /*! Slave is not participating in active bus communication */
  ECM_IF_STATE_INIT_ERR = 0x11, /*!< Slave is in INIT+ERR */
  ECM_IF_STATE_PREOP_ERR = 0x12,  /*!< Slave is in PREOP+ERR */
  ECM_IF_STATE_BOOT_ERR = 0x13, /*!< Slave is in BOOT+ERR */
  ECM_IF_STATE_SAFEOP_ERR = 0x14, /*!< Slave is in SAFEOP+ERR */
  ECM_IF_STATE_UNKNOWN = 0xFF, /*!< Slave state is unknown due to Master being Bus off */
};

/******************************************************************************/

#define ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ            0x9E00 /*!< Set Master Target State Request */
#define ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF            0x9E01 /*!< Set Master Target State Confirmation */

#define ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ           0x9E02 /*!< Get Master Current State Request */
#define ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF           0x9E03 /*!< Get Master Current State Confirmation */

#define ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ             0x9E04 /*!< Set Slave Target State Request */
#define ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF             0x9E05 /*!< Set Slave Target State Confirmation */

#define ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ            0x9E06 /*!< Get Slave Current State Request */
#define ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF            0x9E07 /*!< Get Slave Current State Confirmation */

#define ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ              0x9E08 /*!< Get Master State Diag Request */
#define ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF              0x9E09 /*!< Get Master State Diag Confirmation */

#define ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ                0x9E0A /*!< Manual Mode Control Request */
#define ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF                0x9E0B /*!< Manual Mode Control Confirmation */

#define ECM_IF_CMD_EXT_START_BUSSCAN_REQ                  0x9E0E /*!< Extended Start Bus Scan Request */
#define ECM_IF_CMD_EXT_START_BUSSCAN_CNF                  0x9E0F /*!< Extended Start Bus Scan Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ/ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF
 */

/* request packet */
/*! Set Master Target State Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_Ttag
{
  uint8_t bTargetState; /*!< Target State */
} ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_T;

/*! Set Master Target State Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ, ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_MASTER_TARGET_STATE_REQ_T;


/* confirmation packet */
/*! Set Master Target State Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_Ttag
{
  uint8_t bTargetState; /*!< Target state */
} ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_T;

/*! Set Master Target State Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF, ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_MASTER_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_MASTER_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_MASTER_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_MASTER_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_MASTER_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ/ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF
 */

/* request packet */
/*! Get Master Current State Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ, ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T;


/* confirmation packet */
/*! Get Master Current State Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_Ttag
{
  uint8_t bCurrentState; /*!< Current state of master */
  uint8_t bTargetState; /*!< Target state of master */
  uint32_t ulStopReason; /*!< If this value equals 0, the state change is either progressing or successful.
  * If this value is unequal to 0, the last state change has been aborted. */
  uint32_t ulMasterStatusFlags; /*!< Master status flags */
} ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_T;

/*! Get Master Current State Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF, ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_GET_MASTER_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T tReq;
  ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T tCnf;
} ECM_IF_GET_MASTER_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ/ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF
 */

/* request packet */
/*! Set Slave Target State Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bTargetState; /*!< Target state */
} ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_T;

/*! Set Slave Target State Request Packet Descriotion
 *
 * \param ulCmd <<ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ, ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ>>
 * \param ulLen 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T;


/* confirmation packet */
/*! Set Slave Target State Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Slave address */
  uint8_t bTargetState; /*!< Current slave state */
} ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_T;

/*! Set Slave Target State Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF, ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF>>
 * \param ulLen 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_SLAVE_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_SLAVE_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ/ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF
 */

/* request packet */
/*! Get Slave Current State Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Slave address */
} ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_T;

/*! Get Slave Current State Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ, ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T;


/* confirmation packet */
/*! Get Slave Current State Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Slave address */
  uint8_t bCurrentState; /*!< Slave current state */
  uint8_t bTargetState; /*!< Slave target state */
  uint32_t ulActiveError; /*!< Active error code */
} ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_T;

/*! Get Slave Current State Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF, ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF>>
 * \param ulLen 8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T tReq;
  ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T tCnf;
} ECM_IF_GET_SLAVE_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ/ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF
 */

/* request packet */
/*! Get Master State Diag Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ, ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_GET_MASTER_STATE_DIAG_REQ_T;


/* confirmation packet */
/*! Get Master State Diag Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_Ttag
{
  uint8_t bCurrentState; /*!< Current state */
  uint8_t abReserved[3];
  uint32_t ulCurrentActivity; /*!< Current master activity */
} ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_T;

/* ulCurrentActivity */
/*! Current activity
 *
 */
enum ECM_IF_ACTIVITY_Etag
{
  ECM_IF_ACTIVITY_IDLE = 0x00000000, /*!< ECM_IF_ACTIVITY_IDLE */
  ECM_IF_ACTIVITY_ACTIVE = 0x00000001, /*!< ECM_IF_ACTIVITY_ACTIVE */
  ECM_IF_ACTIVITY_BUS_SCAN = 0x00000100, /*!< ECM_IF_ACTIVITY_BUS_SCAN */
  ECM_IF_ACTIVITY_MANUAL = 0x00000200, /*!< ECM_IF_ACTIVITY_MANUAL */
  ECM_IF_ACTIVITY_ERROR_DETECTED = 0x00000300, /*!< ECM_IF_ACTIVITY_ERROR_DETECTED */

  ECM_IF_ACTIVITY_WAIT_FOR_SLAVES = 0x00010000, /*!< ECM_IF_ACTIVITY_WAIT_FOR_SLAVES */
  ECM_IF_ACTIVITY_OPEN_PORTS = 0x00010001, /*!< ECM_IF_ACTIVITY_OPEN_PORTS */

  ECM_IF_ACTIVITY_IDENTIFY_SLAVES = 0x00010100, /*!< ECM_IF_ACTIVITY_IDENTIFY_SLAVES */
  ECM_IF_ACTIVITY_VERIFY_SLAVES = 0x00010200, /*!< ECM_IF_ACTIVITY_VERIFY_SLAVES */
  ECM_IF_ACTIVITY_HOTCONNECT_DETECT = 0x00010300, /*!< ECM_IF_ACTIVITY_HOTCONNECT_DETECT */

  ECM_IF_ACTIVITY_WAIT_FOR_TIMESYNC = 0x00010400, /*!< ECM_IF_ACTIVITY_WAIT_FOR_TIMESYNC */
  ECM_IF_ACTIVITY_WAIT_FOR_LINK = 0x00010500, /*!< ECM_IF_ACTIVITY_WAIT_FOR_LINK */

  ECM_IF_ACTIVITY_RESET_SLAVES = 0x00020000, /*!< ECM_IF_ACTIVITY_RESET_SLAVES */

  ECM_IF_ACTIVITY_INIT_TO_PREOP = 0x00030000, /*!< ECM_IF_ACTIVITY_INIT_TO_PREOP */
  ECM_IF_ACTIVITY_PREOP_TO_SAFEOP = 0x00030100, /*!< ECM_IF_ACTIVITY_PREOP_TO_SAFEOP */
  ECM_IF_ACTIVITY_SAFEOP_TO_OP = 0x00030200, /*!< ECM_IF_ACTIVITY_SAFEOP_TO_OP */
  ECM_IF_ACTIVITY_OP_TO_INIT = 0x00030300, /*!< ECM_IF_ACTIVITY_OP_TO_INIT */
  ECM_IF_ACTIVITY_OP_TO_PREOP = 0x00030400, /*!< ECM_IF_ACTIVITY_OP_TO_PREOP */
  ECM_IF_ACTIVITY_OP_TO_SAFEOP = 0x00030500, /*!< ECM_IF_ACTIVITY_OP_TO_SAFEOP */
  ECM_IF_ACTIVITY_SAFEOP_TO_INIT = 0x0030600, /*!< ECM_IF_ACTIVITY_SAFEOP_TO_INIT */
  ECM_IF_ACTIVITY_SAFEOP_TO_PREOP = 0x0030700, /*!< ECM_IF_ACTIVITY_SAFEOP_TO_PREOP */
  ECM_IF_ACTIVITY_PREOP_TO_INIT = 0x0030800, /*!< ECM_IF_ACTIVITY_PREOP_TO_INIT */
  ECM_IF_ACTIVITY_TO_BUS_OFF = 0x0030900, /*!< ECM_IF_ACTIVITY_TO_BUS_OFF */

  ECM_IF_ACTIVITY_PS_RESET_DC = 0x00040000, /*!< ECM_IF_ACTIVITY_PS_RESET_DC */
  ECM_IF_ACTIVITY_PS_WAIT_FOR_RX_LATCH_READY = 0x00040100, /*!< ECM_IF_ACTIVITY_PS_WAIT_FOR_RX_LATCH_READY */
  ECM_IF_ACTIVITY_PS_WAIT_FOR_READ_DC_SLAVE_RX_PORTS = 0x00040200, /*!< ECM_IF_ACTIVITY_PS_WAIT_FOR_READ_DC_SLAVE_RX_PORTS */
  ECM_IF_ACTIVITY_PS_BEGIN_DC_SYNC = 0x00040300, /*!< ECM_IF_ACTIVITY_PS_BEGIN_DC_SYNC */
  ECM_IF_ACTIVITY_PS_WAIT_FOR_DC_SYNC = 0x00040400, /*!< ECM_IF_ACTIVITY_PS_WAIT_FOR_DC_SYNC */

  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_RETRIGGER_SYNC = 0x00050000, /*!< ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_RETRIGGER_SYNC */
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_RX_LATCH_READY = 0x00050100, /*!< ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_RX_LATCH_READY */
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_READ_DC_SLAVE_RX_PORTS = 0x00050200, /*!< ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_READ_DC_SLAVE_RX_PORTS */
  ECM_IF_ACTIVITY_RESYNC_BEGIN_DC_SYNC = 0x00050300, /*!< ECM_IF_ACTIVITY_RESYNC_BEGIN_DC_SYNC */
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_DC_SYNC = 0x00050400, /*!< ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_DC_SYNC */
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_REACHING_STATE = 0x00050500, /*!< ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_REACHING_STATE */
};

/*! Get Master State Diag Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF, ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF>>
 * \param ulLen 8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_MASTER_STATE_DIAG_CNF_T;

/* packet union */
typedef union ECM_IF_GET_MASTER_STATE_DIAG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_STATE_DIAG_REQ_T tReq;
  ECM_IF_GET_MASTER_STATE_DIAG_CNF_T tCnf;
} ECM_IF_GET_MASTER_STATE_DIAG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_EXT_START_BUSSCAN_REQ/ECM_IF_CMD_EXT_START_BUSSCAN_CNF
 *
 * to be used instead of HIL_BUSSCAN_REQ ulAction=HIL_BUSSCAN_CMD_START
 */

/* request packet */
/*! Extended Start Bus Scan Request packet structure
 *
 * This packet is an alternative to HIL_BUSSCAN_REQ (ulActio = HIL_BUSSCAN_CMD_START) for starting the actual bus scan with EtherCAT master specific options
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_EXT_START_BUSSCAN_REQ_DATA_Ttag
{
  uint32_t ulAction; /*!< always HIL_BUSSCAN_CMD_START */
  uint32_t ulBusScanFlags; /*!< Bus Scan Flags */
  uint16_t usVlanId; /*!< only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
  uint8_t bVlanPriority; /*!< only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
} ECM_IF_EXT_START_BUSSCAN_REQ_DATA_T;

/*! Bus Scan Flags
 *
 */
enum ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_Etag
{
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_WITHOUT_PREOP = 0x00000001, /*!< Simplified Bus Scan without entering PREOP */
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG = 0x00000002, /*!< MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG */
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_ENABLE_VLAN_TAG = 0x00000004, /*!< enable VLAN tag only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_SKIP_TIME_SYNC_LATCH = 0x00000008, /*!< Skip Time Sync Latch handling if that is activated */
};

/*! Extended Start Bus Scan Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_EXT_START_BUSSCAN_REQ, ECM_IF_CMD_EXT_START_BUSSCAN_REQ>>
 * \param ulLen 11
 * \param ulSta 0
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_EXT_START_BUSSCAN_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_EXT_START_BUSSCAN_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_EXT_START_BUSSCAN_REQ_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ/ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF
 */

/* request packet */
/*! Manual Mode Control Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_Ttag
{
  uint32_t ulCmd; /*!< Manual mode command */
  uint16_t usPortSelect; /*!< Port select */
  uint16_t usAdp; /*!< Station address */
  uint16_t usAdo; /*!< Register offset */
  uint16_t usLength; /*!< Access length */
  uint16_t usReserved;
  uint8_t abData[1024]; /*!< Request data length specified by usLength */
} ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_T;

/* ulCmd */
/*! Manual mode command
 *
 */
enum ECM_IF_MANUAL_MODE_CONTROL_CMD_Etag
{
  ECM_IF_MANUAL_MODE_CONTROL_CMD_STOP = 0,     /*!< Stop manual mode */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_START = 1,    /*!< Start manual mode */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APRD = 0x0101,/*!< Send APRD */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APWR = 0x0102,/*!< Send APWR */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APRW = 0x0103,/*!< Send APRW */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPRD = 0x0104,/*!< Send FPRD */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPWR = 0x0105,/*!< Send FPWR */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPRW = 0x0106,/*!< Send FPRW */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BRD = 0x0107, /*!< Send BRD */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BWR = 0x0108, /*!< Send BWR */
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BRW = 0x0109, /*!< Send BRW */
};

/* usPortSelect */
/*!< Port select
 *
 */
enum ECM_IF_MANUAL_MODE_REQUEST_PORT_SELECTOR_E
{
  ECM_IF_MANUAL_MODE_REQUEST_ALL_PORTS = 0,/*!< Send on all ports */
  ECM_IF_MANUAL_MODE_REQUEST_PORT_0 = 1,   /*!< Send via port 0 only */
  ECM_IF_MANUAL_MODE_REQUEST_PORT_1 = 2    /*!< Send via port 1 only */
};

/*! Manual Mode Control Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ, ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ>>
 * \param ulLen 14+usLength
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_MANUAL_MODE_CONTROL_REQ_T;


/* confirmation packet */
/*! Manual Mode Control Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_Ttag
{
  uint32_t ulCmd; /*!< Manual mode command */
  uint16_t usPortReceived; /*!< Received on port */
  uint16_t usAdp; /*!< Station address */
  uint16_t usAdo; /*!< Register offset */
  uint16_t usLength; /*!< Access length */
  uint16_t usRetWkc; /*!< Returning working counter */
  uint8_t abData[1024]; /*!< Returned data */
} ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_T;

/* usPortReceived */
/*! Port Received
 *
 */
enum ECM_IF_MANUAL_MODE_REG_RECEIVE_PORT_Etag
{
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_NOT_DETERMINED = 0,/*!< Undetermined */
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_0 = 1,             /*!< Port 0 */
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_1 = 2,             /*!< Port 1 */
  ECM_IF_MANUAL_MODE_RECEIVE_ALL_PORTS = 3           /*!< All Ports */
};

/*! Manual Mode Control Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF, ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF>>
 * \param ulLen 14+usLength
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_MANUAL_MODE_CONTROL_CNF_T;


/* packet union */
typedef union ECM_IF_MANUAL_MODE_CONTROL_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MANUAL_MODE_CONTROL_REQ_T tReq;
  ECM_IF_MANUAL_MODE_CONTROL_CNF_T tCnf;
} ECM_IF_MANUAL_MODE_CONTROL_PCK_T;


/**********************************************************************************************************************

 #####   #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#        #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/* usTransportType */
/*! CoE Transport type
 *
 */
typedef enum ECM_IF_COE_TRANSPORT_TYPE_Etag
{
  ECM_IF_COE_TRANSPORT_COE = 0,/*!< CoE is accessed directly */
  ECM_IF_COE_TRANSPORT_AOE = 1 /*!< CoE is accessed through AoE */
} ECM_IF_COE_TRANSPORT_TYPE_E;


#define ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ                   0x9A00 /*!< CoE SDO Download Request */
#define ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF                   0x9A01 /*!< CoE SDO Download Confirmation */

#define ECM_IF_CMD_COE_SDO_UPLOAD_REQ                     0x9A02 /*!< CoE SDO Upload Request */
#define ECM_IF_CMD_COE_SDO_UPLOAD_CNF                     0x9A03 /*!< CoE SDO Upload COnfirmation */

#define ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ              0x9A04 /*!< CoE SDOINFO GetOdList Request */
#define ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF              0x9A05 /*!< CoE SDOINFO GetOdList Confirmation */

#define ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ             0x9A06 /*!< CoE SDOINFO GetObjDesc Request */
#define ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF             0x9A07 /*!< CoE SDOINFO GetObjDesc Confirmation */

#define ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ           0x9A08 /*!< CoE SDOINFO GetEntryDesc Request */
#define ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF           0x9A09 /*!< CoE SDOINFO GetEntryDesc Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ/ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! CoE SDO Download Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< Transport type see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex; /*!< Object index */
  uint8_t bSubIndex; /*!< Sub index */
  uint8_t fCompleteAccess; /*!< != 0 if complete access is to be used */
  uint32_t ulTotalBytes; /*!< has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs; /*!< timeout in ms */
  uint8_t abData[1024]; /*!< data to be written to SDO */
} ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T;

/*! CoE SDO Download Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ, ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ>>
 * \param uLLen 18+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDO_DOWNLOAD_REQ_T;


/* confirmation packet */
/*! CoE SDO Download Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usObjIndex; /*!< same as in request */
  uint8_t bSubIndex; /*!< same as in request */
  uint8_t fCompleteAccess; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
} ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_T;

/*! CoE SDO Download Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF, ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDO_DOWNLOAD_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDO_DOWNLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_DOWNLOAD_REQ_T tReq;
  ECM_IF_COE_SDO_DOWNLOAD_CNF_T tCnf;
} ECM_IF_COE_SDO_DOWNLOAD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDO_UPLOAD_REQ/ECM_IF_CMD_COE_SDO_UPLOAD_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! CoE SDO Upload Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< Transport type */ /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE port used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex; /*!< Object index */
  uint8_t bSubIndex; /*!< Subindex */
  uint8_t fCompleteAccess; /*!< != 0 if complete access is to be used */
  uint32_t ulTimeoutMs; /*!< timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes to returned */
} ECM_IF_COE_SDO_UPLOAD_REQ_DATA_T;

/*! CoE SDO Upload Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDO_UPLOAD_REQ, ECM_IF_CMD_COE_SDO_UPLOAD_REQ>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDO_UPLOAD_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDO_UPLOAD_REQ_T;


/* confirmation packet */
/*! CoE SDO Upload Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usObjIndex; /*!< same as in request */
  uint8_t bSubIndex; /*!< same as in request */
  uint8_t fCompleteAccess; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T, abData) */
} ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T;

/*! CoE SDO Upload Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDO_UPLOAD_CNF, ECM_IF_CMD_COE_SDO_UPLOAD_CNF>>
 * \param ulLen 18+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDO_UPLOAD_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDO_UPLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_UPLOAD_REQ_T tReq;
  ECM_IF_COE_SDO_UPLOAD_CNF_T tCnf;
} ECM_IF_COE_SDO_UPLOAD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ/ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! CoE SDOINFO GetOdList data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< Transport type */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usListType; /*!< List type */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Max total bytes */
} ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_T;

/*! CoE SDOINFO GetOdList Types
 *
 */
typedef enum ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_Etag
{
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_COUNTS = 0,       /*!< Object count list of the other five lists */
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_ALL = 1,          /*!< All objects */
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_RXPDOMAPPABLE = 2,/*!< Only RxPDO-mappable */
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_TXPDOMAPPABLE = 3,/*!< Only TxPDO-mappable */
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_BACKUP = 4,       /*!< Only backup objects */
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_SETTINGS = 5      /*!< Only settings objects */
} ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_E;

/*! CoE SDOINFO GetOdList Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ, ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ>>
 * \param ulLen 16
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDOINFO_GETODLIST_REQ_T;


/* confirmation packet */
/*! CoE SDOINFO GetOdList Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usListType; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint16_t ausObjectIDs[512]; /*!< actual byte length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T;

/*! CoE SDOINFO GetOdList Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF, ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF>>
 * \param ulLen 16+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDOINFO_GETODLIST_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETODLIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETODLIST_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETODLIST_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETODLIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ/ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! CoE SDOINFO GetObjDesc Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< CoE transport type */
  uint16_t usAoEPort; /*!< AoE POrt used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex; /*!< Object index */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes requested */
} ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_T;

/*! CoE SDOINFO GetObjDesc Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ, ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ>>
 * \param ulLen 16
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_T;


/* confirmation packet */
/*! CoE SDOINFO GetObjDesc Confirmation data structur
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usObjIndex; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T;

/*! CoE SDOINFO GetObjDesc Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF, ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF>>
 * \param ulLen 16+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETOBJDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETOBJDESC_PCK_T;


/*! Format of abData in GetObjDesc Confirmation
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_Ttag
{
  uint16_t usReserved;
  uint16_t usDataType; /*!< Data type */
  uint8_t bMaxSubIndex; /*!< Max subindex */
  uint8_t bObjectCode; /*!< Object code */
  uint8_t abName[256]; /*!< Name of object. actual length is defined by ulTotalBytes - offsetof(ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_T, abName) */
} ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ/ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/*! SDOINFO GetEntryDesc Value Flags
 *
 */
typedef enum ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_Etag
{
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_UNIT_TYPE = 0x08,    /*!< Unit */
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_DEFAULT_VALUE = 0x10,/*!< Default value */
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MINIMUM_VALUE = 0x20,/*!< Minimum value */
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MAXIMUM_VALUE = 0x40,/*!< Maximum value */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_E;

/* request packet */
/*! CoE SDOINFO GetEntryDesc Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< CoE transport types */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex; /*!< Object index */
  uint8_t bSubIndex; /*!< Sub index */
  uint8_t bRequestedValueInfo; /*!< Requested value flags */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Max total data bytes to be returned */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_T;

/*! CoE SDOINFO GetEntryDesc Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ, ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_T tData; /*!< Packe data */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_T;


/* confirmation packet */
/*! CoE SDOINFO GetEntryDesc Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usObjIndex; /*!< same as in request */
  uint8_t bSubIndex; /*!< same as in request */
  uint8_t bValueInfo; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T;

/*! CoE SDOINFO GetEntryDesc Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF,ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF>>
 * \param ulLen 18+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETENTRYDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETENTRYDESC_PCK_T;

/*! Format of abData in CoE SDOINFO GetEntryDesc
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_Ttag
{
  uint32_t ulReserved;
  uint16_t usDataType; /*!< Data type */
  uint16_t usBitLength; /*!< Bit length */
  uint16_t usObjAccess; /*!< Object access flags */
  uint8_t abData[1024]; /*!< actual length is defined by ulTotalBytes - offsetof(ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_T, abName)
   * order of data
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_UNIT_TYPE): uint32_t ulEcatUnit
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MINIMUM_VALUE): uint8_t abMinimumValue[usFieldSize]
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MAXIMUM_VALUE): uint8_t abMaximumValue[usFieldSize]
   * uint8_t abName[remainingbytes]
   */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_T;




/**********************************************************************************************************************

 #####   #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
 #####   #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
      #  #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/*! SoE TransportType
 *
 */
typedef enum ECM_IF_SOE_TRANSPORT_TYPE_Etag
{
  ECM_IF_SOE_TRANSPORT_SOE = 0, /*!< use SoE directly */
  ECM_IF_SOE_TRANSPORT_AOE = 1 /*!< SoE via AoE */
} ECM_IF_SOE_TRANSPORT_TYPE_E;


/* bElementFlags */
/*! SoE Element Flags
 *
 */
typedef enum ECM_IF_SOE_ELEMENT_FLAGS_Etag
{
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_DATASTATE = 0x01,/*!< Data State */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_NAME = 0x02, /*!< Name */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_ATTRIBUTE = 0x04, /*!< Attribute */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_UNIT = 0x08, /*!< Unit */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_MIN = 0x10, /*!< Minimum value */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_MAX = 0x20,  /*!< Maximum value */
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_VALUE = 0x40 /*!< Value */
} ECM_IF_SOE_ELEMENT_FLAGS_E;


#define ECM_IF_CMD_SOE_WRITE_REQ                          0x9B00 /*!< SoE Write Request */
#define ECM_IF_CMD_SOE_WRITE_CNF                          0x9B01 /*!< SoE Write Confirmation */

#define ECM_IF_CMD_SOE_READ_REQ                           0x9B02 /*!< SoE Read Request */
#define ECM_IF_CMD_SOE_READ_CNF                           0x9B03 /*!< SoE Read Confirmation */

#define ECM_IF_CMD_SOE_EXEC_PROCCMD_REQ                   0x9B04 /*!< SoE Execute Procedure Command Request */
#define ECM_IF_CMD_SOE_EXEC_PROCCMD_CNF                   0x9B05 /*!< SoE Execute Procedure COmmadn Confirmation */


/******************************************************************************
 * Packet: ECM_IF_CMD_SOE_WRITE_REQ/ECM_IF_CMD_SOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! SoE Write Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< SoE transport type */ /* see ECM_IF_SOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_SOE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usIDN; /*!< IDN number */
  uint32_t ulTotalBytes; /*!< has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint8_t bDriveNo; /*!< Drive number to be accessed */
  uint8_t bElementFlags; /*!< SoE Element Flags */ /* see ECM_IF_SOE_ELEMENT_FLAGS_E */
  uint8_t abData[1024]; /*!< Data to be written */
} ECM_IF_SOE_WRITE_REQ_DATA_T;

/*! SoE Write Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SOE_WRITE_REQ, ECM_IF_CMD_SOE_WRITE_REQ>>
 * \param ulLen 18+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SOE_WRITE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SOE_WRITE_REQ_T;


/* confirmation packet */
/*! SoE Write Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usIDN; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint8_t bDriveNo; /*!< same as in request */
  uint8_t bElementFlags; /*!< same as in request */
} ECM_IF_SOE_WRITE_CNF_DATA_T;

/*! SoE Write Confirmation Packet Description
 *
 * \param ulCme <<ECM_IF_CMD_SOE_WRITE_CNF, ECM_IF_CMD_SOE_WRITE_CNF>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SOE_WRITE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_SOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_SOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_WRITE_REQ_T tReq;
  ECM_IF_SOE_WRITE_CNF_T tCnf;
} ECM_IF_SOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SOE_READ_REQ/ECM_IF_CMD_SOE_READ_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! SoE Read Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< SoE transport type *//* see ECM_IF_SOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_SOE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usIDN; /*!< IDN number */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint8_t bDriveNo; /*!< Drive number */
  uint8_t bElementFlags; /*!< Element flags */ /* see ECM_IF_SOE_ELEMENT_FLAGS_E */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes to be requested */
} ECM_IF_SOE_READ_REQ_DATA_T;

/*! SoE Read Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SOE_READ_REQ, ECM_IF_CMD_SOE_READ_REQ>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SOE_READ_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SOE_READ_REQ_T;


/* confirmation packet */
/*! SoE Read Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint16_t usIDN; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint8_t bDriveNo; /*!< same as in request */
  uint8_t bElementFlags; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data.
  * actual length is given by ulLen - offsetof(ECM_IF_SOE_READ_CNF_DATA_T, abData) */
} ECM_IF_SOE_READ_CNF_DATA_T;

/*! SoE Read Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SOE_READ_CNF, ECM_IF_CMD_SOE_READ_CNF>>
 * \param ulLen 18+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SOE_READ_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_SOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_SOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_READ_REQ_T tReq;
  ECM_IF_SOE_READ_CNF_T tCnf;
} ECM_IF_SOE_READ_PCK_T;


/**********************************************************************************************************************

#######  #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#        #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#####    #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#        #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
#        #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/* usTransportType */
/*! FoE transport type
 *
 */
typedef enum ECM_IF_FOE_TRANSPORT_TYPE_Etag
{
  ECM_IF_FOE_TRANSPORT_FOE = 0,/*!< use FoE directly */
  ECM_IF_FOE_TRANSPORT_AOE = 1 /*!< FoE via AoE */
} ECM_IF_FOE_TRANSPORT_TYPE_E;


#define ECM_IF_CMD_FOE_WRITE_REQ                          0x9900 /*!< FoE Write Request */
#define ECM_IF_CMD_FOE_WRITE_CNF                          0x9901 /*!< FoE Write Confirmation */

#define ECM_IF_CMD_FOE_READ_REQ                           0x9902 /*!< FoE Read Request */
#define ECM_IF_CMD_FOE_READ_CNF                           0x9903 /*!< FoE Read Confirmation */


#define ECM_IF_FOE_MAX_ERROR_TEXT_BYTE_LEN 256 /*!< Max Length of FoE Error Text */

/******************************************************************************
 * Packet: ECM_IF_CMD_FOE_WRITE_REQ/ECM_IF_CMD_FOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0 (data format as in tFirstSegData)
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! FoE Write Request data structure - First segment
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_FIRST_Ttag
{
  /* structure for first fragment */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< FoE transport type *//* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTotalBytes; /*!< has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulPassword; /*!< Password */
  uint32_t ulFileNameBytes; /*!< Number of bytes used for file name including its NUL terminator */
  uint8_t abData[1024]; /*!< [0 - (ulFileNameBytes - 1)] is a NUL-terminated filename */
} ECM_IF_FOE_WRITE_REQ_DATA_FIRST_T;

/*! FoE Write Request data structure - Following segments
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_SEG_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< FoE Transport type */ /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE Port used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTotalBytes; /*!< has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint8_t abData[1024]; /*!< Data to be sent */
} ECM_IF_FOE_WRITE_REQ_DATA_SEG_T;

/*! FoE Write Request data structure
 *
 */
typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_Ttag
{
  ECM_IF_FOE_WRITE_REQ_DATA_FIRST_T tFirst; /*!< First segment */
  ECM_IF_FOE_WRITE_REQ_DATA_SEG_T tSeg; /*!< Following segments */
} ECM_IF_FOE_WRITE_REQ_DATA_T;

/*! FoE Write Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_FOE_WRITE_REQ, ECM_IF_CMD_FOE_WRITE_REQ>>
 * \param ulLen 10+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_FOE_WRITE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_FOE_WRITE_REQ_T;


/* confirmation packet */
/*! FoE Write Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint8_t abErrorText[ECM_IF_FOE_MAX_ERROR_TEXT_BYTE_LEN]; /*!< NUL-terminated error text, valid when ulSta != 0 */
} ECM_IF_FOE_WRITE_CNF_DATA_T;

/*! FoE Write Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_FOE_WRITE_CNF, ECM_IF_CMD_FOE_WRITE_CNF>>
 * \param ulLen 10+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_FOE_WRITE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_FOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_FOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_WRITE_REQ_T tReq;
  ECM_IF_FOE_WRITE_CNF_T tCnf;
} ECM_IF_FOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_FOE_READ_REQ/ECM_IF_CMD_FOE_READ_CNF
 *
 * first fragment has ulDestId == 0 (data format as in tFirstSegData)
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! FoE Read Request data structure - First segment
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_FIRST_Ttag
{
  /* structure for first fragment */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< FoE transport type *//* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE port used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes to be requested */
  uint32_t ulPassword; /*!< Password */
  uint8_t abFileName[1024]; /*!< byte size defined by tHead.ulLen - offsetof(ECM_IF_FOE_READ_REQ_DATA_FIRST_T, tData) */
} ECM_IF_FOE_READ_REQ_DATA_FIRST_T;

/*! FoE Read Request data structure - Following segments
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_SEG_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usTransportType; /*!< FoE transport type */ /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /*!< AoE port used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes to be requested */
} ECM_IF_FOE_READ_REQ_DATA_SEG_T;

/*! FoE Read Request data structure
 *
 */
typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_Ttag
{
  ECM_IF_FOE_READ_REQ_DATA_FIRST_T tFirst; /*!< First segment */
  ECM_IF_FOE_READ_REQ_DATA_SEG_T tSeg; /*!< Following segments */
}ECM_IF_FOE_READ_REQ_DATA_T;

/*! FoE Read Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_FOE_READ_REQ, ECM_IF_CMD_FOE_READ_REQ>>
 * \param ulLen 14+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_FOE_READ_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_FOE_READ_REQ_T;


/* confirmation packet */
/*! FoE Read Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usTransportType; /*!< same as in request */
  uint16_t usAoEPort; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_FOE_READ_CNF_DATA_T, abData)
  * in case of ulSta != 0, abData contains a NUL-terminated error string */
} ECM_IF_FOE_READ_CNF_DATA_T;

/*! FoE Read Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_FOE_READ_CNF, ECM_IF_CMD_FOE_READ_CNF>>
 * \param ulLen 14+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_FOE_READ_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_FOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_FOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_READ_REQ_T tReq;
  ECM_IF_FOE_READ_CNF_T tCnf;
} ECM_IF_FOE_READ_PCK_T;


/**********************************************************************************************************************

   #     #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
  # #    #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
 #   #   #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#     #  #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#######  #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
#     #  #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/*! AoE address
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_ADDRESS_Ttag
{
  uint8_t abNetId[6]; /*!< AoE net id */
  uint16_t usPort; /*!< AoE port */
} ECM_IF_AOE_ADDRESS_T;

/******************************************************************************/
#define ECM_IF_CMD_AOE_WRITE_REQ                          0x9D00 /*!< AoE Write Request */
#define ECM_IF_CMD_AOE_WRITE_CNF                          0x9D01 /*!< AoE Write Confirmation */

#define ECM_IF_CMD_AOE_READ_REQ                           0x9D02 /*!< AoE Read Request */
#define ECM_IF_CMD_AOE_READ_CNF                           0x9D03 /*!< AoE Read Confirmation */

#define ECM_IF_CMD_AOE_READWRITE_REQ                      0x9D04 /*!< AoE ReadWrite Request */
#define ECM_IF_CMD_AOE_READWRITE_CNF                      0x9D05 /*!< AoE ReadWrite Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_WRITE_REQ/ECM_IF_CMD_AOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! AoE Write Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< AoE address */
  uint32_t ulIndexGroup; /*!< Index group */
  uint32_t ulIndexOffset; /*!< Index offset */
  uint32_t ulTotalBytes; /*!< has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint8_t abData[1024]; /*!< Data to be written. Actual length determined by tHead.ulLen */
} ECM_IF_AOE_WRITE_REQ_DATA_T;

/*! AoE Write Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_WRITE_REQ, ECM_IF_CMD_AOE_WRITE_REQ>>
 * \param ulLen 24+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_WRITE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_WRITE_REQ_T;


/* confirmation packet */
/*! AoE Write Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< same as in request */
  uint32_t ulIndexGroup; /*!< same as in request */
  uint32_t ulIndexOffset; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
} ECM_IF_AOE_WRITE_CNF_DATA_T;

/*! AoE Write Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_WRITE_CNF, ECM_IF_CMD_AOE_WRITE_CNF>>
 * \param ulLen 24
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_WRITE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_WRITE_REQ_T tReq;
  ECM_IF_AOE_WRITE_CNF_T tCnf;
} ECM_IF_AOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_READ_REQ/ECM_IF_CMD_AOE_READ_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! AoE Read Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< AoE address */
  uint32_t ulIndexGroup; /*!< Index group */
  uint32_t ulIndexOffset; /*!< Index offset */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulMaxTotalBytes; /*!< Maximum total bytes to be read */
} ECM_IF_AOE_READ_REQ_DATA_T;

/*! AoE Read Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_READ_REQ, ECM_IF_CMD_AOE_READ_REQ>>
 * \param ulLen 22
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_READ_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_READ_REQ_T;


/* confirmation packet */
/*! AoE Read Confirmation data structure */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< same as in request */
  uint32_t ulIndexGroup;  /*!< same as in request */
  uint32_t ulIndexOffset; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalBytes; /*!< summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_AOE_READ_CNF_DATA_T, abData) */
} ECM_IF_AOE_READ_CNF_DATA_T;

/*! AoE Read Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_READ_REQ, ECM_IF_CMD_AOE_READ_REQ>>
 * \param ulLen 22+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_READ_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READ_REQ_T tReq;
  ECM_IF_AOE_READ_CNF_T tCnf;
} ECM_IF_AOE_READ_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_READWRITE_REQ/ECM_IF_CMD_AOE_READWRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! AoE ReadWrite Request data structure
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< AoE address */
  uint32_t ulIndexGroup; /*!< Index group */
  uint32_t ulIndexOffset; /*!< Index offset */
  uint32_t ulTotalWriteBytes; /*!< has to be set to summed length of all abData of all write fragments */
  uint32_t ulMaxTotalReadBytes; /*!< Maximum total data bytes to be read */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint8_t abData[1024]; /*!< Data to be written */
} ECM_IF_AOE_READWRITE_REQ_DATA_T;

/*! AoE ReadWrite Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_READWRITE_REQ, ECM_IF_CMD_AOE_READWRITE_REQ>>
 * \param ulLen 26+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_READWRITE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_READWRITE_REQ_T;


/* confirmation packet */
/*! AoE ReadWrite Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget; /*!< same as in request */
  uint32_t ulIndexGroup; /*!< same as in request */
  uint32_t ulIndexOffset; /*!< same as in request */
  uint32_t ulTotalWriteBytes; /*!< same as in request */
  uint32_t ulTotalReadBytes; /*!< summed length of all abData confirmation fragments */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint8_t abData[1024]; /*!< Read data. actual length is given by ulLen - offsetof(ECM_IF_AOE_READWRITE_CNF_DATA_T, abData) */
} ECM_IF_AOE_READWRITE_CNF_DATA_T;

/*! AoE ReadWrite Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_AOE_READWRITE_CNF, ECM_IF_CMD_AOE_READWRITE_CNF>>
 * \param ulLen 26+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_AOE_READWRITE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_AOE_READWRITE_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_READWRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READWRITE_REQ_T tReq;
  ECM_IF_AOE_READWRITE_CNF_T tCnf;
} ECM_IF_AOE_READWRITE_PCK_T;


/**********************************************************************************************************************

 #####   #######     #     #######  #######        ###  #     #  ######   ###   #####      #     #######  ###  #######  #     #   #####
#     #     #       # #       #     #               #   ##    #  #     #   #   #     #    # #       #      #   #     #  ##    #  #     #
#           #      #   #      #     #               #   # #   #  #     #   #   #         #   #      #      #   #     #  # #   #  #
 #####      #     #     #     #     #####           #   #  #  #  #     #   #   #        #     #     #      #   #     #  #  #  #   #####
      #     #     #######     #     #               #   #   # #  #     #   #   #        #######     #      #   #     #  #   # #        #
#     #     #     #     #     #     #               #   #    ##  #     #   #   #     #  #     #     #      #   #     #  #    ##  #     #
 #####      #     #     #     #     #######        ###  #     #  ######   ###   #####   #     #     #     ###  #######  #     #   #####

 */

#define ECM_IF_CMD_MASTER_CURRENT_STATE_IND               0x9E10 /*!< Master Current State Indication */
#define ECM_IF_CMD_MASTER_CURRENT_STATE_RES               0x9E11 /*!< Master Current State Response */

#define ECM_IF_CMD_SLAVE_CURRENT_STATE_IND                0x9E12 /*!< Slave Current State Indication */
#define ECM_IF_CMD_SLAVE_CURRENT_STATE_RES                0x9E13 /*!< Slave Current STate Response */

#define ECM_IF_CMD_REGISTER_APP_REQ                       0x9E1C
#define ECM_IF_CMD_REGISTER_APP_CNF                       0x9E1D

#define ECM_IF_CMD_UNREGISTER_APP_REQ                     0x9E1E
#define ECM_IF_CMD_UNREGISTER_APP_CNF                     0x9E1F


/******************************************************************************
 * Packet: ECM_IF_CMD_MASTER_CURRENT_STATE_IND/ECM_IF_CMD_MASTER_CURRENT_STATE_RES
 */

/* indication packet */
/*! Master Current State Indication data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_IND_DATA_Ttag
{
  uint8_t bCurrentState; /*!< Current state */
  uint8_t bTargetState; /*!< Target state */
  uint32_t ulStopReason; /*!< if != 0, it shows the reason why the state switch stopped */
  uint32_t ulMasterStatusFlags; /*!< Master status flags */
} ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T;

/*! Master Current State Indication Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_MASTER_CURRENT_STATE_IND, ECM_IF_CMD_MASTER_CURRENT_STATE_IND>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T tData; /*!< Packet data */
} ECM_IF_MASTER_CURRENT_STATE_IND_T;


/* confirmation packet */
/*! Master Current State Response Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_MASTER_CURRENT_STATE_RES, ECM_IF_CMD_MASTER_CURRENT_STATE_RES>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet ehader */
} ECM_IF_MASTER_CURRENT_STATE_RES_T;


/* packet union */
typedef union ECM_IF_MASTER_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MASTER_CURRENT_STATE_IND_T tInd;
  ECM_IF_MASTER_CURRENT_STATE_RES_T tRes;
} ECM_IF_MASTER_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SLAVE_CURRENT_STATE_IND/ECM_IF_CMD_SLAVE_CURRENT_STATE_RES
 */

/* indication packet */
/*! Slave Current State Indication data structure slave entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usCurrentStatus; /*!< Current status */
  uint32_t ulLastError; /*!< Last error */
} ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T;

/*!< Maximum entries in Indication */
#define ECM_IF_SLAVE_CURRENT_STATE_IND_MAX_ENTRIES (HIL_MAX_DATA_SIZE / sizeof(ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T))

/*! Slave Current State Indication data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_Ttag
{
  ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T atEntries[ECM_IF_SLAVE_CURRENT_STATE_IND_MAX_ENTRIES]; /*!< Slave entry table */
} ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_T;

/*! Slave Current State Indication Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SLAVE_CURRENT_STATE_IND, ECM_IF_CMD_SLAVE_CURRENT_STATE_IND>>
 * \param ulLen n*8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_T tData; /*!< Packet data */
} ECM_IF_SLAVE_CURRENT_STATE_IND_T;


/* confirmation packet */
/*! Slave Current State Response Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SLAVE_CURRENT_STATE_RES, ECM_IF_CMD_SLAVE_CURRENT_STATE_RES>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_SLAVE_CURRENT_STATE_RES_T;


/* packet union */
typedef union ECM_IF_SLAVE_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SLAVE_CURRENT_STATE_IND_T tInd;
  ECM_IF_SLAVE_CURRENT_STATE_RES_T tRes;
} ECM_IF_SLAVE_CURRENT_STATE_PCK_T;


/**********************************************************************************************************************

######   ###     #      #####   #     #  #######   #####   #######  ###   #####    #####
#     #   #     # #    #     #  ##    #  #     #  #     #     #      #   #     #  #     #
#     #   #    #   #   #        # #   #  #     #  #           #      #   #        #
#     #   #   #     #  #  ####  #  #  #  #     #   #####      #      #   #         #####
#     #   #   #######  #     #  #   # #  #     #        #     #      #   #              #
#     #   #   #     #  #     #  #    ##  #     #  #     #     #      #   #     #  #     #
######   ###  #     #   #####   #     #  #######   #####      #     ###   #####    #####

 */

#define ECM_IF_CMD_GET_DC_DEVIATION_REQ 0x9E60 /*!< Get DC Deviation Request */
#define ECM_IF_CMD_GET_DC_DEVIATION_CNF 0x9E61 /*!< Get DC Deviation Confirmation */

#define ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ 0x9E62 /*!< Get Slave DC Info Request */
#define ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF 0x9E63 /*!< Get Slave DC Info Confirmation */

#define ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ 0x9E64 /*!< Reset DC Max Deviations Request */
#define ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF 0x9E65 /*!< Reset DC Max Deviations Confirmation */

#define ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ 0x9E66 /*!< Get Ext Sync Info Request */
#define ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF 0x9E67 /*!< Get Ext Sync Info Confirmation */

#define ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ 0x9E52 /*!< Reset Ext Sync Max Deviations Request */
#define ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF 0x9E53 /*!< Reset Ext Sync Max Deviations Confirmation */

#define ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ 0x9E68 /*!< Get Slave Handle Bit List Request */
#define ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_CNF 0x9E69 /*!< Get SLave Handle Bit List Confirmation */

#define ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ 0x9E6A /*!< Get Frame Loss Counters Request */
#define ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF 0x9E6B /*!< Get Frame Loss Counters Confirmation */

#define ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ 0x9E6C /*!< Get Threshold Counters Request */
#define ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF 0x9E6D /*!< Get Threshold Counters Confirmation */

#define ECM_IF_CMD_GET_ERROR_COUNTERS_REQ 0x9E6E /*!< Get Error Counters Request */
#define ECM_IF_CMD_GET_ERROR_COUNTERS_CNF 0x9E6F /*!< Get Error Counters Confirmation */

#define ECM_IF_CMD_READ_EMERGENCY_REQ 0x9E54 /*!< Read Emergency Request */
#define ECM_IF_CMD_READ_EMERGENCY_CNF 0x9E55 /*!< Read Emergency Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_DC_DEVIATION_REQ/ECM_IF_CMD_GET_DC_DEVIATION_CNF
 */

/* ulDcStatusFlags */
/*! DC Control Status Flags
 *
 */
typedef enum ECM_IF_DC_CONTROL_STATUS_FLAGS_Etag
{
  ECM_IF_DC_CONTROL_STATUS_INACTIVE = 0, /*!< Inactive */

  /* actual flags */
  ECM_IF_DC_CONTROL_STATUS_ACTIVE = 1, /*!< Active */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_DC_RX_STATUS_MAIN = 2, /*!< Stopped due unexpected DC Rx Status on main port */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_DC_RX_STATUS_RED = 4, /*!< Stopped due unexpected DC Rx Status on redundancy port */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_BRD_ALSTATUS_WKC_MAIN = 8, /*!< Stopped due to unexpected BRD ALStatus Wkc on main port */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_BRD_ALSTATUS_WKC_RED = 16, /*!< Stopped due to unexpected BRD ALStatus Wkc on redundancy port */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_DL_STATUS_IRQ = 32,                 /*!< Stopped due DLStatus Ecat IRQ */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_DC_ALL_PORTS_RX_STATUS_TIMEOUT = 64,/*!< Stopped due all ports having Dc RxStatus timeout */
  ECM_IF_DC_CONTROL_STATUS_STOPPED_BY_INTERNAL_REQUEST = 128           /*!< Stopped by internal request */
} ECM_IF_DC_CONTROL_STATUS_FLAGS_E;

/* request packet */
/*! Get DC Deviation Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_DC_DEVIATION_REQ, ECM_IF_CMD_GET_DC_DEVIATION_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_GET_DC_DEVIATION_REQ_T;


/* confirmation packet */
/*! Get DC Deviation Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_CNF_DATA_Ttag
{
  uint32_t ulDcSlaveBrdDeviationSignMag; /*! current deviation seen on BRD 92C */
  uint32_t ulDcBusDeviationSignMag; /*!< current bus cycle deviation */
  uint32_t ulDcLocalSysTimeDeviationSignMag; /*!< current deviation on Master Systime control loop */
  uint32_t ulDcStatusFlags; /*!< DC Status Flags */

  /* max values */
  uint32_t ulDcSlaveBrdDeviationMaxMag; /*!< maximum deviation seen on BRD 92C */
  uint32_t ulDcBusDeviationPosMaxMag; /*!< maximum positive bus cycle deviation */
  uint32_t ulDcBusDeviationNegMaxMag; /*!< maximum negative bus cycle deviation */
  uint32_t ulDcLocalSysTimeDeviationPosMaxMag; /*!< maximum positive deviation on Master Systime control loop */
  uint32_t ulDcLocalSysTimeDeviationNegMaxMag; /*!< maximum negative deviation on Master Systime control loop */
} ECM_IF_GET_DC_DEVIATION_CNF_DATA_T;

/*! Get DC Deviation Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_DC_DEVIATION_CNF, ECM_IF_CMD_GET_DC_DEVIATION_CNF>>
 * \param ulLen 36
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_DC_DEVIATION_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_DC_DEVIATION_CNF_T;


/* packet union */
typedef union ECM_IF_GET_DC_DEVIATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_DC_DEVIATION_REQ_T tReq;
  ECM_IF_GET_DC_DEVIATION_CNF_T tCnf;
} ECM_IF_GET_DC_DEVIATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ/ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF
 */

/* request packet */
/*! Get Slave DC Info Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_T;

/*! Get Slave DC Info Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ, ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_DC_INFO_REQ_T;


/* confirmation packet */
/*! Get Slave DC Info Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usFlags; /*!< Slave DC Info Flags */
  uint32_t ulDcSystimeDelayNs; /*!< DC Systime Delay configured to slave in ns */
  uint64_t ullDcSystimeOffsetNs; /*!< DC Systime Offset configured to slave in ns */
  uint64_t ullDcSyncShiftTimeNs; /*!< Configured DC Sync0 shift time based on bus cycle reference in ns */
  uint32_t ulDcCyc0Time; /*!< DC Cyc0Time as written to register */
  uint32_t ulDcCyc1Time; /*!< DC Cyc1Time as written to register */
  uint64_t ullRxLatchTime0Ns; /*!< Last latched local time of DC slave for Port 0 */
  uint32_t ulRxLatchTime1Ns; /*!< Last latched local time of DC slave for Port 1 */
  uint32_t ulRxLatchTime2Ns; /*!< Last latched local time of DC slave for Port 2 */
  uint32_t ulRxLatchTime3Ns; /*!< Last latched local time of DC slave for Port 3 */
  uint32_t ulPort1SumDelayNs; /*!< Total summed delay of all slaves connected to Port 1 of slave in ns */
  uint32_t ulPort2SumDelayNs; /*!< Total summed delay of all slaves connected to Port 2 of slave in ns */
  uint32_t ulPort3SumDelayNs; /*!< Total summed delay of all slaves connected to Port 3 of slave in ns */
  uint32_t ulTotalSumDelayNs; /*!< Total summed delay of all slaves connected to Port1, 2 and 3 */
  uint64_t ullDcSync0StartingDelayTimeNs; /*!< Last used delay for starting Sync signal generation on slave */
  uint64_t ullDcResyncSystimeOffsetNs; /*!< DC Systime Offset configured to slave in ns during Resync */
} ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_T;

/*! Meaning of usFlags
 *
 */
typedef enum ECM_IF_GET_SLAVE_DC_INFO_FLAGS_Etag
{
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_IN_TOPOLOGY = 0x0001, /*!< Slave is connected and actively participating in topology */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_TIME_CONFIGURED = 0x0002, /*!< DC SysTime has been configured on slave */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_SYNC_CONFIGURED = 0x0004, /*!< DC Sync Unit has been configured with parameters */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_IS_64BIT = 0x0008, /*!< Slave supports 64bit DC */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_IS_SUPPORTED = 0x0010, /*!< CM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_IS_SUPPORTED */

  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE = 0x0100, /*!< DC Sync Unit is configured to be activated*/
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE_SYNC0 = 0x0200, /*!< DC Sync0 is configured to be activated */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE_SYNC1 = 0x0400, /*!< DC Sync1 is configured to be activated */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_RX_TIMESTAMP_LATCH_SUPPORTED = 0x0800, /*!< Port Rx Timestamp Latch supported */

  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT0_EXISTS = 0x1000, /*!< Slave has a port 0 */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT1_EXISTS = 0x2000, /*!< Slave has a port 1 */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT2_EXISTS = 0x4000, /*!< Slave has a port 2 */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT3_EXISTS = 0x8000 /*!< Slave has a port 3 */
} ECM_IF_GET_SLAVE_DC_INFO_FLAGS_E;

/*! Get Slave DC Info Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF, ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF>>
 * \param ulLen Y
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_DC_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_DC_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_DC_INFO_REQ_T tReq;
  ECM_IF_GET_SLAVE_DC_INFO_CNF_T tCnf;
} ECM_IF_GET_SLAVE_DC_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ/ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF
 */

/* request packet */
/*! Reset DC Max Deviations Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ, ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_T;


/* confirmation packet */
/*! Reset DC Max Deviations Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF, ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_T;


/* packet union */
typedef union ECM_IF_RESET_DC_MAX_DEVIATIONS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_T tReq;
  ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_T tCnf;
} ECM_IF_RESET_DC_MAX_DEVIATIONS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ/ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF
 */

/* request packet */
/*! Get Ext Sync Info Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ, ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_GET_EXT_SYNC_INFO_REQ_T;


/* confirmation packet */
/*! Get Ext Sync Info Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_Ttag
{
  uint32_t ulExtSyncInfoFlags; /*!< Ext Sync Info Flags */
  uint64_t ullInternalTimestampNs; /*!< Last known 32 bit / 64 bit own network DC timestamp as seen by ExtSync device */
  uint64_t ullExternalTimestampNs; /*!< Last known 32 bit / 64 bit external timestamp as seen by ExtSync device */
  int32_t lTimeControlValueBySlave; /*!< Time value provided from ExtSync device data
  * Not used by master for actual ExtSync implementation */
  uint16_t usExtSyncStationAddress; /*!< Fixed station address of Ext Sync device */
  uint64_t ullDcToExtTimeOffsetNs; /*!< Time difference between this network and the other network providing the ExtSync base time.
  * Relationship of timestamps:
  * DcToExtTimeOffsetNs = ExternalTimestampNs - InternalTimestampNs
  */ /* internal DC timestamp (ns) + ullDcToExtTimeOffsetNs => external clock time (ns) */
  uint32_t ulLastUpdateDiffNs; /*!< Last delta time between ExtSync processing in ns */
  int32_t lLastControlDeltaDiffNs; /*!< ExtSync diagnostic data. Internal use */
  int32_t lLastControlDeltaDeltaDiffNs; /*!< ExtSync diagnostic data. Internal use */
  uint16_t usControlledStationAddress; /*!< Fixed station address of DC reference clock */
  uint32_t ulExtSyncUpdateCount; /*!< Counter of processed ExtSync updates */
  uint32_t ulDeviationPosMaxMag; /*!< Positive deviation maximum magnitude */
  uint32_t ulDeviationNegMaxMag; /*!< Negative deviation maximum magnitude */
} ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_T;

/* ulExtSyncInfoFlags */
/*! Definition of ulExtSyncInfoFlags */
typedef enum ECM_IF_EXT_SYNC_INFO_FLAGS_Etag
{
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_SYNC_MODE_SLAVE = 0x00000001, /*!< Master is using ExtSync reference provided by other network */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_IS_64BIT = 0x00000004, /*!< ExtSync timestamp size
  * If not set, the timestamp has 32 bit.
  * If set, the timestamp has 64 bit. */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_NOT_CONNECTED = 0x00000010, /*!< ExtSync device is not connected when bit is set */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_SYNC_MODE_MASTER = 0x00000020, /*!< Master is providing ExtSync to other network */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONNECTED_AS_SLAVE = 0x00008000, /*!< Ext Sync Device is connected as slave */ /* result of !(External Device Not Connected) && (Sync Mode.Bit 1) */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 0x00FF0000, /*!< Used internally */
  SRT_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 16, /*!< Used internally */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_DC_TO_EXT_OFFSET_VALID = 0x20000000, /*!< ullDcToExtTimeOffsetNs is valid */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_ACTIVE = 0x40000000, /*!< Master is actively using External synchronization on device */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONFIGURED = 0x80000000 /*!< Ext Sync device has been configured */
} ECM_IF_EXT_SYNC_INFO_FLAGS_E;

/*! Get Ext Sync Info Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF, ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF>>
 * \param ulLen X
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_EXT_SYNC_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_EXT_SYNC_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_EXT_SYNC_INFO_REQ_T tReq;
  ECM_IF_GET_EXT_SYNC_INFO_CNF_T tCnf;
} ECM_IF_GET_EXT_SYNC_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ/ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF
 */

/* request packet */
/*! Reset Ext Sync Max Deviations Request Packet Descriptions
 *
 * \param ulCmd <<ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ, ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_T;


/* confirmation packet */
/*! Reset Ext Sync Max Deviations Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF, ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_T;


/* packet union */
typedef union ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_T tReq;
  ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_T tCnf;
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ/ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_CNF
 */

/* request packet */
/*! Get Slave Handle Bit List Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_Ttag
{
  uint32_t ulListType; /*!< same enum as in RCX_GET_SLAVE_HANDLES_REQ */
  uint32_t ulStartHandle; /*!< first bit position in confirmation refers to the handle ulStartHandle */
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_T;

/*! Get Slave Handle Bit List Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ, ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ>>
 * \param ulLen 8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_T tData; /*!< Packet header */
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_T;


/* confirmation packet */
/*! Get Slave Handle Bit List Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_Ttag
{
  uint32_t ulListType; /*!< same enum as in RCX_GET_SLAVE_HANDLES_REQ */
  uint32_t ulStartHandle; /*!< same as in request */
  uint32_t ulNumHandleBits; /*!< number of handle bits in confirmation */
  uint8_t abBitMap[1024]; /*!< handle bit list */
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_T;

/*! Get Slave Handle Bit List Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF, ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF>>
 * \param ulLen 12+(ulNumHandleBits+7)/8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_T tReq;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_T tCnf;
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ/ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF
 */

/* request packet */
/*! Get Frame Loss Counters Request data structure
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetAfterRead; /*!< != 0 if values shall be reset to zero after reading */
} ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_T;

/*! Get Frame Loss Counters Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ, ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_T;


/* confirmation packet */
/*! Get Frame Loss Counter Entry
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_Ttag
{
  uint32_t ulMainPortCount; /*!< main port counter */
  uint32_t ulRedPortCount; /*!< redundancy port counter */
} ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_T;

#define ECM_IF_GET_FRAME_LOSS_COUNTERS_MAX_NUM_ENTRIES 8 /*!< Maximum number of entries */

/*! Get Frame Loss Counter Confirmation data structure
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_Ttag
{
  uint32_t fRedundancyEnabled; /*!< != 0 if redundancy is enabled */
  ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_T atEntries[ECM_IF_GET_FRAME_LOSS_COUNTERS_MAX_NUM_ENTRIES]; /*!< oer frane entries *//* has to be last entry */
} ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_T;

/*! Get Frame Loss Counter Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF, ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF>>
 * \param ullen 4+8*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_FRAME_LOSS_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_T tReq;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_FRAME_LOSS_COUNTERS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ/ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF
 */

/* request packet */
/*! Get Threshold Counters Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetOutOfRxWindowCumulativeCntAfterRead; /*!< reset out of rx window cumulative count after reading */
  uint32_t fResetFrameLostCumulativeCntAfterRead; /*!< reset frame lost cumulative count after reading */
} ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T;

typedef ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T ECM_IF_GET_THRESHOLD_COUNTERS_DATA_REQ_T;

/*! Get Threshold Counters Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ, ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ>>
 * \param ulLen 8
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_THRESHOLD_COUNTERS_REQ_T;


/* confirmation packet */
/*! Get Threshold Counters Confirmation data entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_THRESHOLD_COUNTER_ENTRY_Ttag
{
  uint32_t ulThresholdCnt; /*!< threshold count */
  uint32_t ulThreshold; /*!< threshold */
  uint32_t ulCumulativeCnt; /*!< cumulative count */
} ECM_IF_THRESHOLD_COUNTER_ENTRY_T;

/*! Get Threshold Counters Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_Ttag
{
  ECM_IF_THRESHOLD_COUNTER_ENTRY_T tOutOfRxWindow; /*!< Out Of Rx Window counters */
  ECM_IF_THRESHOLD_COUNTER_ENTRY_T tFrameLost; /*!< Frame Lost counters */
} ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_T;

/*! Get Threshold Counters Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF, ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF>>
 * \param ulLen 24
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_THRESHOLD_COUNTERS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_THRESHOLD_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_THRESHOLD_COUNTERS_REQ_T tReq;
  ECM_IF_GET_THRESHOLD_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_THRESHOLD_COUNTERS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_ERROR_COUNTERS_REQ/ECM_IF_CMD_GET_ERROR_COUNTERS_CNF
 */

/* request packet */
/*! Get Error Counters Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetAfterRead; /*!< if != 0 reset counters after reset */
} ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_T;

/*! Get Error Counters Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_ERROR_COUNTERS_REQ, ECM_IF_CMD_GET_ERROR_COUNTERS_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_ERROR_COUNTERS_REQ_T;

/* confirmation packet */
/*! Get Error Counters Confirmation port data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_Ttag
{
  uint32_t fValid;
  uint32_t ulTransmittedOk;
  uint32_t ulLinkDownDuringTransmission;
  uint32_t ulUtxUnderflowDuringTransmission;
  uint32_t ulFramesReceivedOk;
  uint32_t ulFcsErrors;
  uint32_t ulAlignmentErrors;
  uint32_t ulFrameTooLongErrors;
  uint32_t ulRuntFramesReceived;
  uint32_t ulCollisionFragmentsReceived;
  uint32_t ulDroppedDueLowResource;
  uint32_t ulDroppedDueUrxOverflow;
  uint32_t ulRxFatalError;
} ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T;

/*! Get Error Counters Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_Ttag
{
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T tMainPort; /*!< Main port */
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T tRedPort; /*!< Redundancy port */
} ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_T;

/*! Get Error Counters Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_ERROR_COUNTERS_CNF, ECM_IF_CMD_GET_ERROR_COUNTERS_CNF>>
 * \param ulLen X
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_ERROR_COUNTERS_CNF_T;

/* packet union */
typedef union ECM_IF_GET_ERROR_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_ERROR_COUNTERS_REQ_T tReq;
  ECM_IF_GET_ERROR_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_ERROR_COUNTERS_PCK_T;

/******************************************************************************
 * Packet: ECM_IF_CMD_READ_EMERGENCY_REQ/ECM_IF_CMD_READ_EMERGENCY_CNF
 */

/* request packet */
/*! Read Emergency Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_EMERGENCY_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t fDeleteEmergency; /*!< If != 0, clear the emergencies from internal buffer queue */
} ECM_IF_READ_EMERGENCY_REQ_DATA_T;

/*! Read Emergency Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_EMERGENCY_REQ, ECM_IF_CMD_READ_EMERGENCY_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_EMERGENCY_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_EMERGENCY_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_EMERGENCY_REQ_T;


/* confirmation packet */
/*! Read Emergency Confirmation Emergency Buffer
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_EMERGENCY_CNF_DATA_BUF_Ttag
{
  uint16_t usEmergencyBufBytes; /*!< counts all bytes of this structure from the start */
  uint16_t usErrorCode;/*!< CoE error code */
  uint8_t bErrorRegister; /*!< CoE error register (object 0x1001) */
  uint8_t abErrorData[__HIL_VARIABLE_LENGTH_ARRAY]; /*!< CoE Diagnostic data */
} ECM_IF_READ_EMERGENCY_CNF_DATA_BUF_T;

/*! Read Emergency Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_EMERGENCY_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Same as in request */
  uint16_t fDeleteEmergency; /*!< Same as in request */
  uint16_t fOverflowOccured;  /*!< If != 0 an overflow occured */
  uint16_t usNumberOfEmergencies; /*!< Emergencies returned in buffer */
  ECM_IF_READ_EMERGENCY_CNF_DATA_BUF_T atEmergencyBufferStart[__HIL_VARIABLE_LENGTH_ARRAY]; /*!< Multiple buffers put back to back. Actual size of each specified by the included buffer length */
} ECM_IF_READ_EMERGENCY_CNF_DATA_T;

/*! Read Emergency Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_EMERGENCY_CNF, ECM_IF_CMD_READ_EMERGENCY_CNF>>
 * \param ulLen 8+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_EMERGENCY_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;  /*!< Packet header */
  ECM_IF_READ_EMERGENCY_CNF_DATA_T tData;  /*!< Packet data */
} ECM_IF_READ_EMERGENCY_CNF_T;


/* packet union */
typedef union ECM_IF_READ_EMERGENCY_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_EMERGENCY_REQ_T tReq;
  ECM_IF_READ_EMERGENCY_CNF_T tCnf;
} ECM_IF_READ_EMERGENCY_PCK_T;

/**********************************************************************************************************************/


#define ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ 0x9E58 /*!< Select Sync Configuration Request */
#define ECM_IF_CMD_SELECT_SYNC_CONFIG_CNF 0x9E59 /*!< Select Sync Configuration Confirmation */

#define ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ 0x9E5A /*!< Enum Sync Configuration Request */
#define ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF 0x9E5B /*!< Enum Sync Configuration Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ/ECM_IF_CMD_SELECT_SYNC_CONFIG_CNF
 */

/* request packet */
/*! Select Sync Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_Ttag
{
  uint32_t ulSyncModuleId; /*! Sync module id */
  uint32_t ulTimeSyncModuleId; /*! Time sync module id */
  uint16_t usSlaveAddress; /*! Station address when using ExtSync configuration */
  uint32_t aulSyncModuleParameters[10]; /*! depends on ulSyncModuleTypeId */
  uint32_t aulTimeSyncModuleParameters[10]; /*! depends on ulTimeSyncModuleTypeId */
} ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_T;

/*! Select Sync Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ, ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ>>
 * \param ulLen 90
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SELECT_SYNC_CONFIG_REQ_T;

#define ECM_IF_SELECT_FIRST_EXTSYNC_SLAVE 0xFFFF /*!< Special value for usSlaveAddress to select first ExtSync suitable slave */

/* confirmation packet */
/*! Select Sync Configuration Confirmation Packet Description
 *
 * \param ulCmd ECM_IF_CMD_SELECT_SYNC_CONFIG_CNF
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_SELECT_SYNC_CONFIG_CNF_T;


/* packet union */
typedef union ECM_IF_SELECT_SYNC_CONFIG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SELECT_SYNC_CONFIG_REQ_T tReq;
  ECM_IF_SELECT_SYNC_CONFIG_CNF_T tCnf;
} ECM_IF_SELECT_SYNC_CONFIG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ/ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF
 */

/* request packet */
/*! Enum Sync Config Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_Ttag
{
  uint32_t ulIdx; /*!< Start index */
} ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_T;

/*! Enum Sync Config Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ, ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ENUM_SYNC_CONFIG_REQ_T;

/* special values for ulIdx */
#define ECM_IF_ENUM_SYNC_ACTIVE_SYNC_MODULE 0xFFFFFFFF /*!< Retrieve active sync module */
#define ECM_IF_ENUM_SYNC_ACTIVE_TIMESYNC_MODULE 0xFFFFFFFE /*!< Retrieve active time sync module */


/* confirmation packet */
/*! Enum Sync Config Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_Ttag
{
  uint32_t ulIdx; /*!< Module index */
  uint32_t ulEntryType; /*!< Entry type */
  uint32_t ulModuleId; /*!< Module id */
  uint32_t aulExcludes[(HIL_MAX_DATA_SIZE - sizeof(uint16_t) * 5) / sizeof(uint32_t)]; /*!< Exclusion List for module */
} ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_T;

/* ulEntryType */
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_SYNC 0 /*!< Entry denotes a sync module */
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_TIMESYNC 1 /*!< Entry denotes a time sync module */
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_EXTSYNC 2 /*!< Entry denotes an ExtSync capable slave. ulModuleId is now station address */

/* ulModuleId for ECM_IF_ENUM_SYNC_CONFIG_TYPE_SYNC */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_DEACTIVATED 0 /*!< Sync module deactivated */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_RXEND_POSEDGE 1 /*!< Sync module generates SYNC0 Pos Edge on RxEnd time point */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_RXEND_POSEDGE 2 /*!< Sync module generates SYNC1 Pos Edge on RxEnd time point */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_CYCLESTART_POSEDGE 3 /*!< Sync module generates SYNC0 Pos Edge on Cycle Start */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_CYCLESTART_POSEDGE 4 /*!< Sync module generates SYNC1 Pos Edge on Cycle Start */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_CYCLESTART_NEGEDGE 5 /*!< Sync module generates SYNC0 Neg Edge on Cycle Start */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_CYCLESTART_NEGEDGE 6 /*!< Sync module generates SYNC1 Neg Edge on Cycle Start */

/* ulModuleId for ECM_IF_ENUM_SYNC_CONFIG_TYPE_TIMESYNC */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_DEACTIVATED 0 /*!< Time sync module deactivated */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC0_LATCH_POSEDGE 1 /*!< Synchronize on SYNC0 Pos Edge */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC1_LATCH_POSEDGE 2 /*!< Synchronize on SYNC1 Pos Edge */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC0_LATCH_NEGEDGE 3 /*!< Synchronize on SYNC0 Neg Edge */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC1_LATCH_NEGEDGE 4 /*!< Synchronize on SYNC1 Neg Edge */

/*! Enum Sync Config Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF, ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF>>
 * \param ulLen 12+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ENUM_SYNC_CONFIG_CNF_T;


/* packet union */
typedef union ECM_IF_ENUM_SYNC_CONFIG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ENUM_SYNC_CONFIG_REQ_T tReq;
  ECM_IF_ENUM_SYNC_CONFIG_CNF_T tCnf;
} ECM_IF_ENUM_SYNC_CONFIG_PCK_T;


/**********************************************************************************************************************

#######  #######  ######   #######  #        #######   #####   #     #        ######   #######     #     ######   #######  #     #  #######
   #     #     #  #     #  #     #  #        #     #  #     #   #   #         #     #  #          # #    #     #  #     #  #     #     #
   #     #     #  #     #  #     #  #        #     #  #          # #          #     #  #         #   #   #     #  #     #  #     #     #
   #     #     #  ######   #     #  #        #     #  #  ####     #           ######   #####    #     #  #     #  #     #  #     #     #
   #     #     #  #        #     #  #        #     #  #     #     #           #   #    #        #######  #     #  #     #  #     #     #
   #     #     #  #        #     #  #        #     #  #     #     #           #    #   #        #     #  #     #  #     #  #     #     #
   #     #######  #        #######  #######  #######   #####      #           #     #  #######  #     #  ######   #######   #####      #

 */

#define ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ 0x9E50 /*!< Get Topology Info Request */
#define ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF 0x9E51 /*!< Get Topology Info Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ/ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
/*! Get Topology Info Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ, ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ>>
 * \param ulLen 0
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_GET_TOPOLOGY_INFO_REQ_T;


/* confirmation packet */
/*! Get Topology Info Confirmation data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_Ttag
{
  uint16_t usThisStationAddress; /*!< station address */
  uint16_t ausPortConnectedTo[4]; /*!< 0xFFFF = NOT CONNECTED, 0 = CONNECTED TO MASTER */
  /* Entries in order of auto-increment position */
} ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T;

#define ECM_IF_GET_TOPOLOGY_INFO_MAX_ENTRIES (1024 / sizeof(ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T))

#define ECM_IF_TOPOLOGY_INFO_PORT_NOT_CONNECTED 65535 /*!< Not Connected */

/*! Get Topology Info Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_Ttag
{
  uint32_t ulTotalNumOfListEntries; /*!< Number of list entries */
  uint32_t ulStartOfUnconnectedListEntries; /*!< First entry index of unconnected slaves. may point at table end */
  ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T atEntries[ECM_IF_GET_TOPOLOGY_INFO_MAX_ENTRIES]; /*!< List entries */
} ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_T;

/*! Get Topology Info Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF, ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF>>
 * \param ulLen 8+10*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_TOPOLOGY_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_TOPOLOGY_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TOPOLOGY_INFO_REQ_T tReq;
  ECM_IF_GET_TOPOLOGY_INFO_CNF_T tCnf;
} ECM_IF_GET_TOPOLOGY_INFO_PCK_T;


/**********************************************************************************************************************

 #####   #######  #     #  #######  ###   #####         ######   #######     #     ######   #######  #     #  #######
#     #  #     #  ##    #  #         #   #     #        #     #  #          # #    #     #  #     #  #     #     #
#        #     #  # #   #  #         #   #              #     #  #         #   #   #     #  #     #  #     #     #
#        #     #  #  #  #  #####     #   #  ####        ######   #####    #     #  #     #  #     #  #     #     #
#        #     #  #   # #  #         #   #     #        #   #    #        #######  #     #  #     #  #     #     #
#     #  #     #  #    ##  #         #   #     #        #    #   #        #     #  #     #  #     #  #     #     #
 #####   #######  #     #  #        ###   #####         #     #  #######  #     #  ######   #######   #####      #

 */

#define ECM_IF_CMD_GET_TIMING_INFO_REQ 0x9E20 /*!< Get Timing Info Request */
#define ECM_IF_CMD_GET_TIMING_INFO_CNF 0x9E21 /*!< Get Timing Info Confirmation */

#define ECM_IF_CMD_GET_WC_STATE_INFO_REQ 0x9E22 /*!< Get WcState Info Request */
#define ECM_IF_CMD_GET_WC_STATE_INFO_CNF 0x9E23 /*!< Get WcState Info Confirmation */

#define ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ 0x9E24 /*!< Get Cyclic Cmd Mapping Request */
#define ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF 0x9E25 /*!< Get Cyclic Cmd Mapping Confirmation */

#define ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ 0x9E26 /*!< Get Cyclic Slave Mapping Request */
#define ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF 0x9E27 /*!< Get Cyclic Slave Mapping Confirmation */

#define ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ 0x9E28 /*!< Get Slave Mbx Protocols Request */
#define ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF 0x9E29 /*!< Get Slave Mbx Protocols Confirmation */

#define ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ 0x9E2A /*!< Get Slave AoE Addresses Request */
#define ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF 0x9E2B /*!< Get Slave AoE Addresses Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_TIMING_INFO_REQ/ECM_IF_CMD_GET_TIMING_INFO_CNF
 */

/* request packet */
/*! Get Timing Info Request Packet Description
 *
 * \param ulCmd ECM_IF_CMD_GET_TIMING_INFO_REQ
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_TIMING_INFO_REQ_T;


/* confirmation packet */
/*! Get Timing Info Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_CNF_DATA_Ttag
{
  uint32_t ulBusCycleTimeNs; /*!< Bus cycle time in ns */
  uint32_t ulFrameTransmitTimeNs; /*!< Frame transmit time in ns */
  uint32_t ulExpectedBusDelayNs; /*!< Expected bus delay in ns */
  uint32_t ulExpectedRxEndTimeNs; /*!< Expected RxEnd time position from start of bus cycle transmission */
  uint32_t ulExpectedTxDataTimeNs; /*!< TxData time position from start of bus cycle transmission */
} ECM_IF_GET_TIMING_INFO_CNF_DATA_T;

/*! Get Timing Info Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_TIMING_INFO_CNF, ECM_IF_CMD_GET_TIMING_INFO_CNF>>
 * \param ulLen 20
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_TIMING_INFO_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_TIMING_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_TIMING_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TIMING_INFO_REQ_T tReq;
  ECM_IF_GET_TIMING_INFO_CNF_T tCnf;
} ECM_IF_GET_TIMING_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_WC_STATE_INFO_REQ/ECM_IF_CMD_GET_WC_STATE_INFO_CNF
 */

/* request packet */
/*! Get WcState Info Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< Start offset from when to read */
} ECM_IF_GET_WCSTATE_INFO_REQ_DATA_T;

/*! Get WcState Info Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_WC_STATE_INFO_REQ, ECM_IF_CMD_GET_WC_STATE_INFO_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_WCSTATE_INFO_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_WCSTATE_INFO_REQ_T;


/* confirmation packet */
/*! Get WcState Info Confirmation data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WCSTATE_INFO_ENTRY_Ttag
{
  uint32_t ulWcStateBitPosition; /*!< Bit position in Rx data image */
  uint16_t usTxImageStartByteOffset; /*!< TxData image start which is governed by this WcState bit */
  uint16_t usRxImageStartByteOffset; /*!< RxData image start which is governed by this WcState bit */
  uint16_t usImageByteLength; /*!< Length of images which are governed by this WcState bit */
  uint16_t usDirection; /*!< Images to which the WcState bit applies */
} ECM_IF_WCSTATE_INFO_ENTRY_T;

/*! WcState Direction Flags
 *
 */
enum ECM_IF_WCSTATE_DIRECTION_Etag
{
  MSK_ECM_IF_WCSTATE_INFO_DIRECTION_TXDATA = 0x0001,/*!< WcState bit applies to TxData */
  MSK_ECM_IF_WCSTATE_INFO_DIRECTION_RXDATA = 0x0002,/*!< WcState bit applies to RxData */
};

#define ECM_IF_MAX_WCSTATE_INFO_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T)) /*!< Maximum number of WcState entries per packet */

/*! Get WcState Info Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< same as in request */
  uint32_t ulTotalEntries; /*!< total number of available entries. this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_WCSTATE_INFO_ENTRY_T atEntries[ECM_IF_MAX_WCSTATE_INFO_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T;

/*! Get WcState Info Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_WC_STATE_INFO_CNF, ECM_IF_CMD_GET_WC_STATE_INFO_CNF>>
 * \param ulLen 8+n*12
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_WCSTATE_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_WCSTATE_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_WCSTATE_INFO_REQ_T tReq;
  ECM_IF_GET_WCSTATE_INFO_CNF_T tCnf;
} ECM_IF_GET_WCSTATE_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ/ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF
 */

/* request packet */
/*! Get Cyclic Cmd Mapping Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< Start offset from when to read */
} ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_T;

/*! Get Cyclic Cmd Mapping Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ, ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_T;


/* confirmation packet */
/*! Get Cyclic Cmd Mapping Confirmation data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_Ttag
{
  uint16_t usTransmitType; /*!< Transmit type */
  uint16_t usReceiveType; /*!< Receive type */
  uint16_t usTxImageStartByteOffset; /*!< Transmit data start byte offset */
  uint16_t usRxImageStartByteOffset; /*!< Receive data start byte offset */
  uint16_t usImageByteLength; /*!< Byte length of data */
  uint16_t usWkcCompareReceiveByteOffset; /*!< Wkc Byte offset only valid if not set to 0xFFFF */
} ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T;

/* usTransmitType / usReceiveType */
/*! Transmit and Receive Type
 *
 */
enum ECM_IF_CYCLIC_CMD_DATATYPE_Etag
{
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_UNUSED = 0,             /*!< Unused */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_PROCESS_DATA = 1,       /*!< Process Data */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_DC_SYSTIME = 2,         /*!< DC SysTime */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_ALSTATUS = 3,       /*!< BRD ALStatus data */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_DC_SYSTIME_DIFF = 4,/*!< BRD DC Systime Diff data */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_WCSTATE_BITS = 5,       /*!< WcState bits */
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_EXTSYNC_STATUS = 6      /*!< ExtSync status */
};

#define ECM_IF_MAX_CYCLIC_CMD_MAPPING_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T)) /*!< Maximum number of Cmd mapping entries per packet */

/*! Get Cyclic Cmd Mapping Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< same as in request */
  uint32_t ulTotalEntries; /*!< total number of available entries. this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T atEntries[ECM_IF_MAX_CYCLIC_CMD_MAPPING_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_T;

/*! Get Cyclic Cmd Mapping Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF, ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF>>
 * \param ulLen 8+12*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_T;


/* packet union */
typedef union ECM_IF_GET_CYCLIC_CMD_MAPPING_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_T tReq;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_T tCnf;
} ECM_IF_GET_CYCLIC_CMD_MAPPING_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ/ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF
 */

/* request packet */
/*! Get Cyclic Slave Mapping Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< Start offset from when to read */
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_T;

/*! Get Cyclic Slave Mapping Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ, ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_T;


/* confirmation packet */
/*! Get Cyclic Slave Mapping Confirmation data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_Ttag
{
  uint16_t usDirection; /*!< Data direction of entry */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usWkcCompareReceiveByteOffset; /*! Related wkc byte count, only valid if not set to 0xFFFF */
  uint32_t ulWcStateBitOffset; /*!< Related WcState bit, only valid if not set to 0xFFFFFFFF */
  uint32_t ulImageStartBitOffset; /*!< Image start bit offset */
  uint32_t ulImageBitLength; /*!< Image bit length */

  uint32_t ulBitOffsetWithin; /* when bSmNo == 0xFF and bFmmmuNo = 0xFF, we provide register address multiplied by 8 */
  uint8_t bSmNo; /*!< related Sync Manager, 0xFF not set */
  uint8_t bFmmuNo; /*!< related FMMU, 0xFF not set */

  uint16_t usReserved;
} ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T;

/* usDirection */
/*! Cyclic Slave Mapping Direction Type
 *
 */
enum ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_Etag
{
  VAL_ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_TRANSMIT = 1,/*!< Transmit Data */
  VAL_ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_RECEIVE = 2  /*!< Receive Data */
};

#define ECM_IF_MAX_CYCLIC_SLAVE_MAPPING_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T)) /*!< Maximum number of cyclic slave mapping entries per packet */

/*! Get Cyclic Slave Mapping Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset; /*!< same as in request */
  uint32_t ulTotalEntries; /*!< total number of available entries. this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T atEntries[ECM_IF_MAX_CYCLIC_SLAVE_MAPPING_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_T;

/*! Get Cyclic Slave Mapping Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF, ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF>>
 * \param ulLen 8+26*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_T;


/* packet union */
typedef union ECM_IF_GET_CYCLIC_SLAVE_MAPPING_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_T tReq;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_T tCnf;
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_PCK_T;

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ/ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF
 */

#define ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES 128 /*!< Max number of Mbx Protocols entries per packet */

/* request packet */
/*! Get Slave Mbx Protocols Request data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usMbxProtocols; /*!< filled in on confirmation and will hold the available mailbox protocols */
  uint32_t ulReserved;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T;

/*! Get Slave Mbx Protocols data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_Ttag
{
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_T;

/*! Get Slave Mbx Protocols Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ, ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ>>
 * \param ulLen 8*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_T;


/* confirmation packet */
/*! Get Slave Mbx Protocols Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_Ttag
{
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES];
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_T;

/*! Get Slave Mbx Protocols Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF, ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF>>
 * \param ulLen 8*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_MBX_PROTOCOLS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_T tReq;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_T tCnf;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ/ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF
 */

#define ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES 128 /*!< Max number of AoE address entries per packet */

/* request packet */
/*! Get Slave AoE Addresses Request data structure entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t abNetId[6]; /*!< AoE address, filled in on confirmation (SBZ if no aoe support) */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T;

/*! Get Slave AoE Addresses Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_Ttag
{
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_T;

/*! Get Slave AoE Addresses Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ, ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ>>
 * \param ulLen 8*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_T;


/* confirmation packet */
/*! Get Slave AoE Addresses Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_Ttag
{
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES]; /*!< Entry table */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_T;

/*! Get Slave AoE Addresses Confirmation PAcket Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF, ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF>>
 * \param ulLen 8*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_AOE_ADDRESSES_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_T tReq;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_T tCnf;
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_PCK_T;


/**********************************************************************************************************************

######   ###     #      #####   #     #  #######   #####   #######  ###   #####         #        #######   #####
#     #   #     # #    #     #  ##    #  #     #  #     #     #      #   #     #        #        #     #  #     #
#     #   #    #   #   #        # #   #  #     #  #           #      #   #              #        #     #  #
#     #   #   #     #  #  ####  #  #  #  #     #   #####      #      #   #              #        #     #  #  ####
#     #   #   #######  #     #  #   # #  #     #        #     #      #   #              #        #     #  #     #
#     #   #   #     #  #     #  #    ##  #     #  #     #     #      #   #     #        #        #     #  #     #
######   ###  #     #   #####   #     #  #######   #####      #     ###   #####         #######  #######   #####

 */

/* 9E30-9E4F */

#define ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ 0x9E30 /*!< Read Diag Log Entry Request */
#define ECM_IF_CMD_READ_DIAG_LOG_ENTRY_CNF 0x9E31 /*!< Read Diag Log Entry Confirmation */

#define ECM_IF_CMD_CLEAR_DIAG_LOG_REQ 0x9E32 /*!< Clear Diag Log Request */
#define ECM_IF_CMD_CLEAR_DIAG_LOG_CNF 0x9E33 /*!< Clear Diag Log Confirmation */

#define ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND 0x9E34 /*!< New Diag Log Entries Indication */
#define ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES 0x9E35 /*!< New Diag Log Entries Response */

#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ 0x9E36 /*!< Diag Log Indications Register Request */
#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF 0x9E37 /*!< Diag Log Indications Register Confirmation */

#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ 0x9E38 /*!< Diag Log Indications Unregister Request */
#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF 0x9E39 /*!< Diag Log Indications Unregister Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ/ECM_IF_CMD_READ_DIAG_LOG_ENTRY_CNF
 */

/* request packet */
/*! Read Diag Log Entry Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ, ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_REQ_Ttag
{
  HIL_PACKET_HEADER_T                               tHead; /*!< Packet header */
} ECM_IF_READ_DIAG_LOG_ENTRY_REQ_T;


/* confirmation packet */
/*! Read Diag Log Entry Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_Ttag
{
  uint32_t ulLostEntries; /*!< Number of lost entries in between */
  ECM_DIAG_ENTRY_T tDiagEntry; /*!< Diag log entry */
} ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_T;

/*! Read Diag Log Entry Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ, ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ>>
 * \param ulLen X
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_READ_DIAG_LOG_ENTRY_CNF_T;


/* packet union */
typedef union ECM_IF_READ_DIAG_LOG_ENTRY_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_DIAG_LOG_ENTRY_REQ_T tReq;
  ECM_IF_READ_DIAG_LOG_ENTRY_CNF_T tCnf;
} ECM_IF_READ_DIAG_LOG_ENTRY_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_CLEAR_DIAG_LOG_REQ/ECM_IF_CMD_CLEAR_DIAG_LOG_CNF
 */

/* request packet */
/*! Clear Diag Log Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_CLEAR_DIAG_LOG_REQ, ECM_IF_CMD_CLEAR_DIAG_LOG_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CLEAR_DIAG_LOG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_CLEAR_DIAG_LOG_REQ_T;


/* confirmation packet */
/*! Clear Diag Log Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_CLEAR_DIAG_LOG_CNF, ECM_IF_CMD_CLEAR_DIAG_LOG_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CLEAR_DIAG_LOG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_CLEAR_DIAG_LOG_CNF_T;


/* packet union */
typedef union ECM_IF_CLEAR_DIAG_LOG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_CLEAR_DIAG_LOG_REQ_T tReq;
  ECM_IF_CLEAR_DIAG_LOG_CNF_T tCnf;
} ECM_IF_CLEAR_DIAG_LOG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND/ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES
 */

/* indication packet */
/*! New Diag Log Entries Indication data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_Ttag
{
  uint16_t usNumOfDiagEntries; /*!< Number of diag entries available */
} ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_T;

/*! New Diag Log Entries Indication Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND, ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_T tData; /*!< Packet data */
} ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_T;


/* response packet */
/*! New Diag Log Entries Response Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES, ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_T;


/* packet union */
typedef union ECM_IF_NEW_DIAG_LOG_ENTRIES_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_T tInd;
  ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_T tRes;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ/ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF
 */

/* request packet */
/*! Diag Log Indications Register Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ, ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_T;


/* confirmation packet */
/*! Diag Log Indications Register Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_Ttag
{
  uint16_t usReserved;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_T;

/*! Diag Log Indications Register Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF, ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_T;


/* packet union */
typedef union ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_T tReq;
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_T tCnf;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ/ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF
 */

/* request packet */
/*! Diag Log Indications Unregister Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ, ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_T;


/* confirmation packet */
/*! Diag Log Indications Unregister Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF, ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_T;


/* packet union */
typedef union ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_T tReq;
  ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_T tCnf;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_PCK_T;


/**********************************************************************************************************************

 #####   #     #  ######   ######   #######  ######   #######  #######  ######            #     ######   ###   #####
#     #  #     #  #     #  #     #  #     #  #     #     #     #        #     #          # #    #     #   #   #     #
#        #     #  #     #  #     #  #     #  #     #     #     #        #     #         #   #   #     #   #   #
 #####   #     #  ######   ######   #     #  ######      #     #####    #     #        #     #  ######    #    #####
      #  #     #  #        #        #     #  #   #       #     #        #     #        #######  #         #         #
#     #  #     #  #        #        #     #  #    #      #     #        #     #        #     #  #         #   #     #
 #####    #####   #        #        #######  #     #     #     #######  ######         #     #  #        ###   #####

 */

#define ECM_IF_CMD_GET_SUPPORTED_APIS_REQ 0x9E90 /*!< Get Supported APIs Request */
#define ECM_IF_CMD_GET_SUPPORTED_APIS_CNF 0x9E91 /*!< Get Supported APIs Confirmation */

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SUPPORTED_APIS_REQ/ECM_IF_CMD_GET_SUPPORTED_APIS_CNF
 */

/* request packet */
/*! Get Supported APIs Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SUPPORTED_APIS_REQ,ECM_IF_CMD_GET_SUPPORTED_APIS_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_SUPPORTED_APIS_REQ_T;


/* confirmation packet */
/*! Get Supported APIs Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_Ttag
{
  uint32_t aulSupportedApis[256]; /*!< Entry table */
} ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_T;

/*! Get Supported APIs Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_GET_SUPPORTED_APIS_CNF, ECM_IF_CMD_GET_SUPPORTED_APIS_CNF>>
 * \param ulLen 4*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_GET_SUPPORTED_APIS_CNF_T;

/*! Get SUpported APIs coding
 *
 */
enum ECM_IF_GET_SUPPORTED_APIS_CODING_Etag
{
  MSK_ECM_IF_SUPPORTED_API_TYPE = 0xFFFF0000, /*!< API Type mask */
  MSK_ECM_IF_SUPPORTED_API_VERSION = 0x0000FFFF, /*!< API type version */

  VAL_ECM_IF_SUPPORTED_API_TYPE_ECMV4 = 0x00010000, /*!< EcmV4 Base API, always used with version zero */
  VAL_ECM_IF_SUPPORTED_API_TYPE_MANUAL_MODE = 0x00020000, /*!< Manual Mode Control API */
};

/* packet union */
typedef union ECM_IF_GET_SUPPORTED_APIS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SUPPORTED_APIS_REQ_T tReq;
  ECM_IF_GET_SUPPORTED_APIS_CNF_T tCnf;
} ECM_IF_GET_SUPPORTED_APIS_PCK_T;


/**********************************************************************************************************************

 #####   #######  #     #  #######  ###   #####         ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  ##    #  #         #   #     #         #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  # #   #  #         #   #               #   # #   #     #     #        #     #  #         #   #   #        #
#        #     #  #  #  #  #####     #   #  ####         #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #   # #  #         #   #     #         #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #    ##  #         #   #     #         #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #     #  #        ###   #####         ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

#define ECM_IF_CMD_BEGIN_CONFIGURATION_REQ 0x9EA0 /*!< Begin Configuration Request */
#define ECM_IF_CMD_BEGIN_CONFIGURATION_CNF 0x9EA1 /*!< Begin Configuration Confirmation */

#define ECM_IF_CMD_END_CONFIGURATION_REQ 0x9EA2 /*!< End Configuration Request */
#define ECM_IF_CMD_END_CONFIGURATION_CNF 0x9EA3 /*!< End Configuration Confirmation */

#define ECM_IF_CMD_ABORT_CONFIGURATION_REQ 0x9EA4 /*!< Abort Configuration Request */
#define ECM_IF_CMD_ABORT_CONFIGURATION_CNF 0x9EA5 /*!< ABort Configuration Confirmation */

#define ECM_IF_CMD_LOAD_ENI_REQ 0x9EA6 /*!< Load ENI Request */
#define ECM_IF_CMD_LOAD_ENI_CNF 0x9EA7 /*!< Load ENI Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_REQ 0x9EA8 /*!< Add Slave Request */
#define ECM_IF_CMD_ADD_SLAVE_CNF 0x9EA9 /*!< Add Slave Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ 0x9EAA /*!< Add Slave Mailbox Request */
#define ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF 0x9EAB /*!< Add Slave Mailbox Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_REQ 0x9EAC /*!< Add Slave Mbx InitCmd Request */
#define ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_CNF 0x9EAD /*!< Add Slave Mbx InitCmd Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ 0x9EAE /*!< Add Slave CoE InitCmd Request */
#define ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF 0x9EAF /*!< Add Slave CoE InitCmd Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ 0x9EB0 /*!< Add Slave SoE InitCmd Request */
#define ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF 0x9EB1 /*!< Add Slave SoE InitCmd Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ 0x9EB2 /*!< Add Slave Register InitCmd Request */
#define ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_CNF 0x9EB3 /*!< Add Slave Register InitCmd Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ 0x9EB4 /*!< Add Slave DC Params Request */
#define ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF 0x9EB5 /*!< Add Slave DC Params Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ 0x9EB6 /*!< Add Slave ESM Timeouts Request */
#define ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF 0x9EB7 /*!< Add Slave ESM Timeouts Confirmation */

#define ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ 0x9EB8 /*!< Add Cyclic Frame Request */
#define ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF 0x9EB9 /*!< Add Cyclic Frame Confirmation */

#define ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ 0x9EBA /*!< Add Cyclic Telegram Request */
#define ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF 0x9EBB /*!< Add Cyclic Telegram Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ 0x9EBC /*!< Add Slave SM Cfg Request */
#define ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF 0x9EBD /*!< Add Slave SM Cfg Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ 0x9EBE /*!< Add Slave FMMU Cfg Request */
#define ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF 0x9EBF /*!< Add Slave FMMU Cfg Confirmation */

#define ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ 0x9EC0 /*!< Add Mandatory Slave List Request */
#define ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF 0x9EC1 /*!< Add Mandatory SLave List Confirmation */

#define ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ 0x9EC2 /*!< Unload Configuration Request */
#define ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF 0x9EC3 /*!< Unload COnfiguration Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ 0x9EC4 /*!< Add Slave EoE IP Param Request */
#define ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF 0x9EC5 /*!< Add Slave EoE IP Param Confirmation */

#define ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ 0x9EC6 /*!< Set Default Target State Request */
#define ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF 0x9EC7 /*!< Set Default Target State Confirmation */

#define ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ 0x9EC8 /*!< Add Hot Connect Group Request */
#define ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF 0x9EC9 /*!< Add Hot Connect Group Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ 0x9ECA /*!< Add Slave ESC Timeouts Request */
#define ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF 0x9ECB /*!< Add Slave ESC Timeouts Confirmation */

#define ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ 0x9ECC /*! Set Base Sync Offset Percentage Request */
#define ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF 0x9ECD /*! Set Base Sync Offset Percentage Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ 0x9ECE /*! Add Slave AoE NetID Cfg Request */
#define ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF 0x9ECF /*! Add Slave AoE NetID Cfg Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ 0x9ED2 /*! Add Slave Assign PDO Cfg Request */
#define ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_CNF 0x9ED3 /*! Add Slave Assign PDO Cfg Confirmation */

#define ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_REQ 0x9ED4 /*! Add Slave PDO Mapping Cfg Request */
#define ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_CNF 0x9ED5 /*! Add Slave PDO Mapping Cfg Confirmation */

/******************************************************************************
 * bMinState
 */
/*! Definitions for bMinState
 *
 */
typedef enum ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_Etag
{
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_BOOT = 0,  /*!< Minimum required state is BOOT */
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_PREOP = 1, /*!< Minimum required state is PREOP */
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_SAFEOP = 2,/*!< Minimum required state is SAFEOP */
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_OP = 3,    /*!< Minimum required state is OP */
} ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_E;

/******************************************************************************
 * ulTransitionFlags
 */
/*! Definitions for ulTransitionFlags
 *
 */
typedef enum ECM_IF_TRANSITION_FLAGS_Etag
{
  MSK_ECM_IF_TRANSITION_FLAGS_IP_BEFORE_ALCONTROL = 1 << 0, /*!< INIT->PREOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PS_BEFORE_ALCONTROL = 1 << 1, /*!< PREOP->SAFEOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PI_BEFORE_ALCONTROL = 1 << 2, /*!< PREOP->INIT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SP_BEFORE_ALCONTROL = 1 << 3, /*!< SAFEOP->PREOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SO_BEFORE_ALCONTROL = 1 << 4, /*!< SAFEOP->OP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SI_BEFORE_ALCONTROL = 1 << 5, /*!< SAFEOP->INIT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OS_BEFORE_ALCONTROL = 1 << 6, /*!< OP->SAFEOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OP_BEFORE_ALCONTROL = 1 << 7, /*!< OP->PREOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OI_BEFORE_ALCONTROL = 1 << 8, /*!< OP->INIT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_IB_BEFORE_ALCONTROL = 1 << 9, /*!< INIT->BOOT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_BI_BEFORE_ALCONTROL = 1 << 10, /*!< BOOT->INIT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_II_BEFORE_ALCONTROL = 1 << 11, /*!< INIT+ERR->INIT before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PP_BEFORE_ALCONTROL = 1 << 12, /*!< PREOP+ERR->PREOP before ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SS_BEFORE_ALCONTROL = 1 << 13, /*!< SAFEOP+ERR->SAFEOP before ALControl */

  MSK_ECM_IF_TRANSITION_FLAGS_IP_AFTER_ALCONTROL = 1 << 16, /*!< INIT->PREOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PS_AFTER_ALCONTROL = 1 << 17, /*!< PREOP->SAFEOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PI_AFTER_ALCONTROL = 1 << 18, /*!< PREOP->INIT after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SP_AFTER_ALCONTROL = 1 << 19, /*!< SAFEOP->PREOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SO_AFTER_ALCONTROL = 1 << 20, /*!< SAFEOP->OP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SI_AFTER_ALCONTROL = 1 << 21, /*!< SAFEOP->INIT after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OS_AFTER_ALCONTROL = 1 << 22, /*!< OP->SAFEOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OP_AFTER_ALCONTROL = 1 << 23, /*!< OP->PREOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_OI_AFTER_ALCONTROL = 1 << 24, /*!< OP->INIT after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_IB_AFTER_ALCONTROL = 1 << 25, /*!< INIT->BOOT after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_II_AFTER_ALCONTROL = 1 << 26, /*!< INIT+ERR->INIT after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_PP_AFTER_ALCONTROL = 1 << 27, /*!< PREOP+ERR->PREOP after ALControl */
  MSK_ECM_IF_TRANSITION_FLAGS_SS_AFTER_ALCONTROL = 1 << 28, /*!< SAFEOP+ERR->SAFEOP after ALControl */
} ECM_IF_TRANSITION_FLAGS_E;

/******************************************************************************
 * usSlaveReconnectRestartMode
 */
/*! Definitions for usSlaveReconnectRestartMode
 */
typedef enum ECM_IF_SLAVE_RECONNECT_RESTART_MODE_Etag
{
  ECM_IF_SLAVE_RECONNECT_RESTART_NO_SLAVES = 0, /*!< Slave Reconnect Restart No Slaves */
  ECM_IF_SLAVE_RECONNECT_RESTART_ALL_SLAVES = 2 /*!< Slave Reconnect Restart All Slaves */
} ECM_IF_SLAVE_RECONNECT_RESTART_MODE_E;

/******************************************************************************
 * Packet: ECM_IF_CMD_BEGIN_CONFIGURATION_REQ/ECM_IF_CMD_BEGIN_CONFIGURATION_CNF
 */

/* request packet */
/*! Begin Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_Ttag
{
  uint16_t usTxProcDataImageBytes; /*!< Maximum used Tx process data image bytes */
  uint16_t usRxProcDataImageBytes; /*!< Maximum used Rx process data image bytes */
  uint16_t usCcProcDataImageBytes; /*!< Maximum used cross communication process data image bytes */
  uint16_t usSlaveReconnectRestartMode; /*!< Slave reconnect restart mode */
  /* 8 bytes */
  uint32_t ulStackConfigFlags; /*!< Stack configuration flags */
  /* 12 bytes */
  uint32_t ulWatchdogTime; /*!< Watchdog time in ms */
  /* 16 bytes */
  uint32_t ulBaseSyncOffsetPercentage; /*!< Base sync offset percentage. value range 0-10000 resembling 0-100.00%. Enabled with ECM_IF_STACK_CONFIG_FLAGS_USE_CUSTOM_BASE_SYNC_OFFSET_PERCENTAGE */
  uint16_t usVlanId; /*!< Vlan Id if selected in ulStackConfigFlags */
  uint8_t bCyclicVlanPriority; /*!< Vlan priority for cyclic frames if selected in ulStackConfigFlags */
  uint8_t bAcyclicVlanPriority; /*!< Vlan priority for acyclic frames if selected in ulStackConfigFlags */
  uint32_t ulBaseCycleTimeNs; /*!< Base cycle time in ns */
  uint32_t aulReserved[14]; /*!< set to zero */
  /* 84 bytes */
} ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_T;

#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV1_SIZE 8
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV2_SIZE 12
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV3_SIZE 16
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV4_SIZE 84

/* ulStackConfigFlags */
/*! Definitions for ulStackConfigFlags
 *
 */
typedef enum ECM_IF_STACK_CONFIG_FLAGS_Etag
{
  ECM_IF_STACK_CONFIG_FLAGS_STAY_IN_INIT_WHEN_MASTER_LINK_DOWN = 0x00000001, /*!< Stay in INIT when master detects link down */
  ECM_IF_STACK_CONFIG_FLAGS_USE_CUSTOM_BASE_SYNC_OFFSET_PERCENTAGE = 0x00000004,/*!< Use custom base sync offset percentage */
  ECM_IF_STACK_CONFIG_FLAGS_USE_VLAN_TAGGING = 0x00000008, /*!< Enable VLAN tagging */
} ECM_IF_STACK_CONFIG_FLAGS_E;

/*! Begin Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_BEGIN_CONFIGURATION_REQ, ECM_IF_CMD_BEGIN_CONFIGURATION_REQ>>
 * \param ulLen 84
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_BEGIN_CONFIGURATION_REQ_T;


/* confirmation packet */
/*! Begin Configuration Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_BEGIN_CONFIGURATION_CNF, ECM_IF_CMD_BEGIN_CONFIGURATION_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_BEGIN_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_BEGIN_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_BEGIN_CONFIGURATION_REQ_T tReq;
  ECM_IF_BEGIN_CONFIGURATION_CNF_T tCnf;
} ECM_IF_BEGIN_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_END_CONFIGURATION_REQ/ECM_IF_CMD_END_CONFIGURATION_CNF
 */

/* request packet */
/*! End Configuration Request Packet Description
 *
 * End a configuration packet stream
 *
 * \param ulCmd <<ECM_IF_CMD_END_CONFIGURATION_REQ, ECM_IF_CMD_END_CONFIGURATION_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*! < Packet header. */
} ECM_IF_END_CONFIGURATION_REQ_T;


/* confirmation packet */
/*! End Configuration Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_CNF_DATA_Ttag
{
  uint32_t ulNumSlavesInConfig; /*!< Number of slaves in config */
  uint32_t ulActualTxDataBytes; /*!< Actual used TxData bytes */
  uint32_t ulActualRxDataBytes; /*!< Actual used RxData bytes */
} ECM_IF_END_CONFIGURATION_CNF_DATA_T;

/*! End Configuration Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_END_CONFIGURATION_CNF, ECM_IF_CMD_END_CONFIGURATION_CNF>>
 * \param ulLen 12
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_END_CONFIGURATION_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_END_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_END_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_END_CONFIGURATION_REQ_T tReq;
  ECM_IF_END_CONFIGURATION_CNF_T tCnf;
} ECM_IF_END_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ABORT_CONFIGURATION_REQ/ECM_IF_CMD_ABORT_CONFIGURATION_CNF
 */

/* request packet */
/*! Abort Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ABORT_CONFIGURATION_REQ, ECM_IF_CMD_ABORT_CONFIGURATION_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ABORT_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_ABORT_CONFIGURATION_REQ_T;


/* confirmation packet */
/*! Abort Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ABORT_CONFIGURATION_CNF, ECM_IF_CMD_ABORT_CONFIGURATION_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ABORT_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_ABORT_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_ABORT_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ABORT_CONFIGURATION_REQ_T tReq;
  ECM_IF_ABORT_CONFIGURATION_CNF_T tCnf;
} ECM_IF_ABORT_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ/ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF
 */

/* request packet */
/*! Unload Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ, ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_UNLOAD_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_UNLOAD_CONFIGURATION_REQ_T;


/* confirmation packet */
/*! Unload Configuration Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF, ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_UNLOAD_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_UNLOAD_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_UNLOAD_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_UNLOAD_CONFIGURATION_REQ_T tReq;
  ECM_IF_UNLOAD_CONFIGURATION_CNF_T tCnf;
} ECM_IF_UNLOAD_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_LOAD_ENI_REQ/ECM_IF_CMD_LOAD_ENI_CNF
 */

/* request packet */
/*! Definitions for ulLoadType
 *
 */
typedef enum ECM_IF_LOAD_ENI_REQ_TYPE_Etag
{
  ECM_IF_LOAD_ENI_XML_UNCOMPRESSED = 0,/*!< Load plain ENI XML file */
  ECM_IF_LOAD_ENI_CONFIG_NXD_ECMv3 = 1,/*!< Load config.nxd */
  ECM_IF_LOAD_ENI_XML_LZMA = 2,        /*!< Load LZMA compressed ENI XML file */
} ECM_IF_LOAD_ENI_REQ_TYPE_E;

/*! Load ENI Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_REQ_DATA_Ttag
{
  uint32_t ulLoadType; /*!< ENI laod type */
} ECM_IF_LOAD_ENI_REQ_DATA_T;

/*! Load ENI Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_LOAD_ENI_REQ, ECM_IF_CMD_LOAD_ENI_REQ>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_LOAD_ENI_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_LOAD_ENI_REQ_T;


/* confirmation packet */
/*! Load ENI Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_LOAD_ENI_CNF, ECM_IF_CMD_LOAD_ENI_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_LOAD_ENI_CNF_T;


/* packet union */
typedef union ECM_IF_LOAD_ENI_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_LOAD_ENI_REQ_T tReq;
  ECM_IF_LOAD_ENI_CNF_T tCnf;
} ECM_IF_LOAD_ENI_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_REQ/ECM_IF_CMD_ADD_SLAVE_CNF
 */

/* request packet */
/*! Add Slave Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulIdentificationFlags; /*!< Identification values. Default value 0x00000017 */
  uint32_t ulVendorId; /*!< Vendor id */
  uint32_t ulProductCode; /*!< Product code */
  uint32_t ulRevisionNumber; /*!< Revision number */
  uint32_t ulSerialNumber; /*!< Serial number */
  uint32_t ulSlaveConfigFlags; /*!< Slave configuration flags */
  uint8_t abSlaveName[__HIL_VARIABLE_LENGTH_ARRAY]; /*!< Slave name [optional] */
} ECM_IF_ADD_SLAVE_REQ_DATA_T;

/* ulIdentificationFlags */
/*! Definitions for ulIdenticationFlags
 *
 */
typedef enum ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_Etag
{
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_VENDORID = 0x00000001, /*!< Check vendor id */
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_PRODUCTCODE = 0x00000002, /*!< Check product code */
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO_LO = 0x00000004, /*!< Check low word of revision number */
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO_HI = 0x00000010, /*!< Check high word of revision number */
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO = 0x00000014, /*!< Check full revision number */
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_SERIALNO = 0x00000008, /*!< Check serial number */
} ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_E;

/* ulSlaveConfigFlags */
/*! Definitions for ulSlaveConfigFlags
 *
 */
typedef enum ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_Etag
{
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_REINIT_AFTER_COMM_ERROR = 0x00000001, /*!< Re-Init after Comm Errors */
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_AUTO_RESTORE_STATE = 0x00000002, /*!< Auto-Restore state after reconnect */
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_OPTIONAL_SLAVE = 0x00000004, /*!< every slave behind will be optional as segment as well even though the flag is not set on those */
} ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_E;

/*! Add Slave Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_REQ, ECM_IF_CMD_ADD_SLAVE_REQ>>
 * \param ulLen 26
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_REQ_T;


/* confirmation packet */
/*! Add Slave Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_ADD_SLAVE_CNF_DATA_T;

/*! Add Slave Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_CNF,ECM_IF_CMD_ADD_SLAVE_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REQ_T tReq;
  ECM_IF_ADD_SLAVE_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ/ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF
 */

/* usMbxProtocols */
/*! Definitions for usMbxProtocols
 *
 */
typedef enum ECM_IF_SLAVE_MAILBOX_CFG_PROTOCOL_FLAGS_Etag
{
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_AOE = 1 << 1, /*!< AoE mailbox protocol */
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_EOE = 1 << 2, /*!< EoE mailbox protocol */
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_COE = 1 << 3, /*!< CoE mailbox protocol */
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_FOE = 1 << 4, /*!< FoE mailbox protocol */
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_SOE = 1 << 5, /*!< SoE mailbox protocol */
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_VOE = 1 << 15,/*!< VoE mailbox protocol */
} ECM_IF_SLAVE_MAILBOX_CFG_PROTOCOL_FLAGS_E;

/* request packet */
/*! Add Slave Mailbox Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */

  uint16_t usStdTxMbxPhysOffset; /*!< Std Tx Mbx phys offset */
  uint16_t usStdTxMbxSize; /*!< Std Tx Mbx phys size */

  uint16_t usStdRxMbxPhysOffset; /*!< Std Rx Mbx phys offset */
  uint16_t usStdRxMbxSize; /*!< Std Rx Mbx phys size */

  uint8_t bStdTxSmControlByte; /*!< Std Tx Mbx SM control byte */
  uint8_t bStdRxSmControlByte; /*!< Std Rx Mbx SM control byte */

  uint8_t bStdTxMbxSmNo; /*!< Std Tx Mbx SM number */
  uint8_t bStdRxMbxSmNo; /*!< Std Rx Mbx SM number */

  uint16_t usMbxStateBitNo; /*!< MbxState bit number */

  uint16_t usBootTxMbxPhysOffset; /*!< Boot Tx Mbx phys offset */
  uint16_t usBootTxMbxSize; /*!< Boot Tx Mbx phys size */

  uint16_t usBootRxMbxPhysOffset; /*!< Boot Rx Mbx phys offset */
  uint16_t usBootRxMbxSize; /*!< Boot Rx Mbx phys size */

  uint8_t bBootTxSmControlByte; /*!< Boot Tx Mbx SM control byte */
  uint8_t bBootRxSmControlByte; /*!< Boot Rx Mbx SM control byte */

  uint8_t bBootTxMbxSmNo; /*!< Boot Tx Mbx SM number */
  uint8_t bBootRxMbxSmNo; /*!< Boot Rx Mbx SM number */

  uint16_t usMbxProtocols; /*!< Available mailbox protocols */
} ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_T;

/*! Add Slave Mailbox Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ, ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ>>
 * \param ulLen 30
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_MAILBOX_REQ_T;


/* confirmation packet */
/*! Add Slave Mailbox Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_T;

/*! Add Slave Mailbox Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF, ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_MAILBOX_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_MAILBOX_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_MAILBOX_REQ_T tReq;
  ECM_IF_ADD_SLAVE_MAILBOX_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_MAILBOX_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_CNF
 */

/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF
 */

/* request packet */
/*! Definitions for ulAction
 *
 */
typedef enum ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_WRITE_SINGLE = 0, /*!< Write single subindex */
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_COMPARE_SINGLE = 1, /*!< Compare value of single subindex */
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_WRITE_COMPLETE = 2, /*!< Write object with Complete Access */
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_COMPARE_COMPLETE = 3, /*!< Compare object value with Complete Access */
} ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_E;

/*! Add Slave CoE InitCmd Request data structure
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulTransitionFlags; /*!< Transition flags */
  uint32_t ulAction; /*!< CoE InitCmd action */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usIndex; /*!< Object index */
  uint8_t bSubIndex; /*!< Subindex */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulTotalSizeBytes; /*!< Total bytes of data. All fragments summed. */
  uint8_t abData[1024]; /*!< data to be used for action. Can be fragmented */
} ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_T;

/*! Add Slave CoE InitCmd Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ, ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ>>
 * \param ulLen 19+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_T tData; /*!< Packet data */
}  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_T;


/* confirmation packet */
/*! Add Slave CoE InitCmd Confirmation data structure */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulTransitionFlags; /*!< same as in request */
  uint32_t ulAction; /*!< same as in request */
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usIndex; /*!< same as in request */
  uint8_t bSubIndex; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalSizeBytes; /*!< same as in request */
} ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_T;

/*! Add Slave CoE InitCmd Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF, ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF>>
 * \param ulLen 19
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_COE_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_COE_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF
 */

/* request packet */
/*! Definitions for ulAction
 *
 */
typedef enum ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_WRITE = 0, /*!< Write */
  ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_COMPARE = 1,/*!< Compare Value */
} ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_E;

/*! Add Slave SoE InitCmd Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulTransitionFlags; /*!< Transtiion flags */
  uint32_t ulAction; /*!< SoE InitCmd action types */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usIdn; /*!< IDN number */
  uint8_t bDriveNo; /*!< Drive number */
  uint8_t bElementFlags; /*!< Element flags */
  uint32_t ulTimeoutMs; /*!< Timeout in ms */
  uint32_t ulTotalSizeBytes; /*!< Total bytes of data. All fragments summed. */
  uint8_t abData[1024]; /*!< data to be used for action. CAn be fragmented */
} ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_T;

/*! Add Slave SoE InitCmd Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ, ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ>>
 * \param ulLen 22+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_T;


/* confirmation packet */
/*! Add Slave SoE InitCmd Confirmation data structure */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulTransitionFlags; /*!< same as in request */
  uint32_t ulAction; /*!< same as in request */
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usIdn; /*!< same as in request */
  uint8_t bDriveNo; /*!< same as in request */
  uint8_t bElementFlags; /*!< same as in request */
  uint32_t ulTimeoutMs; /*!< same as in request */
  uint32_t ulTotalSizeBytes; /*!< same as in request */
} ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_T;

/*! Add Slave SoE InitCmd Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF, ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF>>
 * \param ulLen 22
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_SOE_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_SOE_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_CNF
 */

/* request packet */
/*! Definitions for ulAction
 *
 */
typedef enum ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_WRITE = 0, /*!< Write */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE = 1, /*!< Compare if equal */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_EQ = ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE, /*!< Compare if equal */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_NE = 2,                                          /*!< Compare if not-equal */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GE_UNSIGNED = 3,                                 /*!< Compare if greater than or equal (unsigned) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LE_UNSIGNED = 4,                                 /*!< Compare if less than or equal (unsigned) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GT_UNSIGNED = 5,                                 /*!< Compare if greater than (unsigned)  */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LT_UNSIGNED = 6,                                 /*!< Compare if less than (unsigned)  */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GE_SIGNED = 7,                                   /*!< Compare if greater than or equal (signed) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LE_SIGNED = 8,                                   /*!< Compare if less than or equal (signed) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GT_SIGNED = 9,                                   /*!< Compare if greater than (signed) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LT_SIGNED = 10,                                  /*!< Compare if less than (signed) */
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_NONE = 11,                                       /*!< Read without compare */
} ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_E;

/*! Add Slave Reg InitCmd Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulAction; /*!< Reg InitCmd action type */
  uint32_t ulTransitionFlags; /*!< Transition flags */
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usAdo; /*!< Register offset */
  uint8_t abData[1024]; /*!< Data to be used for action */
} ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_T;

/*! Add Slave Reg InitCmd Request Packet Description
 *
 * \param ulcmd <<ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ, ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ>>
 * \param ulLen 12+n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_T;


/* confirmation packet */
/*! Add Slave Reg InitCmd Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulAction; /*!< same as in request */
  uint32_t ulTransitionFlags; /*!< same as in request */
  uint16_t usStationAddress; /*!< same as in request */
  uint16_t usAdo; /*!< same as in request */
  uint16_t usLength; /*!< same as in request */
} ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_T;

/*! Add Slave Reg InitCmd Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ,ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ>>
 * \param ulLen 12
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_REG_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_REG_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ/ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF
 */

/* request packet */
/*! Add Slave DC Params Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bCyclicUnitControl; /*!< Cyclic unit control */
  uint8_t bDcActivate; /*!< Dc Activate */
  uint32_t ulDcCyc0Time; /*!< Dc Cyc0Time */
  uint32_t ulDcCyc1Time; /*!< Dc Cyc1TIme */
  uint64_t ullDcSyncShiftTime; /*!< Dc Sync Shift Time */
  uint8_t abDcLatchControl[2]; /*!< Dc Latch Control */
} ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_T;

/*! Add Slave DC Params Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ, ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ>>
 * \param ulLen 22
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_T;


/* confirmation packet */
/*! Add Slave DC Params Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< same as in request */
} ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_T;

/*! Add Slave DC Params Configuration Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF, ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_DC_PARAMS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_DC_PARAMS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ/ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF
 */

/* request packet */
/*! Add Slave ESM Timeouts Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulPreOpTimeoutMs; /*!< PREOP timeout in ms */
  uint32_t ulSafeOpTimeoutMs; /*!< SAFEOP timeout in ms */
  uint32_t ulBackToInitTimeoutMs; /*!< Back to INIT timeout in ms */
  uint32_t ulBackToSafeOpTimeoutMs; /*!< Back to SAFEOP timeout in ms */
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_T;

/*! Add Slave ESM Timeouts Request Packet Description
 *
 * \param ulCMd <<ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ,ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ>>
 * \param ulLen 18
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_T;


/* confirmation packet */
/*! Add Slave ESM Timeouts Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_Ttat
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_T;

/*! Add Slave ESM Timeouts Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF,ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ/ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF
 */

/* request packet */
/*! Definitions for usSetFlags
 *
 */
typedef enum ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_FLAGS_Etag
{
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_TO_ALL_SLAVES = 0x8000, /*!< Use for all slaves */
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_WDG_TIME_PROCDATA = 0x0001,/*!< Set Wdg Time Process Data */
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_WDG_TIME_PDI = 0x0002,     /*!< Set Wdg Time PDI */
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_FLAGS_E;

/*! Add Slave ESC Timeouts Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usSetFlags; /*!< Slave ESC timeout flags */
  uint16_t usWdgDivider; /*!< Watchdog divider */
  uint16_t usWdgTimePdi; /*!< PDI watchdog time */
  uint16_t usWdgTimeProcData; /*!< ProcData watchdog time */
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_T;

/*! Add Slave ESC Timeouts Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ, ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_T;


/* confirmation packet */
/*! Add Slave ESC Timeouts Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint16_t usSetFlags; /*!< Slave ESC timeout flags */
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_T;

/*! Add Slave ESC Timeouts Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF, ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF>>
 * \param ulLen 4
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ/ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF
 */

/* request packet */
/*! Add Cyclic Frame Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_Ttag
{
  uint32_t ulCycleTimeMultiplier; /*!< 1 => cycle time is equal to ulBaseCycleTimeNs */
  uint32_t ulCyclePrescalerStartIndex; /*!< value between 0 and ([Smallest common nominator of all ulCycleTimeMultiplier] - 1) */
  uint32_t ulRepetitionCount; /*!< 0 == one frame, 1 == two frames */
} ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_T;

/*! Add Cyclic Frame Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ, ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ>>
 * \param ulLen 12
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_CYCLIC_FRAME_REQ_T;


/* confirmation packet */
/*! Add Cyclic Frame Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_Ttag
{
  uint8_t bCyclicFrameIdx; /*!< first frame in configuration is returned as 0. Following frames are incremented */
} ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_T;

/*! Add Cyclic Frame Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF, ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_CYCLIC_FRAME_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_CYCLIC_FRAME_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_FRAME_REQ_T tReq;
  ECM_IF_ADD_CYCLIC_FRAME_CNF_T tCnf;
} ECM_IF_ADD_CYCLIC_FRAME_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ/ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF
 */

/* request packet */
/*! Add Cyclic Telegram Request data structure */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_Ttag
{
  uint8_t bCyclicFrameIdx; /*!< Cyclic frame index */
  uint8_t bMinState; /*!< Minimum required state except BOOT */
  uint8_t bCmd; /*!< Telegram command */
  ECM_IF_SLAVE_ADDR_T tAddr; /*!< Address to be used in command */
  uint16_t usLength; /*!< Data length of command */
  uint16_t usInitialWkc; /*!< Initial Wkc to be sent */
  uint16_t usExpectedWkc; /*!< Expected wkc (only some bReceiveDataFLow do this check) */
  uint8_t bTransmitDataFlow; /*!< Transmit data flow */
  uint16_t usTransmitSourceOffset; /*!< Offset in TxData to be copied into telegram */
  uint8_t bReceiveDataFlow; /*!< Receive data flow */
  uint16_t usReceiveDestinationOffset; /*!< Offset in RxData to copy to */
  uint16_t usWkcCompareReceiveDestinationOffset; /*!< Offset in RxData to copy the actual Wkc to */
} ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_T;

/* bTransmitDataFlow/bReceiveDataFlow */
/*! Data flow types for bTransmitDataFlow and bReceiveDataFlow */
typedef enum ECM_IF_CYCLIC_DATAFLOW_Etag
{
  /*!< Transmit telegram data filled with zeroes */
  ECM_IF_CYCLIC_DATA_FLOW_TX_ZEROFILL = 0,

  /*!< Ignore receive telegram data */
  ECM_IF_CYCLIC_DATA_FLOW_RX_UNUSED = ECM_IF_CYCLIC_DATA_FLOW_TX_ZEROFILL,

  /*!< Transmit buffer within telegram data */
  ECM_IF_CYCLIC_DATA_FLOW_TX_PROCESS_DATA_IMAGE = 1,

  /*!< Copy telegram data to receive buffer */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE = 2,

  /*!< Copy telegram data to receive buffer (clear if WKC does not match) */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE_CLEAR_IF_WKC_MISMATCH = 3,

  /*!< Copy telegram data to cross communication buffer */
  ECM_IF_CYCLIC_DATA_FLOW_CC_PROCESS_DATA_IMAGE = 4,

  /*!< Copy telegram data to mailbox state buffer */
  ECM_IF_CYCLIC_DATA_FLOW_MBX_STATE = 5,

  /*! DC SysTime control command for external synchronization
   * Allowed commands:
   * FPWR (ADP=FixedAddr, ADO=0x910, Length=4 or 8)
   * APWR (ADP=AutoIncAddr, ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_DC_SYSTIME_EXT_SYNC = 6,

  /*! DC SysTime receive timestamp
   * Allowed commands:
   * ARMW (ADO=0x910, Length=4 or 8)
   * FRMW (ADO=0x910, Length=4 or 8)
   * FPRD (ADP=FixedAddr, ADO=0x910, Length=4 or 8)
   * APRD (ADP=AutoIncAddr, ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_RX_DC_SYSTIME = 7,

  /*! DC SysTime update control
   * Allowed commands:
   * ARMW (ADO=0x910, Length=4 or 8)
   * FRMW (ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_DC_SYSTIME_CONTROL = 8,

  /*! Broadcast AlStatus read
   * Allowed commands:
   * BRD (ADO=0x130, Length=2)
   */
  ECM_IF_CYCLIC_DATA_FLOW_BRD_ALSTATUS = 9,

  /*! Broadcast DC SysTimeDiff read
   * Allowed commands:
   * BRD (ADO=0x92C, Length=2)
   */
  ECM_IF_CYCLIC_DATA_FLOW_BRD_DC_SYSTIME_DIFF  = 10,

  /*! Copy telegram data to receive buffer (freeze if WKC does not match) */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE_FREEZE_IF_WKC_MISMATCH = 11,

  /*! DC SysTime Latch Trigger Tx
   * Allowed commands:
   * BWR (ADO=0x900, Length=4)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_BWR_DC_RX_LATCH = 12,

  /*! DC SysTime Latch Trigger Rx
   * Allowed commands:
   * BWR (ADO=0x900, Length=4)
   */
  ECM_IF_CYCLIC_DATA_FLOW_RX_BWR_DC_RX_LATCH = ECM_IF_CYCLIC_DATA_FLOW_TX_BWR_DC_RX_LATCH,
} ECM_IF_CYCLIC_DATAFLOW_E;

/*! Offsets in DPM when not mapped
 * usTransmitSourceOffset, usReceiveDestinationOffset, usWkcCompareReceiveDestinationOffset
 */
typedef enum ECM_IF_CYCLIC_CMD_NOT_MAPPED_Etag
{
  /*! Not Mapped */
  ECM_IF_CYCLIC_CMD_NOT_MAPPED_IN_PROCIMAGE = 0xFFFF
} ECM_IF_CYCLIC_CMD_NOT_MAPPED_E;

/*! Add Cyclic Telegram Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ, ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ>>
 * \param ulLen 21
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_T;


/* confirmation packet */
/*! Add Cyclic Telegram Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_Ttag
{
  uint8_t bCyclicFrameIdx; /*!< same as in request */
} ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_T;


/*! Add Cyclic Telegram Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF, ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_CYCLIC_TELEGRAM_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_T tReq;
  ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_T tCnf;
} ECM_IF_ADD_CYCLIC_TELEGRAM_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ/ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF
 */

/* add slave SM configs, not to be used for mailbox Sync Managers */

/* request packet */
/*! Add Slave SMCfg Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bSmNo; /*!< Sync Manager number */
  uint8_t bMinState; /*!< Minimum state required */
  uint16_t usSmPhysAddr; /*!< SM physical address */
  uint16_t usSmPhysLength; /*!< SM physical length */
  uint8_t bControlByte; /*!< SM control byte */
  uint8_t bEnableByte; /*!< SM enable byte */
} ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_T;

/*! Add Slave SMCfg Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ, ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_SMCFG_REQ_T;


/* confirmation packet */
/*! Add Slave SMCfg Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bSmNo; /*!< Sync Manager number */
} ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_T;

/*! Add Slave SMCfg Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF, ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF>>
 * \param ulLen 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_SMCFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_SMCFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SMCFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_SMCFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_SMCFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ/ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF
 */

/* bFmmuType */
/*! FMMU types
 *
 */
typedef enum ECM_IF_SLAVE_FMMU_CFG_TYPE_Etag
{
  ECM_IF_SLAVE_FMMU_CFG_TYPE_E_FROM_SLAVE = 1,/*!< Data sent by slave */
  ECM_IF_SLAVE_FMMU_CFG_TYPE_E_TO_SLAVE = 2,  /*!< Data sent to slave */
} ECM_IF_SLAVE_FMMU_CFG_TYPE_E;

/* request packet */
/*! Add Slave FMMUCfg Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bFmmuNo; /*!< FMMU number */
  uint8_t bFmmuType; /*!< FMMU type */
  uint8_t bMinState; /*!< Minimum state this is required */
  uint32_t ulLogStartAddr; /*!< Logical start address */
  uint16_t usLogLength; /*!< Logical length */
  uint16_t usSmPhysAddr; /*!< SM physical address */
  uint8_t bLogStartBit; /*!< Logical start bit */
  uint8_t bLogEndBit; /*!< Logical end bit */
  uint8_t bPhysStartBit; /*!< Physical start bit */
} ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_T;

/*! Add Slave FMMUCfg Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ, ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ>>
 * \param ulLen 16
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_FMMUCFG_REQ_T;


/* confirmation packet */
/*! Add Slave FMMUCfg Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t bFmmuNo; /*!< FMMU number */
} ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_T;

/*! Add Slave FMMUCfg Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF, ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF>>
 * \param ulLen 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_FMMUCFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_FMMUCFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_FMMUCFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_FMMUCFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_FMMUCFG_PCK_T;

/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ/ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF
 */

#define ECM_IF_AOE_NETID_CFG_INITIALIZE_NETID 0x0002 /*!< AoE Flag to state that Initialize NetID is needed */

/* request packet */
/*! Add Slave AoE NetID Cfg Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint8_t abNetId[6]; /*!< AoE NetId */
  uint16_t usFlags; /*!< AoE Configuration flags */
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_T;

/*! Add Slave AoE NetID Cfg Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ, ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ>>
 * \param ulLen 10
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_T;


/* confirmation packet */
/*! Add Slave AoE NetID Cfg Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_T;

/*! Add Slave AoE NetID Cfg Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF, ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_AOE_NETID_CFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ/ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF
 */

#define ECM_IF_EOE_FLAGS_MAC_INCLUDED 0x01 /*!< MAC address is included */
#define ECM_IF_EOE_FLAGS_IP_ADDR_INCLUDED 0x02 /*!< IP address is included */
#define ECM_IF_EOE_FLAGS_SUBNET_MASK_INCLUDED 0x04 /*!< Subnet mask is included */
#define ECM_IF_EOE_FLAGS_DEFAULT_GW_INCLUDED 0x08 /*!< Default gateway is included */
#define ECM_IF_EOE_FLAGS_DNS_IP_INCLUDED 0x10 /*!< DNS IP address is included */
#define ECM_IF_EOE_FLAGS_DNS_NAME_INCLUDED 0x20 /*!< DNS name is included */

/* request packet */
/*! Add Slave EoE IP Param Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
  uint32_t ulEoEFlags; /*!< EoE flags */
  uint8_t abMacAddress[6]; /*!< MAC address */
  uint32_t ulIPAddress; /*!< UP address, in little endian order */
  uint32_t ulSubnetMask; /*!< Subnetmask, in little endian order */
  uint32_t ulDefGatewayAddress; /*!< Gateway address, in little endian order */
  uint32_t ulDnsIpAddr; /*!< DNS IP address, in little endian order */
  uint8_t abDnsName[32]; /*!< DNS name */
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_T;

/*! Add Slave EoE IP Param Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ, ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ>>
 * \param ulLen 60
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_T;


/* confirmation packet */
/*! Add Slave EoE IP Param Confirmation data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station address */
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_T;

/*! Add Slave EoE IP Param Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF, ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF>>
 * \param ulLen 2
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_EOE_IP_PARAM_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_T tReq;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ/ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_CNF
 *
 */

/* request packet */
#define ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_MAX_PDO 255 /*!< Maximum number of PDOs */

/*! Add Slave EoE Assign PDO Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station Address */
  uint8_t bSmNo; /*!< Sync Manager number */
  uint16_t ausPdoIdx[ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_MAX_PDO]; /*!< PDO entries up to 255 */
} ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_DATA_T;

/*! Add Slave EoE Assign PDO Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ, ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ>>
 * \param ulLen 2*n + 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_T;

#define ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_PCK_LEN(numpdos) (offsetof(ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_DATA_T, ausPdoIdx) + sizeof(uint16_t) * (numpdos))

/* confirmation packet */
/*! Add Slave EoE Assign PDO Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station Address */
  uint8_t bSmNo; /*!< Sync Manager number */
} ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_DATA_T;

/*! Add Slave EoE Assign PDO Configuration Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_CNF, ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_CNF>>
 * \param ulLen 3
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_ASSIGN_PDO_CFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_REQ/ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_CNF
 *
 */

/* request packet */
#define ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_MAX_PDO 255 /*!< Maximum number of PDO entries */

/*! Add Slave EoE PDO Mapping Configuration Request data entry
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_ENTRY_Ttag
{
  uint16_t usIndex; /*!< Mapped index */
  uint8_t bSubIndex; /*!< mapped subindex */
  uint8_t bBitLength; /*!< Bit length */
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_ENTRY_T;

/*! Add Slave EoE PDO Mapping Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station Address */
  uint16_t usPdoIndex; /*!< PDO number */
  uint8_t bSmNo; /*!< Sync Manager number */
  ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_ENTRY_T atEntries[ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_MAX_PDO]; /*!< PDO mapping entries up to 255 */
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_T;

/*! Add Slave EoE PDO Mapping Configuration Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ, ECM_IF_CMD_ADD_SLAVE_ASSIGN_PDO_CFG_REQ>>
 * \param ulLen n * 4 + 5
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_T;

#define ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_PCK_LEN(numentries) (offsetof(ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_T, atEntries) + sizeof(ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_DATA_ENTRY_T) * (numentries))

/* confirmation packet */
/*! Add Slave EoE PDO Mapping Configuration Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress; /*!< Station Address */
  uint16_t usPdoIndex; /*!< PDO number */
  uint8_t bSmNo; /*!< Sync Manager number */
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_DATA_T;

/*! Add Slave EoE PDO Mapping Configuration Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_CNF, ECM_IF_CMD_ADD_SLAVE_PDO_MAPPING_CFG_CNF>>
 * \param ulLen 5
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_PDO_MAPPING_CFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ/ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF
 *
 * Packet supports fragmentation
 *
 * Only a single transfer is allowed
 */

/*! Mandatory Slave List Request data structure entry
 *
 * index in lists represents auto-increment position
 *
 * table index 0 in slave table entry 0 is always 0 for connected to main port
 * position equals auto-inc position
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_Ttag
{
  uint16_t usThisStationAddress; /*!< Station address */
  uint16_t ausPortConnectedTo[4]; /*!< 0xFFFF == not connected, 0 == connected to redundancy port (only one entry valid), otherwise slave station address */
} ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T;


/* request packet */
/*! Mandatory Slave List Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_Ttag
{
  /* actual number of entries in packet is ptPck->tHead.ulLen / sizeof(ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T) */
  ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T)]; /*!< Entry table */
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_T;

/*! Mandatory Slave List Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ, ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ>>
 * \param ulLen 10*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_T;


/* confirmation packet */
/*! Mandatory Slave List Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF, ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_MANDATORY_SLAVE_LIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_T tReq;
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_T tCnf;
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ/ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF
 *
 * Packet supports fragmentation
 *
 * Only a single transfer is allowed
 */

/*! Add Hot COnnect Group Request data structure entry
 *
 * index in lists represents auto-increment position
 *
 * table index 0 in slave table entry 0 is always 0xFFFF for in port of hot connect slave
 * position equals auto-inc position
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_HOT_CONNECT_GROUP_ENTRY_Ttag
{
  uint16_t usThisStationAddress; /*!< Station address */
  uint16_t ausPortConnectedTo[4]; /*!< 0xFFFF == not connected */
} ECM_IF_HOT_CONNECT_GROUP_ENTRY_T;


/* request packet */
/*! Add Hot Connect Group Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_Ttag
{
  uint16_t usIdentificationReg; /*!< Identification register */
  uint16_t usIdentificationValue; /*!< Identification value */
  uint32_t ulIdentificationTimeoutMs; /*!< Timeout in ms, needed for Explicit Device Identification via ALStatus */
  /* actual number of entries in packet is ptPck->tHead.ulLen / sizeof(ECM_IF_HOT_CONNECT_GROUP_ENTRY_T) */
  ECM_IF_HOT_CONNECT_GROUP_ENTRY_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_HOT_CONNECT_GROUP_ENTRY_T)];
} ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_T;

/*! some well-known identification registers */
typedef enum ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_Etag
{
  ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_SECOND_STATION_ADDRESS = 0x0012, /*!< Second station address */
  ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_ALSTATUSCODE = 0x0134 /*!< this represents the identification via ALCONTROL/ALSTATUS */
} ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_E;

/*! Add Hot Connect Group Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ, ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ>>
 * \param ulLen 8+10*n
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_T;


/* confirmation packet */
/*! Add Hot Connect Group Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF, ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_HOT_CONNECT_GROUP_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_T tReq;
  ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_T tCnf;
} ECM_IF_ADD_HOT_CONNECT_GROUP_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ/ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF
 */

/* request packet */
/*! Set Default Target State Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_Ttag
{
  uint8_t bDefaultTargetState; /*!< Default target state for master */
} ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_T;

/*! Set Default Target State Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ, ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ>>
 * \param ulLen 1
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_T;


/* confirmation packet */
/*! Set Default Target State Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF, ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_DEFAULT_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_DEFAULT_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet:  ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ/ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF
 */

/* request packet */
/*! Set Base Sync Offset Percentage Request data structure
 *
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_Ttag
{
  uint32_t ulBaseSyncOffsetPercentage; /*!< value range 0-10000 resembling 0%-100% */
  uint32_t ulFlags; /*!< set to zero */
  uint32_t aulReserved[16]; /*!< set to zero */
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_T;

/*! Set Base Sync Offset Percentage Request Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ, ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ>>
 * \param ulLen 72
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_T tData; /*!< Packet data */
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_T;


/* confirmation packet */
/*! Set Base Sync Offset Percentage Confirmation Packet Description
 *
 * \param ulCmd <<ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF, ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF>>
 * \param ulLen 0
 * \param ulDest 0x20
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead; /*!< Packet header */
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_T;


/* packet union */
typedef union ECM_IF_BASE_SYNC_OFFSET_PERCENTAGE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_T tReq;
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_T tCnf;
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_PCK_T;


/******************************************************************************/
#define ECM_IF_CMD_AUTO_CONFIGURE_REQ 0x9ED0
#define ECM_IF_CMD_AUTO_CONFIGURE_CNF 0x9ED1

/******************************************************************************
 * Packet:  ECM_IF_CMD_AUTO_CONFIGURE_REQ/ECM_IF_CMD_AUTO_CONFIGURE_CNF
 */

/* ulACfgFlags */
typedef enum ECM_IF_ACFG_FLAGS_Etag
{
  MSK_ECM_IF_ACFG_FLAGS_ENABLE_IP_CONFIG = 0x00000001,
  MSK_ECM_IF_ACFG_FLAGS_ENABLE_FIRST_ASSIGNED = 0x00000004,
} ECM_IF_ACFG_FLAGS_E;

/* ulSystemFlags */
#define MSK_ECM_IF_ACFG_SYSTEM_FLAGS_APP_CONTROLLED 0x00000001

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_REQ_DATA_Ttag
{
  uint32_t ulSystemFlags;
  uint32_t ulWatchdogTime;
  uint32_t ulBusCycleNs;
  /* 12 bytes */
  uint16_t usTxProcDataImageBytes;
  uint16_t usRxProcDataImageBytes;
  uint16_t usCcProcDataImageBytes;
  uint16_t usSlaveReconnectRestartMode;
  /* 20 bytes */
  uint32_t ulStackConfigFlags;
  uint32_t ulACfgFlags;
  /* 28 bytes */
  uint16_t usFirstAssignedFirstStationAddress;
  uint8_t bTargetState;
  /* 31 bytes */
  uint8_t abMasterIpAddress[4];
  uint8_t abSubnetMask[4];
  uint8_t abDefaultGateway[4];
  /* 43 bytes */
  uint8_t abReserved[5];
  /* 48 bytes */
  uint32_t ulBaseSyncOffsetPercentage; /* value range 0-10000 */
  /* 52 bytes */
  uint16_t usVlanId;
  uint8_t bCyclicVlanPriority;
  uint8_t bAcyclicVlanPriority;
  uint32_t aulReserved[15]; /* set to zero */
  /* 116 bytes */
} ECM_IF_AUTO_CONFIGURE_REQ_DATA_T;

#define ECM_IF_AUTO_CONFIGURE_REQ_REV1_SIZE 43
#define ECM_IF_AUTO_CONFIGURE_REQ_REV2_SIZE 116

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_REQ_DATA_T tData;
} ECM_IF_AUTO_CONFIGURE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_Ttag
{
  uint16_t usTopologyPosition;
  uint16_t usIdentityValidFlags;
  uint32_t ulVendorId;
  uint32_t ulProductCode;
  uint32_t ulRevisionNumber;
  uint32_t ulSerialNumber;
  uint32_t ulProcessedAction;
  uint16_t usIndex;
  uint8_t bSubIndex;
  uint32_t ulDetailResult;
} ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_CNF_DATA_Ttag
{
  uint32_t ulNumSlavesInConfig;
  uint32_t ulActualTxDataBytes;
  uint32_t ulActualRxDataBytes;
  ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T)];
} ECM_IF_AUTO_CONFIGURE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_CNF_DATA_T tData;
} ECM_IF_AUTO_CONFIGURE_CNF_T;


/* packet union */
typedef union ECM_IF_AUTO_CONFIGURE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_REQ_T tReq;
  ECM_IF_AUTO_CONFIGURE_CNF_T tCnf;
} ECM_IF_AUTO_CONFIGURE_PCK_T;

/* pragma pack */
#ifdef __HIL_PRAGMA_PACK_ENABLE
#pragma __HIL_PRAGMA_UNPACK_1(ECM_IF_PUBLIC)
#endif

#endif
