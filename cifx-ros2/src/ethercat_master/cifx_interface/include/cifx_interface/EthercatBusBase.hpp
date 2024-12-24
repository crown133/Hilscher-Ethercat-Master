/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the cifx_interface.
**
** The cifx_interface is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The seom_interface is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the cifx_interface.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <cassert>
// std
#include <atomic>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <chrono>

// cifx
#include <cifx/cifxlinux.h>
#include <cifx/cifXUser.h>
#include <cifx/cifXEndianess.h>

#include <cifx/Hil_Packet.h>
#include <cifx/Hil_SystemCmd.h>

#include <cifx/EcmIF_Public.h>

#include "message_logger/message_logger.hpp"

// cifx_interface
#include "cifx_interface/common/Macros.hpp"
#include "cifx_interface/common/ThreadSleep.hpp"

namespace cifx_interface {

#define MAX_SDO_SIZE 32 // n bytes

// forward declaration for EthercatSlaveBase
class EthercatSlaveBase;
using EthercatSlaveBasePtr = std::shared_ptr<EthercatSlaveBase>;

/**
 * @brief      Class for managing an ethercat bus containing one or multpile
 *             slaves
 */
class EthercatBusBase {
 public:
  using PdoSizePair = std::pair<uint16_t, uint16_t>;
  using PdoSizeMap = std::unordered_map<std::string, PdoSizePair>;

  EthercatBusBase() = delete;
  /*!
   * Constructor.
   * @param name Name of the bus, e.g. "cifx0".
   */
  explicit EthercatBusBase(const std::string& name, bool verbose=false);

  /*!
   * Destructor.
   */
  ~EthercatBusBase() = default;

  /*!
   * Get the name of the bus.
   * @return Name of the bus.
   */
  const std::string& getName() const { return name_; }

  /*!
   * Get the time of the last successful IO/Data reading.
   * @return Stamp.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateReadStamp() const { return updateReadStamp_; }

  /*!
   * Get the time of the last successful IO/Data writing.
   * @return Stamp.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateWriteStamp() const { return updateWriteStamp_; }

  /*!
   * Check if a bus is available.
   * @param name Name of the bus.
   * @return True if available.
   */
  static bool busIsAvailable(const std::string& name);

  /*!
   * Print all available busses.
   */
  static void printAvailableBusses();

  /*!
   * Function to display driver information
   *   \param  hDriver  Handle to cifX driver
   *   \param  ptVTable Pointer to cifX API function table
   *   \return CIFX_NO_ERROR on success     
   */
  void DisplayDriverInformation();

  /*!
   * Check if this bus is available.
   * @return True if available.
   */
  bool busIsAvailable() const;

  /*!
   * Get the number of slaves which were detected on this bus.
   * @return Number of slaves.
   */
  int getNumberOfSlaves() const;

  /*!
   * Add an ANYdrive EtherCAT slave.
   * @slave ANYdrive EtherCAT slave.
   * @return True if successful.
   */
  bool addSlave(const EthercatSlaveBasePtr& slave);

  /*!
   * Startup the bus communication.
   * @param sizeCheck	perform a check of the Rx and Tx Pdo sizes defined in the PdoInfo oject of the slaves
   * @return True if successful.
   */
  bool startup(const bool sizeCheck = true);

  /*!
   * Update step 1: Read all PDOs.
   */
  void updateRead();

  /*!
   * Update step 2: Write all PDOs.
   */
  void updateWrite();

  /*!
   * Shutdown the bus communication.
   */
  void shutdown();

  /*!
   * Set the desired EtherCAT state machine state.
   * @param state Desired state.
   * @param slave Address of the slave, 0 for master.
   */
  void setState(const uint16_t state, const uint16_t slave = 0);

  /*!
   * Wait for an EtherCAT state machine state to be reached.
   * @param state      Desired state.
   * @param slave      Address of the slave, 0 for master.
   * @param maxRetries Maximum number of retries.
   * @param retrySleep Duration to sleep between the retries.
   * @return True if the state has been reached within the timeout.
   */
  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 40, const double retrySleep = 0.001);

  /*!
   * Check if an error for the SDO index of the slave exists.
   * @param slave   Address of the slave.
   * @param index   Index of the SDO.
   * @return True if an error for the index exists.
   */
  // bool checkForSdoErrors(const uint16_t slave, const uint16_t index);

  /*!
   * Synchronize the distributed clocks.
   *
   * @param      slave     Address of the slave.
   * @param      activate  True to activate the distr. clock, false to
   *                       deactivate.
   * @param[in]  timeStep  The time step
   */
  void syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift);

  /*!
   * Returns a map of the actually requested PDO sizes (Rx & Tx) This is useful
   * for slaves where the PDO size at startup is unknown This method shall be
   * used after adding the slaves and after executing the "startup" method
   *
   * @return     std::unordered_map with the addresses and the corresponding Pdo
   *             sizes
   */
  PdoSizeMap getHardwarePdoSizes();

  /*!
   * Returns a pair with the TxPdo and RxPdo sizes for the requested address
   * Overloads the "PdoSizeMap getHardwarePdoSizes()" method.
   *
   * @param      slave  Address of the slave
   *
   * @return     std::pair with the rx (first) and tx (second) Pdo sizes
   */
  // PdoSizePair getHardwarePdoSizes(const uint16_t slave);

  /*!
   * Send a writing SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t slave_station_address, const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    // assert(static_cast<int>(slave) <= getNumberOfSlaves());
    const int size = sizeof(Value);
    
    CIFX_PACKET tSendPkt       = {{0}};
    CIFX_PACKET tRecvPkt       = {{0}};

    ECM_IF_COE_SDO_DOWNLOAD_REQ_T sdoWriteReq;
    sdoWriteReq.tHead.ulDest = 0x20;
    sdoWriteReq.tHead.ulLen = 18 + size;
    sdoWriteReq.tHead.ulCmd = ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ;
    sdoWriteReq.tData.usStationAddress = slave_station_address; //slaves_[slave]->getStationAddress();
    sdoWriteReq.tData.usTransportType = 0;
    sdoWriteReq.tData.usObjIndex = index;
    if(completeAccess) {
      sdoWriteReq.tData.bSubIndex = 0x00;  // TODO: check DS402; 0 or 1?
    } else {
      sdoWriteReq.tData.bSubIndex = subindex;
    }
    sdoWriteReq.tData.fCompleteAccess = static_cast<bool>(completeAccess);
    sdoWriteReq.tData.ulTotalBytes = size;
    sdoWriteReq.tData.ulTimeoutMs = 10;
    memcpy(sdoWriteReq.tData.abData, &value, sizeof(Value));

    // tSendPkt.tHeader = sdoWriteReq.tHead;
    // memcpy(&tSendPkt.abData, &sdoWriteReq.tData, sizeof(sdoWriteReq.tData));
    memcpy(&tSendPkt, &sdoWriteReq, sizeof(sdoWriteReq));

    if(CIFX_NO_ERROR != xChannelPutPacket(hChannel, &tSendPkt, 10))
    {
      MELO_ERROR_STREAM("Error sending packet to device!");
      return false;
    } else
    {
      ECM_IF_COE_SDO_DOWNLOAD_CNF_T sdoWriteCnf;

      if(CIFX_NO_ERROR != xChannelGetPacket(hChannel, sizeof(sdoWriteCnf), &tRecvPkt, 20))
      {
        MELO_ERROR_STREAM("Error getting packet from device!");
        return false;
      } 
    }

    return true;
  }

  /*!
   * Send a reading SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t slave_station_address, const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    // assert(static_cast<int>(slave) <= getNumberOfSlaves());
    int size = sizeof(Value);
    
    CIFX_PACKET tSendPkt       = {{0}};
    CIFX_PACKET tRecvPkt       = {{0}};

    ECM_IF_COE_SDO_UPLOAD_REQ_T sdoReadReq;
    sdoReadReq.tHead.ulDest = 0x20;
    sdoReadReq.tHead.ulLen = 18;
    sdoReadReq.tHead.ulCmd = ECM_IF_CMD_COE_SDO_UPLOAD_REQ;
    sdoReadReq.tData.usStationAddress = slave_station_address; //slaves_[slave]->getStationAddress();
    sdoReadReq.tData.usTransportType = 0;
    sdoReadReq.tData.usObjIndex = index;
    if(completeAccess) {
      sdoReadReq.tData.bSubIndex = 0x00;  // TODO: check DS402; 0 or 1?
    } else {
      sdoReadReq.tData.bSubIndex = subindex;
    }
    sdoReadReq.tData.fCompleteAccess = static_cast<bool>(completeAccess);
    sdoReadReq.tData.ulTimeoutMs = 10;
    sdoReadReq.tData.ulMaxTotalBytes = MAX_SDO_SIZE;

    // tSendPkt.tHeader = sdoReadReq.tHead;
    // memcpy(&tSendPkt.abData, &sdoReadReq.tData, sizeof(sdoReadReq.tData));
    memcpy(&tSendPkt, &sdoReadReq, sizeof(sdoReadReq));

    if(CIFX_NO_ERROR != xChannelPutPacket(hChannel, &tSendPkt, 10))
    {
      MELO_ERROR_STREAM("Error sending packet to device!");
      return false;
    } else
    {
      ECM_IF_COE_SDO_UPLOAD_CNF_T sdoReadCnf;
      if(CIFX_NO_ERROR != xChannelGetPacket(hChannel, sizeof(sdoReadCnf), &tRecvPkt, 20))
      {
        MELO_ERROR_STREAM("Error getting packet from device!");
        return false;
      } 
      memcpy(&sdoReadCnf.tData, tRecvPkt.abData, sizeof(sdoReadCnf.tData));
      if(sdoReadCnf.tData.usStationAddress != slave_station_address) //slaves_[slave]->getStationAddress())
      {
        MELO_ERROR_STREAM("Wrong SDO Returned Station Address!");
        return false;
      }
      memcpy(&value, sdoReadCnf.tData.abData, sdoReadCnf.tData.ulTotalBytes);
    }

    return true;
  }

   /*!
   * Check if the bus is ok.
   * @return True if bus is ok.
   */
  bool busIsOk() const;

  /*!
   * Read a TxPDO from the buffer.
   * @param slave Address of the slave.
   * @param txPdo Return argument, TxPDO container.
   */
  template <typename TxPdo>
  void readTxPdo(const uint16_t slave, TxPdo& txPdo) const {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    // assert(sizeof(TxPdo) == ecatContext_.slavelist[slave].Ibytes);
    // memcpy(&txPdo, ecatContext_.slavelist[slave].inputs, sizeof(TxPdo));
  }

  /*!
   * Write an RxPDO to the buffer.
   * @param slave Address of the slave.
   * @param rxPdo RxPDO container.
   */
  template <typename RxPdo>
  void writeRxPdo(const uint16_t slave, const RxPdo& rxPdo) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    // assert(sizeof(RxPdo) == ecatContext_.slavelist[slave].Obytes);
    // memcpy(ecatContext_.slavelist[slave].outputs, &rxPdo, sizeof(RxPdo));
  }

 protected:

  //! Name of the bus.
  std::string name_;

  bool verbose_{false};
  
  CIFXHANDLE hDriver = NULL;
  CIFXHANDLE hChannel = NULL;

  //! Whether the bus has been initialized successfully
  bool initlialized_{false};

  //! cifx card driver Driver initialization structure 
  /*!< see CIFX_DRIVER_INIT_XXX defines */
  int init_options_ = CIFX_DRIVER_INIT_AUTOSCAN; 

  /*!< base directory for device configuration */
  const char* base_dir_ = nullptr; 

  /*!< Poll interval in ms for non-irq cards   */
  unsigned long poll_interval_ = 0; 

  /*!< Poll thread priority */ 
  int poll_priority_;  
  
  /*!< see TRACE_LVL_XXX defines in cifX Device Driver - Linux DRV 15 EN Page 33*/
  // Trace Level = 0x00 Tracing disabled
  // Trace Level = 0x01 Debug messages will be logged
  // Trace Level = 0x02 Information messages will be logged
  // Trace Level = 0x04 Warning messages will be logged
  // Trace Level = 0x08 Errors messages will be logged
  // Trace Level = 0xFF All messages will be logged
  unsigned long trace_level_ = 0;    
  
  /*!< Number of user defined cards */
  int user_card_cnt_; 

  /*!< Pointer to Array of user cards (must be user_card_cnt long) */
  struct CIFX_DEVICE_T* user_cards_ = nullptr;

  int iCardNumber_;

  int fEnableCardLocking_;
  /*!< Stack size of polling thread */
  int poll_StackSize_ = 0; //set to 0 to use default 

  /*!< Schedule policy of poll thread          */
  int poll_schedpolicy_ = SCHED_OTHER; // SCHED_FIFO SCHED_RR SCHED_OTHER 

// Note: does not use dynamic memory allocation (new/delete). Therefore
//   // all context pointers must be null or point to an existing member.
  struct CIFX_LINUX_INIT cifx_init_ = {init_options_,
                                       base_dir_, 
                                       poll_interval_,
                                       poll_priority_,
                                       trace_level_,
                                       user_card_cnt_,
                                       user_cards_, 
                                       iCardNumber_,
                                       fEnableCardLocking_, 
                                       poll_StackSize_,
                                       poll_schedpolicy_,
                                       NULL};

  //! List of slaves.
  std::vector<EthercatSlaveBasePtr> slaves_;

  //! Bool indicating whether PDO data has been sent and not read yet.
  bool sentProcessData_{false};

  //! Time of the last successful PDO reading.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateReadStamp_;
  //! Time of the last successful PDO writing.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateWriteStamp_;

  //! Maximal number of retries to configure the EtherCAT bus.
  const unsigned int ecatConfigMaxRetries_{5};
  //! Time to sleep between the retries.
  const double ecatConfigRetrySleep_{1.0};

  // EtherCAT input/output mapping of the slaves within the datagrams.
  char ioMap_[4096];
  unsigned char ioRecvData_[4096];
  unsigned char ioSendData_[4096];

  //! Board Information structure                                     
  char       abBoardName[CIFx_MAX_INFO_NAME_LENTH];        /*!< Global board name              */
  char       abBoardAlias[CIFx_MAX_INFO_NAME_LENTH];       /*!< Global board alias name        */
  uint32_t   ulBoardID;                                    /*!< Unique board ID, driver created*/
  uint32_t   ulPhysicalAddress;                            /*!< Physical memory address        */
  uint32_t   ulChannelCnt;                                 /*!< Number of available channels   */

  uint32_t   ulDeviceNumber;                               /*!< Global board device number     */
  uint32_t   ulSerialNumber;                               /*!< Global board serial number     */

//   // EtherCAT context data elements:

//   // Port reference.
//   ecx_portt ecatPort_;
//   // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
//   ec_slavet ecatSlavelist_[EC_MAXSLAVE];
//   // Number of slaves found in the network.
//   int ecatSlavecount_{0};
//   // Slave group structure.
//   ec_groupt ecatGrouplist_[EC_MAXGROUP];
//   // Internal, reference to EEPROM cache buffer.
//   uint8 ecatEsiBuf_[EC_MAXEEPBUF];
//   // Internal, reference to EEPROM cache map.
//   uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
//   // Internal, reference to error list.
//   ec_eringt ecatEList_;
//   // Internal, reference to processdata stack buffer info.
//   ec_idxstackT ecatIdxStack_;
//   // Boolean indicating if an error is available in error stack.
//   boolean ecatError_{FALSE};
//   // Reference to last DC time from slaves.
//   int64 ecatDcTime_{0};
//   // Internal, SM buffer.
//   ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
//   // Internal, PDO assign list.
//   ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
//   // Internal, PDO description list.
//   ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
//   // Internal, SM list from EEPROM.
//   ec_eepromSMt ecatSm_;
//   // Internal, FMMU list from EEPROM.
//   ec_eepromFMMUt ecatFmmu_;

  mutable std::recursive_mutex contextMutex_;
//   // EtherCAT context data.
//   // Note: SOEM does not use dynamic memory allocation (new/delete). Therefore
//   // all context pointers must be null or point to an existing member.
  // ecx_contextt ecatContext_ = {&ecatPort_,
//                                &ecatSlavelist_[0],
//                                &ecatSlavecount_,
//                                EC_MAXSLAVE,
//                                &ecatGrouplist_[0],
//                                EC_MAXGROUP,
//                                &ecatEsiBuf_[0],
//                                &ecatEsiMap_[0],
//                                0,
//                                &ecatEList_,
//                                &ecatIdxStack_,
//                                &ecatError_,
//                               //  0,
//                               //  0,
//                                &ecatDcTime_,
//                                &ecatSmCommtype_[0],
//                                &ecatPdoAssign_[0],
//                                &ecatPdoDesc_[0],
//                                &ecatSm_,
//                                &ecatFmmu_,
//                                nullptr};

}; //class EthercatBusBase

using EthercatBusBasePtr = std::shared_ptr<EthercatBusBase>;

// /*!
//  * Send a reading SDO - specialization for strings
//  * @param slave          Address of the slave.
//  * @param index          Index of the SDO.
//  * @param subindex       Sub-index of the SDO.
//  * @param completeAccess Access all sub-indices at once.
//  * @param value          Return argument, will contain the value which was read. The string needs to be preallocated to the correct size!
//  * @return True if successful.
//  */
// template<>
// bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value);

// /*!
//    * Send a writing SDO - specialization for strings
//    * @param slave          Address of the slave.
//    * @param index          Index of the SDO.
//    * @param subindex       Sub-index of the SDO.
//    * @param completeAccess Access all sub-indices at once.
//    * @param value          Value to write.
//    * @return True if successful.
//    */
// template<>
// bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const std::string value);

}  // namespace cifx_interface
