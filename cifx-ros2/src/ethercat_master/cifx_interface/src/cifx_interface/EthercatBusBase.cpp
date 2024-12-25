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

#include <cifx_interface/EthercatBusBase.hpp>
#include <cifx_interface/EthercatSlaveBase.hpp>

namespace cifx_interface {

EthercatBusBase::EthercatBusBase(const std::string& name, bool verbose) : name_(name), 
                                                                          verbose_(verbose) {
  // Initialize all data pointers that are not used with null.

  if(verbose_) {
    trace_level_ = 255;
  }
}

bool EthercatBusBase::busIsAvailable(const std::string& name) {
  CIFXHANDLE hDriver = NULL;
  int32_t    lRet    = xDriverOpen(&hDriver);

  printf("---------- Board Checking ----------\r\n");

  if(CIFX_NO_ERROR == lRet)
  {
    /* Driver/Toolkit successfully opened */
    unsigned long     ulBoard    = 0;
    BOARD_INFORMATION tBoardInfo = {0};

    /* Iterate over all boards */
    while(CIFX_NO_ERROR == xDriverEnumBoards(hDriver, ulBoard, sizeof(tBoardInfo), &tBoardInfo))
    {
      if(tBoardInfo.abBoardName == name) {
      printf("Found Board %s\r\n", tBoardInfo.abBoardName);
      if(strlen( (char*)tBoardInfo.abBoardAlias) != 0)
        printf(" Alias        : %s\r\n", tBoardInfo.abBoardAlias);

      printf(" DeviceNumber : %lu\r\n",(long unsigned int)tBoardInfo.tSystemInfo.ulDeviceNumber);
      printf(" SerialNumber : %lu\r\n",(long unsigned int)tBoardInfo.tSystemInfo.ulSerialNumber);
      printf(" Board ID     : %lu\r\n",(long unsigned int)tBoardInfo.ulBoardID);
      printf(" System Error : 0x%08lX\r\n",(long unsigned int)tBoardInfo.ulSystemError);
      printf(" Channels     : %lu\r\n",(long unsigned int)tBoardInfo.ulChannelCnt);
      printf(" DPM Size     : %lu\r\n",(long unsigned int)tBoardInfo.ulDpmTotalSize);
      return true;
      }
      ++ulBoard;
    }

    /* close previously opened driver */
    xDriverClose(hDriver);
  }

  return false;
}

void EthercatBusBase::printAvailableBusses() {
    CIFXHANDLE hDriver = NULL;
  int32_t    lRet    = xDriverOpen(&hDriver);

  printf("---------- Board/Channel enumeration demo ----------\r\n");

  if(CIFX_NO_ERROR == lRet)
  {
    /* Driver/Toolkit successfully opened */
    unsigned long     ulBoard    = 0;
    BOARD_INFORMATION tBoardInfo = {0};

    /* Iterate over all boards */
    while(CIFX_NO_ERROR == xDriverEnumBoards(hDriver, ulBoard, sizeof(tBoardInfo), &tBoardInfo))
    {
      printf("Found Board %s\r\n", tBoardInfo.abBoardName);
      if(strlen( (char*)tBoardInfo.abBoardAlias) != 0)
        printf(" Alias        : %s\r\n", tBoardInfo.abBoardAlias);

      printf(" DeviceNumber : %lu\r\n",(long unsigned int)tBoardInfo.tSystemInfo.ulDeviceNumber);
      printf(" SerialNumber : %lu\r\n",(long unsigned int)tBoardInfo.tSystemInfo.ulSerialNumber);
      printf(" Board ID     : %lu\r\n",(long unsigned int)tBoardInfo.ulBoardID);
      printf(" System Error : 0x%08lX\r\n",(long unsigned int)tBoardInfo.ulSystemError);
      printf(" Channels     : %lu\r\n",(long unsigned int)tBoardInfo.ulChannelCnt);
      printf(" DPM Size     : %lu\r\n",(long unsigned int)tBoardInfo.ulDpmTotalSize);

      unsigned long       ulChannel    = 0;
      CHANNEL_INFORMATION tChannelInfo = {{0}};

      /* iterate over all channels on the current board */
      while(CIFX_NO_ERROR == xDriverEnumChannels(hDriver, ulBoard, ulChannel, sizeof(tChannelInfo), &tChannelInfo))
      {
        printf(" - Channel %lu:\r\n", ulChannel);
        printf("    Firmware : %s\r\n", tChannelInfo.abFWName);
        printf("    Version  : %u.%u.%u build %u\r\n", 
               tChannelInfo.usFWMajor,
               tChannelInfo.usFWMinor,
               tChannelInfo.usFWRevision,
               tChannelInfo.usFWBuild);
        printf("    Date     : %02u/%02u/%04u\r\n", 
               tChannelInfo.bFWMonth,
               tChannelInfo.bFWDay,
               tChannelInfo.usFWYear);

        printf("  Device Nr. : %lu\r\n",(long unsigned int)tChannelInfo.ulDeviceNumber);
        printf("  Serial Nr. : %lu\r\n",(long unsigned int)tChannelInfo.ulSerialNumber);
        printf("  netX Flags : 0x%08X\r\n", tChannelInfo.ulNetxFlags);
        printf("  Host Flags : 0x%08X\r\n", tChannelInfo.ulHostFlags);
        printf("  Host COS   : 0x%08X\r\n", tChannelInfo.ulHostCOSFlags);
        printf("  Device COS : 0x%08X\r\n", tChannelInfo.ulDeviceCOSFlags);

        ++ulChannel;
      }

      ++ulBoard;
    }

    /* close previously opened driver */
    xDriverClose(hDriver);
  }

  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");
}
void ShowError( int32_t lError )
{
  if( lError != CIFX_NO_ERROR)
  {
    char szError[1024] ={0};
    xDriverGetErrorDescription( lError,  szError, sizeof(szError));
    printf("Error: 0x%X, <%s>\n", (unsigned int)lError, szError);
  }
}

void EthercatBusBase::DisplayDriverInformation (void)
{
  int32_t            lRet             = CIFX_NO_ERROR;
  DRIVER_INFORMATION tDriverInfo      = {{0}};
  char               szDrvVersion[32] = "";
  CIFXHANDLE         hDriver          = NULL;
 
  if (CIFX_NO_ERROR == (lRet = xDriverOpen(&hDriver)))
  {
    printf("\n---------- Display Driver Version ----------\n");
    if( CIFX_NO_ERROR != (lRet = xDriverGetInformation(hDriver, sizeof(tDriverInfo), &tDriverInfo)) )
      ShowError( lRet);
    else if ( CIFX_NO_ERROR != (lRet = cifXGetDriverVersion( sizeof(szDrvVersion)/sizeof(*szDrvVersion), szDrvVersion)))
      ShowError( lRet);
    else
      printf("Driver Version: %s, based on %.32s \n\n", szDrvVersion, tDriverInfo.abDriverVersion);
    
    /* close previously opened driver */
    xDriverClose(hDriver);
    
  } 
  
  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");
}

bool EthercatBusBase::busIsAvailable() const { return busIsAvailable(name_); }

int EthercatBusBase::getNumberOfSlaves() const {
  // std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return slaves_.size();
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave) {
  for (const auto& existingSlave : slaves_) {
    if (slave->getAddress() == existingSlave->getAddress()) {
      MELO_ERROR_STREAM("[" << getName() << "] "
                            << "Slave '" << existingSlave->getName() << "' and slave '" << slave->getName()
                            << "' have identical addresses (" << slave->getAddress() << ").");
      return false;
    }
  }

  slaves_.push_back(slave);
  return true;
}

bool EthercatBusBase::startup(const bool sizeCheck) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  /*
   * Followed by start of the application we need to set up the NIC to be used as
   * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
   * SOEM comes with support for cable redundancy we call ec_init_redundant that
   * will open a second port as backup. You can send NULL as ifname if you have a
   * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
   */
  if(CIFX_NO_ERROR != cifXDriverInit(&cifx_init_))
  {
    MELO_ERROR_STREAM("[" << getName() << "] "
                            << "Driver init failed.");
    return false;
  }
  if(verbose_) {
    DisplayDriverInformation();
  }
  if (!busIsAvailable()) {
    MELO_ERROR_STREAM("[" << getName() << "] "
                          << "Bus is not available.");
    printAvailableBusses();
    return false;
  }

  if(CIFX_NO_ERROR != xDriverOpen(&hDriver)) {
    MELO_ERROR_STREAM("[" << getName() << "] "
                            << "Driver open failed.");
    return false;
  }

  /* Driver/Toolkit successfully opened */
  if(CIFX_NO_ERROR != xChannelOpen(hDriver, name_.c_str(), 0, &hChannel)) {
    MELO_ERROR_STREAM("[" << getName() << "] "
                            << "Channel open failed.");
    return false;
  }

    // Todo：get slave numbers
    // Print the slaves which have been detected.
    MELO_INFO_STREAM("The following " << getNumberOfSlaves() << " slaves have been found and configured:");
    // for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    //   MELO_INFO_STREAM("Address: " << slave << " - Name: '" << std::string(ecatContext_.slavelist[slave].name) << "'");
    // }

    // Todo： Check slave addresses
    // // Check if the given slave addresses are valid.
    // bool slaveAddressesAreOk = true;
    // for (const auto& slave : slaves_) {
    //   auto address = static_cast<int>(slave->getAddress());
    //   if (address == 0) {
    //     MELO_ERROR_STREAM("[" << getName() << "] "
    //                           << "Slave '" << slave->getName() << "': Invalid address " << address << ".");
    //     slaveAddressesAreOk = false;
    //   }
    //   if (address > getNumberOfSlaves()) {
    //     MELO_ERROR_STREAM("[" << getName() << "] "
    //                           << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
    //                           << "only " << getNumberOfSlaves() << " slave(s) found.");
    //     slaveAddressesAreOk = false;
    //   }
    // }
    // if (!slaveAddressesAreOk) {
    //   return false;
    // }

    uint32_t rxPdoId = 0;
    uint32_t txPdoId = 0;
    for (const auto& slave : slaves_) {
      slave->setPdoId(rxPdoId, txPdoId);

      rxPdoId += slave->getCurrentPdoInfo().rxPdoSize_;
      txPdoId += slave->getCurrentPdoInfo().txPdoSize_;
    }
 

    //Todo：Initialize slaves in host application instead of netx firmware
    // Initialize the communication interfaces of all slaves.
    for (auto& slave : slaves_) {
      if (!slave->startup()) {
        MELO_ERROR_STREAM("[" << getName() << "] "
                              << "Slave '" << slave->getName() << "' was not initialized successfully.");
        return false;
      }
    }

    // Check if the size of the IO mapping fits our slaves.
    bool ioMapIsOk = true;
    // do this check only if 'sizeCheck' is true
    // if (sizeCheck) {
      // for (const auto& slave : slaves_) {
        // const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
        //TODO: auto get the size of PDO from netx
        // if (pdoInfo.rxPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Obytes) {
        //   MELO_ERROR_STREAM("[" << getName() << "] "
        //                         << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of " << pdoInfo.rxPdoSize_
        //                         << " bytes but the slave found at its address " << slave->getAddress() << " requests "
        //                         << ecatContext_.slavelist[slave->getAddress()].Obytes << " bytes).");
        //   ioMapIsOk = false;
        // }
        // if (pdoInfo.txPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Ibytes) {
        //   MELO_ERROR_STREAM("[" << getName() << "] "
        //                         << "TxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of " << pdoInfo.txPdoSize_
        //                         << " bytes but the slave found at its address " << slave->getAddress() << " requests "
        //                         << ecatContext_.slavelist[slave->getAddress()].Ibytes << " bytes).");
        //   ioMapIsOk = false;
        // }

      // }
    // }

    auto pdoRXSize = 0;
    auto pdoTXSize = 0;
    for (const auto& slave : slaves_) {
      const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
      pdoRXSize += pdoInfo.rxPdoSize_;
      pdoTXSize += pdoInfo.txPdoSize_;
    }
    //TODO: change the size of the IO mapping accroding to the actual size
    // ioRecvData_.reset(new unsigned char[pdoRXSize]);
    // ioSendData_.reset(new unsigned char[pdoTXSize]);

    if (!ioMapIsOk) {
      return false;
    }

    // Initialize the memory with zeroes.
    // for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    //   memset(ecatContext_.slavelist[slave].inputs, 0, ecatContext_.slavelist[slave].Ibytes);
    //   memset(ecatContext_.slavelist[slave].outputs, 0, ecatContext_.slavelist[slave].Obytes);
    // }

    initlialized_ = true;

    unsigned long ulState;
    if(CIFX_NO_ERROR != xChannelBusState(hChannel, CIFX_BUS_STATE_ON,(uint32_t*) &ulState, 10000))
    {
      MELO_ERROR_STREAM("[" << getName() << "] "
                            << "Error setting Bus state");
      xChannelClose(hChannel);
      xDriverClose(hDriver);
      return false;
    }

    return true;
}

void EthercatBusBase::updateRead() {
  if (!sentProcessData_) {
    MELO_DEBUG_STREAM("No process data to read.");
    return;
  }

  //! Receive the EtherCAT data.
  updateReadStamp_ = std::chrono::high_resolution_clock::now();
  if(CIFX_NO_ERROR != xChannelIORead(hChannel, 0, 0, sizeof(ioRecvData_), ioRecvData_, 10))
  {
    MELO_ERROR_STREAM("Error reading IO Data area!\r\n");
  }
  sentProcessData_ = false;

  //! Each slave attached to this bus reads its data to the buffer.
  for (auto& slave : slaves_) {
    slave->updateRead();
  }
}

void EthercatBusBase::updateWrite() {
  if (sentProcessData_) {
    MELO_DEBUG_STREAM("Sending new process data without reading the previous one.");
  }

  //! Each slave attached to this bus write its data to the buffer.
  for (auto& slave : slaves_) {
    slave->updateWrite();
  }

  //! Send the EtherCAT data.
  updateWriteStamp_ = std::chrono::high_resolution_clock::now();
  // std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  if(CIFX_NO_ERROR != xChannelIOWrite(hChannel, 0, 0, sizeof(ioSendData_), ioSendData_, 10))
  {
    MELO_ERROR_STREAM("Error writing to IO Data area!\r\n");
  } else {
    sentProcessData_ = true;
  }
}

void EthercatBusBase::shutdown() {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  // Set the slaves to state Init.
  if (getNumberOfSlaves() > 0) {
    setState(ECM_IF_STATE_INIT); //set master to init
    waitForState(ECM_IF_STATE_INIT);
  }

  for (auto& slave : slaves_) {
    slave->shutdown();
  }

  xChannelClose(hChannel);
  xDriverClose(hDriver);

  unsigned long ulState;
  xChannelBusState(hChannel, CIFX_BUS_STATE_OFF, (uint32_t*) &ulState, 10000);

  initlialized_ = false;
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave) {
  // std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  if(!initlialized_) {
    MELO_ERROR_STREAM("Bus " << name_ << " was not successfully initialized, skipping operation");
    return;
  }
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  
  int32_t lRet;
  CIFX_PACKET tSendPkt       = {{0}};
  CIFX_PACKET tRecvPkt       = {{0}};

  if(slave == 0) {
    ECM_IF_SET_MASTER_TARGET_STATE_REQ_T master_stateSetReq;
    master_stateSetReq.tHead.ulCmd = ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ;
    master_stateSetReq.tHead.ulLen = 1;
    master_stateSetReq.tHead.ulDest = 0x20;
    master_stateSetReq.tData.bTargetState = state;

    // tSendPkt.tHeader = master_stateSetReq.tHead;
    // memcpy(&tSendPkt.abData, &master_stateSetReq.tData, sizeof(master_stateSetReq.tData));
    memcpy(&tSendPkt, &master_stateSetReq, sizeof(master_stateSetReq));
   
    if(CIFX_NO_ERROR != (lRet = xChannelPutPacket(hChannel, &tSendPkt, 10)))
    {
      MELO_ERROR_STREAM("Master state set failed, Error sending packet to device " << lRet);
    } else {
      ECM_IF_SET_MASTER_TARGET_STATE_CNF_T master_stateSetCnf;
      if(CIFX_NO_ERROR != (lRet = xChannelGetPacket(hChannel, sizeof(master_stateSetCnf), &tRecvPkt, 20)) )
      {
        MELO_ERROR_STREAM("Error getting state from master " << lRet);
      }
      MELO_DEBUG_STREAM("Master " << name_ << ": State " << state << " has been set.");
    }
  }
  else if(slave > 0) {
    ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T slave_stateSetReq;
    slave_stateSetReq.tHead.ulCmd = ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ;
    slave_stateSetReq.tHead.ulLen = 3;
    slave_stateSetReq.tHead.ulDest = 0x20;
    slave_stateSetReq.tData.usStationAddress = slaves_[slave-1]->getStationAddress();
    slave_stateSetReq.tData.bTargetState = state;

    // tSendPkt.tHeader = slave_stateSetReq.tHead;
    // memcpy(&tSendPkt.abData, &slave_stateSetReq.tData, sizeof(slave_stateSetReq.tData));
    memcpy(&tSendPkt, &slave_stateSetReq, sizeof(slave_stateSetReq));

    if(CIFX_NO_ERROR != (lRet = xChannelPutPacket(hChannel, &tSendPkt, 10)))
    {
      MELO_ERROR_STREAM("Slave state set failed, Error sending packet to device " << lRet);
    } else {
      ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T slave_stateSetCnf;
      if(CIFX_NO_ERROR != (lRet = xChannelGetPacket(hChannel, sizeof(slave_stateSetCnf), &tRecvPkt, 20)) )
      {
        MELO_ERROR_STREAM("Error getting state from slave " << lRet);
      }
      MELO_DEBUG_STREAM("Slave " << slave << ": State " << state << " has been set.");
    }
  }

}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries, const double retrySleep) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  // std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  for (unsigned int retry = 0; retry <= maxRetries; retry++) {
    int32_t lRet;
    CIFX_PACKET tSendPkt       = {{0}};
    CIFX_PACKET tRecvPkt       = {{0}};

    if(slave == 0) {
      ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T master_stateGetReq;
      master_stateGetReq.tHead.ulCmd = ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ;
      master_stateGetReq.tHead.ulLen = 0;
      master_stateGetReq.tHead.ulDest = 0x20;
      // tSendPkt.tHeader = master_stateGetReq.tHead;
      // memcpy(&tSendPkt.abData, &master_stateGetReq.tData, sizeof(master_stateGetReq.tData));
      memcpy(&tSendPkt, &master_stateGetReq, sizeof(master_stateGetReq));

      if(CIFX_NO_ERROR != (lRet = xChannelPutPacket(hChannel, &tSendPkt, 10)))
      {
        MELO_ERROR_STREAM("Master state get failed, Error sending packet to device " << lRet);
      } else {
        ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T master_stateGetCnf;
        if(CIFX_NO_ERROR != (lRet = xChannelGetPacket(hChannel, sizeof(master_stateGetCnf), &tRecvPkt, 20)) )
        {
          MELO_ERROR_STREAM("Error getting state from master " << lRet);
        }
        memcpy(&master_stateGetCnf.tData, &tRecvPkt.abData, sizeof(master_stateGetCnf.tData));
        if(master_stateGetCnf.tData.bCurrentState == state) {
          MELO_DEBUG_STREAM("Master " << name_ << ": State " << state << " has been reached.");
          return true;
        }
      }
      MELO_DEBUG_STREAM("Master " << name_ << ": State " << state << " has been set.");
    }
    else if(slave > 0) {
      ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T slave_stateGetReq;
      slave_stateGetReq.tHead.ulCmd = ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ;
      slave_stateGetReq.tHead.ulLen = 2;
      slave_stateGetReq.tHead.ulDest = 0x20;
      slave_stateGetReq.tData.usStationAddress = slaves_[slave-1]->getStationAddress();

      // tSendPkt.tHeader = slave_stateGetReq.tHead;
      // memcpy(&tSendPkt.abData, &slave_stateGetReq.tData, sizeof(slave_stateGetReq.tData));
      memcpy(&tSendPkt, &slave_stateGetReq, sizeof(slave_stateGetReq));

      if(CIFX_NO_ERROR != (lRet = xChannelPutPacket(hChannel, &tSendPkt, 10)))
      {
        MELO_ERROR_STREAM("Slave state set failed, Error sending packet to device " << lRet);
      } else {
        ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T slave_stateGetCnf;
        if(CIFX_NO_ERROR != (lRet = xChannelGetPacket(hChannel, sizeof(slave_stateGetCnf), &tRecvPkt, 20)) )
        {
          MELO_ERROR_STREAM("Error getting state from slave " << lRet);
        }
        memcpy(&slave_stateGetCnf.tData, &tRecvPkt.abData, sizeof(slave_stateGetCnf.tData));
        if(slave_stateGetCnf.tData.bCurrentState == state) {
          MELO_DEBUG_STREAM("Slave " << slave << ": State " << state << " has been reached.");
          return true;
        }
      }
      MELO_DEBUG_STREAM("Slave " << slave << ": State " << state << " has been set.");
    }
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(retrySleep); //retrySleep = s
    ts.tv_nsec = static_cast<long>((std::fmod(retrySleep, 1.0)) * 1e9);
    nanosleep(&ts, NULL);
  }

  MELO_WARN_STREAM("Slave " << slave << ": State " << state << " has not been reached.");
  return false;
}

bool EthercatBusBase::busIsOk() const { return true; }  //TODO: check the state of bus

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
  // MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activating" : "Deactivating")
  //                          << " distributed clock synchronization...");

  // ecx_dcsync0(&ecatContext_, slave, static_cast<uint8_t>(activate), static_cast<uint32_t>(cycleTime * 1e9),
  //             static_cast<int32_t>(1e9 * cycleShift));

  // MELO_INFO_STREAM("Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activated" : "Deactivated")
  //                          << " distributed clock synchronization.");
  //TODO: set the distributed clock
}

//TODO: auto get the size of PDO from netx
EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes() {
  PdoSizeMap pdoMap;

  // for (const auto& slave : slaves_) {
  //   pdoMap.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
  // }

  return pdoMap;
}

// EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) {
//   // return std::make_pair(ecatContext_.slavelist[slave].Obytes, ecatContext_.slavelist[slave].Ibytes);
  
// }


// template<>
// bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value) {
//     assert(static_cast<int>(slave) <= getNumberOfSlaves());
//     //Expected length of the string. String needs to be preallocated
//     int size = value.length();
//     //Store for check at the end
//     int expected_size = size;
//     //Create buffer with the length of the string
//     char buffer[size];
//     int wkc = 0;
//     {
//       std::lock_guard<std::recursive_mutex> guard(contextMutex_);
//       wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<bool>(completeAccess), &size, buffer, EC_TIMEOUTRXM);
//       //Convert read data to a std::string
//       value = std::string(buffer,size);
//     }
//     if (wkc <= 0) {
//       MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
//                                  << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
//                                  << static_cast<uint16_t>(subindex) << ").");
//       return false;
//     }

//     if (size != (int)expected_size) {
//       MELO_ERROR_STREAM("Slave " << slave << ": Size mismatch (expected " << expected_size << " bytes, read " << size
//                                  << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index
//                                  << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").");
//       return false;
//     }
//     return true;
// }

// template<>
// bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const std::string value) {
//     assert(static_cast<int>(slave) <= getNumberOfSlaves());
//     const int size = value.length();
//     const char* dataPtr = value.data();
//     int wkc = 0;
//     {
//         std::lock_guard<std::recursive_mutex> guard(contextMutex_);
//         wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &dataPtr, EC_TIMEOUTRXM);
//     }
//     if (wkc <= 0) {
//         MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x" << std::setfill('0')
//                                    << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
//                                    << static_cast<uint16_t>(subindex) << ").");
//         return false;
//     }
//     return true;
// }
}  // namespace cifx_interface
