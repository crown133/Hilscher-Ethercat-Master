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

//  cifx_interface
#include <cifx_interface/EthercatSlaveBase.hpp>

namespace cifx_interface {

EthercatSlaveBase::EthercatSlaveBase(EthercatBusBase* bus, const uint32_t address) : bus_(bus), address_(address) {}
EthercatSlaveBase::EthercatSlaveBase() : bus_(nullptr), address_(0) {}


bool EthercatSlaveBase::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                           const std::string& valueTypeString, std::string& valueString) {
  printWarnNotImplemented();
  return false;
}

bool EthercatSlaveBase::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, const std::string& valueString) {
  printWarnNotImplemented();
  return false;
}

// bool EthercatSlaveBase::sendSdoReadVisibleString(const uint16_t index, const uint8_t subindex, std::string& value) {
//   std::lock_guard<std::recursive_mutex> lock(mutex_);
//   return bus_->sendSdoReadVisibleString(address_, index, subindex, value);
// }

}  // namespace cifx_interface
