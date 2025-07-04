/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
**
** This file is part of the elmo_ethercat_sdk.
** The elmo_ethercat_sdk is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The elmo_ethercat_sdk is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the elmo_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>

#include "elmo_ethercat_sdk/Configuration.hpp"
#include "elmo_ethercat_sdk/DriveState.hpp"
#include "elmo_ethercat_sdk/Error.hpp"
#include "elmo_ethercat_sdk/Statusword.hpp"

namespace elmo {
/*!
 * aliases for time_points, durations and clocks
 */
using ReadingClock = std::chrono::steady_clock;
using ReadingDuration = std::chrono::duration<double, std::milli>;
using ReadingTimePoint = std::chrono::time_point<ReadingClock>;

/*!
 * An alias for a pair of ErrorType and time point
 */
using ErrorPair = std::pair<ErrorType, ReadingTimePoint>;
using FaultPair = std::pair<uint16_t, ReadingTimePoint>;
using ErrorTimePairDeque = std::deque<std::pair<ErrorType, double>>;
using FaultTimePairDeque = std::deque<std::pair<uint16_t, double>>;

class Reading {
 public:
  /*!
   * raw get methods
   */
  int32_t getActualPositionRaw() const;
  int32_t getAuxiliaryPositionRaw() const;

  int32_t getActualVelocityRaw() const;
  uint16_t getRawStatusword() const;
  int16_t getActualCurrentRaw() const;
  uint16_t getAnalogInputRaw() const;
  uint32_t getBusVoltageRaw() const;

  /*!
   * User units get methods
   */
  double getActualPosition() const;
  double getAuxiliaryPosition() const;

  double getActualVelocity() const;
  double getActualCurrent() const;
  double getActualTorque() const;
  double getAnalogInput() const;
  double getAgeOfLastReadingInMicroseconds() const;
  double getBusVoltage() const;

  /*!
   * Other get methods
   */
  int32_t getDigitalInputs() const;
  Statusword getStatusword() const;
  std::string getDigitalInputString() const;
  DriveState getDriveState() const;

  /*!
   * set methods (only raw)
   */
  void setActualPosition(int32_t actualPosition);

  void setAuxiliaryPosition(int32_t auxiliaryPosition);

  void setDigitalInputs(int32_t digitalInputs);

  void setActualVelocity(int32_t actualVelocity);

  void setStatusword(uint16_t statusword);

  void setAnalogInput(int16_t analogInput);

  void setActualCurrent(int16_t actualCurrent);

  void setBusVoltage(uint32_t busVoltage);

  void setTimePointNow();

  void setPositionFactorIntegerToRad(double positionFactor);

  void setAuxiliaryPositionFactorIntegerToRad(double positionFactor);

  void setVelocityFactorIntegerPerSecToRadPerSec(double velocityFactor);

  void setCurrentFactorIntegerToAmp(double currentFactor);

  void setTorqueFactorIntegerToNm(double torqueFactor);

 protected:
  int32_t actualPosition_{0};
  int32_t auxiliaryPosition_{0}; //zcy

  int32_t digitalInputs_{0};
  int32_t actualVelocity_{0};
  uint16_t statusword_{0};
  int16_t analogInput_{0};
  int16_t actualCurrent_{0};
  uint32_t busVoltage_{0};

  double positionFactorIntegerToRad_{1};
  double auxiliaryPositionFactorIntegerToRad_{1};
  double velocityFactorIntegerPerSecToRadPerSec_{1};
  double currentFactorIntegerToAmp_{1};
  double torqueFactorIntegerToNm_{1};

  ReadingTimePoint lastReadingTimePoint_;

 public:
  /*!
   * returns the age of the last added error in microseconds
   * @return	the age
   */
  double getAgeOfLastErrorInMicroseconds() const;
  /*!
   * returns the age of the last added fault in microseconds
   * @return	the age
   */
  double getAgeOfLastFaultInMicroseconds() const;

  /*!
   * get all stored errors and their age in microseconds
   *
   * @return	deque of all stored errors
   */
  ErrorTimePairDeque getErrors() const;
  /*!
   * get all stored faults and ther age in microseconds
   * @return	deque of all stored faults
   */
  FaultTimePairDeque getFaults() const;

  /*!
   * Returns the last Error that occured
   * @return	The error type of tha last error
   */
  ErrorType getLastError() const;

  /*!
   * Returns the last fault that occured
   * @return	the code of the last occuring fault
   */
  uint16_t getLastFault() const;

  /*!
   * Adds an error type to the reading
   * A time point is set automatically
   * @param errorType	The type of the error
   */
  void addError(ErrorType errorType);
  /*!
   * Adds a fault code to the reading
   * A time point is set automatically
   * @param faultCode	The Code of the fault
   */
  void addFault(uint16_t faultCode);

  /*!
   * The default constructor
   * This is used for Readings generated by the user.
   * No configuration of the capacities and appending equal faults / errors is
   * necessary.
   */
  Reading() = default;

  bool hasUnreadError() const;

  bool hasUnreadFault() const;

  /*!
   * @brief	Load parameters from Configuration object
   * @param[in] configuration	The Configuration with the requested
   * configuration parameters
   */
  void configureReading(const Configuration& configuration);

  /*!
   * The configuration constructor
   * This is called for readings generated inside of the elmo_ethercat_sdk.
   * @param errorStorageCapacity	the number of errors that are stored
   * @param faultStorageCapacity	the number of faults that are stored
   * @param forceAppendEqualError	true if a new errer shall be appended even
   * though it is equal to the last one
   * @param forceAppendEqualFault	true if a new fault shall be appended even
   * though it is equal to the last one
   */
  Reading(unsigned int errorStorageCapacity, unsigned int faultStorageCapacity, bool forceAppendEqualError, bool forceAppendEqualFault);

 private:
  std::deque<ErrorPair> errors_;
  std::deque<FaultPair> faults_;

  ErrorPair lastError_;
  FaultPair lastFault_;

  mutable bool hasUnreadError_{false};
  mutable bool hasUnreadFault_{false};

  /*!
   * paramaters changeable with a Configuration object
   */
  unsigned int errorStorageCapacity_{25};
  unsigned int faultStorageCapacity_{25};
  bool forceAppendEqualError_{false};
  bool forceAppendEqualFault_{false};
};

}  // namespace elmo

// stream operator in global namespace
std::ostream& operator<<(std::ostream& os, const elmo::Reading& reading);
