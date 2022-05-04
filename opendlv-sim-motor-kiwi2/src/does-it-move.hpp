/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DOES_IT_MOVE
#define DOES_IT_MOVE

#include <mutex>

#include "opendlv-standard-message-set.hpp"

class Does_It_Move {
 private:
  Does_It_Move(Does_It_Move const &) = delete;
  Does_It_Move(Does_It_Move &&) = delete;
  Does_It_Move &operator=(Does_It_Move const &) = delete;
  Does_It_Move &operator=(Does_It_Move &&) = delete;

 public:
  Does_It_Move() noexcept;
  ~Does_It_Move() = default;

 public:
  void setObjectDirection(opendlv::proxy::ObjectDirectionRequest const &) noexcept;
  void setGroundSteeringAngle(opendlv::proxy::GroundSteeringRequest const &) noexcept;
  void setPedalPosition(opendlv::proxy::PedalPositionRequest const &) noexcept;
  opendlv::sim::KinematicState step(double) noexcept;

 private:
  std::mutex m_ObjectDirectionMutex;
  std::mutex m_groundSteeringAngleMutex;
  std::mutex m_pedalPositionMutex;
  double m_longitudinalSpeed;
  double m_lateralSpeed;
  double m_yawRate;
  float m_groundSteeringAngle;
  float m_pedalPosition;
};

#endif
