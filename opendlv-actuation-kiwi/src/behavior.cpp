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

#include "behavior.hpp"
#include <cmath>
#include<iostream>


Behavior::Behavior() noexcept:

  m_closestBlueCone{},
  m_closestYellowCone{},
  m_carBack{},
  m_closestOrangeCone{},
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_closestBlueConeMutex{},
  m_closestYellowConeMutex{},
  m_carBackMutex{},
  m_closestOrangeConeMutex{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{}
{
}


opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setBlueCones(opendlv::logic::perception::Cones const &closestBlueCone) noexcept
{ 
  std::lock_guard<std::mutex> lock(m_closestBlueConeMutex);
  m_closestBlueCone = closestBlueCone;
}

void Behavior::setYellowCones(opendlv::logic::perception::Cones const &closestYellowCone) noexcept
{ 
  std::lock_guard<std::mutex> lock(m_closestYellowConeMutex);
  m_closestYellowCone = closestYellowCone;
}

void Behavior::setCarBack(opendlv::logic::perception::Cones const &carBack) noexcept
{
  std::lock_guard<std::mutex> lock(m_carBackMutex);
  m_carBack = carBack;
}

void Behavior::setOrangeCones(opendlv::logic::perception::Cones const &closestOrangeCone) noexcept
{
  std::lock_guard<std::mutex> lock(m_closestOrangeConeMutex);
  m_closestOrangeCone = closestOrangeCone;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}


void Behavior::step() noexcept
{
  opendlv::logic::perception::Cones closestBlueCone;
  opendlv::logic::perception::Cones closestYellowCone;
  opendlv::logic::perception::Cones carBack;
  opendlv::logic::perception::Cones closestOrangeCone;
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  {
    std::lock_guard<std::mutex> lock1(m_closestBlueConeMutex);
    std::lock_guard<std::mutex> lock2(m_closestYellowConeMutex);
    std::lock_guard<std::mutex> lock3(m_carBackMutex);
    std::lock_guard<std::mutex> lock4(m_closestOrangeConeMutex);
    std::lock_guard<std::mutex> lock5(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock6(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock7(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock8(m_rightIrReadingMutex);

    closestBlueCone = m_closestBlueCone;
    closestYellowCone = m_closestYellowCone;
    carBack = m_carBack;
    closestOrangeCone = m_closestOrangeCone;
    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
  }

  int32_t blueConeX = closestBlueCone.x();
  int32_t blueConeY = closestBlueCone.y();
  int32_t yellowConeX = closestYellowCone.x();
  int32_t yellowConeY = closestYellowCone.y();
  int32_t otherCarX = carBack.x();
  int32_t otherCarY = carBack.y();
  int32_t orangeConeX = closestOrangeCone.x();
  int32_t orangeConeY = closestOrangeCone.y();

  // double resultvec = 0.0f;
  double anglefromzero = 0.0f;
  double errorangle = 0.0f;
  
  //float frontDistance = frontUltrasonicReading.distance();
  //float rearDistance = rearUltrasonicReading.distance();
  //double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  //double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());

  float pedalPosition = 0.2f;
  float groundSteeringAngle = 0.0f;
  std::cout<< "Blue Cone X: " << blueConeX<< " Y: " << blueConeY << std::endl;
  std::cout<< "Yellow Cone X: " << yellowConeX << " Y: " << yellowConeY << std::endl;
  std::cout<< "Orange Cone X: " << orangeConeX << " Y: " << orangeConeY << std::endl;
  
 
  // intesection behavior
  if ((orangeConeX == 10000) || (orangeConeY == 10000)) {
    std::cout << "no orange cones visible. DRIVE ON" << std::endl;
  } else if ((orangeConeX > 0) && (orangeConeY < 300) && (otherCarX >= 100) && (otherCarX < 500)) {
    pedalPosition = 0.0f;
    std::cout << "STOPPING: at intersection, car incoming" << std::endl; 
    } else if ((orangeConeX > 0) && (orangeConeY < 300) && (otherCarX > 500)) {
    std::cout << "passing intersection, no incoming cars from right" << std::endl;
    }
  
  //car in front behavior
  if ((otherCarY < 300) && ((otherCarX <= 200) || (otherCarX >= -200))) {
    pedalPosition = 0.1f;
    std::cout << "Car in front, slowing down" <<std::endl; }
  // normal track behavior
  if ((blueConeX == 10000) || (blueConeY == 10000)) {
    // groundSteeringAngle = 0.2f;
    groundSteeringAngle = -0.2f;
    std::cout << "no blue cones visible. Turning right" << std::endl;
   } else if ((yellowConeX == 10000) || (yellowConeY == 10000)) {
      // groundSteeringAngle = -0.2f;
      groundSteeringAngle = 0.2f;
      std::cout << "no yellow cones visible. Turning left" << std::endl;
    } else if ((blueConeX == 0) && (yellowConeX == 0)) {
      groundSteeringAngle = 0.0f;
      pedalPosition = 0.01f;
      std::cout << "Initialising.." << std::endl;
    }
    else {
      std::cout << "track lane found. Moving forward" << std::endl;
      // resultvec = double(blueConeY + yellowConeY)/double(blueConeX + yellowConeX);
      anglefromzero = atan2(double(blueConeY + yellowConeY),double(blueConeX + yellowConeX))*180/M_PI;
      errorangle = anglefromzero - 90;
      std::cout<<"moving to ["<<blueConeX + yellowConeX<<", "<<blueConeY + yellowConeY<<"]"<<std::endl;
      // anglefromzero = atan(resultvec) * 180/M_PI;
      std::cout<< "angle from zero is " << anglefromzero <<" degrees"<<std::endl;
      // groundSteeringAngle = -
      if (errorangle > 5) {
        groundSteeringAngle = 0.1f;
        // groundSteeringAngle = -0.3f;
        std::cout<<"turn LEFT!"<<std::endl;
      } else if (errorangle < -5) {
        groundSteeringAngle = -0.1f;
        // groundSteeringAngle = 0.3f;
        std::cout<<"turn RIGHT!"<<std::endl;
      } else{
        std::cout<<"moving at low speed"<<std::endl;
        groundSteeringAngle = 0.0f;
        pedalPosition = 0.1f;
      }
    }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = (2.5 - sensorVoltage) / 0.07;
  return distance;
}
