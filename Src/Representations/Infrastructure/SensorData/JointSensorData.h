#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/RobotParts/Joints.h"
#include <array>

STREAMABLE_WITH_BASE(JointSensorData, JointAngles,
{
  JointSensorData();
  ,
  (std::array<short, Joints::numOfJoints>) currents, /**< The currents of all motors. */
  (std::array<unsigned char, Joints::numOfJoints>) temperatures, /**< The currents of all motors. */
});

inline JointSensorData::JointSensorData() :
  JointAngles()
{
  currents.fill(SensorData::off);
  temperatures.fill(0);
}
//STREAMABLE_WITH_BASE(JointSensorData, JointAngles,
//{
//  JointSensorData();
//
//  (ENUM_INDEXED_ARRAY(short, (Joints) Joint)) currents, /**< The currents of all motors. */
//  (ENUM_INDEXED_ARRAY(unsigned char, (Joints) Joint)) temperatures, /**< The currents of all motors. */
//});
//
//inline JointSensorData::JointSensorData() :
//  JointAngles()
//{
//  currents.fill(SensorData::off);
//  temperatures.fill(0);
//}
