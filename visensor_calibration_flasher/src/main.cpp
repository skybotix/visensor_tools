/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 *
 * All rights reserved.
 *
 * Redistribution and non-commercial use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of the {organization} nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include "visensor_calibration_flasher.hpp"

void printArgs(void)
{
  std::cout << "visensor_calibration_flasher <SENSOR_IP> <COMMAND> [CMD_ARGS]" << std::endl;
  std::cout << std::endl;
  std::cout << "  Available commands are:" << std::endl;
  std::cout << "     show                   prints all the saved calibration." << std::endl;
  std::cout << "     add <path> <slot_id>   add a new calibration to the sensor." << std::endl;
  std::cout << "     delete <cam_id> <slot_id> <is_flipped> <projection_type> <lens_model_type>\n";
  std::cout << "                            delete the defined calibration" << std::endl;
  std::cout << std::endl;
}

bool cmdDelete(ros::NodeHandle& nh, std::string& target_name, std::vector<std::string>& args)
{
  // check if delete <cam_id> <slot_id> <is_flipped> <projection_type> <lens_model_type>
  if (args.size() != 5) {
    printArgs();
    return false;
  }

  visensor::SensorId::SensorId cam_id = static_cast<visensor::SensorId::SensorId>(std::stoi(
      args.at(0)));
  int slot_id = std::stoi(args.at(1));
  int is_flipped = std::stoi(args.at(2));

  visensor::ViCameraProjectionModel::ProjectionModelTypes projection_type;
  visensor::ViCameraLensModel::LensModelTypes lens_model_type;
  if (args.at(3) == "pinhole") {
    projection_type = visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE;
  } else if (args.at(3) == "omnidirectional") {
    projection_type = visensor::ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL;
  } else if (args.at(3) == "unknown") {
    projection_type = visensor::ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN;
  } else {
    ROS_WARN("Projection Model not defined. Use either pinhole or omnidirectional.\n");
    return false;
  }

  if (args.at(4) == "radtan") {
    lens_model_type = visensor::ViCameraLensModel::LensModelTypes::RADTAN;
  } else if (args.at(4) == "equidistant") {
    lens_model_type = visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT;
  } else if (args.at(3) == "unknown") {
    lens_model_type = visensor::ViCameraLensModel::LensModelTypes::UNKNOWN;
  } else {
    ROS_WARN("Lens Model not defined. Use either radtan or equidistant.\n");
    return false;
  }

  CalibrationFlasher calibration_flasher;
  if (!calibration_flasher.deleteCalibration(cam_id, slot_id, is_flipped, projection_type,
                                             lens_model_type)) {

    ROS_WARN("No matching calibration found\n");
  }

  return true;
}

bool cmdShow(ros::NodeHandle& nh, std::string& target_name, std::vector<std::string>& args)
{
  // check if command match to show
  if (args.size() != 0) {
    printArgs();
    return false;
  }
  CalibrationFlasher calibration_flasher;
  calibration_flasher.printAllCameraCalibration();
  return true;
}

bool cmdAdd(ros::NodeHandle& nh, std::string& target_name, std::vector<std::string>& args)
{
  // check if command match to: add <path> <slot_id>
  if (args.size() != 2) {
    printArgs();
    return false;
  }
  std::string path = args.at(0);
  int slot_id = std::stoi(args.at(1));

  YAML::Node config = YAML::LoadFile(path);
  CalibrationFlasher calibration_flasher;
  calibration_flasher.addCalibration(config, slot_id);
  return false;
}

bool cmdSetFactory(ros::NodeHandle& nh, std::string& target_name, std::vector<std::string>& args)
{
  // check if command match to setFactory <path>
  if (args.size() != 1) {
    printArgs();
    return false;
  }
  std::string path = args.at(0);

  CalibrationFlasher calibration_flasher;
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visensor_update");
  ros::NodeHandle nh("~");

  // command arguments
  typedef bool (*commandFunction)(ros::NodeHandle&, std::string&, std::vector<std::string>&);  // function pointer type
  std::map<std::string, commandFunction> argCmds = { { "delete", cmdDelete }, { "show", cmdShow }, {
      "add", cmdAdd }, { "setFactory", cmdSetFactory }, };

  //parse args
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++)
    args.push_back(std::string(argv[i]));

  /* find all sensors using auto discovery */

  //make IP optional
  std::string targetname, command;

  if (args.size() >= 2) {
    //args <IP> <COMMAND>
    targetname = args[0];
    command = args[1];
    args.erase(args.begin(), args.begin() + 2);
  } else {
    printArgs();
    exit(1);
  }

  //check command and run it
  bool success = false;

  if (!argCmds.count(command)) {
    //invalid command
    printArgs();
    exit(-1);
  }

  // run the command
  success = argCmds[command](nh, targetname, args);

  return success;
}
