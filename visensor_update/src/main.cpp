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
#include "SensorUpdater.hpp"

#include <iostream>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <ros/package.h>

void printArgs(void)
{
    std::cout << "visensor_update <SENSOR_IP> <COMMAND> <CMD_ARGS>" << std::endl;
    std::cout << std::endl;

    std::cout << "  Available commands are:" << std::endl;
    std::cout << "     update               updates the sensor to the newest software on the online repo" << std::endl;
    std::cout << "     update-16488         updates the sensor with ADIS 16488 to the newest software on the online repo" << std::endl;
    std::cout << "     clean                removes all software on the sensor" << std::endl;
    std::cout << "     version              shows installed packages " << std::endl;
    std::cout << "     reboot               reboot sensor " << std::endl;
    //std::cout << "     install <PKG-FILE>   install custom package " << std::endl;
    //std::cout << "     remove <PKG-NAME>    removes custom package " << std::endl;
    std::cout << std::endl;

    std::cout << "  Perform sensor update:" << std::endl;
    std::cout << "     visensor_update 10.0.0.1 update" << std::endl;
    std::cout << std::endl;
}

bool update(SensorUpdater &updater, SensorUpdater::REPOS repo)
{
  /* print version before update */
  std::cout << "Before update:\n";
  updater.printVersionsInstalled();

  /* install the newest version of all mandatory packages */
  bool success = updater.sensorUpdate(repo);

  /* print version before update */
  std::cout << "After update:\n";
  updater.printVersionsInstalled();

  /* reboot the sensor */
  updater.sensorReboot();

  return success;
}

bool cmdUpdate(SensorUpdater &updater)
{
  return update(updater, SensorUpdater::REPOS::REPO_RELEASE);
}

bool cmdUpdate16488(SensorUpdater &updater)
{
  return update(updater, SensorUpdater::REPOS::REPO_16488_RELEASE);
}

bool cmdUpdateDevelop(SensorUpdater &updater)
{
  return update(updater, SensorUpdater::REPOS::REPO_DEV);
}

bool cmdUpdate16488Develop(SensorUpdater &updater)
{
  return update(updater, SensorUpdater::REPOS::REPO_16488_DEV);
}

bool cmdConvertCalibration(SensorUpdater &updater)
{
  return updater.convertCalibration();
}

bool cmdClean(SensorUpdater &updater)
{
  return updater.sensorClean();
}

bool cmdVersion(SensorUpdater &updater)
{
  return updater.printVersionsInstalled();
}

bool cmdReboot(SensorUpdater &updater)
{
  return updater.sensorReboot();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visensor_update");
  ros::NodeHandle nh;

  // command arguments
  typedef bool (*commandFunction)(SensorUpdater&); // function pointer type
  std::map<std::string, commandFunction> argCmds =
  {
      {"update", cmdUpdate},
      {"update-16488", cmdUpdate16488},
      {"update-devel", cmdUpdateDevelop},
      {"update-16488-devel", cmdUpdate16488Develop},
      {"convert-calibration", cmdConvertCalibration},
      {"clean", cmdClean},
      {"reboot", cmdReboot},
      {"version", cmdVersion},
  };

  //parse args
  std::vector<std::string> args;
  for(int i=1; i<argc; i++)
    args.push_back( std::string(argv[i]) );

  /* find all sensors using auto discovery */

  //make IP optional
  std::string hostname,
              command;

  if(args.size() == 2)
  {
    //args <IP> <COMMAND>
    hostname = args[0];
    command = args[1];
  }
  else if (args.size() == 1)
  {
    //args <COMMAND>, use default hostname
    std::cout << "Sensor IP address not specified; using default (10.0.0.1)\n\n";
    hostname = std::string("10.0.0.1");
    command = args[0];
  }
  else
  {
    printArgs();
    exit(1);
  }

  //check command and run it
  bool success=false;

  if( !argCmds.count( command ) )
  {
    //invalid command
    printArgs();
    exit(-1);
  }

  // try to connect to the specified hostname/IP
  SensorUpdater updater(hostname);

  // run the command
  success = argCmds[command](updater);

  return success;
}

