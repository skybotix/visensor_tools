/*
 * main.cpp
 *
 *  Created on: Dec 1, 2013
 *      Author: skybotix
 */

#include "SensorUpdater.hpp"

#include <iostream>
#include <vector>
#include <map>

void printArgs(void)
{
    std::cout << "visensor_update <SENSOR_IP> <COMMAND> <CMD_ARGS>" << std::endl;
    std::cout << std::endl;

    std::cout << "  Available commands are:" << std::endl;
    std::cout << "     update               updates the sensor to the newest on the online repo" << std::endl;
    std::cout << "     update-develop       updates the sensor to the newest develop on the online repo" << std::endl;
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

bool cmdUpdate(SensorUpdater &updater)
{
  /* print version before update */
  std::cout << "Before update:\n";
  updater.printVersionsInstalled();

  /* delete all installed packages */
  updater.sensorClean();
  std::cout << "\n";

  /* install the newest version of all mandatory packages */
  bool success = updater.sensorUpdate(UpdateConfig::REPOS::REPO_RELEASE);
  std::cout << "\n";

  /* print version before update */
  std::cout << "After update:\n";
  updater.printVersionsInstalled();

  /* reboot the sensor */
  updater.sensorReboot();

  return success;
}

bool cmdUpdateDevelop(SensorUpdater &updater)
{
  /* print version before update */
  std::cout << "Before update:\n";
  updater.printVersionsInstalled();

  /* delete all installed packages */
  updater.sensorClean();
  std::cout << "\n";

  /* install the newest version of all mandatory packages */
  bool success = updater.sensorUpdate(UpdateConfig::REPOS::REPO_DEV);
  std::cout << "\n";

  /* print version before update */
  std::cout << "After update:\n";
  updater.printVersionsInstalled();

  /* reboot the sensor */
  updater.sensorReboot();

  return success;
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
  // command arguments
  typedef bool (*commandFunction)(SensorUpdater&); // function pointer type
  std::map<std::string, commandFunction> argCmds =
  {
      {"update", cmdUpdate},
      {"update-develop", cmdUpdateDevelop},
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

