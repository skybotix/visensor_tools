/*
 * Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)
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

#include <boost/regex.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include "SensorUpdater.hpp"

void printArgs(void)
{
    std::cout << "visensor_update <SENSOR_IP> <COMMAND> <CMD_ARGS>" << std::endl;
    std::cout << std::endl;

    std::cout << "  Available commands are:" << std::endl;
    std::cout << "     update               updates the sensor to the newest software on the online repo, check for the correct IMU first" << std::endl;
    std::cout << "     update <imu-type>    updates the sensor with the specified IMU version to the newest software on the online repo.\n"
                 "                          The imu-type can be 16448 or 16488" << std::endl;
    std::cout << "     convert-calibration  converts the calibration format from libvisensor version 1.2.X to 2.0.X"  << std::endl;
    std::cout << "     update <fpga-version> <kernel-version> <embedded-version>\n"
                 "                          updates the sensor to the given version from the online repo, check for the correct IMU first" << std::endl;
    std::cout << "     update <imu-type> <fpga-version> <kernel-version> <embedded-version>\n"
                 "                          updates the sensor to the given version from the online repo, select the IMU" << std::endl;
    std::cout << "     download_to <path> <imu-type>\n"
                 "                          download newest packages to a given path. The imu-type can be 16448 or 16488" << std::endl;
    std::cout << "     download_to <path> <imu-type> <fpga-version> <kernel-version> <embedded-version>\n"
                 "                          download given version to a given path. The imu-type can be 16448 or 16488" << std::endl;
    std::cout << "     update_from <path>\n"
                 "                          upload previous downlaoded software to the path." << std::endl;
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

SensorUpdater::VersionEntry parse_version(std::string version_string) {
  SensorUpdater::VersionEntry package;
  boost::regex version_expression("([0-9]+)\\.([0-9]+)\\.([0-9]+)");
  boost::cmatch what;
  /* find matches of the version */
  if( regex_match(version_string.c_str(), what, version_expression) )
  {
   // what[0] contains whole version
   // what[1] contains the major version number
   // what[2] contains the minor version number
   // what[3] contains the patch version number
   package.version_major = boost::lexical_cast<unsigned int>( what[1] );
   package.version_minor = boost::lexical_cast<unsigned int>( what[2] );
   package.version_patch = boost::lexical_cast<unsigned int>( what[3] );
  }
  else {
   //regex match failed (file is not a valid package name...)
   std::cout << "failed to parse version: " << version_string.c_str() << "\n";
   printArgs();
   exit(1);
  }
  return package;
}

void parse_versions(const std::vector<std::string> &args, SensorUpdater::VersionList& requestedVersions) {
  SensorUpdater::VersionEntry arg_fpga_version = parse_version(args[0]);
  arg_fpga_version.package_name = "visensor-fpga-bitstream";
  requestedVersions.push_back(arg_fpga_version);
  SensorUpdater::VersionEntry arg_kernel_version = parse_version(args[1]);
  arg_kernel_version.package_name = "visensor-kernel-modules";
  requestedVersions.push_back(arg_kernel_version);
  SensorUpdater::VersionEntry arg_embedded_version = parse_version(args[2]);
  arg_embedded_version.package_name = "visensor-linux-embedded";
  requestedVersions.push_back(arg_embedded_version);
}

bool update(SensorUpdater &updater, SensorUpdater::REPOS repo, SensorUpdater::VersionList & requestedVersionList)
{
  /* print version before update */
  std::cout << "Before update:\n";
  updater.printVersionsInstalled();

  /* install the newest version of all mandatory packages */
  bool success = updater.sensorUpdate(repo, requestedVersionList);

  /* print version before update */
  std::cout << "After update:\n";
  updater.printVersionsInstalled();

  /* reboot the sensor */
  if (success)
    updater.sensorReboot();

  return success;
}


bool cmdUpdateFrom(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  std::string path;
  SensorUpdater::VersionList packageList;
  // check if command: upload_from <path>
  if(args.size() != 1)  {
    printArgs();
    return false;
  }
  path = args[0];

  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  /* print version before update */
  std::cout << "Before update:\n";
  updater.printVersionsInstalled();

  /* install the downloaded version */
  std::cout << "Installing packages from: " << path << std::endl;
  bool success = updater.sensorUploadFrom(path);

  /* print version before update */
  std::cout << "After update:\n";
  updater.printVersionsInstalled();

  /* reboot the sensor */
  if (success)
    updater.sensorReboot();

  return success;
}

bool cmdUpdate(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args) {
  SensorUpdater::VersionList requestedVersions;
  std::map<std::string, SensorUpdater::REPOS> arg_repos =
  {
    {"16448", SensorUpdater::REPOS::REPO_16448_RELEASE},
    {"16488", SensorUpdater::REPOS::REPO_16488_RELEASE}
  };

  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  // check if command: update <imu-type>
  if(args.size() == 1)  {
    SensorUpdater::VersionList requestedVersions;
    std::string arg_repo = args[0];
    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      exit(-1);
    }
    return update(updater, arg_repos.at(arg_repo), requestedVersions);
  }
  // check if command: update <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 3)  {
    parse_versions(args, requestedVersions);
    return update(updater, SensorUpdater::REPOS::REPO_RELEASE, requestedVersions);
  }
  // check if command: update <imu-type> <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 4)  {
    std::string arg_repo = args[0];
    args.erase(args.begin());
    parse_versions(args, requestedVersions);

    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      exit(-1);
    }
    return update(updater, arg_repos.at(arg_repo), requestedVersions);
  }
  // use the default
  return update(updater, SensorUpdater::REPOS::REPO_RELEASE, requestedVersions);
}

bool cmdUpdateDevelop(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  SensorUpdater::VersionList requestedVersions;
  std::map<std::string, SensorUpdater::REPOS> arg_repos =
  {
    {"16448", SensorUpdater::REPOS::REPO_16448_DEV},
    {"16488", SensorUpdater::REPOS::REPO_16488_DEV}
  };

  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  if(args.size() == 1)  {
  // check if command: update-devel <imu-type>
    std::string arg_repo = args[0];
    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      exit(-1);
    }
    return update(updater, arg_repos.at(arg_repo), requestedVersions);
  }
  // check if command: update-devel <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 3)  {
    parse_versions(args, requestedVersions);
    return update(updater, SensorUpdater::REPOS::REPO_DEV, requestedVersions);
  }
  // check if command: update-devel <imu-type> <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 4)  {
    std::string arg_repo = args[0];
    args.erase(args.begin());
    parse_versions(args, requestedVersions);

    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      exit(-1);
    }
    return update(updater, arg_repos.at(arg_repo), requestedVersions);
  }
  // use the default
  return update(updater, SensorUpdater::REPOS::REPO_DEV, requestedVersions);
}

bool cmdDownloadTo(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args) {
  SensorUpdater::VersionList requestedVersions;
  SensorUpdater::REPOS repository;
  std::string path;
  std::map<std::string, SensorUpdater::REPOS> arg_repos =
  {
    {"16448", SensorUpdater::REPOS::REPO_16448_RELEASE},
    {"16488", SensorUpdater::REPOS::REPO_16488_RELEASE}
  };

  // check if command: download-devel_to <path> <imu-type>
  if(args.size() == 2)  {
    SensorUpdater::VersionList requestedVersions;
    path = args[0];
    std::string arg_repo = args[1];
    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      return false;
    }
    repository = arg_repos.at(arg_repo);
  }
  // check if command: download_to <path> <imu-type> <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 5)  {
    path = args[0];
    std::string arg_repo = args[1];
    args.erase(args.begin(),args.begin() + 2);
    parse_versions(args, requestedVersions);

    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      return false;
    }
    repository = arg_repos.at(arg_repo);
  }
  else {
    // print help
    printArgs();
    return false;
  }
  return updater.sensorDownloadTo(repository, path, requestedVersions);
}

bool cmdDownloadDevelTo(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args) {
  SensorUpdater::VersionList requestedVersions;
  SensorUpdater::REPOS repository;
  std::string path;
  std::map<std::string, SensorUpdater::REPOS> arg_repos =
  {
    {"16448", SensorUpdater::REPOS::REPO_16448_DEV},
    {"16488", SensorUpdater::REPOS::REPO_16488_DEV}
  };

  // check if command: download_to <path> <imu-type>
  if(args.size() == 2)  {
    SensorUpdater::VersionList requestedVersions;
    path = args[0];
    std::string arg_repo = args[1];
    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      return false;
    }
    repository = arg_repos.at(arg_repo);
  }
  // check if command: download-devel_to <path> <imu-type> <fpga-version> <kernel-version> <embedded-version>
  else if(args.size() == 5)  {
    path = args[0];
    std::string arg_repo = args[1];
    args.erase(args.begin(),args.begin() + 2);
    parse_versions(args, requestedVersions);

    if( !arg_repos.count( arg_repo ) )
    {
      //invalid command
      printArgs();
      return false;
    }
    repository =  arg_repos.at(arg_repo);
  }
  else {
    // print help
    printArgs();
    return false;
  }
  return updater.sensorDownloadTo(repository, path, requestedVersions);
}

bool cmdConvertCalibration(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  return updater.convertCalibration();
}

bool cmdClean(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  return updater.sensorClean();
}

bool cmdVersion(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  return updater.printVersionsInstalled();
}

bool cmdReboot(SensorUpdater& updater, std::string& target_name, std::vector<std::string>& args)
{
  // try to connect to the specified hostname/IP
  updater.connect(target_name);

  return updater.sensorReboot();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visensor_update");
  ros::NodeHandle nh;

  // command arguments
  typedef bool (*commandFunction)(SensorUpdater&, std::string&, std::vector<std::string>&); // function pointer type
  std::map<std::string, commandFunction> argCmds =
  {
      {"update", cmdUpdate},
      {"update-devel", cmdUpdateDevelop},
      {"download_to", cmdDownloadTo},
      {"download-devel_to", cmdDownloadDevelTo},
      {"update_from", cmdUpdateFrom},
      {"convert-calibration", cmdConvertCalibration},
      {"clean", cmdClean},
      {"reboot", cmdReboot},
      {"version", cmdVersion}
  };

  //parse args
  std::vector<std::string> args;
  for(int i=1; i<argc; i++)
    args.push_back( std::string(argv[i]) );

  //make IP optional
  std::string targetname,
              command;

  if(args.size() >= 2)
  {
    //args <IP> <COMMAND>
    targetname = args[0];
    command = args[1];
    args.erase(args.begin(),args.begin()+2);
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

  // initialize updater without connecting to device
  SensorUpdater updater;

  // run the command
  success = argCmds[command](updater, targetname, args);

  return success;
}

