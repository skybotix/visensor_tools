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

#include <dirent.h>
#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/regex.hpp>


#include "SensorUpdater.hpp"

SensorUpdater::SensorUpdater() :
  is_ssh_initialized_(false) {
}

SensorUpdater::~SensorUpdater()
{
}

void SensorUpdater::connect(const std::string &target_ip) {
  pSsh_ =  boost::make_shared<visensor::SshConnection>();
  pSsh_->sshConnect(target_ip, sshUsername(), sshPassword());
  pFile_transfer_ =  boost::make_shared<visensor::FileTransfer>(pSsh_);
  is_ssh_initialized_ = true;
}

/**
 * function returns a vector of pairs with (package_name, version)
 * for all packages that start with the given packagename prefix
 */
bool SensorUpdater::getVersionInstalled(VersionList* outPackageList)
{
  //clear the output VersionList
  outPackageList->clear();
  for (parse_function_map::const_iterator iter =  possible_pkgs_.begin(); iter != possible_pkgs_.end(); ++iter) {
    VersionEntry package;
    if (!( (this->*(iter->second))(&package, iter->first) )) {
      std::cout << "Failed to parse the version of the package " << iter->first << std::endl;
    }
    outPackageList->push_back(package);
  }
  return true;
}


bool SensorUpdater::parseVersionDefault(VersionEntry* outPackage, const std::string &prefix)
{
  int exitcode = 127;
  std::string output;
  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  pSsh_->runCommand(std::string("dpkg -l | grep ") + prefix, &output, exitcode);

  if (exitcode != 0) {
    std::cout << "failed to run " << std::string("dpkg -l | grep ") + prefix << std::endl;
    return false;
  }

  //typical line to parse:
  //  ii  vim   :7.3.547-4ubuntu1.1  amd64    Vi IMproved - enhanced vi editor
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep("\n");
  tokenizer tokens(output, sep);

  //divide by new lines
  for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
    //get the filename
    std::string line = *tok_iter;

    //parse package file names
    //typical line: visensor-linux-1.0.1-Linux.deb
    //design and check on: http://regexpal.com/
    //exract version number
    boost::regex expression("[\\s\\t]*[A-Za-z0-9]*[\\s\\t]+([A-Za-z0-9\\-]+)[\\s\\t]+"
                            "([0-9]+)\\."
                            "([0-9]+)\\."
                            "([0-9]+)[\\s\\t]+"
                            "([A-Za-z0-9-]+)[A-Za-z0-9\\-\\.\\s.()]*");
    boost::cmatch what;
     // find matches
    if (regex_match(line.c_str(), what, expression)) {
      // what[0] contains whole filename
      // what[1] contains the package name
      // what[2] contains the major version number
      // what[3] contains the minor version number
      // what[4] contains the patch version number
      // what[5] contains the arch
      outPackage->package_name = what[1];
      outPackage->version_major = boost::lexical_cast<unsigned int>(what[2]);
      outPackage->version_minor = boost::lexical_cast<unsigned int>(what[3]);
      outPackage->version_patch = boost::lexical_cast<unsigned int>(what[4]);
       // assume that there is only one package
      return true;
    } else {
      //regex match failed (file is not a valid package name...)
      std::cout << "regex failed: " << line.c_str() << "\n";
    }
  }

  // return true if exit-code = 0
  return false;
}

bool SensorUpdater::parseVersionFpgaBitstream(VersionEntry* package, const std::string &prefix) {
  std::string output;
  int exitcode=127;

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  // run command
  pSsh_->runCommand( std::string("/home/root/fpga/fpga_version.bash"), &output, exitcode );


  if (exitcode != 0) {
    std::cout << "failed to run " <<  "/home/root/fpga/fpga_version.bash" << std::endl;
    return false;
  }

  //typical lines to parse:
  //Version:        <Major.Minor.Patch>
  //Note:           config_<SensorType>_<ImuType>
  // or:
  //Version:        <Major.Minor.Patch>
  //Note:           version_<SensorType>
  typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;
  boost::char_separator<char> sep("\n");
  tokenizer tokens(output, sep);

  tokenizer::iterator tok_iter = tokens.begin();
  std::string version_line = *tok_iter;
  std::string note_line = *(++tok_iter);

  //exract version number
  boost::regex version_expression("[\\s\\t]*[A-Za-z:]*[\\s\\t]+([0-9]+)\\."
      "([0-9]+)\\."
      "([0-9]+)[\\s\\t]*[A-Za-z0-9\\-\\.\\s.()]*");
  boost::cmatch what;
  // find matches of the version
  if( regex_match(version_line.c_str(), what, version_expression) )
  {
   // what[0] contains whole filename
   // what[1] contains the major version number
   // what[2] contains the minor version number
   // what[3] contains the patch version number
   package->package_name = "visensor-fpga-bitstream";
   package->version_major = boost::lexical_cast<unsigned int>( what[1] );
   package->version_minor = boost::lexical_cast<unsigned int>( what[2] );
   package->version_patch = boost::lexical_cast<unsigned int>( what[3] );
  }
  else {
   //regex match failed (file is not a valid package name...)
   std::cout << "regex failed: " << version_line.c_str() << "\n";
   return false;
  }

  // find matches for the sensor type. Try first the new format config_<SensorType>_<ImuType>
  boost::regex type_expression("[\\s\\t]*[A-Za-z:]*[\\s\\t]+[A-Za-z]*_([a-z]+)_"
      "([A-Za-z0-9]+)");
  boost::cmatch what_type;
  if( regex_match(note_line.c_str(), what_type, type_expression) )  {
   // what_type[0] contains whole filename
   // what_type[1] contains the sensor type: a => normal configuration, c => flir configuration
   // what_type[2] contains the IMU type: A48 or A88
    try {
      package->sensor_type = supported_fpga_configs_.at(boost::lexical_cast<std::string>( what_type[1] ));
      package->imu_type = supported_imu_.at(boost::lexical_cast<std::string>( what_type[2] ));
    }
    catch (const std::exception& ex) {
      std::cout << "failed to parse FPGA types: " << ex.what() << std::endl;
      return false;
    }
  }
  else {
    // check if version was for the ADIS 16488. There is only one version for the ADIS 16488 in the old format
    VersionEntry fpga_version_adis_16488(1,0,22);
    if ( *package == fpga_version_adis_16488 ) {
      package->sensor_type = SUPPORTED_FPGA_CONFIGS::NORMAL;
      package->imu_type = SUPPORTED_IMU::ADIS_16488;
    }
    else {
      // find matches for the sensor type. Try old format version_<SensorType>
      boost::regex type_expression("[\\s\\t]*[A-Za-z:]*[\\s\\t]+[A-Za-z]*_([a-z]+)");
      // find matches
      boost::cmatch what_type;
      if( regex_match(note_line.c_str(), what_type, type_expression) ) {
        // what_type[0] contains whole filename
        // what_type[1] contains the sensor type: a => normal configuration, c => flir configuration
        try {
          package->sensor_type = supported_fpga_configs_.at(boost::lexical_cast<std::string>( what_type[1] ));
          package->imu_type = SUPPORTED_IMU::ADIS_16448;
        }
        catch (const std::exception& ex) {
          std::cout << "failed to parse FPGA types: " << ex.what() << std::endl;
          return false;
        }
      }
      else {
        //regex match failed (file is not a valid package name...)
        std::cout << "failed to parse the fpga bitstream type: " << note_line.c_str() << "\n";
        return false;
      }
    }
  }
  return true;
}

bool SensorUpdater::getVersionsOnServer(VersionList* outPackageList, const REPOS& repo) {
  // clear the output list
  outPackageList->clear();

  // query the ftp server
  std::string filelist;
  std::string repo_ftppath = REPOS_PATH.at(repo);

  // open ftp connection
  WebClient web_client(servername());

  bool success = web_client.dirList(repo_ftppath, filelist);

  if (!success)
    return false;

  //extract filenames from filelist
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(";");
  tokenizer tokensL(filelist, sep);

  //loop through all filenames
  for (tokenizer::iterator tokL_iter = tokensL.begin(); tokL_iter != tokensL.end(); ++tokL_iter) {
    //get the filename
    std::string filename = *tokL_iter;

    //parse package file names
    //typical line: visensor-linux-1.0.1-Linux.deb
    //design and check on: http://regexpal.com/
    boost::regex version_expression("([A-Za-z0-9-]+)-"
        "([0-9]+)\\."
        "([0-9]+)\\."
        "([0-9]+)-"
        "([A-Za-z0-9-]+)\\.deb");
    boost::cmatch what;

    // find matches
    if( regex_match(filename.c_str(), what, version_expression) ) {
      // what[0] contains whole filename
      // what[1] contains the package name
      // what[2] contains the major version number
      // what[3] contains the minor version number
      // what[4] contains the patch version number
      // what[5] contains the arch

      VersionEntry package;
      package.package_name = what[1];
      package.version_major = boost::lexical_cast<unsigned int>(what[2]);
      package.version_minor = boost::lexical_cast<unsigned int>(what[3]);
      package.version_patch = boost::lexical_cast<unsigned int>(what[4]);

      // store the relative ftp path (if we want to downlaod it later...)
      package.path = REPOS_PATH.at( repo ) + "/" + filename;

      // store the packages
      outPackageList->push_back(package);
    } else {
      //regex match failed (file is not a valid package name...)
      std::cout << "regex failed: " << filename.c_str() << "\n";
    }
  }

  //now remove old version (only the newest version of each package should remain in the list
  return true;
}
bool SensorUpdater::getVersionsFromLocalPath(VersionList* outPackageList, std::string path) {
  // clear the output list
  outPackageList->clear();

  std::string filelist;

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (path.c_str())) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      for (parse_function_map::const_iterator  iter =  possible_pkgs_.begin(); iter != possible_pkgs_.end(); ++iter) {
        if(ent->d_name == (iter->first + ".deb")){
          VersionEntry package;
          package.package_name = iter->first;
          package.path =  path + "/" + ent->d_name;
          outPackageList->push_back(package);
        }
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    return false;
  }

  return true;
}

bool SensorUpdater::printVersionsInstalled(void)
{
  VersionList listSensor;
  bool success = getVersionInstalled(&listSensor);

  if (!success) {
    std::cout << "Error: could not get installed versions from sensor!\n";
    return false;
  }

  std::cout << "Name\t\t\t\tVersion\n";
  std::cout << "-----------------------------------------\n";
  if (!listSensor.size()) {
    std::cout << "No packages installed!\n";
    return true;
  }

  for (size_t i = 0; i < listSensor.size(); i++) {
    std::cout << listSensor[i].package_name << "\t\t" << listSensor[i].version_major << "."
              << listSensor[i].version_minor << "." << listSensor[i].version_patch << "\n";
  }
  std::cout << std::endl;
  return true;
}

bool SensorUpdater::printVersionsRepo(const REPOS& repo)
{
  VersionList listFtp;
  bool success = getVersionsOnServer(&listFtp, repo);

  if (!success) {
    std::cout << "Error: could not get installed versions from repository!\n";
    return false;
  }

  std::cout << "Name\t\tVersion\t\tftppath\n";
  for (size_t i = 0; i < listFtp.size(); i++) {
    std::cout << listFtp[i].package_name << "\t\t" << listFtp[i].version_major << "."
              << listFtp[i].version_minor << "." << listFtp[i].version_patch << "\t\t"
              << listFtp[i].path << "\n";
  }
  return true;
}

/* reboot the sensor */
bool SensorUpdater::sensorReboot(void) const
{
  std::string output;
  int exitcode = 127;

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  // run command
  pSsh_->runCommand(std::string("reboot"), &output, exitcode);

  std::cout << "Rebooting sensor...\n";

  // return true if exit-code = 0
  return (exitcode == 0);
}


bool SensorUpdater::sensorInstallDebFile(const std::string& remotefile)
{
  std::string output;
  int exitcode = 127;

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  // mount read-write for changes
  bool success = sensorSetMountRW(true);

  if (!success)
    return false;

  // run command
  pSsh_->runCommand(std::string("dpkg -i ") + remotefile, &output, exitcode);
  // mount read-only after changes
  success = sensorSetMountRW(false);

  // return true if exit-code = 0
  return (exitcode == 0);
}

/* remove the deb package with the name package_name */
bool SensorUpdater::sensorRemoveDeb(const std::string& package_name)
{
  std::string output;
  int exitcode = 127;

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  // mount read-write for changes
  bool success = sensorSetMountRW(true);

  if (!success)
    return false;

  // run command
  pSsh_->runCommand(std::string("dpkg -P ") + package_name, &output, exitcode);
  // mount read-only after changes
  success = sensorSetMountRW(false);

  // return true if exit-code = 0
  return (exitcode == 0);
}

//RW: true ==> read-write, RW: false ==> read-only
bool SensorUpdater::sensorSetMountRW(bool RW)
{
  std::string output;
  int exitcode = 127;

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  //command
  std::string cmd;
  if (RW)
    cmd = "mount -o remount,rw /";
  else
    cmd = "mount -o remount,ro /";

  // run command
  pSsh_->runCommand(cmd, &output, exitcode);

  //0 and 255 are success exit codes
  if (exitcode != 0 && exitcode != 255) {
    std::cout << "Error: could not change RW mode on sensor!\n";
    return false;
  }

  // return true
  return true;
}


/* clean all installed packages on the sensor with the given prefix */
bool SensorUpdater::sensorClean(void)
{
  VersionList listToClean;
  // get all the installed packages with the given prefix
  getVersionInstalled(&listToClean);

  return sensorClean(listToClean);
}

/* clean all installed packages on the sensor with the given prefix */
bool SensorUpdater::sensorClean(VersionList listToClean)
{
  bool success;
  // remove package after package
  for (size_t i = 0; i < listToClean.size(); i++) {
    std::cout << "Removing " << listToClean[i].package_name << " from Sensor ... ";
    success = sensorRemoveDeb(listToClean[i].package_name);
    if (success)
      std::cout << "done.\n";
    else
      std::cout << "failed!\n";
  }
  std::cout << std::endl;
  return success;
}

/* get a list of the newest/defined versions from the repo */
bool SensorUpdater::getUpdateList(VersionList* outList, const VersionList &packageVersionList, const REPOS &repo)
{
  // get the newest version from the repos
  VersionList allPackages;
  bool success = getVersionsOnServer(&allPackages, repo);

  //extract the newst version of all mandatory packages
  VersionList updatePackages;
  int i;
  parse_function_map::const_iterator iter;
  for (iter =  possible_pkgs_.begin(), i = 0; iter != possible_pkgs_.end(); ++iter, ++i) {
    // extract all packages which are mandatory to install
    VersionList temp;

    for(size_t j=0; j<allPackages.size(); j++) {
      if( allPackages[j].package_name == iter->first) {
        if (!packageVersionList.empty()) {
          // check if the version is the requested version
          if (allPackages[j] == packageVersionList[i]) {
            if (allPackages[j].package_name != packageVersionList[i].package_name) {
              std::cerr << "requested version package name and found package name does not match" << std::endl;
              exit(-1);
            }

            temp.push_back( allPackages[j] );
            break;
          }
        }
        else {
          temp.push_back( allPackages[j] );
        }
      }
    }

    // check we found the mandatory package
    if (temp.size() < 1) {
      std::cout << "[ERROR]: Could not find the required package \""
                << iter->first << "\" in the online repository!\n";
      exit(1);
    }

    // sort for version
    std::sort(temp.begin(), temp.end());

    // add the newest version of the mandatory package to the update list
    updatePackages.push_back(temp.back());
  }
  *outList = updatePackages;
  return success;
}

/* download packages defined in packageList from repo to local path */
bool SensorUpdater::downloadPackagesToPath(const VersionList& packageList,
                                           const std::string& localPath)
{
  // download and install the needed packages
   WebClient web_client(servername());

  for (size_t i = 0; i < packageList.size(); i++) {
    std::cout << "Downloading " << packageList[i].package_name << " ...  ";

     // download
    std::string pkg_filename = localPath + packageList[i].package_name + std::string(".deb");

    bool ret = web_client.getFileToFile(packageList[i].path, pkg_filename);
    if (!ret) {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not fetch update package from online repository! \n";
      exit(1);
    }

    std::cout << "done.\n";
  }
  std::cout << std::endl;
  return true;
}

/* install packages defined in packageList from local path to sensor*/
bool SensorUpdater::installPackagesFromPath(const VersionList& packageList,
                                            const std::string& localPath)
{
  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }
  for (size_t i = 0; i < packageList.size(); i++) {
    std::cout << "Installing " << packageList[i].package_name << " ...  ";

    // download
    std::string pkg_filename = localPath + packageList[i].package_name + std::string(".deb");
    std::string pkg_remote_filename = remotePath() + packageList[i].package_name + std::string(".deb");

    // transfer file to sensor
    bool ret = pSsh_->sendFile(pkg_filename, pkg_remote_filename);
    if (!ret) {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not upload file to sensor!\n";
      exit(1);
    }

    // install
    ret = sensorInstallDebFile(pkg_remote_filename);

    if (!ret) {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not install debfile on sensor " << pkg_filename << "\n";
      exit(1);
    }
    std::cout << "done.\n";
  }
  std::cout << std::endl;
  return true;
}

bool SensorUpdater::loadPropertyTree(const std::string& calibration_filename,
                                     boost::property_tree::ptree* tree)
{
  try {
    read_xml(calibration_filename, *tree, boost::property_tree::xml_parser::trim_whitespace);
  } catch (std::exception const& ex) {
    std::cout << "failed.\n";
    std::cout << "[ERROR]: Could not load the calibration file!\n";
    std::cout << "Exception: " << ex.what() << "\n";
    return false;
  }
  return true;
}

bool SensorUpdater::loadXmlCameraCalibrationFile(const std::string& local_calibration_filename)
{
  std::string remote_calibration_filename = std::string("/calibration.xml");

  if (!is_ssh_initialized_) {
    std::cout << "sensor updater is not connected to any sensor\n";
    return false;
  }

  // transfer calibration file from the sensor
  if (!pSsh_->getFile(remote_calibration_filename, local_calibration_filename)) {
    return false;
  }
  return true;
}

/**
 * tries to load the camera calibration
 *
 * The camera calibration with the format of the libvisensor < 2.0.0 is loaded and parsed to
 * ViCameraCalibration vector. If no Xml format could be found or parsed a empty vector is returned.
 *
 * @return vector of the saved configurations
 *
 */
std::vector<visensor::ViCameraCalibration> SensorUpdater::parseXmlCameraCalibration(
    const std::string& xml_filename)
{
  std::vector<visensor::ViCameraCalibration> output_vector;
  boost::property_tree::ptree calibration_tree;

  if (!loadPropertyTree(xml_filename, &calibration_tree)) {
    exit(1);
  }

  BOOST_FOREACH(const boost::property_tree::ptree::value_type & iter, calibration_tree.get_child("") ){
    visensor::ViCameraCalibration calibration(visensor::ViCameraLensModel::LensModelTypes::RADTAN,
        visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
    std::vector<std::string> elements;
    boost::split(elements, iter.first, boost::is_any_of("_"));

    if( (elements.size() == 3) && (elements[0] == "cam") ) {
      try
      {
        calibration.cam_id_ = std::stoi(elements[1]);
        calibration.slot_id_ = std::stoi(elements[2])/2;
        calibration.is_flipped_ = std::stoi(elements[2])%2;
        calibration.resolution_[0] = 752;
        calibration.resolution_[1] = 480;
        //build childtree name
        std::string cam_id_str = boost::lexical_cast<std::string>(calibration.cam_id_);
        std::string slot_str = boost::lexical_cast<std::string>(std::stoi(elements[2]));
        std::string child_tree = std::string("cam_") + cam_id_str + std::string("_") + slot_str + std::string(".");

        //load data
        visensor::ViCameraLensModelRadtan::Ptr lens_model = calibration.getLensModel<visensor::ViCameraLensModelRadtan>();
        visensor::ViCameraProjectionModelPinhole::Ptr projection_model = calibration.getProjectionModel<visensor::ViCameraProjectionModelPinhole>();
        projection_model->focal_length_u_ = calibration_tree.get<double>(child_tree + "fu");
        projection_model->focal_length_v_ = calibration_tree.get<double>(child_tree + "fv");
        projection_model->principal_point_u_ = calibration_tree.get<double>(child_tree + "cu");
        projection_model->principal_point_v_ = calibration_tree.get<double>(child_tree + "cv");

        lens_model->k1_ = calibration_tree.get<double>(child_tree + "K0");
        lens_model->k2_ = calibration_tree.get<double>(child_tree + "K1");
        lens_model->r1_ = calibration_tree.get<double>(child_tree + "K2");
        lens_model->r2_ = calibration_tree.get<double>(child_tree + "K3");

        calibration.R_.resize(9);
        calibration.R_[0] = calibration_tree.get<double>(child_tree + "R00");
        calibration.R_[1] = calibration_tree.get<double>(child_tree + "R01");
        calibration.R_[2] = calibration_tree.get<double>(child_tree + "R02");
        calibration.R_[3] = calibration_tree.get<double>(child_tree + "R10");
        calibration.R_[4] = calibration_tree.get<double>(child_tree + "R11");
        calibration.R_[5] = calibration_tree.get<double>(child_tree + "R12");
        calibration.R_[6] = calibration_tree.get<double>(child_tree + "R20");
        calibration.R_[7] = calibration_tree.get<double>(child_tree + "R21");
        calibration.R_[8] = calibration_tree.get<double>(child_tree + "R22");

        calibration.t_.resize(3);
        calibration.t_[0] = calibration_tree.get<double>(child_tree + "t0");
        calibration.t_[1] = calibration_tree.get<double>(child_tree + "t1");
        calibration.t_[2] = calibration_tree.get<double>(child_tree + "t2");

      } catch(std::exception const& ex)
      {
        std::cout << "failed.\n";
        std::cout << "Exception: " << ex.what() << "\n";
        return output_vector;
      }
      output_vector.push_back(calibration);
    }
  }
  return output_vector;
}

bool SensorUpdater::convertCalibration()
{

  if (!is_ssh_initialized_){
    std::cout << "ssh is not initialized" << std::endl;
    return false;
  }
  visensor::ViSensorConfiguration::Ptr config_server = boost::make_shared<visensor::ViSensorConfiguration>(pFile_transfer_);

  std::cout << "Convert calibration to new format: ";

  std::cout << "Try to load old config... ";
  std::string tmp_calibration_filename("/tmp/calibration.xml");
  if (!loadXmlCameraCalibrationFile(tmp_calibration_filename)) {
    std::cout << "failed.\n" << std::endl;
    std::cout << "no calibration file was found, assume that the sensor is not yet calibrate" << std::endl;
    std::cout << std::endl;
    return checkConfiguration(config_server);
  }
  std::cout << "done." << std::endl;

  std::cout << "Load new configuration in case the conversation is done multiple time... ";
  // try to load existing configuration already saved in the new format
  try {
    config_server->loadConfig();
    std::cout << "done." << std::endl;
  } catch (visensor::exceptions const &ex) {
    std::cout << "ignore" << std::endl
              << "no new configurations were found, assume that the sensor has none" << std::endl;
    std::cout << "Exception was: " << ex.what() << std::endl;
  }

  std::cout << "Convert calibration to new format ... ";
  std::vector<visensor::ViCameraCalibration> calibration_list =
      parseXmlCameraCalibration(tmp_calibration_filename);
  if (calibration_list.size() == 0) {
    std::cout << "failed\n";
    std::cout << "no calibrations were found" << std::endl;
    checkConfiguration(config_server);
    return false;
  }
  try {
    for (std::vector<visensor::ViCameraCalibration>::iterator it = calibration_list.begin();
        it != calibration_list.end(); ++it) {
      config_server->setCameraCalibration(*it);
    }
    config_server->saveConfig();
  } catch (visensor::exceptions const &ex) {
    std::cout << "failed" << std::endl << "Setting of the new calibration failed!" << std::endl;
    std::cout << "Exception was: " << ex.what() << std::endl;
    checkConfiguration(config_server);
    return false;
  }
  std::cout << "done." << std::endl << std::endl;

  return checkConfiguration(config_server);
}

bool SensorUpdater::checkConfiguration(visensor::ViSensorConfiguration::Ptr& config_server) {
  if ( (!config_server->isValid()) || (config_server->getViSensorId() < 0) ) {
    int sensorID;
    std::cout << std::endl << "The new configuration requested the Vi-Sensor ID." << std::endl;
    while ((std::cout << "Sensor ID (integer): ") && !(std::cin >> sensorID)) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      std::cout << "Invalid input; please re-enter." << std::endl;
    }
    try {
      config_server->setViSensorId(sensorID);
      config_server->saveConfig();
    }
    catch (visensor::exceptions const &ex) {
      std::cout << "Setting of sensor ID failed!" << std::endl;
      std::cout <<  "Exception was: " << ex.what() << std::endl;
      return false;
    }
    std::cout << std::endl;
  }
  return true;
}

bool SensorUpdater::checkCalibrationConvertion(const VersionList& old_list,
                                               const VersionList& new_list)
{
  VersionEntry linux_embedded_entry;
  VersionEntry version_of_cali_change(2,0,0);
  //check if the calibration need to be converted
  if (old_list.size() == 0) {
    std::cout << "Try to copy possible available calibration ... " << std::endl;
    if (!convertCalibration()) {
      std::cerr << "Could not convert calibration to new format" << std::endl;
      return false;
    }
  } else {
    size_t i;
    for (i = 0; i < old_list.size(); i++) {
      if (old_list[i].package_name == "visensor-linux-embedded") {
        break;
      }
    }
    if ((i >= old_list.size()) && (old_list.size() != new_list.size())) {
      std::cerr << "could not found linux embedded version" << std::endl;
      return false;
    }
    if ((old_list[i] < version_of_cali_change)
        && ((new_list[i] > version_of_cali_change) || (new_list[i] == version_of_cali_change))) {
      if (!convertCalibration()) {
        std::cerr << "could not convert calibration to new format" << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool SensorUpdater::checkRepo(REPOS &repo) {
  // check if fpga version is supported
  VersionEntry fpga_version;
  if ((this->*(possible_pkgs_.at("visensor-fpga-bitstream")))(&fpga_version, "visensor-fpga-bitstream") ) {
    if (fpga_version.sensor_type != SUPPORTED_FPGA_CONFIGS::NORMAL) {
      std::cout << "Please use the manual update. The update tool does not support your FPGA configuration" << std::endl;
      return false;
    }
  }
  else {
    std::cout << "failed to get fpga config!\n" << std::endl;
    return false;
  }

  switch (repo) {
    case REPOS::REPO_RELEASE:
      if (fpga_version.imu_type == SUPPORTED_IMU::ADIS_16488) {
        repo = REPOS::REPO_16488_RELEASE;
      }
      else {
        repo = REPOS::REPO_16448_RELEASE;
      }
      break;
    case REPOS::REPO_DEV:
      if (fpga_version.imu_type == SUPPORTED_IMU::ADIS_16488) {
        repo = REPOS::REPO_16488_DEV;
      }
      else {
        repo = REPOS::REPO_16448_DEV;
      }
      break;
    case REPOS::REPO_16448_RELEASE:
    case REPOS::REPO_16448_DEV:
    case REPOS::REPO_16488_RELEASE:
    case REPOS::REPO_16488_DEV:
    default:
      break;
  }
  return true;
}

/* remove all old packages and install the newest/given version of all packages in the repo which are mandatory */
bool SensorUpdater::sensorUpdate(REPOS &repo, const VersionList& requestedVersionList) {
  VersionList list;
  VersionList currentList;
  std::string localPath = std::string("/tmp/");

  if(!getVersionInstalled(&currentList)) {
    std::cout << "No ViSensor packages were installed on the sensor. Please check your settings or flash your sensor manualy" << std::endl;
    return false;
  }

  if(!checkRepo(repo)) {
    return false;
  }

  if(!getUpdateList(&list, requestedVersionList, repo)) {
    return false;
  }

  if(!downloadPackagesToPath(list, localPath)) {
    return false;
  }

  if (!checkCalibrationConvertion(currentList, list)) {
    return false;
  }

  if(!sensorClean(list)) {
    return false;
  }

  // update to newest version
  if(!installPackagesFromPath(list, localPath)) {
    return false;
  }

  return true;
}


bool SensorUpdater::sensorDownloadTo(REPOS &repo, const std::string path, const VersionList& requestedVersionList) {
  VersionList list;
  VersionList currentList;

  if(!getUpdateList(&list, requestedVersionList, repo)) {
    return false;
  }

  if(!downloadPackagesToPath(list, path)) {
    return false;
  }

  return true;
}



bool SensorUpdater::sensorUploadFrom(const std::string path) {
  VersionList list;
  VersionList currentList;

  getVersionInstalled(&currentList);

  if(!getVersionsFromLocalPath(&list, path) || (list.size() == 0)) {
    std::cerr << "no binaries were found." << std::endl << std::endl;
    return false;
  }

  // try to convert calibration in any case
  std::cout << "Try to copy possible available calibration ... " << std::endl;
  if (!convertCalibration()) {
    std::cerr << "Could not convert calibration to new format" << std::endl;
  }

  if(!sensorClean(list)) {
    return false;
  }

  // update to saved version
  if(!installPackagesFromPath(list, path)) {
    return false;
  }
  return true;
}
