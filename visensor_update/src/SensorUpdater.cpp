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

#include "SensorUpdater.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/regex.hpp>

#include <map>


SensorUpdater::SensorUpdater(const std::string &hostname) {
  pSsh_ =  boost::make_shared<visensor::SshConnection>(hostname, UpdateConfig::ssh_username, UpdateConfig::ssh_password);
  pFile_transfer_ =  boost::make_shared<visensor::FileTransfer>(pSsh_);
}


SensorUpdater::~SensorUpdater()
{
}

//function returns a vector of pairs with (package_name, version)
//for all packages that start with the given packagename prefix
bool SensorUpdater::getVersionInstalled(VersionList &outPackageList, const std::string &prefix, bool dontParseVersion)
{
  int exitcode=127;

  /* run command */
  std::string output;
  pSsh_->runCommand( std::string("dpkg -l | grep ") + prefix,
                     output,
                     exitcode );

  //clear the output VersionList
  outPackageList.clear();

  //typical line to parse:
  //  ii  vim                                         2:7.3.547-4ubuntu1.1                       amd64        Vi IMproved - enhanced vi editor
  typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;
  boost::char_separator<char> sep("\n");
  tokenizer tokens(output, sep);


  //divide by new lines
  for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
  {

    //get the filename
     std::string line = *tok_iter;

     //parse package file names
     //typical line: visensor-linux-1.0.1-Linux.deb
     //design and check on: http://regexpal.com/

     boost::regex *expression;

     if(!dontParseVersion)
     {
       //exract version number
       expression = new boost::regex("[\\s\\t]*[A-Za-z0-9]*[\\s\\t]+([A-Za-z0-9\\-]+)[\\s\\t]+([0-9]+)\\.([0-9]+)\\.([0-9]+)[\\s\\t]+([A-Za-z0-9-]+)[A-Za-z0-9\\-\\.\\s.()]*");
     } else {
       //do not extract version number
       expression = new boost::regex("[\\s\\t]*[A-Za-z0-9]{2}[\\s\\t]+([A-Za-z0-9\\-]+)[\\s\\t]+([A-Za-z0-9\\-\\.\\s.()]+)");
     }


     boost::cmatch what;

     /* find matches */
     if( regex_match(line.c_str(), what, *expression) )
     {
       // what[0] contains whole filename
       // what[1] contains the package name
       // what[2] contains the major version number
       // what[3] contains the minor version number
       // what[4] contains the patch version number
       // what[5] contains the arch

       VersionEntry package;
       package.package_name = what[1];
       if(!dontParseVersion)
       {
         package.version_major = boost::lexical_cast<unsigned int>( what[2] );
         package.version_minor = boost::lexical_cast<unsigned int>( what[3] );
         package.version_patch = boost::lexical_cast<unsigned int>( what[4] );
       }

       /* add package to the list of packages */
       outPackageList.push_back( package );

     } else {
       //regex match failed (file is not a valid package name...)
       std::cout << "regex failed: " << line.c_str() << "\n";
     }
  }

  /* return true if exit-code = 0 */
  return true;
}



bool SensorUpdater::getVersionsOnServer(VersionList &outPackageList, UpdateConfig::REPOS repo)
{
  /* clear the output list*/
  outPackageList.clear();

  /* query the ftp server */
  std::string filelist;
  std::string repo_ftppath = UpdateConfig::REPOS_PATH[ static_cast<size_t>( repo ) ];

  /* open ftp connection */
  WebClient web_client(UpdateConfig::hostname);

  bool success = web_client.dirList(repo_ftppath, filelist);

  if(!success)
    return false;

  //extract filenames from filelist
  typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;
  boost::char_separator<char> sep(";");
  tokenizer tokensL(filelist, sep);

  //loop through all filenames
  for (tokenizer::iterator tokL_iter = tokensL.begin(); tokL_iter != tokensL.end(); ++tokL_iter)
  {

    //get the filename
    std::string filename = *tokL_iter;

    //parse package file names
    //typical line: visensor-linux-1.0.1-Linux.deb
    //design and check on: http://regexpal.com/
    boost::regex expression("([A-Za-z0-9-]+)-([0-9]+)\\.([0-9]+)\\.([0-9]+)-([A-Za-z0-9-]+)\\.deb");
    boost::cmatch what;

    /* find matches */
    if( regex_match(filename.c_str(), what, expression) )
    {
      // what[0] contains whole filename
      // what[1] contains the package name
      // what[2] contains the major version number
      // what[3] contains the minor version number
      // what[4] contains the patch version number
      // what[5] contains the arch

      VersionEntry package;
      package.package_name = what[1];
      package.version_major = boost::lexical_cast<unsigned int>( what[2] );
      package.version_minor = boost::lexical_cast<unsigned int>( what[3] );
      package.version_patch = boost::lexical_cast<unsigned int>( what[4] );

      /* store the relative ftp path (if we want to downlaod it later...) */
      package.path = UpdateConfig::REPOS_PATH[ static_cast<size_t>( repo ) ] + "/" + filename;


      /* store the packages */
      outPackageList.push_back( package );

    } else {
      //regex match failed (file is not a valid package name...)
      //std::cout << "regex failed: " << filename.c_str() << "\n";
    }
  }

  //now remove old version (only the newest version of each package should remain in the list
  return true;
}

bool SensorUpdater::printVersionsInstalled(void)
{
  VersionList listSensor;
  bool success = getVersionInstalled(listSensor, UpdateConfig::prefix);

  if(!success)
  {
    std::cout << "Error: could not get installed versions from sensor!\n";
    return false;
  }

  std:: cout << "Name\t\t\t\tVersion\n";
  std:: cout << "-----------------------------------------\n";

  if( !listSensor.size() )
  {
    std::cout << "No packages installed!\n";
    return true;
  }

  for(size_t i=0; i<listSensor.size(); i++)
  {
    std::cout <<  listSensor[i].package_name << "\t\t"  <<
                  listSensor[i].version_major <<
                  "." <<
                  listSensor[i].version_minor <<
                  "." <<
                  listSensor[i].version_patch <<
                  "\n";
  }

  std::cout << std::endl;

  return true;
}

bool SensorUpdater::printVersionsRepo(UpdateConfig::REPOS repo)
{
  VersionList listFtp;
  bool success = getVersionsOnServer(listFtp, repo);

  if(!success)
  {
    std::cout << "Error: could not get installed versions from repository!\n";
    return false;
  }


  std:: cout << "Name\t\tVersion\t\tftppath\n";

  for(size_t i=0; i<listFtp.size(); i++)
  {
    std::cout <<  listFtp[i].package_name << "\t\t"  <<
                  listFtp[i].version_major <<
                  "." <<
                  listFtp[i].version_minor <<
                  "." <<
                  listFtp[i].version_patch << "\t\t"  <<
                  listFtp[i].path <<
                  "\n";
  }

  return true;
}


/* reboot the sensor */
bool SensorUpdater::sensorReboot(void) const
{
  std::string output;
  int exitcode=127;

  /* run command */
  pSsh_->runCommand( std::string("reboot"),
                     output,
                     exitcode );

  std::cout << "Rebooting sensor...\n";

  /* return true if exit-code = 0 */
  return (exitcode == 0);
}


/* install a debian package which is on the sensor */
bool sensorInstallDebMemory(const std::string &debian_package);


/* install a debian package which is on the sensor */
bool SensorUpdater::sensorInstallDebFile(const std::string &remotefile)
{
  std::string output;
  int exitcode=127;

  /* mount read-write for changes */
  bool success = sensorSetMountRW(true);

  if(!success)
    return false;

  /* run command */
  pSsh_->runCommand( std::string("dpkg -i ") + remotefile,
                     output,
                     exitcode );


  /* mount read-only after changes */
  success = sensorSetMountRW(false);

  /* return true if exit-code = 0 */
  return (exitcode == 0);
}


/* remove the deb package with the name package_name */
bool SensorUpdater::sensorRemoveDeb(const std::string &package_name)
{
  std::string output;
  int exitcode=127;

  /* mount read-write for changes */
  bool success = sensorSetMountRW(true);

  if(!success)
    return false;

  /* run command */
  pSsh_->runCommand( std::string("dpkg -P ") + package_name,
                     output,
                     exitcode );


  /* mount read-only after changes */
  success = sensorSetMountRW(false);

  /* return true if exit-code = 0 */
  return (exitcode == 0);
}

//RW: true ==> read-write, RW: false ==> read-only
bool SensorUpdater::sensorSetMountRW(bool RW)
{

  std::string output;
  int exitcode=127;

  /*command */
  std::string cmd;
  if(RW)
    cmd = "mount -o remount,rw /";
  else
    cmd = "mount -o remount,ro /";

  /* run command */
  pSsh_->runCommand( cmd,
                     output,
                     exitcode );

  //0 and 255 are success exit codes
  if( exitcode != 0 && exitcode != 255)
  {
    std::cout << "Error: could not change RW mode on sensor!\n";
    return false;
  }

  /* return true */
  return true;
}


/* clean all installed packages on the sensor with the given prefix */
bool SensorUpdater::sensorClean(void)
{
  VersionList listSensor;

  /* get all the installed packages with the given prefix */
  bool success = getVersionInstalled(listSensor, UpdateConfig::prefix, true);

  if(!success)
  {
    std::cout << "Error: could not get installed versions from sensor!\n";
    return false;
  }

  /* remove package after package */
  for(size_t i=0; i<listSensor.size(); i++)
  {
    std::cout << "Removing " << listSensor[i].package_name << " from Sensor ... ";
    success = sensorRemoveDeb(listSensor[i].package_name);
    if(success)
      std::cout << "done.\n";
    else
      std::cout << "failed!\n";
  }
  std::cout << std::endl;
  return success;
}

/* get a list of the newest versions from the repo */
bool SensorUpdater::getUpdateList(VersionList &outList, const UpdateConfig::REPOS &repo)
{
  /* get the newest version from the repos */
  VersionList allPackages;
  bool success = getVersionsOnServer(allPackages, repo);


  //extract the newst version of all mandatory packages
  VersionList updatePackages;

  for(size_t i=0; i<UpdateConfig::repo_mandatory_pkgs.size(); i++)
  {
    /* extract all packages which are mandatory to install */
    VersionList temp;

    for(size_t j=0; j<allPackages.size(); j++)
      if( allPackages[j].package_name == UpdateConfig::repo_mandatory_pkgs[i])
        temp.push_back( allPackages[j] );

    /* check we found the mandatory package */
    if(temp.size()<1)
    {
      std::cout << "[ERROR]: Could not find the required package \"" << UpdateConfig::repo_mandatory_pkgs[i] << "\" in the online repository!\n";
      exit(1);
    }

    /* sort for version */
    std::sort(temp.begin(), temp.end());

    /* add the newest version of the mandatory package to the update list */
    updatePackages.push_back( temp.back() );
  }

  outList = updatePackages;

  return success;
}

/* download packages defined in packageList from repo to local path */
bool SensorUpdater::downloadPackagesToPath(VersionList &packageList, const std::string &localPath)
{
  /* download and install the needed packages */
   WebClient web_client(UpdateConfig::hostname);

   for(size_t i=0; i<packageList.size(); i++)
   {
     std::cout << "Downloading " << packageList[i].package_name << " ...  ";

     /* download */
     std::string pkg_filename = localPath + packageList[i].package_name + std::string(".deb");

     bool ret = web_client.getFileToFile(packageList[i].path, pkg_filename);
     if(!ret)
     {
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
bool SensorUpdater::installPackagesFromPath(VersionList &packageList, const std::string &localPath)
{
  for(size_t i=0; i<packageList.size(); i++)
  {
    std::cout << "Installing " << packageList[i].package_name << " ...  ";

    /* download */
    std::string pkg_filename = localPath + packageList[i].package_name + std::string(".deb");

    /* transfer file to sensor */
    bool ret = pSsh_->sendFile(pkg_filename, pkg_filename);
    if(!ret)
    {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not upload file to sensor!\n";
      exit(1);
    }

    /* install */
    ret = sensorInstallDebFile(pkg_filename);

    if(!ret)
    {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not install debfile on sensor " << pkg_filename << "\n";
      exit(1);
    }

    std::cout << "done.\n";
  }
  std::cout << std::endl;
  return true;
}


bool SensorUpdater::loadPropertyTree(std::string calibration_filename, boost::property_tree::ptree& tree) {
  try
  {
    read_xml(calibration_filename, tree, boost::property_tree::xml_parser::trim_whitespace );
  } catch(std::exception const&  ex)
  {
    std::cout << "failed.\n";
    std::cout << "[ERROR]: Could not load the calibration file!\n";
    std::cout << "Exception: " << ex.what() << "\n";
    return false;
  }

  return true;
}
bool SensorUpdater::loadXmlCameraCalibrationFile(std::string local_calibration_filename) {
    std::string remote_calibration_filename = std::string("/calibration.xml");
    /* transfer calibration file from the sensor */
    if(!pSsh_->getFile(remote_calibration_filename, local_calibration_filename))
    {
      std::cout << "failed.\n";
      std::cout << "[ERROR]: Could not download calibration file from the sensor!\n";
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
std::vector<visensor::ViCameraCalibration>  SensorUpdater::parseXmlCameraCalibration(std::string xml_filename) {
  std::vector<visensor::ViCameraCalibration> output_vector;
  boost::property_tree::ptree calibration_tree;

  if (!loadPropertyTree(xml_filename, calibration_tree) ) {
    exit(1);
  }

  BOOST_FOREACH(const boost::property_tree::ptree::value_type & iter, calibration_tree.get_child("") ) {
    visensor::ViCameraCalibration calibration(visensor::ViCameraLensModel::LensModelTypes::RADIAL,
                                              visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
    std::vector<std::string> elements;
    boost::split(elements, iter.first, boost::is_any_of("_"));

    if( (elements.size() == 3) && (elements[0] == "cam") ) {
      try
      {
        calibration.cam_id_ = std::stoi(elements[1]);
        calibration.slot_id_ = std::stoi(elements[2])/2;
        calibration.is_flipped_ = std::stoi(elements[2])%2;
        calibration.resolution_ = {752, 480};
        //build childtree name
        std::string cam_id_str = boost::lexical_cast<std::string>(calibration.cam_id_);
        std::string slot_str = boost::lexical_cast<std::string>(std::stoi(elements[2]));
        std::string child_tree = std::string("cam_") + cam_id_str + std::string("_") + slot_str + std::string(".");

        //load data
        visensor::ViCameraLensModelRadial::Ptr lens_model = calibration.getLensModel<visensor::ViCameraLensModelRadial>();
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

      } catch(std::exception const&  ex)
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

bool SensorUpdater::convertCalibration() {
  visensor::ViSensorConfiguration::Ptr config_server = boost::make_shared<visensor::ViSensorConfiguration>(pFile_transfer_);
  std::cout << "Convert calibration to new format ... ";

  std::string tmp_calibration_filename("/tmp/calibration.xml");
  if (!loadXmlCameraCalibrationFile(tmp_calibration_filename)) {
    std::cout <<  "no calibration file was found, assume that the sensor is not yet calibrate" << std::endl;
    return true;
  }

  // try to load existing configuration already saved in the new format
  try {
    config_server->loadConfig();
  }
  catch (visensor::exceptions const &ex) {
    std::cout <<  "ignore" << std::endl
        << "no new configurations were found, assume that the sensor has no" << std::endl;
    std::cout <<  "Exception was: " << ex.what() << std::endl;
  }

  std::vector<visensor::ViCameraCalibration> calibration_list = parseXmlCameraCalibration(tmp_calibration_filename);
  if (calibration_list.size() == 0) {
    std::cout << "failed\n";
    std::cout <<  "no calibration were found" << std::endl;
    exit(1);
  }
  try {
    for (std::vector<visensor::ViCameraCalibration>::iterator it = calibration_list.begin();  it != calibration_list.end(); ++it) {
      config_server->cleanCameraCalibration(static_cast<SensorId::SensorId>(it->cam_id_), it->slot_id_, it->is_flipped_,
                                          visensor::ViCameraLensModel::LensModelTypes::UNKNOWN,
                                          visensor::ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);

      config_server->setCameraCalibration(*it);
    }
    config_server->saveConfig();
  }
  catch (visensor::exceptions const &ex) {
    std::cout <<  "failed" << std::endl
        << "Setting of the new calibration failed!" << std::endl;
    std::cout <<  "Exception was: " << ex.what() << std::endl;
    exit(1);
  }
  std::cout << "done." << std::endl;
  return true;
}

bool SensorUpdater::checkCalibrationConvertion(VersionList old_list, VersionList new_list) {

  VersionEntry linux_embedded_entry;
  VersionEntry version_of_cali_change;
  version_of_cali_change.version_major = 2;
  version_of_cali_change.version_minor = 0;
  version_of_cali_change.version_patch = 0;
  //check if the calibration need to be converted
  if (old_list.size() == 0) {
    std::cout << "Try to copy possible available calibration ... ";

    if (("/tmp/calibration.xml")) {
      std::cout << "done." << std::endl;
      if (!convertCalibration()) {
        std::cerr << "Could not convert calibration to new format" << std::endl;
        return false;
      }
    }
    else {
      std::cout << "No calibration file found, nothing to convert" << std::endl;
    }
  }
  else {
    size_t i;
    for (i = 0; i < old_list.size(); i++) {
      if (old_list[i].package_name == "visensor-linux-embedded" ) {
        break;
      }
    }
    if ((i >= old_list.size()) && (old_list.size() != new_list.size())) {
      std::cerr << "could not found linux embedded version" << std::endl;
      return false;
    }
    if ( (old_list[i] < version_of_cali_change) && ( (new_list[i] > version_of_cali_change) || (new_list[i] == version_of_cali_change))) {
      if (!convertCalibration()) {
        std::cerr << "could not convert calibration to new format" << std::endl;
        return false;
      }
    }
  }
  return true;

}
/* remove all old packages and install the newest version of all packages in the repo which are mandatory */
bool SensorUpdater::sensorUpdate(const UpdateConfig::REPOS &repo)
{
  VersionList list;
  VersionList currentList;
  std::string localPath = std::string("/tmp/");

  bool success = false;

  if(getUpdateList(list, repo))
  {
    if(!getVersionInstalled(currentList,  UpdateConfig::prefix, false))
      return false;

    if(downloadPackagesToPath(list, localPath))
    {
      if(sensorClean())
      {
        if (checkCalibrationConvertion(currentList, list)) {
          // update to newest version
          if(installPackagesFromPath(list, localPath)) {
            success = true;
          }
        }
      }
    }
  }

  return success;
}


