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

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <map>


SensorUpdater::SensorUpdater(const std::string &hostname) :
  pSsh_( new SshConnection(hostname,
                           UpdateConfig::ssh_username,
                           UpdateConfig::ssh_password) )
{
}


SensorUpdater::~SensorUpdater()
{
}

//function returns a vector of pairs with (package_name, version)
//for all packages that start with the given packagename prefix
bool SensorUpdater::getVersionInstalled(SensorUpdater::VersionList &outPackageList, const std::string &prefix, bool dontParseVersion)
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

       SensorUpdater::VersionEntry package;
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



bool SensorUpdater::getVersionsOnServer(SensorUpdater::VersionList &outPackageList, UpdateConfig::REPOS repo)
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

      SensorUpdater::VersionEntry package;
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
  SensorUpdater::VersionList listSensor;
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
  SensorUpdater::VersionList listFtp;
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
  SensorUpdater::VersionList listSensor;

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
bool SensorUpdater::getUpdateList(SensorUpdater::VersionList &outList, const UpdateConfig::REPOS &repo)
{
  /* get the newest version from the repos */
  SensorUpdater::VersionList allPackages;
  bool success = getVersionsOnServer(allPackages, repo);


  //extract the newst version of all mandatory packages
  SensorUpdater::VersionList updatePackages;

  for(size_t i=0; i<UpdateConfig::repo_mandatory_pkgs.size(); i++)
  {
    /* extract all packages which are mandatory to install */
    SensorUpdater::VersionList temp;

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
bool SensorUpdater::downloadPackagesToPath(SensorUpdater::VersionList &packageList, const std::string &localPath)
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
bool SensorUpdater::installPackagesFromPath(SensorUpdater::VersionList &packageList, const std::string &localPath)
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

/* remove all old packages and install the newest version of all packages in the repo which are mandatory */
bool SensorUpdater::sensorUpdate(const UpdateConfig::REPOS &repo)
{
  SensorUpdater::VersionList list;
  std::string localPath = std::string("/tmp/");
  bool success = false;

  if(getUpdateList(list, repo))
  {
    if(downloadPackagesToPath(list, localPath))
    {
      if(sensorClean())
      {
        if(installPackagesFromPath(list, localPath))
          success = true;
      }
    }
  }

  return success;
}


