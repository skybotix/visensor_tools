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

#ifndef SENSORUPDATER_HPP_
#define SENSORUPDATER_HPP_
#include <vector>
#include <tuple>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "update_config.hpp"

#include "communication_layers/SshConnection.hpp"
#include "communication_layers/WebClient.hpp"


//#include <visensor/visensor.hpp>
#include <visensor_impl.hpp>



class SensorUpdater {
 public:

  /* some typedefs */
  struct VersionEntry {
    std::string package_name;
    unsigned int version_major;
    unsigned int version_minor;
    unsigned int version_patch;
    std::string path;

    /* lexicographical version comparison */
    bool operator<(const VersionEntry& rhs) const
    {
        // compares n to rhs.n,
        // then s to rhs.s,
        // then d to rhs.d

        return std::tie(version_major, version_minor, version_patch) < std::tie(rhs.version_major, rhs.version_minor, rhs.version_patch);
    }

    bool operator==(const VersionEntry& rhs) const
    {
        return (version_major == rhs.version_major) && (version_minor == rhs.version_minor) && (version_patch == rhs.version_patch);
    }

    bool operator>(const VersionEntry& rhs) const
    {
      return !(*this<rhs) && !(*this==rhs);
    }

  };

  typedef std::vector<VersionEntry> VersionList;


  SensorUpdater(const std::string &hostname);
  virtual ~SensorUpdater();

  /* repo functions */
  bool getVersionInstalled(VersionList &outPackageList, const std::string &prefix, bool dontParseVersion=false);
  bool getVersionsOnServer(SensorUpdater::VersionList &outPackageList, UpdateConfig::REPOS repo);
  bool printVersionsInstalled(void);
  bool printVersionsRepo(UpdateConfig::REPOS repo);

  bool getUpdateList(SensorUpdater::VersionList &outList, const UpdateConfig::REPOS &repo);


  /* package functions */
  bool downloadPackagesToPath(SensorUpdater::VersionList &packageList, const std::string &localPath);
  bool installPackagesFromPath(SensorUpdater::VersionList &packageList, const std::string &localPath);

  /* calibration functions */
  bool convertCalibration();
  std::vector<visensor::ViCameraCalibration>  loadXmlCameraCalibration();
  bool loadPropertyTree(std::string calibration_filename, boost::property_tree::ptree& tree);

  /* sensor functions */
  bool sensorInstallDebMemory(const std::string &debian_package);
  bool sensorInstallDebFile(const std::string &file);
  bool sensorRemoveDeb(const std::string &package_name);

  bool sensorClean(void);
  bool sensorReboot(void) const;

  bool sensorSetMountRW(bool RW);


  /* high level update functions */
  bool sensorUpdate(const UpdateConfig::REPOS &repo);


 private:
  SshConnection *pSsh_; //ssh connection to sensor

};

#endif /* SENSORUPDATER_HPP_ */
