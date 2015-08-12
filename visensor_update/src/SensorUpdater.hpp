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

#ifndef SENSORUPDATER_HPP_
#define SENSORUPDATER_HPP_
#include <vector>
#include <tuple>
#include <map>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// include from libvisensor
#include "communication_layers/ssh_connections.hpp"
#include "communication_layers/WebClient.hpp"
#include "visensor_impl.hpp"

class SensorUpdater {
 public:
  /* where are the repos on the ftp server (ftp relative path) */
  enum class REPOS
    {
      REPO_RELEASE,
      REPO_DEV,
      REPO_16448_RELEASE,
      REPO_16448_DEV,
      REPO_16488_RELEASE,
      REPO_16488_DEV
    };
  enum class SUPPORTED_IMU
    {
      UNKNOWN,
      ADIS_16448,
      ADIS_16488
    };
  enum class SUPPORTED_FPGA_CONFIGS
    {
      UNKNOWN,
      NORMAL,
      P0_8_CAM_SYSTEM,
      FLIR
    };

  /* some typedefs */
  struct VersionEntry {
    std::string package_name;
    unsigned int version_major;
    unsigned int version_minor;
    unsigned int version_patch;
    SensorUpdater::SUPPORTED_FPGA_CONFIGS sensor_type;
    SensorUpdater::SUPPORTED_IMU imu_type;
    std::string path;

    VersionEntry() :
        package_name(""),
        version_major(0),
        version_minor(0),
        version_patch(0),
        sensor_type(SUPPORTED_FPGA_CONFIGS::UNKNOWN),
        imu_type(SUPPORTED_IMU::UNKNOWN),
        path("")
    {
    }
    VersionEntry(unsigned int major, unsigned int minor, unsigned int patch) :
        package_name(""),
        version_major(major),
        version_minor(minor),
        version_patch(patch),
        sensor_type(SUPPORTED_FPGA_CONFIGS::UNKNOWN),
        imu_type(SUPPORTED_IMU::UNKNOWN),
        path("")
    {
    }

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

  typedef bool (SensorUpdater::*parseFunction)(SensorUpdater::VersionEntry* package, const std::string& prefix); // function pointer type
  typedef std::map<std::string, parseFunction> parse_function_map;

  SensorUpdater();
  virtual ~SensorUpdater();

  void connect(const std::string &target_ip);

  /* repo functions */
  bool getVersionInstalled(VersionList* outPackageList);
  bool parseVersionDefault(VersionEntry* package, const std::string &prefix);
  bool parseVersionFpgaBitstream(VersionEntry* package, const std::string &prefix);
  bool getVersionsOnServer(SensorUpdater::VersionList* outPackageList, const REPOS& repo);
  bool getVersionsFromLocalPath(VersionList* outPackageList, std::string path);
  bool printVersionsInstalled(void);
  bool printVersionsRepo(const REPOS& repo);

  bool getUpdateList(SensorUpdater::VersionList* outList, const VersionList &packageVersionList, const REPOS &repo);

  /* package functions */
  bool downloadPackagesToPath(const SensorUpdater::VersionList& packageList, const std::string& localPath);
  bool installPackagesFromPath(const SensorUpdater::VersionList& packageList, const std::string& localPath);

  /* calibration functions */
  bool convertCalibration();
  bool checkConfiguration(visensor::ViSensorConfiguration::Ptr& config_server);
  std::vector<visensor::ViCameraCalibration>  parseXmlCameraCalibration(const std::string& xml_filename);
  bool checkCalibrationConvertion(const VersionList& old_list, const VersionList& new_list);
  bool loadXmlCameraCalibrationFile(const std::string& local_calibration_filename);
  bool loadPropertyTree(const std::string& calibration_filename, boost::property_tree::ptree* tree);

  /* sensor functions */
  bool sensorInstallDebMemory(const std::string& debian_package);
  bool sensorInstallDebFile(const std::string& file);
  bool sensorRemoveDeb(const std::string& package_name);

  bool sensorClean(void);
  bool sensorClean(VersionList listToClean);
  bool sensorReboot(void) const;
  bool sensorSetMountRW(bool RW);

  /* high level update functions */
  bool sensorUpdate(REPOS &repo, const VersionList& requestedVersionList);

  bool sensorDownloadTo(REPOS &repo, const std::string path, const VersionList& requestedVersionList);
  bool sensorUploadFrom(const std::string path);
  bool checkRepo(REPOS &repo);

 private:
  visensor::SshConnection::Ptr pSsh_; //ssh connection to sensor
  visensor::FileTransfer::Ptr pFile_transfer_; //class for the file transfer to the sensor
  bool is_ssh_initialized_; ///< check if updater is connected to the sensor

  /* sensor ssh login configuration */
  const std::string sshUsername() const {
      return "root";
  }
  const std::string servername() const {
      return "http://skybotix.com/downloads/vi-firmware";
  }

  /* update repo configuration */
  const std::string sshPassword() const {
      return "";
  }

  /*standard prefix for debian package filenames */
  const std::string prefix() const {
      return "visensor";
  }
  const std::string remotePath() const {
    return "/tmp/";
  }
};

static const std::map< SensorUpdater::REPOS, std::string> REPOS_PATH = {
  {  SensorUpdater::REPOS::REPO_RELEASE, std::string("release") },
  {  SensorUpdater::REPOS::REPO_DEV, std::string("develop") },
  {  SensorUpdater::REPOS::REPO_16448_RELEASE, std::string("release") },
  {  SensorUpdater::REPOS::REPO_16448_DEV, std::string("develop") },
  {  SensorUpdater::REPOS::REPO_16488_RELEASE, std::string("release/adis16488") },
  {  SensorUpdater::REPOS::REPO_16488_DEV, std::string("develop/adis16488") }
  };

/* which packages do we want to install, if we do a sensor update */
static const SensorUpdater::parse_function_map possible_pkgs_ {
  {"visensor-fpga-bitstream", &SensorUpdater::parseVersionFpgaBitstream},
  {"visensor-kernel-modules", &SensorUpdater::parseVersionDefault},
  {"visensor-linux-embedded", &SensorUpdater::parseVersionDefault}
};

static const std::map<const std::string, const SensorUpdater::SUPPORTED_IMU> supported_imu_ {
  { "A48",  SensorUpdater::SUPPORTED_IMU::ADIS_16448},
  { "A88",  SensorUpdater::SUPPORTED_IMU::ADIS_16488} };

static const std::map<std::string, SensorUpdater::SUPPORTED_FPGA_CONFIGS> supported_fpga_configs_ {
  { "a",  SensorUpdater::SUPPORTED_FPGA_CONFIGS::NORMAL },
  { "c",  SensorUpdater::SUPPORTED_FPGA_CONFIGS::FLIR },
  { "e",  SensorUpdater::SUPPORTED_FPGA_CONFIGS::P0_8_CAM_SYSTEM} };

#endif /* SENSORUPDATER_HPP_ */
