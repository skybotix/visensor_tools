/*
 * SensorUpdater.hpp
 *
 *  Created on: Dec 1, 2013
 *      Author: skybotix
 */

#ifndef SENSORUPDATER_HPP_
#define SENSORUPDATER_HPP_

#include "update_config.hpp"

#include "communication_layers/SshConnection.hpp"
#include "communication_layers/WebClient.hpp"

#include <vector>
#include <tuple>


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
