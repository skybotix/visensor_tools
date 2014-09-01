/*
 * SensorUpdater.hpp
 *
 *  Created on: Dec 1, 2013
 *      Author: skybotix
 */


#ifndef UPDATE_CONFIG_HPP_
#define UPDATE_CONFIG_HPP_

#include <string>
#include <vector>

namespace UpdateConfig
{
  /* sensor ssh login configuration */
  const std::string ssh_username = "root";
  const std::string ssh_password = "";

  /* update repo configuration */
  const std::string hostname = "http://skybotix.com/downloads/vi-firmware";

  /*standard prefix for debian package filenames */
  const std::string prefix("visensor");

  /* where are the repos on the ftp server (ftp relative path) */
  enum class REPOS : std::size_t
    {
      REPO_RELEASE,
      REPO_DEV
    };

  const std::string REPOS_PATH[] =
    {
        "release",
        "develop"
    };


  /* which packages do we want to install, if we do a sensor update */
  const std::vector<std::string> repo_mandatory_pkgs =
      {
          std::string("visensor-fpga-bitstream"),
          std::string("visensor-kernel-modules"),
          std::string("visensor-linux-embedded")
      };


}

#endif /* UPDATE_CONFIG_HPP_ */


