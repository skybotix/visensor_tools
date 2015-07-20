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
      REPO_DEV,
      REPO_16488_RELEASE,
      REPO_16488_DEV
    };

  const std::string REPOS_PATH[] =
    {
        "release",
        "develop",
        "release/adis16488",
        "develop/adis16488"
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


