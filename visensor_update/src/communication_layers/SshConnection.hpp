/*
 * SensorUpdater.hpp
 *
 *  Created on: Dec 1, 2013
 *      Author: skybotix
 */

#ifndef SSHCONNECTION_HPP_
#define SSHCONNECTION_HPP_

#include <libssh2.h>
#include <string>

#include <boost/smart_ptr.hpp>

class SshConnection {


 public:
  typedef boost::shared_ptr<SshConnection> Ptr;
  SshConnection(const std::string &hostname, const std::string &username, const std::string &password, const unsigned int port=22);
  virtual ~SshConnection();


  /* protocol-lvl functionality */
  bool runCommand(const std::string &command, std::string &output, int &exitcode) const;
  bool sendFile(const std::string &srcpath, std::string &scppath) const;
  bool getFile(const std::string &scppath, std::string &dstpath) const;




 private:
  /* ssh session stuff mgmt */
  bool sshConnect();
  void sshDisconnect();


  LIBSSH2_SESSION *session_ssh_;   //ssh session
  int socket_;                     //network socket

  const std::string hostname_;    //ssh server details
  const std::string username_;
  const std::string password_;
  const unsigned int port_;



};

#endif /* SSHCONNECTION_HPP_ */

