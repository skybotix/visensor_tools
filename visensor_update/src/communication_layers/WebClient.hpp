/*
 * SensorUpdater.hpp
 *
 *  Created on: Dec 1, 2013
 *      Author: skybotix
 */

#ifndef FTPCLIENT_HPP_
#define FTPCLIENT_HPP_

#include <curl/curl.h>
#include <string>


class WebClient {


 public:
  WebClient(const std::string &hostname, bool verbose=false);
  virtual ~WebClient();

  /* protocol-lvl functionality */
  bool dirList(const std::string &ftppath, std::string &outputBuffer);
  bool getFileToMemory(const std::string &ftppath, std::string &outputBuffer);
  bool getFileToFile(const std::string &ftppath, const std::string &localpath);



 private:
  CURL *curl_;                  //curl instance

  const std::string hostname_;    //server data
  const bool verbose_;


};

#endif /* FTPCLIENT_HPP_ */

