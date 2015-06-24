
#include "WebClient.hpp"

#include <curl/curl.h>

#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>


static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *outputBuffer)
{
  size_t realsize = size * nmemb;

  for(size_t i=0; i<realsize; i++)
    *(std::string*)outputBuffer += ((char*)contents)[i];

  return realsize;
}



WebClient::WebClient(const std::string &hostname, bool verbose) :
    hostname_(hostname),
    verbose_(verbose)
{
  /* init curl */
  curl_global_init(CURL_GLOBAL_DEFAULT);
  curl_ = curl_easy_init();
}


WebClient::~WebClient()
{
  /* release curl */
  curl_global_cleanup();
}


bool WebClient::getFileToFile(const std::string &ftppath, const std::string &localpath)
{

  /* download from ftp to string object */
  std::string outputBuffer;
  bool success = getFileToMemory(ftppath, outputBuffer);

  /* write string to file (if download was successful */
  if(success)
  {
    std::ofstream dstFile;
    try
    {
      dstFile.open( localpath.c_str() );
      if (!dstFile) {
        perror ("[ERROR]:  Could not open local file! Reason was");
        return false;
      }
      if (!dstFile.is_open()) {
        perror ("[ERROR]:  Could not open local file! Reason was");
        return false;
      }
      dstFile << outputBuffer;
    }
    catch (std::ofstream::failure e)
    {
      std::cout << "[ERROR]: Could not save update package!\n";

      if (dstFile.is_open()) {
        dstFile.close();
      }
      return false;
    }

    /* close file */
    dstFile.close();
  }

  return success;
}


bool WebClient::getFileToMemory(const std::string &path, std::string &outputBuffer)
{
  CURLcode res;

  /* build the download url */
  std::string URL = hostname_+"/"+path;

  /* clear output buffer */
  outputBuffer.clear();

  /* (re-)init curl */
  curl_ = curl_easy_init();

  if(curl_) {

    curl_easy_setopt(curl_, CURLOPT_URL, URL.c_str());

    /* Define our callback to get called when there's data to be written */
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, (void *)&outputBuffer);

    /* Switch on full protocol/debug output */
    if(verbose_)
      curl_easy_setopt(curl_, CURLOPT_VERBOSE, 1L);

    res = curl_easy_perform(curl_);

    /* always cleanup */
    curl_easy_cleanup(curl_);

    if(CURLE_OK != res) {
      /* we failed */
      std::cout << "[ERROR]: Could not access online repository! (check internet connection)\n";
      exit(1);
    }
  }

  return true;
}



bool WebClient::dirList(const std::string &path, std::string &outputBuffer)
{
  CURLcode res;

  /* build the download url */
  std::string URL = hostname_+"/"+path+"/dirlist.php";

  /* clear output buffer */
  outputBuffer.clear();

  /* (re-)init curl */
  curl_ = curl_easy_init();

  if(curl_)
  {
    curl_easy_setopt(curl_, CURLOPT_URL, URL.c_str());

    /* Define our callback to get called when there's data to be written */
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, (void *)&outputBuffer);

    /* Switch on full protocol/debug output */
    if(verbose_)
      curl_easy_setopt(curl_, CURLOPT_VERBOSE, 1L);

    res = curl_easy_perform(curl_);

    /* always cleanup */
    curl_easy_cleanup(curl_);

    if(CURLE_OK != res)
    {
      /* we failed */
      std::cout << "[ERROR]: Could not access online repository! (check internet connection)\n";
      exit(1);
    }
  }

  return true;
}




