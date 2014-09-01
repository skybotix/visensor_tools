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
#include <ros/ros.h>
#include <ros/package.h>

#include <visensor/visensor.hpp>

void printSensorConfig(const visensor::ViCameraCalibration& config){

  std::cout << "Focal Length: \n";
  std::cout << config.focal_point[0] << "\t" << config.focal_point[1] << std::endl;
  std::cout << "Principcal Point: \n";
  std::cout << config.principal_point[0] << "\t" << config.principal_point[1] << std::endl;
  std::cout << "Principcal Coefficient: \n";
  for (int i = 0; i< 5; ++i){
    std::cout << config.dist_coeff[i] << " ";
  }
  std::cout << "\nR: \n";
  for (int i = 0; i< 3; ++i){
    std::cout << config.R[i] << "\t" << config.R[i + 3] << "\t" << config.R[i + 6] << "\n";
  }
  std::cout << "T: \n";
  for (int i = 0; i< 3; ++i){
    std::cout << config.t[i] << "\t";
  }
  std::cout << std::endl;
}

static const std::map<int, std::string> ROS_CAMERA_NAMES {
  { visensor::SensorId::SensorId::CAM0, "/cam0" },
  { visensor::SensorId::SensorId::CAM1, "/cam1" },
  { visensor::SensorId::SensorId::CAM2, "/cam2" },
  { visensor::SensorId::SensorId::CAM3, "/cam3" },
  { visensor::SensorId::SensorId::FLIR0, "/tau0" },
  { visensor::SensorId::SensorId::FLIR1, "/tau1" },
  { visensor::SensorId::SensorId::FLIR2, "/tau2" },
  { visensor::SensorId::SensorId::FLIR3, "/tau3" } };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vi_calibration_flasher");
  ros::NodeHandle nh("~");

  visensor::ViSensorDriver drv;

  try {
    drv.init();
  } catch (visensor::exceptions const &ex) {
    std::cout << ex.what() << "\n";
    exit(1);
  }

  std::vector<visensor::SensorId::SensorId> list_of_camera_ids = drv.getListOfCameraIDs();
  
  std::vector<visensor::ViCameraCalibration> camCalibration;
  camCalibration.resize(list_of_camera_ids.size());

  char answer;
  std::cout << "Overwrite factory calibration? [y/N]" << std::endl;
  std::cin >> answer;
  std::cout << "\n";

  if(answer=='y' || answer=='Y') {
    std::cout << "Going to overwrite factory calibration." << std::endl;
  } else {
    std::cout << "Calibration upload canceled." << std::endl;
    return 0;
  }

  for (auto camera_id : list_of_camera_ids)
  {
    visensor::ViCameraCalibration camera_calibration;

    std::cout << "Reading out " << ROS_CAMERA_NAMES.at(static_cast<int>(camera_id)) << std::endl;
    XmlRpc::XmlRpcValue cam_params;
    nh.getParam(ROS_CAMERA_NAMES.at(static_cast<int>(camera_id)), cam_params);

    assert(cam_params.hasMember("distortion_coeffs"));
    assert(cam_params.hasMember("intrinsics"));
    assert(cam_params.hasMember("resolution"));
    assert(cam_params.hasMember("camera_model"));
    assert(cam_params.hasMember("distortion_model"));
    assert(cam_params.hasMember("T_cam_imu"));
    XmlRpc::XmlRpcValue T_C_I;

    T_C_I = cam_params["T_cam_imu"];
    //EIGEN USES COLUMN MAJOR ORDER!
    camera_calibration.R[0] = (double) T_C_I[0][0];
    camera_calibration.R[3] = (double) T_C_I[0][1];
    camera_calibration.R[6] = (double) T_C_I[0][2];
    camera_calibration.R[1] = (double) T_C_I[1][0];
    camera_calibration.R[4] = (double) T_C_I[1][1];
    camera_calibration.R[7] = (double) T_C_I[1][2];
    camera_calibration.R[2] = (double) T_C_I[2][0];
    camera_calibration.R[5] = (double) T_C_I[2][1];
    camera_calibration.R[8] = (double) T_C_I[2][2];

    camera_calibration.t[0] = (double) T_C_I[0][3];
    camera_calibration.t[1] = (double) T_C_I[1][3];
    camera_calibration.t[2] = (double) T_C_I[2][3];



    XmlRpc::XmlRpcValue distortion_coeffs = cam_params["distortion_coeffs"];
    camera_calibration.dist_coeff[0] = (double) distortion_coeffs[0];
    camera_calibration.dist_coeff[1] = (double) distortion_coeffs[1];
    camera_calibration.dist_coeff[2] = (double) distortion_coeffs[2];
    camera_calibration.dist_coeff[3] = (double) distortion_coeffs[3];
    camera_calibration.dist_coeff[4] = (double) 0.0;
    XmlRpc::XmlRpcValue intrinsics = cam_params["intrinsics"];

    camera_calibration.focal_point[0] = (double) intrinsics[0];
    camera_calibration.focal_point[1] = (double) intrinsics[1];
    camera_calibration.principal_point[0] = (double) intrinsics[2];
    camera_calibration.principal_point[1] = (double) intrinsics[3];

    XmlRpc::XmlRpcValue resolution = cam_params["resolution"];
    int image_width = resolution[0];
    int image_height = resolution[1];

    std::string camera_model;
    std::string distortion_model;

    camera_model.assign(cam_params["camera_model"]);
    distortion_model.assign(cam_params["distortion_model"]);

    if (camera_model != std::string("pinhole")) {
      std::cout << "Camera Model is not pinhole model. Abort, abort!\n";
      return 1;
    }
    if (distortion_model != std::string("radtan")) {
      std::cout << "Distortion Model is not radtan model. Abort, abort!\n";
      return 1;
    }
  
    if(drv.setCameraFactoryCalibration(camera_id, camera_calibration) == false) {
      std::cout << "Calibration upload failed!\n";
      return 1;
    }
  }
  std::cout << "Calibration upload succeeded!\n"; 
  return 0;
}

