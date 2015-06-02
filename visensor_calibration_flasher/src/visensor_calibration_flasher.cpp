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

#include <visensor_impl.hpp>

void printSensorConfig(const visensor::ViCameraCalibration& config){
  visensor::ViCameraLensModelRadial::Ptr lens_model = config.getLensModel<visensor::ViCameraLensModelRadial>();
  visensor::ViCameraProjectionModelPinhole::Ptr projection_model = config.getProjectionModel<visensor::ViCameraProjectionModelPinhole>();
  std::cout << "Focal Length: \n";
  std::cout << projection_model->focal_length_u_ << "\t" << projection_model->focal_length_v_ << std::endl;
  std::cout << "Principcal Point: \n";
  std::cout << projection_model->principal_point_u_ << "\t" << projection_model->principal_point_v_ << std::endl;

  std::cout << "Projection Coefficient: \n";
  std::vector<double> coefficients = lens_model->getCoefficients();
  for (int i = 0; i< 5; ++i){
    std::cout << coefficients[i] << " ";
  }
  std::cout << "\nR: \n";
  for (int i = 0; i< 3; ++i){
    std::cout << config.R_.at(i) << "\t" << config.R_.at(i + 3) << "\t" << config.R_.at(i + 6) << "\n";
  }
  std::cout << "T: \n";
  for (int i = 0; i< 3; ++i){
    std::cout << config.t_[i] << "\t";
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

  visensor::ViSensorDriver::Impl* privat_drv = drv.getPrivateApiAccess();
  for (auto camera_id : list_of_camera_ids)
  {
    visensor::ViCameraCalibration camera_calibration;
    std::string camera_model;
    std::string distortion_model;

    camera_calibration.cam_id_ = static_cast<int>(camera_id);


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



    if (cam_params.hasMember("flip_camera")){
      camera_calibration.is_flipped_ = cam_params["flip_camera"];
    }
    else{
      camera_calibration.is_flipped_ = true;
    }
    std::cout << "flip_camera: " << camera_calibration.is_flipped_ << std::endl;
    T_C_I = cam_params["T_cam_imu"];
    //EIGEN USES COLUMN MAJOR ORDER!
    camera_calibration.R_.resize(9);
    camera_calibration.R_.at(0) = (double) T_C_I[0][0];
    camera_calibration.R_.at(3) = (double) T_C_I[0][1];
    camera_calibration.R_.at(6) = (double) T_C_I[0][2];
    camera_calibration.R_.at(1) = (double) T_C_I[1][0];
    camera_calibration.R_.at(4) = (double) T_C_I[1][1];
    camera_calibration.R_.at(7) = (double) T_C_I[1][2];
    camera_calibration.R_.at(2) = (double) T_C_I[2][0];
    camera_calibration.R_.at(5) = (double) T_C_I[2][1];
    camera_calibration.R_.at(8) = (double) T_C_I[2][2];

    camera_calibration.t_.resize(3);
    camera_calibration.t_.at(0) = (double) T_C_I[0][3];
    camera_calibration.t_.at(1) = (double) T_C_I[1][3];
    camera_calibration.t_.at(2) = (double) T_C_I[2][3];



    distortion_model.assign(cam_params["distortion_model"]);
    XmlRpc::XmlRpcValue distortion_coeffs = cam_params["distortion_coeffs"];
    std::cout << "distortion_model is: " << distortion_model << std::endl;
    if (distortion_model == std::string("radtan"))  {
      visensor::ViCameraLensModelRadial::Ptr lens_model = camera_calibration.getLensModel<visensor::ViCameraLensModelRadial>();
      std::cout << "lens_model radial and params are: " << distortion_coeffs << std::endl;
      lens_model->setType();
      lens_model->k1_ = (double) distortion_coeffs[0];
      lens_model->k2_ = (double) distortion_coeffs[1];
      lens_model->r1_ = (double) distortion_coeffs[2];
      lens_model->r2_ = (double) distortion_coeffs[3];
    }
    else if (distortion_model == std::string("equidistant")) {
      visensor::ViCameraLensModelEquidistant::Ptr lens_model = camera_calibration.getLensModel<visensor::ViCameraLensModelEquidistant>();
      lens_model->setType();
      lens_model->k1_ = (double) distortion_coeffs[0];
      lens_model->k2_ = (double) distortion_coeffs[1];
      lens_model->k3_ = (double) distortion_coeffs[2];
      lens_model->k4_ = (double) distortion_coeffs[3];
    }
    else {
      std::cout << "Distortion Model is not supported. Abort, abort!\n";
      return 1;
    }

    std::cout << "lens model is: " << static_cast<int>(camera_calibration.lens_model_->type_) << std::endl;

    camera_model.assign(cam_params["camera_model"]);
    XmlRpc::XmlRpcValue intrinsics = cam_params["intrinsics"];
    std::cout << "projection_model is: " << camera_model << std::endl;
    if (camera_model == std::string("pinhole")) {
      visensor::ViCameraProjectionModelPinhole::Ptr projection_model = camera_calibration.getProjectionModel<visensor::ViCameraProjectionModelPinhole>();
      projection_model->setType();
      projection_model->focal_length_u_ = (double) intrinsics[0];
      projection_model->focal_length_v_ = (double) intrinsics[1];
      projection_model->principal_point_u_ = (double) intrinsics[2];
      projection_model->principal_point_v_ = (double) intrinsics[3];
    }
    else if (camera_model == std::string("omnidirectional")){
      visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model = camera_calibration.getProjectionModel<visensor::ViCameraProjectionModelOmnidirectional>();
      projection_model->setType();
      projection_model->focal_length_u_ = (double) intrinsics[0];
      projection_model->focal_length_v_ = (double) intrinsics[1];
      projection_model->principal_point_u_ = (double) intrinsics[2];
      projection_model->principal_point_v_ = (double) intrinsics[3];
      // todo(lschmid) how to check if intrinsics has 5 attributes?
      projection_model->mirror_xi_ = (double) intrinsics[4];
    }
    else {
      std::cout << "Current Camera Model is supported. Abort, abort!\n";
      return 1;
    }

    XmlRpc::XmlRpcValue resolution = cam_params["resolution"];
    std::cout << "resolution is: " << resolution << std::endl;
    camera_calibration.resolution_[0] = (int) resolution[0];
    camera_calibration.resolution_[1] = (int) resolution[1];


    camera_calibration.slot_id_ = 0;


    // delete every factroy calibration of the corresponding cam
    privat_drv->cleanCameraCalibrations(camera_id, 0, -1,
                                        visensor::ViCameraLensModel::LensModelTypes::UNKNOWN,
                                        visensor::ViCameraProjectionModel::ProjectionModelTypes::UNKNOWN);
    if(privat_drv->setCameraFactoryCalibration(camera_calibration) == false) {
      std::cout << "Calibration upload failed!\n";
      return 1;
    }
  }
  std::cout << "Calibration upload succeeded!\n"; 
  return 0;
}

