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
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "visensor_calibration_flasher.hpp"

CalibrationFlasher::CalibrationFlasher()
{
  private_drv_ = drv_.getPrivateApiAccess();
  try {
    drv_.init();
  } catch (visensor::exceptions const &ex) {
    std::cerr << ex.what() << "\n";
    exit(1);
  }
}

void CalibrationFlasher::printAllCameraCalibration()
{
  //get all camera calibrations
  std::vector<visensor::ViCameraCalibration> calibrations = private_drv_->getCameraCalibrations(
      static_cast<visensor::SensorId::SensorId>(-1));

  for (std::vector<visensor::ViCameraCalibration>::iterator it = calibrations.begin();
      it != calibrations.end(); ++it) {
    printCameraCalibration(*it);
  }
}
void CalibrationFlasher::printCameraCalibration(const visensor::ViCameraCalibration& config)
{
  std::cout << std::endl << "Calibration of camera " << config.cam_id_ << " is:\n";
  std::cout << "Slot ID:\t\t" << config.slot_id_ << std::endl;
  std::cout << std::endl;
  std::cout << "flip_camera:\t\t" << config.is_flipped_ << std::endl;
  std::cout << std::endl;
  std::cout << "Projection Model:\n";
  std::cout << "\tModel:\t\t";
  std::cout << config.projection_model_->type_name_ << std::endl;
  std::cout << "\tCoefficient:\t";
  std::vector<double> coefficients = config.projection_model_->getCoefficients();
  for (unsigned int i = 0; i < coefficients.size(); ++i) {
    std::cout << coefficients[i] << " ";
  }
  std::cout << std::endl << std::endl;
  std::cout << "Lens Model:\n";
  std::cout << "\tModel:\t\t";
  std::cout << config.lens_model_->type_name_ << std::endl;
  std::cout << "\tCoefficient:\t";
  coefficients = config.lens_model_->getCoefficients();
  for (unsigned int i = 0; i < coefficients.size(); ++i) {
    std::cout << coefficients[i] << " ";
  }
  std::cout << std::endl << std::endl;
  std::cout << "R:";
  for (unsigned int i = 0; i < config.R_.size() / 3; ++i) {
    std::cout << "\t\t\t" << config.R_[i] << "\t" << config.R_[i + 3] << "\t" << config.R_[i + 6]
              << "\n";
  }
  std::cout << "T:\t\t\t";
  for (unsigned int i = 0; i < config.t_.size(); ++i) {
    std::cout << config.t_[i] << "\t";
  }
  std::cout << std::endl << std::endl;
  std::cout << "Resolution is:\t\t" << config.resolution_[0] << ", " << config.resolution_[1]
            << std::endl;
}

bool CalibrationFlasher::addCalibration(const YAML::Node& config, const int slot_id)
{

  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    if (!it->first.as<std::string>().compare(0, 3, "cam")) {
      int cam_id = std::stoi(it->first.as<std::string>().substr(3, std::string::npos));
      std::cout << "found camera calibration for cam " << cam_id << std::endl;
      visensor::ViCameraCalibration camera_calibration;
      if (!parseCameraCalibration(it->second, &camera_calibration))
        return false;

      camera_calibration.cam_id_ = cam_id;
      camera_calibration.slot_id_ = slot_id;

      try {
        printCameraCalibration(camera_calibration);
        if (camera_calibration.slot_id_ == 0) {
          std::cout << "Your going to write the factory calibration for the camera " << slot_id
                    << ". Do you want proceed? [y/n]: " << std::endl;
          std::string input;
          while (!(std::cin >> input)
              && (input != "y" || input != "Y" || input != "n" || input != "N")) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
            std::cout << "Invalid input; Please enter [y/Y/n/N].\n";
          }
          if (input != "y" && input != "Y") {
            return false;
          }
          private_drv_->setCameraFactoryCalibration(camera_calibration);
        } else {
          drv_.setCameraCalibration(camera_calibration);
        }
      } catch (visensor::exceptions const &ex) {
        std::cerr << "Calibration upload failed! Exception was:\n" << ex.what();
        return false;
      }
    }
  }
  return true;
}
bool CalibrationFlasher::parseCameraCalibration(const YAML::Node& cam_params,
                                                visensor::ViCameraCalibration* camera_calibration)
{
  YAML::Node resolution;
  YAML::Node T_C_I;
  YAML::Node intrinsics;
  YAML::Node distortion_coeffs;
  std::string camera_model;
  std::string distortion_model;

  assert(cam_params["distortion_coeffs"]);
  assert(cam_params["intrinsics"]);
  assert(cam_params["resolution"]);
  assert(cam_params["camera_model"]);
  assert(cam_params["distortion_model"]);
  assert(cam_params["T_cam_imu"]);

  if (cam_params["flip_camera"]) {
    camera_calibration->is_flipped_ = cam_params["flip_camera"].as<bool>();
  } else {
    camera_calibration->is_flipped_ = true;
  }
  T_C_I = cam_params["T_cam_imu"];
  //EIGEN USES COLUMN MAJOR ORDER!
  camera_calibration->R_.resize(9);
  camera_calibration->R_.at(0) = T_C_I[0][0].as<double>();
  camera_calibration->R_.at(3) = T_C_I[0][1].as<double>();
  camera_calibration->R_.at(6) = T_C_I[0][2].as<double>();
  camera_calibration->R_.at(1) = T_C_I[1][0].as<double>();
  camera_calibration->R_.at(4) = T_C_I[1][1].as<double>();
  camera_calibration->R_.at(7) = T_C_I[1][2].as<double>();
  camera_calibration->R_.at(2) = T_C_I[2][0].as<double>();
  camera_calibration->R_.at(5) = T_C_I[2][1].as<double>();
  camera_calibration->R_.at(8) = T_C_I[2][2].as<double>();

  camera_calibration->t_.resize(3);
  camera_calibration->t_.at(0) = T_C_I[0][3].as<double>();
  camera_calibration->t_.at(1) = T_C_I[1][3].as<double>();
  camera_calibration->t_.at(2) = T_C_I[2][3].as<double>();

  distortion_model = cam_params["distortion_model"].as<std::string>();
  distortion_coeffs = cam_params["distortion_coeffs"];
  if (distortion_model == std::string("radtan")) {
    camera_calibration->lens_model_ = std::make_shared<visensor::ViCameraLensModelRadtan>();
    visensor::ViCameraLensModelRadtan::Ptr lens_model =
        camera_calibration->getLensModel<visensor::ViCameraLensModelRadtan>();

    if (static_cast<unsigned int>(distortion_coeffs.size())
        < lens_model->getCoefficients().size()) {
      std::cerr << "To few coefficients are given for the radtan projection model. Abort, abort!\n";
      return false;
    }
    lens_model->k1_ = distortion_coeffs[0].as<double>();
    lens_model->k2_ = distortion_coeffs[1].as<double>();
    lens_model->r1_ = distortion_coeffs[2].as<double>();
    lens_model->r2_ = distortion_coeffs[3].as<double>();
  } else if (distortion_model == std::string("equidistant")) {
    camera_calibration->lens_model_ = std::make_shared<visensor::ViCameraLensModelEquidistant>();
    visensor::ViCameraLensModelEquidistant::Ptr lens_model =
        camera_calibration->getLensModel<visensor::ViCameraLensModelEquidistant>();

    if (static_cast<unsigned int>(distortion_coeffs.size())
        < lens_model->getCoefficients().size()) {
      std::cerr
          << "To few coefficients are given for the equidistant projection model. Abort, abort!\n";
      return false;
    }
    lens_model->k1_ = distortion_coeffs[0].as<double>();
    lens_model->k2_ = distortion_coeffs[1].as<double>();
    lens_model->k3_ = distortion_coeffs[2].as<double>();
    lens_model->k4_ = distortion_coeffs[3].as<double>();
  } else {
    std::cerr << "Distortion Model is not supported. Abort, abort!\n";
    return false;
  }

  camera_model = cam_params["camera_model"].as<std::string>();
  intrinsics = cam_params["intrinsics"];
  if (camera_model == std::string("pinhole")) {
      camera_calibration->projection_model_ = std::make_shared<visensor::ViCameraProjectionModelPinhole>();
      visensor::ViCameraProjectionModelPinhole::Ptr projection_model =
          camera_calibration->getProjectionModel<visensor::ViCameraProjectionModelPinhole>();

    if (static_cast<unsigned int>(intrinsics.size()) < projection_model->getCoefficients().size()) {
      std::cerr
          << "To few coefficients are given for the pinhole projection model. Abort, abort!\n";
      return false;
    }
    projection_model->focal_length_u_ = intrinsics[0].as<double>();
    projection_model->focal_length_v_ = intrinsics[1].as<double>();
    projection_model->principal_point_u_ = intrinsics[2].as<double>();
    projection_model->principal_point_v_ = intrinsics[3].as<double>();
  } else if (camera_model == std::string("omnidirectional")) {
    camera_calibration->projection_model_ = std::make_shared<visensor::ViCameraProjectionModelOmnidirectional>();
    visensor::ViCameraProjectionModelOmnidirectional::Ptr projection_model =
        camera_calibration->getProjectionModel<visensor::ViCameraProjectionModelOmnidirectional>();

    if (static_cast<unsigned int>(intrinsics.size()) < projection_model->getCoefficients().size()) {
      std::cerr
          << "To few coefficients are given for the omnidirectional projection model. Abort, abort!\n";
      return false;
    }
    projection_model->focal_length_u_ = intrinsics[0].as<double>();
    projection_model->focal_length_v_ = intrinsics[1].as<double>();
    projection_model->principal_point_u_ = intrinsics[2].as<double>();
    projection_model->principal_point_v_ = intrinsics[3].as<double>();
    projection_model->mirror_xi_ = intrinsics[4].as<double>();
  } else {
    std::cerr << "Current Camera Model is not supported. Abort, abort!\n";
    return false;
  }
  resolution = cam_params["resolution"];
  camera_calibration->resolution_[0] = resolution[0].as<int>();
  camera_calibration->resolution_[1] = resolution[1].as<int>();
  return true;
}

bool CalibrationFlasher::deleteCalibration(
    const visensor::SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
    const visensor::ViCameraProjectionModel::ProjectionModelTypes projection_model_type,
    const visensor::ViCameraLensModel::LensModelTypes lens_model_type)
{
  if (slot_id <= 0) {
    std::cout << "\nYour going to delete the factory calibration. Do you want proceed? [y/n]: "
        << std::endl;

    std::string input;
    while (!(std::cin >> input) && (input != "y" || input != "Y" || input != "n" || input != "N")) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
      std::cout << "Invalid input; Please enter [y/Y/n/N].\n";
    }
    if (input == "y" || input == "Y") {
      if (!private_drv_->cleanCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type,
                                                 projection_model_type)) {
        return false;
      }
    } else
      return true;
  } else {
    if (!drv_.cleanCameraCalibrations(cam_id, slot_id, is_flipped, lens_model_type,
                                      projection_model_type)) {
      return false;
    }
  }
  return true;
}
