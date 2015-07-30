/*
 * calibration_flasher.hpp
 *
 *  Created on: Jun 29, 2015
 *      Author: lukas
 */

#ifndef SRC_VISENSOR_CALIBRATION_FLASHER_HPP_
#define SRC_VISENSOR_CALIBRATION_FLASHER_HPP_

#include "visensor_impl.hpp"

class CalibrationFlasher
{
 public:
  CalibrationFlasher();
  virtual ~CalibrationFlasher()
  {
  }

  void printAllCameraCalibration(void);
  void printCameraCalibration(const visensor::ViCameraCalibration& config);

  bool deleteCalibration(
      const visensor::SensorId::SensorId cam_id, const int slot_id, const int is_flipped,
      const visensor::ViCameraProjectionModel::ProjectionModelTypes projection_model_type,
      const visensor::ViCameraLensModel::LensModelTypes lens_model_type);
  bool parseCameraCalibration(const YAML::Node& cam_params,
                              visensor::ViCameraCalibration* camera_calibration);
  bool addCalibration(const YAML::Node& cam_params, const int slot_id);

 private:
  visensor::ViSensorDriver drv_;
  visensor::ViSensorDriver::Impl* private_drv_;
};

static const std::map<int, std::string> ROS_CAMERA_NAMES {
  { visensor::SensorId::SensorId::CAM0, "/cam0" },
  { visensor::SensorId::SensorId::CAM1, "/cam1" },
  { visensor::SensorId::SensorId::CAM2, "/cam2" },
  { visensor::SensorId::SensorId::CAM3, "/cam3" },
  { visensor::SensorId::SensorId::FLIR0, "/tau0" },
  { visensor::SensorId::SensorId::FLIR1, "/tau1" },
  { visensor::SensorId::SensorId::FLIR2, "/tau2" },
  { visensor::SensorId::SensorId::FLIR3, "/tau3" } };

#endif /* SRC_VISENSOR_CALIBRATION_FLASHER_HPP_ */
