#pragma once

#include <model/hardwareData.hpp>
#include <model/structure.hpp>
#include <model/systemDiagnostics.hpp>

/**
 * @brief The whole model of the system:
 * holds all the data necessary
 */
struct SystemData {
  NonUnitaryFailureDetection updatable_timestamps_;
  NonUnitaryFailureDetection updated_timestamps_;

  R2DLogics r2d_logics_;
  FailureDetection failure_detection_{updated_timestamps_};

  HardwareData hardware_data_;
  Mission mission_{Mission::MANUAL};

  bool mission_finished_{false};
};

