#pragma once

#include <model/hardwareData.hpp>
#include <model/structure.hpp>
#include <model/systemDiagnostics.hpp>

/**
 * @brief The whole model of the system:
 * holds all the data necessary
 */
struct SystemData {
  R2DLogics r2d_logics_;
  FailureDetection failure_detection_;

  HardwareData hardware_data_;
  Mission mission_{Mission::MANUAL};

  bool ready_2_drive_{false};
  bool mission_finished_{false};
};
