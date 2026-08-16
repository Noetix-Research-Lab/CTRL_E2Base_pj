#pragma once

#include "yaml_io.h"

#include <string>
#include <vector>

// Drake YAML schema for /config/pjsol_config.yaml.
// Angle fields (joint_limit.*, drive_map.*.zero_offset) are in deg;
// E2HW converts them to rad before calling pjsol.
// Per drive axis: q_model = direction * (q_motor - zero_offset)
struct PjsolGeomFromYaml {
  std::vector<double> F1;
  std::vector<double> F2;
  std::vector<double> H1;
  std::vector<double> H2;
  double l1{0.0};
  double l2{0.0};
  double r{0.0};
  double e{0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(F1));
    a->Visit(DRAKE_NVP(F2));
    a->Visit(DRAKE_NVP(H1));
    a->Visit(DRAKE_NVP(H2));
    a->Visit(DRAKE_NVP(l1));
    a->Visit(DRAKE_NVP(l2));
    a->Visit(DRAKE_NVP(r));
    a->Visit(DRAKE_NVP(e));
  }
};

struct PjsolJointLimitFromYaml {
  double q1_min{-180.0};
  double q1_max{180.0};
  double q2_min{-180.0};
  double q2_max{180.0};
  double sphere_angle_max{15.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q1_min));
    a->Visit(DRAKE_NVP(q1_max));
    a->Visit(DRAKE_NVP(q2_min));
    a->Visit(DRAKE_NVP(q2_max));
    a->Visit(DRAKE_NVP(sphere_angle_max));
  }
};

// One pjsol active axis <-> software motor channel + calibration.
struct PjsolDriveAxisFromYaml {
  int channel{0};           // joint_data_[index]
  int direction{1};         // +1: 电机正向与解算正向一致; -1: 反向
  double zero_offset{0.0};  // 模型零位对应的电机角度 [deg]

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(channel));
    a->Visit(DRAKE_NVP(direction));
    a->Visit(DRAKE_NVP(zero_offset));
  }
};

struct PjsolDriveMapFromYaml {
  PjsolDriveAxisFromYaml q1;
  PjsolDriveAxisFromYaml q2;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q1));
    a->Visit(DRAKE_NVP(q2));
  }
};

struct PjsolMechanismFromYaml {
  std::string type;
  PjsolGeomFromYaml geom;
  PjsolJointLimitFromYaml joint_limit;
  PjsolDriveMapFromYaml drive_map;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(type));
    a->Visit(DRAKE_NVP(geom));
    a->Visit(DRAKE_NVP(joint_limit));
    a->Visit(DRAKE_NVP(drive_map));
  }
};

struct PjsolOptionsFromYaml {
  PjsolMechanismFromYaml left_ankle;
  PjsolMechanismFromYaml right_ankle;
  PjsolMechanismFromYaml waist;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(left_ankle));
    a->Visit(DRAKE_NVP(right_ankle));
    a->Visit(DRAKE_NVP(waist));
  }
};
