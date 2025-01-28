#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/ImuBias.h>
#include <memory>

namespace gtsam {

std::unique_ptr<Values> default_values();

void clear_values(Values &values);

const Pose3 &values_at_pose3(const Values &values, Key key);

const Vector3 &values_at_vector3(const Values &values, Key key);

const imuBias::ConstantBias &values_at_constant_bias(const Values &values, Key key);

bool values_exists(const Values &values, Key key);

void values_insert_pose3(Values &values, Key key, const Pose3 &value);

void values_insert_constant_bias(Values &values, Key key, const imuBias::ConstantBias &value);

void values_insert_vector3(Values &values, Key key, const Vector3 &value);

} // namespace gtsam
