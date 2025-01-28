#include "values.h"

namespace gtsam {

std::unique_ptr<Values> default_values() { return std::make_unique<Values>(); }

void clear_values(Values &values) { 
    return values.clear(); 
}

const Pose3 &values_at_pose3(const Values &values, Key key) {
  return values.at(key).cast<Pose3>();
}

const Vector3 &values_at_vector3(const Values &values, Key key) {
    return values.at(key).cast<Vector3>();
}

const imuBias::ConstantBias &values_at_constant_bias(const Values &values, Key key) {
    return values.at(key).cast<imuBias::ConstantBias>();
}

bool values_exists(const Values &values, Key key) { return values.exists(key); }

void values_insert_pose3(Values &values, Key key, const Pose3 &value) {
  return values.insert(key, value);
}

void values_insert_constant_bias(Values &values, Key key, const imuBias::ConstantBias &value)
{
    return values.insert(key, value);
}

void values_insert_vector3(Values &values, Key key, const Vector3 &value) {
    return values.insert(key, value);
}

} // namespace gtsam
