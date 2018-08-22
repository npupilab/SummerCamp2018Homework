#pragma once
#include <GSLAM/core/Estimator.h>

extern "C"{
SPtr<GSLAM::Estimator> createEstimatorInstance();
}
