#include <GSLAM/core/Glog.h>
#include <GSLAM/core/Random.h>
#include <Eigen/Core>

namespace ransac{

// Abstract base class for sampling methods.
class Sampler {
 public:
  Sampler(){}
  explicit Sampler(const size_t num_samples);

  // Initialize the sampler, before calling the `Sample` method.
  virtual void Initialize(const size_t total_num_samples) = 0;

  // Maximum number of unique samples that can be generated.
  virtual size_t MaxNumSamples() = 0;

  // Sample `num_samples` elements from all samples.
  virtual std::vector<size_t> Sample() = 0;

  // Sample elements from `X` into `X_rand`.
  //
  // Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
  // should equal `num_samples`.
  template <typename X_t>
  void SampleX(const X_t& X, X_t* X_rand);

  // Sample elements from `X` and `Y` into `X_rand` and `Y_rand`.
  //
  // Note that `X.size()` should equal `num_total_samples` and `X_rand.size()`
  // should equal `num_samples`. The same applies for `Y` and `Y_rand`.
  template <typename X_t, typename Y_t>
  void SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand);
};

// Random sampler for RANSAC-based methods.
//
// Note that a separate sampler should be instantiated per thread.
class RandomSampler : public Sampler {
 public:
  explicit RandomSampler(const size_t num_samples);

  void Initialize(const size_t total_num_samples) override;

  size_t MaxNumSamples() override;

  std::vector<size_t> Sample() override;

 private:
  template <typename T>
  void Shuffle(const uint32_t num_to_shuffle, std::vector<T>* elems) {
    CHECK_LE(num_to_shuffle, elems->size());
    const uint32_t last_idx = static_cast<uint32_t>(elems->size() - 1);
    for (uint32_t i = 0; i < num_to_shuffle; ++i) {
      const auto j = GSLAM::Random::RandomValue<uint32_t>(i, last_idx);
      std::swap((*elems)[i], (*elems)[j]);
    }
  }

  const size_t num_samples_;
  std::vector<size_t> sample_idxs_;
};
RandomSampler::RandomSampler(const size_t num_samples)
    : num_samples_(num_samples) {}

void RandomSampler::Initialize(const size_t total_num_samples) {
  CHECK_LE(num_samples_, total_num_samples);
  sample_idxs_.resize(total_num_samples);
  std::iota(sample_idxs_.begin(), sample_idxs_.end(), 0);
}

size_t RandomSampler::MaxNumSamples() {
  return std::numeric_limits<size_t>::max();
}

std::vector<size_t> RandomSampler::Sample() {
  Shuffle(static_cast<uint32_t>(num_samples_), &sample_idxs_);

  std::vector<size_t> sampled_idxs(num_samples_);
  for (size_t i = 0; i < num_samples_; ++i) {
    sampled_idxs[i] = sample_idxs_[i];
  }

  return sampled_idxs;
}

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename X_t>
void Sampler::SampleX(const X_t& X, X_t* X_rand) {
  const auto sample_idxs = Sample();
  for (size_t i = 0; i < X_rand->size(); ++i) {
    (*X_rand)[i] = X[sample_idxs[i]];
  }
}

template <typename X_t, typename Y_t>
void Sampler::SampleXY(const X_t& X, const Y_t& Y, X_t* X_rand, Y_t* Y_rand) {
  CHECK_EQ(X.size(), Y.size());
  CHECK_EQ(X_rand->size(), Y_rand->size());
  const auto sample_idxs = Sample();
  for (size_t i = 0; i < X_rand->size(); ++i) {
    (*X_rand)[i] = X[sample_idxs[i]];
    (*Y_rand)[i] = Y[sample_idxs[i]];
  }
}

// Measure the support of a model by counting the number of inliers and
// summing all inlier residuals. The support is better if it has more inliers
// and a smaller residual sum.
struct InlierSupportMeasurer {
  struct Support {
    // The number of inliers.
    size_t num_inliers = 0;

    // The sum of all inlier residuals.
    double residual_sum = std::numeric_limits<double>::max();
  };

  // Compute the support of the residuals.
  Support Evaluate(const std::vector<double>& residuals,
                   const double max_residual);

  // Compare the two supports and return the better support.
  bool Compare(const Support& support1, const Support& support2);
};

// Measure the support of a model by its fitness to the data as used in MSAC.
// A support is better if it has a smaller MSAC score.
struct MEstimatorSupportMeasurer {
  struct Support {
    // The number of inliers.
    size_t num_inliers = 0;

    // The MSAC score, defined as the truncated sum of residuals.
    double score = std::numeric_limits<double>::max();
  };

  // Compute the support of the residuals.
  Support Evaluate(const std::vector<double>& residuals,
                   const double max_residual);

  // Compare the two supports and return the better support.
  bool Compare(const Support& support1, const Support& support2);
};

struct RANSACOptions {
  // Maximum error for a sample to be considered as an inlier. Note that
  // the residual of an estimator corresponds to a squared error.
  double max_error = 0.0;

  // A priori assumed minimum inlier ratio, which determines the maximum number
  // of iterations. Only applies if smaller than `max_num_trials`.
  double min_inlier_ratio = 0.1;

  // Abort the iteration if minimum probability that one sample is free from
  // outliers is reached.
  double confidence = 0.99;

  // Number of random trials to estimate model from random subset.
  size_t min_num_trials = 0;
  size_t max_num_trials = std::numeric_limits<size_t>::max();

  void Check() const {
    CHECK_GT(max_error, 0);
    CHECK_GE(min_inlier_ratio, 0);
    CHECK_LE(min_inlier_ratio, 1);
    CHECK_GE(confidence, 0);
    CHECK_LE(confidence, 1);
    CHECK_LE(min_num_trials, max_num_trials);
  }
};

template <typename Estimator, typename SupportMeasurer = InlierSupportMeasurer,
          typename Sampler = RandomSampler>
class RANSAC {
 public:
  struct Report {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Whether the estimation was successful.
    bool success = false;

    // The number of RANSAC trials / iterations.
    size_t num_trials = 0;

    // The support of the estimated model.
    typename SupportMeasurer::Support support;

    // Boolean mask which is true if a sample is an inlier.
    std::vector<unsigned char> inlier_mask;

    // The estimated model.
    typename Estimator::M_t model;
  };

  explicit RANSAC(const RANSACOptions& options);

  // Determine the maximum number of trials required to sample at least one
  // outlier-free random set of samples with the specified confidence,
  // given the inlier ratio.
  //
  // @param num_inliers    The number of inliers.
  // @param num_samples    The total number of samples.
  // @param confidence     Confidence that one sample is outlier-free.
  //
  // @return               The required number of iterations.
  static size_t ComputeNumTrials(const size_t num_inliers,
                                 const size_t num_samples,
                                 const double confidence);

  // Robustly estimate model with RANSAC (RANdom SAmple Consensus).
  //
  // @param X              Independent variables.
  // @param Y              Dependent variables.
  //
  // @return               The report with the results of the estimation.
  Report Estimate(const std::vector<typename Estimator::X_t>& X,
                  const std::vector<typename Estimator::Y_t>& Y);

  // Objects used in RANSAC procedure. Access useful to define custom behavior
  // through options or e.g. to compute residuals.
  Estimator estimator;
  Sampler sampler;
  SupportMeasurer support_measurer;

 protected:
  RANSACOptions options_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename Estimator, typename SupportMeasurer, typename Sampler>
RANSAC<Estimator, SupportMeasurer, Sampler>::RANSAC(
    const RANSACOptions& options)
    : sampler(Sampler(Estimator::kMinNumSamples)), options_(options) {
  options.Check();

  // Determine max_num_trials based on assumed `min_inlier_ratio`.
  const size_t kNumSamples = 100000;
  const size_t dyn_max_num_trials = ComputeNumTrials(
      static_cast<size_t>(options_.min_inlier_ratio * kNumSamples), kNumSamples,
      options_.confidence);
  options_.max_num_trials =
      std::min<size_t>(options_.max_num_trials, dyn_max_num_trials);
}

template <typename Estimator, typename SupportMeasurer, typename Sampler>
size_t RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
    const size_t num_inliers, const size_t num_samples,
    const double confidence) {
  const double inlier_ratio = num_inliers / static_cast<double>(num_samples);

  const double nom = 1 - confidence;
  if (nom <= 0) {
    return std::numeric_limits<size_t>::max();
  }

  const double denom = 1 - std::pow(inlier_ratio, Estimator::kMinNumSamples);
  if (denom <= 0) {
    return 1;
  }

  return static_cast<size_t>(std::ceil(std::log(nom) / std::log(denom)));
}

template <typename Estimator, typename SupportMeasurer, typename Sampler>
typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report
RANSAC<Estimator, SupportMeasurer, Sampler>::Estimate(
    const std::vector<typename Estimator::X_t>& X,
    const std::vector<typename Estimator::Y_t>& Y) {
  CHECK_EQ(X.size(), Y.size());

  const size_t num_samples = X.size();

  Report report;
  report.success = false;
  report.num_trials = 0;

  if (num_samples < Estimator::kMinNumSamples) {
    return report;
  }

  typename SupportMeasurer::Support best_support;
  typename Estimator::M_t best_model;

  bool abort = false;

  const double max_residual = options_.max_error * options_.max_error;

  std::vector<double> residuals(num_samples);

  std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
  std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

  sampler.Initialize(num_samples);

  size_t max_num_trials = options_.max_num_trials;
  max_num_trials = std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
  size_t dyn_max_num_trials = max_num_trials;

  for (report.num_trials = 0; report.num_trials < max_num_trials;
       ++report.num_trials) {
    if (abort) {
      report.num_trials += 1;
      break;
    }

    sampler.SampleXY(X, Y, &X_rand, &Y_rand);

    // Estimate model for current subset.
    const std::vector<typename Estimator::M_t> sample_models =
        estimator.Estimate(X_rand, Y_rand);

    // Iterate through all estimated models.
    for (const auto& sample_model : sample_models) {
      estimator.Residuals(X, Y, sample_model, &residuals);
      CHECK_EQ(residuals.size(), X.size());

      const auto support = support_measurer.Evaluate(residuals, max_residual);

      // Save as best subset if better than all previous subsets.
      if (support_measurer.Compare(support, best_support)) {
        best_support = support;
        best_model = sample_model;

        dyn_max_num_trials = ComputeNumTrials(best_support.num_inliers,
                                              num_samples, options_.confidence);
      }

      if (report.num_trials >= dyn_max_num_trials &&
          report.num_trials >= options_.min_num_trials) {
        abort = true;
        break;
      }
    }
  }

  report.support = best_support;
  report.model = best_model;

  // No valid model was found.
  if (report.support.num_inliers < estimator.kMinNumSamples) {
    return report;
  }

  report.success = true;

  // Determine inlier mask. Note that this calculates the residuals for the
  // best model twice, but saves to copy and fill the inlier mask for each
  // evaluated model. Some benchmarking revealed that this approach is faster.

  estimator.Residuals(X, Y, report.model, &residuals);
  CHECK_EQ(residuals.size(), X.size());

  report.inlier_mask.resize(num_samples);
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (residuals[i] <= max_residual) {
      report.inlier_mask[i] = true;
    } else {
      report.inlier_mask[i] = false;
    }
  }

  return report;
}


// Implementation of LO-RANSAC (Locally Optimized RANSAC).
//
// "Locally Optimized RANSAC" Ondrej Chum, Jiri Matas, Josef Kittler, DAGM 2003.
template <typename Estimator, typename LocalEstimator,
          typename SupportMeasurer = InlierSupportMeasurer,
          typename Sampler = RandomSampler>
class LORANSAC : public RANSAC<Estimator, SupportMeasurer, Sampler> {
 public:
  using typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report;

  explicit LORANSAC(const RANSACOptions& options);

  // Robustly estimate model with RANSAC (RANdom SAmple Consensus).
  //
  // @param X              Independent variables.
  // @param Y              Dependent variables.
  //
  // @return               The report with the results of the estimation.
  Report Estimate(const std::vector<typename Estimator::X_t>& X,
                  const std::vector<typename Estimator::Y_t>& Y);

  // Objects used in RANSAC procedure.
  using RANSAC<Estimator, SupportMeasurer, Sampler>::estimator;
  LocalEstimator local_estimator;
  using RANSAC<Estimator, SupportMeasurer, Sampler>::sampler;
  using RANSAC<Estimator, SupportMeasurer, Sampler>::support_measurer;

 private:
  using RANSAC<Estimator, SupportMeasurer, Sampler>::options_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
          typename Sampler>
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::LORANSAC(
    const RANSACOptions& options)
    : RANSAC<Estimator, SupportMeasurer, Sampler>(options) {}

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
          typename Sampler>
typename LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Report
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Estimate(
    const std::vector<typename Estimator::X_t>& X,
    const std::vector<typename Estimator::Y_t>& Y) {
  CHECK_EQ(X.size(), Y.size());

  const size_t num_samples = X.size();

  typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report report;
  report.success = false;
  report.num_trials = 0;

  if (num_samples < Estimator::kMinNumSamples) {
    return report;
  }

  typename SupportMeasurer::Support best_support;
  typename Estimator::M_t best_model;
  bool best_model_is_local = false;

  bool abort = false;

  const double max_residual = options_.max_error * options_.max_error;

  std::vector<double> residuals(num_samples);

  std::vector<typename LocalEstimator::X_t> X_inlier;
  std::vector<typename LocalEstimator::Y_t> Y_inlier;

  std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
  std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

  sampler.Initialize(num_samples);

  size_t max_num_trials = options_.max_num_trials;
  max_num_trials = std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
  size_t dyn_max_num_trials = max_num_trials;

  for (report.num_trials = 0; report.num_trials < max_num_trials;
       ++report.num_trials) {
    if (abort) {
      report.num_trials += 1;
      break;
    }

    sampler.SampleXY(X, Y, &X_rand, &Y_rand);

    // Estimate model for current subset.
    const std::vector<typename Estimator::M_t> sample_models =
        estimator.Estimate(X_rand, Y_rand);

    // Iterate through all estimated models
    for (const auto& sample_model : sample_models) {
      estimator.Residuals(X, Y, sample_model, &residuals);
      CHECK_EQ(residuals.size(), X.size());

      const auto support = support_measurer.Evaluate(residuals, max_residual);

      // Do local optimization if better than all previous subsets.
      if (support_measurer.Compare(support, best_support)) {
        best_support = support;
        best_model = sample_model;
        best_model_is_local = false;

        // Estimate locally optimized model from inliers.
        if (support.num_inliers > Estimator::kMinNumSamples &&
            support.num_inliers >= LocalEstimator::kMinNumSamples) {
          X_inlier.clear();
          Y_inlier.clear();
          X_inlier.reserve(support.num_inliers);
          Y_inlier.reserve(support.num_inliers);
          for (size_t i = 0; i < residuals.size(); ++i) {
            if (residuals[i] <= max_residual) {
              X_inlier.push_back(X[i]);
              Y_inlier.push_back(Y[i]);
            }
          }

          const std::vector<typename LocalEstimator::M_t> local_models =
              local_estimator.Estimate(X_inlier, Y_inlier);

          for (const auto& local_model : local_models) {
            local_estimator.Residuals(X, Y, local_model, &residuals);
            CHECK_EQ(residuals.size(), X.size());

            const auto local_support =
                support_measurer.Evaluate(residuals, max_residual);

            // Check if non-locally optimized model is better.
            if (support_measurer.Compare(local_support, support)) {
              best_support = local_support;
              best_model = local_model;
              best_model_is_local = true;
            }
          }
        }

        dyn_max_num_trials =
            RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
                best_support.num_inliers, num_samples, options_.confidence);
      }

      if (report.num_trials >= dyn_max_num_trials &&
          report.num_trials >= options_.min_num_trials) {
        abort = true;
        break;
      }
    }
  }

  report.support = best_support;
  report.model = best_model;

  // No valid model was found
  if (report.support.num_inliers < estimator.kMinNumSamples) {
    return report;
  }

  report.success = true;

  // Determine inlier mask. Note that this calculates the residuals for the
  // best model twice, but saves to copy and fill the inlier mask for each
  // evaluated model. Some benchmarking revealed that this approach is faster.

  if (best_model_is_local) {
    local_estimator.Residuals(X, Y, report.model, &residuals);
  } else {
    estimator.Residuals(X, Y, report.model, &residuals);
  }

  CHECK_EQ(residuals.size(), X.size());

  report.inlier_mask.resize(num_samples);
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (residuals[i] <= max_residual) {
      report.inlier_mask[i] = true;
    } else {
      report.inlier_mask[i] = false;
    }
  }

  return report;
}

InlierSupportMeasurer::Support InlierSupportMeasurer::Evaluate(
    const std::vector<double>& residuals, const double max_residual) {
  Support support;
  support.num_inliers = 0;
  support.residual_sum = 0;

  for (const auto residual : residuals) {
    if (residual <= max_residual) {
      support.num_inliers += 1;
      support.residual_sum += residual;
    }
  }

  return support;
}

bool InlierSupportMeasurer::Compare(const Support& support1,
                                    const Support& support2) {
  if (support1.num_inliers > support2.num_inliers) {
    return true;
  } else {
    return support1.num_inliers == support2.num_inliers &&
           support1.residual_sum < support2.residual_sum;
  }
}

MEstimatorSupportMeasurer::Support MEstimatorSupportMeasurer::Evaluate(
    const std::vector<double>& residuals, const double max_residual) {
  Support support;
  support.num_inliers = 0;
  support.score = 0;

  for (const auto residual : residuals) {
    if (residual <= max_residual) {
      support.num_inliers += 1;
      support.score += residual;
    } else {
      support.score += max_residual;
    }
  }

  return support;
}

bool MEstimatorSupportMeasurer::Compare(const Support& support1,
                                        const Support& support2) {
  return support1.score < support2.score;
}

}
