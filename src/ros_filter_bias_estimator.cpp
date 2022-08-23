#include "robot_localization/ros_filter_bias_estimator.h"
#include "robot_localization/filter_utilities.h"
#include "robot_localization/ros_filter_utilities.h"

namespace RobotLocalization
{

// -------------------------------------------------------------------------------------
// Windowed stats

RosFilterBiasEstimator::WindowedStats::WindowedStats() :
    window_duration_(std::numeric_limits<double>::infinity()) {

}

RosFilterBiasEstimator::WindowedStats::WindowedStats(double window_duration) {
    setWindowDuration(window_duration);
}

void RosFilterBiasEstimator::WindowedStats::setWindowDuration(double new_window_duration)
{
  window_duration_ = new_window_duration;
}

void RosFilterBiasEstimator::WindowedStats::clear()
{
    is_full_ = false;
    oldest_index_ = latest_index_;  // No data now
}

void RosFilterBiasEstimator::WindowedStats::add(double t, double v)
{
  // Increment the latest_index_ by 1 and insert the new time
  latest_index_ = (latest_index_ + 1) % ARRAY_SIZE;
  times_[latest_index_] = t;
  values_[latest_index_] = v;

  // If we've caught up with oldest_index, then increment it by 1
  if (latest_index_ == oldest_index_)
  {
    oldest_index_ = (oldest_index_ + 1) % ARRAY_SIZE;
  }

  // Call update to move up the oldest_index
  is_full_ = false;
  while ((t - times_[oldest_index_]) > window_duration_ && oldest_index_ != latest_index_)
  {
    oldest_index_ = (oldest_index_ + 1) % ARRAY_SIZE;
    is_full_ = true;  // Had to move the oldest index, so the window duration is full
  }
}

double RosFilterBiasEstimator::WindowedStats::valueMean()
{
  double sum = 0.0;
  size_t numvals = 0;

  for (size_t i = oldest_index_; i != latest_index_; i = (i + 1) % ARRAY_SIZE)
  {
    sum += values_[i];
    ++numvals;
  }

  return sum / numvals;
}

double RosFilterBiasEstimator::WindowedStats::valueStdDev()
{
  double sum = 0.0;
  size_t numvals = 0;
  double mean = valueMean();

  for (size_t i = oldest_index_; i != latest_index_; i = (i + 1) % ARRAY_SIZE)
  {
    sum += pow(values_[i] - mean, 2);
    ++numvals;
  }

  return sqrt(sum / numvals);
}

double RosFilterBiasEstimator::WindowedStats::valueAbsMedian()
{
  std::vector<double> current_vals;

  // Get the sub-array
  for (size_t i = oldest_index_; i != latest_index_; i = (i + 1) % ARRAY_SIZE)
  {
    current_vals.emplace_back(abs(values_[i]));
  }

  // Sort the sub-array
  std::sort(current_vals.begin(), current_vals.end());

  // Find the median value
  size_t numvals = current_vals.size();
  if (numvals == 0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (numvals % 2 == 0)
  {
    return ((current_vals[(numvals / 2) - 1] + current_vals[numvals / 2]) / 2);
  }

  return current_vals[numvals / 2];
}

std::tuple<double, double, size_t> RosFilterBiasEstimator::WindowedStats::valueStats()
{
  double sum = 0.0;
  size_t numvals = 0;
  double mean = valueMean();

  for (size_t i = oldest_index_; i != latest_index_; i = (i + 1) % ARRAY_SIZE)
  {
    sum += pow(values_[i] - mean, 2);
    ++numvals;
  }

  double std_dev = sqrt(sum / numvals);

  return std::make_tuple(mean, std_dev, numvals);
}

// -------------------------------------------------------------------------------------
// Bias estimator

RosFilterBiasEstimator::RosFilterBiasEstimator() {

}

RosFilterBiasEstimator::RosFilterBiasEstimator(const double min_speed,
                                               const double max_variance,
                                               const double alpha,
                                               const double max_divergence,
                                               const double max_orientation_rate_score,
                                               const double moving_window_size_s) {
    set_min_speed(min_speed);
    set_max_orientation_variance(max_variance);
    set_alpha(alpha);
    set_max_divergence(max_divergence);
    set_max_orientation_rate_score(max_orientation_rate_score);
    set_moving_window_size_s(moving_window_size_s);
}

RosFilterBiasEstimator::~RosFilterBiasEstimator() {

}

void RosFilterBiasEstimator::reset() {
    last_state_received_s_ = 0.0;
    last_speed_ = 0.0;
    for(uint8_t counter = 0; counter < 3; counter++)
    {
        orientation_offset_has_been_set_[counter] = false;
        orientation_offset_is_updating_[counter] = false;
    }
    last_orientation_estimate_.setZero();
    orientation_offset_ << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI;
}

void RosFilterBiasEstimator::setImuDynamicCorrectionData(const Eigen::Vector3d &orientation_estimate,
                                                         const Eigen::Vector3d &orientation_variance,
                                                         double speed,
                                                         double time_s) {
    // For estimating rates
    prior_orientation_estimate_ = last_orientation_estimate_;
    // Save the new data
    last_orientation_estimate_ = orientation_estimate;
    last_orientation_variance_ = orientation_variance;
    last_speed_ = speed;
    last_state_received_s_ = time_s;

    // Calculate and save the rates via back difference
    for(uint8_t axis = 0; axis < ESTIMATION_AXES; axis++)
    {
        if(estimation_axes_[axis])
        {
            orientation_estimate_rate_[axis].add(time_s,
                (last_orientation_estimate_[axis] - prior_orientation_estimate_[axis]) / (time_s - prior_estimate_time_s_));
        }
    }
    // Save for next time
    prior_estimate_time_s_ = time_s;
}

void RosFilterBiasEstimator::updateBiasEstimate(Eigen::Vector3d &orientation_measurement,
                                                Eigen::Vector3d &measurement_variance,
                                                double sensor_timeout,
                                                double time_s,
                                                Eigen::Vector3d &orientation_estimate,
                                                Eigen::Vector3d &estimate_variance,
                                                std::vector<bool> &is_valid,
                                                std::ofstream &debug_stream) {
    is_valid.clear();  // Clear and start fresh
    for(uint8_t axis = 0; axis < 3; axis++)
    {
        if(estimation_axes_[axis])
        {
            // Update the estimate of the rates
            orientation_estimate_rate_[axis].add(time_s,
                (orientation_measurement[axis] - prior_orientation_measurement_[axis]) / (time_s - prior_measurement_time_s_));

            is_valid.push_back(true);  // Assumed true until proven otherwise
            // For logging
            std::string axis_name = (axis == 0) ? "roll" : (axis == 1) ? "pitch" : "yaw";
            if ((last_state_received_s_ > 0.0) && ((time_s - last_state_received_s_) < sensor_timeout))
            {
                if((last_orientation_variance_[axis] < max_orientation_variance_) && (last_speed_ > min_speed_))
                {
                    // For handling the required time until can run a divergence check
                    bool was_updating = orientation_offset_is_updating_[axis];
                    // Updating now
                    orientation_offset_is_updating_[axis] = true;

                    // Should be subtracted (-) since the offset is a difference.
                    // The alpha-beta filter is rather special here, as it has to account for a few issues. First, how do we initialize it?
                    //  If initialized to zero, that means the orientation offset is assumed to be zero to start, with the resulting slow adjustments to
                    //  the filtered correction. The problem is that it's not zero. Without a better set of information (e.g. stored data from
                    //  a previous run), the best way is to initialize it with a jump to the first calculation, then run the filter.
                    // Because the alpha-beta filter is maintaining history, angle wrapping is a problem if not handled.
                    //  To correct this, we will always keep the offset in the range [-PI, PI]. The actual orientation angle
                    //  wrapping is handled by the Kalman filter. The offset angle wrapping needs to be handled here. The additional step is what
                    //  happens when the angle steps over the boundary (e.g. from -(PI-0.0001) to (PI-0.0001)). That is explained and handled in the
                    //  else condition.
                    double offset = FilterUtilities::clampRotation(last_orientation_estimate_[axis] - orientation_measurement[axis]);
                    if(::fabs(orientation_offset_[axis]) < 1e-9)
                    {
                        // Has not been initialized
                        orientation_offset_[axis] = offset;
                        // Should be added (+) since these are variances
                        orientation_offset_variance_[axis] = measurement_variance[axis] + last_orientation_variance_[axis];
                    }
                    else
                    {
                        // Has been initialized - use the alpha-beta filter
                        // Use an alternate formulation to enable handling angle wrapping.
                        // new = alpha*previous + (1-alpha)*current => new = alpha*(previous-current) + current
                        // With rotation clamping to the [-pi, pi] range: clampRotation(alpha*clampRotation(previous-current) + current)
                        //  E.g. previous = 175, current = -175, alpha = 0.8 (in degrees for ease of understanding - the real system is in radians)
                        //  Then: new = clampRotation(0.8*clampRotation(175--175) + -175)
                        //  new = clampRotation(0.8*clampRotation(350) - 175)
                        //  new = clampRotation(0.8*-10 - 175)
                        //  new = clampRotation(-8 - 175)
                        //  new = clampRotation(-183)
                        //  new = 177
                        orientation_offset_[axis] =
                            FilterUtilities::clampRotation(alpha_ * FilterUtilities::clampRotation(orientation_offset_[axis] - offset) + offset);

                        // Should be added (+) since these are variances.
                        // Note that the variance is not an actual angle, so angle wrapping is not required.
                        orientation_offset_variance_[axis] = alpha_ * orientation_offset_variance_[axis] +
                            (1.0 - alpha_) * (last_orientation_variance_[axis] + measurement_variance[axis]);
                    }
                    // Calculated at least once
                    orientation_offset_has_been_set_[axis] = true;

                    // Handle the divergence test initialization
                    if(was_updating == false) {
                        // Clear the rates check
                        orientation_measurement_rate_[axis].clear();
                        orientation_estimate_rate_[axis].clear();
                        orientation_rate_t_scores_[axis].clear();
                        // Calculate the time
                        // Determine the time to drop below the cutoff based on the current difference between the uncorrected and corrected estimators
                        //  and the alpha-beta filter
                        double abs_filter_difference = ::fabs(last_orientation_estimate_[axis] - orientation_measurement[axis]);
                        if((max_divergence_ < abs_filter_difference) && (max_divergence_ > 0.0))
                        {
                            divergence_test_steps_[axis] = round(log(max_divergence_ / abs_filter_difference) / log(alpha_));
                        }
                        else
                        {
                            divergence_test_steps_[axis] = 0;  // Already within tolerance
                        }
                        // Set the start counter
                        divergence_test_steps_[axis] = 0;
                    }
                    if(divergence_test_counter_[axis] <= divergence_test_steps_[axis])
                    {
                        divergence_test_counter_[axis]++;
                    }

                    // Check for the rates divergence
                    if((orientation_measurement_rate_[axis].is_full()) && (orientation_estimate_rate_[axis].is_full()))
                    {
                        // Can calculate the T scores now
                        auto [meas_mean, meas_std, meas_N] = orientation_measurement_rate_[axis].valueStats();
                        auto [est_mean, est_std, est_N] = orientation_estimate_rate_[axis].valueStats();

                        // Check for nans
                        if (isnan(meas_mean) || isnan(meas_std))
                        {
                            meas_mean = 0.0;
                            meas_std = std::numeric_limits<double>::infinity();
                        }
                        if (isnan(est_mean) || isnan(est_std))
                        {
                            est_mean = 0.0;
                            est_std = std::numeric_limits<double>::infinity();
                        }

                        auto [t, v_est] = anovaTest(meas_mean, meas_std, meas_N, est_mean, est_std, est_N);
                        if (isnan(time_s))
                        {
                            t = 0.0;
                        }

                        orientation_rate_t_scores_[axis].add(time_s, t);
                        if(orientation_rate_t_scores_[axis].is_full())
                        {
                            double median_t = orientation_rate_t_scores_[axis].valueAbsMedian();
                            if(median_t > max_orientation_rate_score_)
                            {
                                // Divergence detected - cannot trust this filter's estimate of the orientation
                                orientation_offset_has_been_set_[axis] = false;
                                RF_TOOLS_DEBUG("IMU orientation divergence detected due to rates - ignoring incorrect data");
                            }
                        }
                    }

                    // Handle debugging
                    std::string debug_info;
                    debug_info += "    EKF 1 " + axis_name + ": " + std::to_string(last_orientation_estimate_[axis] * 180 / M_PI) + " deg\n";
                    debug_info += "    EKF 1 " + axis_name + " var: " + std::to_string(last_orientation_variance_[axis]) + " rad^2\n";
                    debug_info += "    Uncorrected IMU " + axis_name + ": " + std::to_string(orientation_measurement[axis] * 180 / M_PI) + " deg\n";
                    debug_info += "    Uncorrected IMU var: " + std::to_string(measurement_variance[axis]) + " rad^2\n";
                    debug_info += "    IMU offset: " + std::to_string(orientation_offset_[axis] * 180 / M_PI) + " deg\n";
                    debug_info += "    IMU offset var: " + std::to_string(orientation_offset_variance_[axis]) + " rad^2\n";
                    RF_TOOLS_VERBOSE("IMU dynamic correction:\n" << debug_info.c_str());
                }
                else if (last_speed_ > min_speed_) {
                    // Moving fast enough, but variance too high. Log since there is a potential divergence case here.
                    ROS_WARN_STREAM_THROTTLE(2.0, "Cannot update dynamic corrections due to variance limit - check for accurate bias estimate.");
                }
                else
                {
                    RF_TOOLS_VERBOSE("Cannot update dynamic correction with speed: " <<
                    last_speed_ << " < " <<
                    min_speed_ << " or yaw variance: " <<
                    last_orientation_variance_[axis] << " > " <<
                    max_orientation_variance_ << "\n");

                    // Run the divergence test
                    if(divergence_test_counter_[axis] >= divergence_test_steps_[axis])
                    {
                        double abs_filter_difference = ::fabs(last_orientation_estimate_[axis] - orientation_measurement[axis]);
                        if(abs_filter_difference > max_divergence_)
                        {
                            // Divergence detected - cannot trust this filter's estimate of the orientation
                            orientation_offset_has_been_set_[axis] = false;
                            RF_TOOLS_DEBUG("IMU orientation divergence detected due to divergence at min speed - ignoring incorrect data");
                        }
                        divergence_test_counter_[axis] = -1;  // Already run
                    }
                }
            }
            else if (orientation_offset_has_been_set_[axis])
            {
                // Only warn if already has received the data. Otherwise, a ton of warnings
                //  during initialization.
                RF_TOOLS_DEBUG("Stale state data for use in calculating " + axis_name + " offset\n");
            }
            if (orientation_offset_has_been_set_[axis])  // Calculated at least once (these don't time out)
            {
                orientation_measurement[axis] += orientation_offset_[axis];
                measurement_variance[axis] += orientation_offset_variance_[axis];

                std::string debug_info;
                debug_info += "    Corrected IMU " + axis_name + ": " + std::to_string(orientation_measurement[axis] * 180 / M_PI) + " deg\n";
                debug_info += "    Corrected IMU var: " + std::to_string(measurement_variance[axis]) + " rad^2\n";
                RF_TOOLS_VERBOSE("IMU dynamic correction:\n" << debug_info.c_str());
            }
            else
            {
                RF_TOOLS_VERBOSE("No offset information for " + axis_name + "\n");
                // Cannot create the pose measurement yet - there is no offset information or it has been flagged as invalid
                is_valid[axis] = false;
            }
        }
        else
        {
            is_valid.push_back(false);  // Not being used/estimated
        }
    }

    // Save for next time
    prior_orientation_measurement_ = orientation_measurement;
    prior_measurement_time_s_ = time_s;
}

std::tuple<double, double> RosFilterBiasEstimator::anovaTest(double x1, double s1, double n1, double x2, double s2, double n2)
{
  if (n1 * n2 == 0)
  {
    return std::make_tuple(0.0, 0.0);
  }

  // Calculate t-statistic
  double t = (x1 - x2) / sqrt((s1 * s1 / n1) + (s2 * s2 / n2));
  // Calculate estimated degrees of freedom
  double v_est = pow((s1 * s1 / n1) + (s2 * s2 / n2), 2) /
                 ((pow(s1, 4) / (n1 * n1 * (n1 - 1))) + (pow(s2, 4) / (n2 * n2 * (n2 - 1))));
  return std::make_tuple(t, v_est);
}

}  // namespace RobotLocalization