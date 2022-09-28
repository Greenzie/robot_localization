#include "robot_localization/ros_filter_bias_estimator.h"
#include "robot_localization/filter_utilities.h"

namespace RobotLocalization
{

RosFilterBiasEstimator::RosFilterBiasEstimator() {

}

RosFilterBiasEstimator::RosFilterBiasEstimator(const double min_speed,
                                               const double max_variance,
                                               const double alpha,
                                               const double max_divergence,
                                               const int max_num_divergences,
                                               const double initial_delay,
                                               const bool is_valid) {
    set_min_speed(min_speed);
    set_max_orientation_variance(max_variance);
    set_alpha(alpha);
    set_max_divergence(max_divergence);
    set_max_num_divergences(max_num_divergences);
    set_initial_delay(initial_delay);
    set_valid(is_valid);
}

RosFilterBiasEstimator::~RosFilterBiasEstimator() {

}

RosFilterBiasEstimator::RosFilterBiasEstimator(const RosFilterBiasEstimator& right) {
    set_min_speed(right.get_min_speed());
    set_max_orientation_variance(right.get_max_orientation_variance());
    set_alpha(right.get_alpha());
    set_max_divergence(right.get_max_divergence());
    set_valid(right.is_valid());
    setDebugInfo(right.getDebug(), right.getVerbose());
    std::vector<bool> estimation_axes;
    right.get_estimation_axes(estimation_axes);
    set_estimation_axes(estimation_axes);
    set_max_num_divergences(right.get_max_num_divergences());
    set_initial_delay(right.get_initial_delay());
}

void RosFilterBiasEstimator::reset() {
    mtx_.lock();
    uncorrected_state_received_s_ = 0.0;
    uncorrected_speed_ = 0.0;
    for(uint8_t counter = 0; counter < ESTIMATION_AXES; counter++)
    {
        orientation_offset_has_been_set_[counter] = false;
        orientation_offset_is_updating_[counter] = false;
        divergence_test_counter_[counter] = -1;
        orientation_offset_[counter] = 0.0;
        orientation_offset_variance_[counter] = M_PI * M_PI;
    }
    uncorrected_orientation_estimate_.setZero();
    num_exceeded_max_divergence_ = 0;
    start_time_ = 0.0;
    initial_delay_met_ = false;
    mtx_.unlock();
}

void RosFilterBiasEstimator::setImuDynamicCorrectionData(const Eigen::Vector3d &orientation_estimate,
                                                         const Eigen::Vector3d &orientation_variance,
                                                         double speed,
                                                         double time_s) {
    // Save the new data
    mtx_.lock();
    uncorrected_orientation_estimate_ = orientation_estimate;
    uncorrected_orientation_variance_ = orientation_variance;
    uncorrected_speed_ = speed;
    uncorrected_state_received_s_ = time_s;
    mtx_.unlock();
}

void RosFilterBiasEstimator::updateBiasEstimate(Eigen::Vector3d &orientation_measurement,
                                                Eigen::Vector3d &measurement_variance,
                                                double sensor_timeout,
                                                double time_s,
                                                Eigen::Vector3d &orientation_estimate,
                                                Eigen::Vector3d &estimate_variance,
                                                std::vector<bool> &is_valid,
                                                std::ofstream &debug_stream) {
    mtx_.lock();
    is_valid.clear();  // Clear and start fresh
    for(uint8_t axis = 0; axis < ESTIMATION_AXES; axis++)
    {
        if(estimation_axes_[axis])
        {
            // Set based on external system. Can be switched to false later based on internal tests
            // Also include the lockout from too many failures
            bool can_still_update = (max_num_divergences_ < 1) || (num_exceeded_max_divergence_ < max_num_divergences_);
            is_valid.push_back(is_valid_ && can_still_update);
            // For logging
            std::string axis_name = (axis == 0) ? "roll" : (axis == 1) ? "pitch" : "yaw";
            if (is_valid[axis]) // Only move forward if still valid
            {
                if ((uncorrected_state_received_s_ > 0.0) && ((time_s - uncorrected_state_received_s_) < sensor_timeout))
                {
                    // Has current, valid data for updating the offset estimate
                    if((uncorrected_orientation_variance_[axis] < max_orientation_variance_) && (uncorrected_speed_ > min_speed_))
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
                        double offset = FilterUtilities::clampRotation(uncorrected_orientation_estimate_[axis] - orientation_measurement[axis]);
                        if(::fabs(orientation_offset_[axis]) < 1e-9)
                        {
                            // Has not been initialized
                            orientation_offset_[axis] = offset;
                            // Should be added (+) since these are variances
                            orientation_offset_variance_[axis] = measurement_variance[axis] + uncorrected_orientation_variance_[axis];
                            // Set the start time to handle the delay
                            start_time_ = time_s;
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
                                (1.0 - alpha_) * (uncorrected_orientation_variance_[axis] + measurement_variance[axis]);
                        }
                        // Handle the initial delay - may be immediate if no delay set
                        initial_delay_met_ |= ((time_s - start_time_) >= initial_delay_);

                        // Calculate the difference between the uncorrected filter estimate and the corrected magnetometer value
                        double abs_filter_to_mag_difference = ::fabs(uncorrected_orientation_estimate_[axis] -
                            (orientation_measurement[axis] + orientation_offset_[axis]));

                        // Dropped below limit or calculated once with no limit
                        orientation_offset_has_been_set_[axis] |= ((initial_delay_met_) &&
                            ((abs_filter_to_mag_difference < max_divergence_) | (max_divergence_ < 1e-9)));

                        // Handle the divergence test initialization
                        if(was_updating == false) {
                            // Calculate the time
                            // Determine the time to drop below the cutoff based on the current difference between the uncorrected and corrected estimators
                            //  and the alpha-beta filter
                            // The calculation is from max_divergence_ = abs_filter_difference * alpha_^(divergence_test_steps_)
                            // divergence_test_steps_ = ln(max_divergence_ / abs_filter_difference) / ln(alpha_)
                            // Note that max_divergence_ must be < abs_filter_difference since alpha_ < 1.0
                            double abs_filter_difference = ::fabs(uncorrected_orientation_estimate_[axis] - orientation_estimate[axis]);
                            double max_difference = std::max(abs_filter_difference, abs_filter_to_mag_difference);
                            if((max_divergence_ < max_difference) && (max_divergence_ > 0.0))
                            {
                                divergence_test_steps_[axis] = round(log(max_divergence_ / max_difference) / log(alpha_));
                            }
                            else
                            {
                                divergence_test_steps_[axis] = 0;  // Already within tolerance
                            }
                            RF_TOOLS_VERBOSE("Required number of steps for axis " << axis_name <<
                                " to test divergence: " << divergence_test_steps_[axis] << " given abs difference " << max_difference
                                << " and max difference " << max_divergence_ << "\n");
                            // Set the start counter
                            divergence_test_counter_[axis] = 0;
                        }
                        if((divergence_test_counter_[axis] <= divergence_test_steps_[axis]) && (divergence_test_counter_[axis] >= 0))
                        {
                            divergence_test_counter_[axis]++;
                        }

                        // Handle debugging
                        std::string debug_info;
                        debug_info += "    EKF 1 " + axis_name + ": " + std::to_string(uncorrected_orientation_estimate_[axis] * 180 / M_PI) + " deg\n";
                        debug_info += "    EKF 1 " + axis_name + " var: " + std::to_string(uncorrected_orientation_variance_[axis]) + " rad^2\n";
                        debug_info += "    Uncorrected IMU " + axis_name + ": " + std::to_string(orientation_measurement[axis] * 180 / M_PI) + " deg\n";
                        debug_info += "    Uncorrected IMU var: " + std::to_string(measurement_variance[axis]) + " rad^2\n";
                        debug_info += "    IMU offset: " + std::to_string(orientation_offset_[axis] * 180 / M_PI) + " deg\n";
                        debug_info += "    IMU offset var: " + std::to_string(orientation_offset_variance_[axis]) + " rad^2\n";
                        RF_TOOLS_VERBOSE("IMU dynamic correction:\n" << debug_info.c_str());
                    }
                    else if (uncorrected_speed_ > min_speed_) {
                        // Moving fast enough, but variance too high. Log since there is a potential divergence case here.
                        RF_TOOLS_VERBOSE("Cannot update dynamic corrections due to variance limit - check for accurate bias estimate.");
                        orientation_offset_is_updating_[axis] = false;  // Not updating
                        // Don't update the estimate, but keep the bias set since it's just avoiding noise at the moment
                    }
                    else
                    {
                        // Still has valid data, but cannot update due to other conditions
                        RF_TOOLS_VERBOSE("Cannot update dynamic correction with speed: " <<
                            uncorrected_speed_ << " < " <<
                            min_speed_ << " or yaw variance: " <<
                            uncorrected_orientation_variance_[axis] << " > " <<
                            max_orientation_variance_ << "\n");

                        // Run the divergence test
                        if(divergence_test_counter_[axis] >= divergence_test_steps_[axis])
                        {
                            double abs_filter_difference = ::fabs(uncorrected_orientation_estimate_[axis] - orientation_estimate[axis]);
                            if(abs_filter_difference > max_divergence_)
                            {
                                // Divergence detected - cannot trust this filter's estimate of the orientation
                                orientation_offset_has_been_set_[axis] = false;  // Wait until divergence drops below limit again
                                RF_TOOLS_DEBUG("IMU orientation divergence of " << abs_filter_difference * 180.0 / M_PI <<
                                    " degrees detected due to divergence at min speed - ignoring incorrect data\n");
                                // Note that we get automatic hysteresis because we only stop using measurements via this method when the speed drops below the cutoff.
                                num_exceeded_max_divergence_ += 1;  // Index the counter
                            }
                            divergence_test_counter_[axis] = -1;  // Already run
                        }
                        orientation_offset_is_updating_[axis] = false;  // Not updating
                    }
                }
                else if (orientation_offset_has_been_set_[axis])
                {
                    // Only warn if already has received the data. Otherwise, a ton of warnings
                    //  during initialization.
                    RF_TOOLS_DEBUG("Stale state data for use in calculating " + axis_name + " offset\n");
                    orientation_offset_is_updating_[axis] = false;  // Not updating
                    // Note: do not reset the has_been_set flag since it's possibly just running at a different rate
                }
            }
            else
            {
                // Invalid data - cannot update, so don't use it
                orientation_offset_has_been_set_[axis] = false;  // Prep for a smooth restart
                orientation_offset_is_updating_[axis] = false;  // No longer updating
            }

            // Independently check since this is different than whether the data itself is valid
            if (orientation_offset_has_been_set_[axis])  // Calculated at least once
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
                RF_TOOLS_VERBOSE("Unavailable offset information for " + axis_name + "\n");
                // Cannot create the pose measurement yet - there is no offset information or it has been flagged as invalid
                is_valid[axis] = false;
            }
        }
        else
        {
            // Not being used/estimated
            is_valid.push_back(false);
        }
    }
    mtx_.unlock();
}

}  // namespace RobotLocalization
