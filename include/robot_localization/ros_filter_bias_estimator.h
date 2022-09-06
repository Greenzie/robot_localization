#ifndef ROBOT_LOCALIZATION_ROS_FILTER_BIAS_ESTIMATOR_H
#define ROBOT_LOCALIZATION_ROS_FILTER_BIAS_ESTIMATOR_H

#include <Eigen/Dense>
#include <fstream>
#include <vector>

#define RF_TOOLS_DEBUG(msg) if (getDebug() || getVerbose()) { debug_stream << msg; }
#define RF_TOOLS_VERBOSE(msg) if (getVerbose()) { debug_stream << msg; }

namespace RobotLocalization
{

class RosFilterBiasEstimator {
    public:
        static const int ESTIMATION_AXES = 3;

        // Default constructor
        RosFilterBiasEstimator();

        /**
         * @brief Special constructor with initializing values
         * 
         * @param min_speed Minimum speed at which to update the bias estimator
         * @param max_variance Maximum variance at which to update the estimator
         * @param alpha Alpha term of the alpha-beta filter for the bias estimator
         * @param max_divergence Maximum divergence between the feeder estimator and this estimator before rejecting the inputs
         * @param is_valid Enables defaulting the validity to false until confirmed by an external system
         */
        RosFilterBiasEstimator(const double min_speed,
                               const double max_variance,
                               const double alpha,
                               const double max_divergence = M_PI * 2.0,
                               const bool is_valid = false);
    
        RosFilterBiasEstimator(const RosFilterBiasEstimator& right);

        // Destructor
        ~RosFilterBiasEstimator();

        /**
         * @brief Resets the estimator
         * 
         */
        void reset();

        void setDebugInfo(bool debug, bool verbose)
        {
            debug_ = debug;
            verbose_ = verbose;
        }
        bool getDebug() const { return debug_; }
        bool getVerbose() const { return verbose_; }

        /**
         * @brief Sets the data from a prior estimator
         * 
         * @param orientation_estimate Roll/Pitch/Yaw in rad
         * @param orientation_variance Roll/Pitch/Yaw variance in rad
         * @param speed Speed in appropriate units
         * @param time_s Time in seconds
         */
        void setImuDynamicCorrectionData(const Eigen::Vector3d &orientation_estimate,
                                         const Eigen::Vector3d &orientation_variance,
                                         double speed,
                                         double time_s);
        
        /**
         * @brief Updates the bias estimate, returns the corrected measurement and associated variance, and determines
         *        whether the resulting update is valid.
         * 
         * @param orientation_measurement Original orientation measurement in, updated/biased measurement out
         * @param measurement_variance Original variance in, updated/biased variance out
         * @param sensor_timeout The timeout of the sensor in seconds
         * @param time_s The current time in seconds
         * @param orientation_estimate The current estimate of the orientation - used for deviation checking
         * @param estimate_variance The current variance of the orientation estimate - used for deviation checking
         * @param is_valid std::vector<bool> whether each axis of the biased orientation measurement and variance are valid for inclusion in the filter
         * @param debug_stream Used for debug output, if configured
         */
        void updateBiasEstimate(Eigen::Vector3d &orientation_measurement,
                                Eigen::Vector3d &measurement_variance,
                                double sensor_timeout,
                                double time_s,
                                Eigen::Vector3d &orientation_estimate,
                                Eigen::Vector3d &estimate_variance,
                                std::vector<bool> &is_valid,
                                std::ofstream &debug_stream);

        // Parameters setters and getters
        void set_estimation_axes(std::vector<bool> is_estimating) {
            for(uint8_t axis = 0; axis < std::min(static_cast<int>(is_estimating.size()), ESTIMATION_AXES); axis++)
            {
                estimation_axes_[axis] = is_estimating[axis];
            }
        }
        void get_estimation_axes(std::vector<bool> &is_estimating) const {
            for(uint8_t axis = 0; axis < ESTIMATION_AXES; axis++)
            {
                is_estimating.push_back(estimation_axes_[axis]);
            }
        }

        void set_min_speed(double min_speed) { min_speed_ = min_speed; }
        double get_min_speed() const { return min_speed_; }
        void set_max_orientation_variance(double max_var) { max_orientation_variance_ = max_var; }
        double get_max_orientation_variance() const { return max_orientation_variance_; }
        void set_alpha(double alpha) { alpha_ = alpha; }
        double get_alpha() const { return alpha_; }
        void set_max_divergence(double max_divergence) { max_divergence_ = max_divergence; }
        double get_max_divergence() const { return max_divergence_; }
        void set_valid(bool valid) { is_valid_ = valid; }
        bool is_valid() const { return is_valid_; }
    private:
        // Time of last state received in seconds
        double uncorrected_state_received_s_{0.0};

        // Estimation axes
        bool estimation_axes_[ESTIMATION_AXES] = {false, false, false};

        // The last orientation estimate and variance in Euler angles received from another estimator
        Eigen::Vector3d uncorrected_orientation_estimate_;
        Eigen::Vector3d uncorrected_orientation_variance_;

        // The estimated orientation offset and variance
        Eigen::Vector3d orientation_offset_;
        Eigen::Vector3d orientation_offset_variance_;

        // The last speed of the vehicle received from a prior estimator
        double uncorrected_speed_{0.0};

        // The minimum speed at which to apply corrections
        double min_speed_{0.0};

        // The maximum variance at which to apply corrections
        double max_orientation_variance_{0.0};

        // Alpha value for the alpha-beta filter (reliance on last measurement) to ensure a low-pass update
        double alpha_{0.0};

        // Maximum divergence between current filter and feeding filter when speed drops below minimum
        double max_divergence_{M_PI * 2.0};

        // External validity check - defaults to true to enable no checks with optional override on construction
        bool is_valid_{true};

        // Handling the times for the divergence test to determine whether it is valid
        int divergence_test_counter_[ESTIMATION_AXES] = {0, 0, 0};
        double divergence_test_steps_[ESTIMATION_AXES] = {0, 0, 0};

        // Whether the orientation offset has been set for future use.
        bool orientation_offset_has_been_set_[ESTIMATION_AXES] = {false, false, false};
        bool orientation_offset_is_updating_[ESTIMATION_AXES] = {false, false, false};

        // Debugging capabilities
        bool debug_{false};
        bool verbose_{false};
};


}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_BIAS_ESTIMATOR_H