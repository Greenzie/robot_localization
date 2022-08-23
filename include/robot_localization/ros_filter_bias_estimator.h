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

        class WindowedStats {
            public:
                /**
                 * @brief Constructor with infinite window
                 */
                WindowedStats();

                /**
                 * @brief Constructor with limited window
                 * 
                 * @param window_duration Window duration in seconds
                 */
                WindowedStats(double window_duration);

                /**
                 * @brief Sets the window duration
                 * 
                 * @param window_duration Window duration in seconds
                 */
                void setWindowDuration(double window_duration);

                /**
                 * @brief Clears all values (resets the number of values stored)
                 * 
                 */
                void clear();

                /**
                 * @brief Adds a value to the stats window
                 * 
                 * @param t The time of the value in seconds
                 * @param v The value
                 */
                void add(double t, double v = 0.0);

                /**
                 * @brief Whether there is enough data to fill the time window
                 * 
                 * @return bool whether there is enough data to fill the time window
                 */
                bool is_full() { return is_full_; }

                /**
                 * @brief Gets the mean of the values in the window
                 * 
                 * @return double The mean of the windowed values
                 */
                double valueMean();

                /**
                 * @brief Gets the standard deviation of the values in the window
                 * 
                 * @return double The mean of the windowed values
                 */
                double valueStdDev();

                /**
                 * @brief Returns the median of the absolute values in the time window
                 * 
                 * @return double The median of the absolute values in the time window
                 */
                double valueAbsMedian();

                /**
                 * @brief Gets the mean, standard deviation, and number of samples in the window
                 * 
                 * @return std::tuple<double, double, size_t> mean, standard deviation, number of samples
                 */
                std::tuple<double, double, size_t> valueStats();
            private:
                #define ARRAY_SIZE 1000
                std::array<double, ARRAY_SIZE> times_{ 0.0 };
                std::array<double, ARRAY_SIZE> values_{ 0.0 };
                size_t latest_index_{ ARRAY_SIZE - 1 };
                size_t oldest_index_{ ARRAY_SIZE - 1 };
                double window_duration_;  // Seconds

                bool is_full_{false};  // Whether the time window is full
        };

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
         * @param max_orientation_rate_score Maximum T-score of difference in orientation rates - used to catch bad orientation measurements
         * @param moving_window_size_s Moving window size for the orientation rates estimation
         */
        RosFilterBiasEstimator(const double min_speed,
                               const double max_variance,
                               const double alpha,
                               const double max_divergence = M_PI * 2.0,
                               const double max_orientation_rate_score = M_PI * 2.0,
                               const double moving_window_size_s = 1.0);

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
        bool getDebug() { return debug_; }
        bool getVerbose() { return verbose_; }

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
         * @param estaimte_variance The current variance of the orientation estimate - used for deviation checking
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
        
        /**
         * @brief https://en.wikipedia.org/wiki/Welch%27s_t-test Welch's ANOVA test
         * 
         * @param x1  Mean value from sample 1
         * @param s1  Standard deviation for sample 1
         * @param n1  Number of measures in sample 1
         * @param x2  Mean value from sample 2
         * @param s2  Standard deviation for sample 2
         * @param n2  Number of measures in sample 2
         * @return tuple of t value and v estimate
         */
        std::tuple<double, double> anovaTest(double x1,
                                             double s1,
                                             double n1,
                                             double x2,
                                             double s2,
                                             double n2);

        // Parameters setters and getters
        void set_estimation_axes(std::vector<bool> is_estimating) {
            for(uint8_t axis = 0; axis < std::min(static_cast<int>(is_estimating.size()), ESTIMATION_AXES); axis++)
            {
                estimation_axes_[axis] = is_estimating[axis];
            }
        }
        void get_estimation_axes(std::vector<bool> is_estimating) {
            for(uint8_t axis = 0; axis < 3; axis++)
            {
                is_estimating.push_back(estimation_axes_[axis]);
            }
        }

        void set_min_speed(double min_speed) { min_speed_ = min_speed; }
        double get_min_speed() { return min_speed_; }
        void set_max_orientation_variance(double max_var) { max_orientation_variance_ = max_var; }
        double get_max_orientation_variance() { return max_orientation_variance_; }
        void set_alpha(double alpha) { alpha_ = alpha; }
        double get_alpha() { return alpha_; }
        void set_max_divergence(double max_divergence) { max_divergence_ = max_divergence; }
        double get_max_divergence() { return max_divergence_; }
        void set_max_orientation_rate_score(double max_score) { max_orientation_rate_score_ = max_score; }
        double get_max_orientation_rate_score() { return max_orientation_rate_score_; }
        void set_moving_window_size_s(double window_size_s) {
            moving_window_size_s_ = window_size_s;
            // Update all time windows
            for(uint8_t axis = 0; axis < ESTIMATION_AXES; axis++)
            {
                orientation_measurement_rate_[axis].setWindowDuration(window_size_s);
                orientation_estimate_rate_[axis].setWindowDuration(window_size_s);
                orientation_rate_t_scores_[axis].setWindowDuration(window_size_s);
            }
        }
        double get_moving_window_size_s() { return moving_window_size_s_; }
    private:
        // Time of last state received in seconds
        double last_state_received_s_{0.0};

        // Estimation axes
        bool estimation_axes_[ESTIMATION_AXES] = {false, false, false};

        // The last orientation estimate and variance in Euler angles received from another estimator
        Eigen::Vector3d last_orientation_estimate_;
        Eigen::Vector3d last_orientation_variance_;

        // The estimated orientation offset and variance
        Eigen::Vector3d orientation_offset_;
        Eigen::Vector3d orientation_offset_variance_;

        // The last speed of the vehicle received from a prior estimator
        double last_speed_{0.0};

        // The minimum speed at which to apply corrections
        double min_speed_{0.0};

        // The maximum variance at which to apply corrections
        double max_orientation_variance_{0.0};

        // Alpha value for the alpha-beta filter (reliance on last measurement) to ensure a low-pass update
        double alpha_{0.0};

        // Maximum divergence between current filter and feeding filter when speed drops below minimum
        double max_divergence_{M_PI * 2.0};

        // Handling the times for the divergence test to determine whether it is valid
        int divergence_test_counter_[ESTIMATION_AXES] = {0, 0, 0};
        double divergence_test_steps_[ESTIMATION_AXES] = {0, 0, 0};

        // Maximum difference in orientation rates using an ANOVA test
        double max_orientation_rate_score_{M_PI * 2.0};

        // Moving window size in seconds for testing the orientation rate difference
        double moving_window_size_s_{1.0};

        // For the ANOVA test
        WindowedStats orientation_measurement_rate_[ESTIMATION_AXES];
        WindowedStats orientation_estimate_rate_[ESTIMATION_AXES];
        WindowedStats orientation_rate_t_scores_[ESTIMATION_AXES];

        Eigen::Vector3d prior_orientation_estimate_;
        double prior_estimate_time_s_{0.0};
        Eigen::Vector3d prior_orientation_measurement_;
        double prior_measurement_time_s_{0.0};

        // Whether the orientation offset has been set for future use.
        bool orientation_offset_has_been_set_[ESTIMATION_AXES] = {false, false, false};
        bool orientation_offset_is_updating_[ESTIMATION_AXES] = {false, false, false};

        // Debugging capabilities
        bool debug_{false};
        bool verbose_{false};
};


}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_BIAS_ESTIMATOR_H