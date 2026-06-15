/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ukf.h"
#include "robot_localization/filter_common.h"

#include <angles/angles.h>
#include <assert.h>
#include <Eigen/Cholesky>

#include <cmath>
#include <vector>


namespace RobotLocalization
{
  Ukf::Ukf(std::vector<double> args) :
    FilterBase(),  // Must initialize filter base!
    uncorrected_(true)
  {
    assert(args.size() == 3);

    double alpha = args[0];
    double kappa = args[1];
    double beta = args[2];

    size_t sigmaCount = (STATE_SIZE << 1) + 1;
    sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

    // Prepare constants
    lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;

    stateWeights_.resize(sigmaCount);
    covarWeights_.resize(sigmaCount);

    stateWeights_[0] = lambda_ / (STATE_SIZE + lambda_);
    covarWeights_[0] = stateWeights_[0] + (1 - (alpha * alpha) + beta);
    sigmaPoints_[0].setZero();
    for (size_t i = 1; i < sigmaCount; ++i)
    {
      sigmaPoints_[i].setZero();
      stateWeights_[i] =  1 / (2 * (STATE_SIZE + lambda_));
      covarWeights_[i] = stateWeights_[i];
    }
  }

  Ukf::~Ukf()
  {
  }

  void Ukf::prepareCorrect(const Measurement &measurement, const std::vector<size_t> &updateIndices,
                           Eigen::VectorXd &innovationSubset, Eigen::MatrixXd &measurementCovarianceSubset,
                           Eigen::MatrixXd &kalmanGainSubset, Eigen::MatrixXd &invInnovCov,
                           Eigen::MatrixXd &predictedMeasCovar)
  {
    // In our implementation, it may be that after we call predict once, we call correct
    // several times in succession (multiple measurements with different time stamps). In
    // that event, the sigma points need to be updated to reflect the current state.
    // Throughout prediction and correction, we attempt to maximize efficiency in Eigen.
    if (!uncorrected_)
    {
      generateSigmaPoints();
    }

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector

    FB_DEBUG("Update indices are:\n" << updateIndices << "\n");

    size_t updateSize = updateIndices.size();

    // Now set up the relevant matrices
    Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                        // z
    Eigen::MatrixXd stateToMeasurementSubset(updateSize, STATE_SIZE);     // H
    Eigen::VectorXd predictedMeasurement(updateSize);
    Eigen::VectorXd sigmaDiff(updateSize);
    Eigen::MatrixXd crossCovar(STATE_SIZE, updateSize);

    std::vector<Eigen::VectorXd> sigmaPointMeasurements(sigmaPoints_.size(), Eigen::VectorXd(updateSize));

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();
    predictedMeasurement.setZero();
    predictedMeasCovar.setZero();
    crossCovar.setZero();
    // Resize since the size may have been unknown generically for the initialization call
    predictedMeasCovar.resize(updateSize, updateSize);

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < updateSize; ++i)
    {
      measurementSubset(i) = measurement.measurement_(updateIndices[i]);
      stateSubset(i) = state_(updateIndices[i]);

      for (size_t j = 0; j < updateSize; ++j)
      {
        measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if (measurementCovarianceSubset(i, i) < 0)
      {
        FB_VERBOSE("WARNING: Negative covariance for index " << i <<
                 " of measurement (value is" << measurementCovarianceSubset(i, i) <<
                 "). Using absolute value...\n");

        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if (measurementCovarianceSubset(i, i) < 1e-9)
      {
        measurementCovarianceSubset(i, i) = 1e-9;

        FB_VERBOSE("WARNING: measurement had very small error covariance for index " <<
                 updateIndices[i] <<
                 ". Adding some noise to maintain filter stability.\n");
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (size_t i = 0; i < updateSize; ++i)
    {
      stateToMeasurementSubset(i, updateIndices[i]) = 1;
    }

    FB_VERBOSE("Current state subset is:\n" << stateSubset <<
             "\nMeasurement subset is:\n" << measurementSubset <<
             "\nMeasurement covariance subset is:\n" << measurementCovarianceSubset <<
             "\nState-to-measurement subset is:\n" << stateToMeasurementSubset << "\n");

    double roll_sum_x {};
    double roll_sum_y {};
    double pitch_sum_x {};
    double pitch_sum_y {};
    double yaw_sum_x {};
    double yaw_sum_y {};

    // (1) Generate sigma points, use them to generate a predicted measurement
    for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
    {
      sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * sigmaPoints_[sigmaInd];
      predictedMeasurement.noalias() += stateWeights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];

      // Euler angle averaging requires special care
      for (size_t i = 0; i < updateSize; ++i)
      {
        if (updateIndices[i] == StateMemberRoll)
        {
          roll_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPointMeasurements[sigmaInd](i));
          roll_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPointMeasurements[sigmaInd](i));
        }
        else if (updateIndices[i] == StateMemberPitch)
        {
          pitch_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPointMeasurements[sigmaInd](i));
          pitch_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPointMeasurements[sigmaInd](i));
        }
        else if (updateIndices[i] == StateMemberYaw)
        {
          yaw_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPointMeasurements[sigmaInd](i));
          yaw_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPointMeasurements[sigmaInd](i));
        }
      }
    }

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll)
      {
        predictedMeasurement(i) = ::atan2(roll_sum_y, roll_sum_x);
      }
      else if (updateIndices[i] == StateMemberPitch)
      {
        predictedMeasurement(i) = ::atan2(pitch_sum_y, pitch_sum_x);
      }
      else if (updateIndices[i] == StateMemberYaw)
      {
        predictedMeasurement(i) = ::atan2(yaw_sum_y, yaw_sum_x);
      }
    }

    // (2) Use the sigma point measurements and predicted measurement to compute a predicted
    // measurement covariance matrix P_zz and a state/measurement cross-covariance matrix P_xz.
    for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
    {
      sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;
      Eigen::VectorXd sigmaStateDiff = sigmaPoints_[sigmaInd] - state_;

      for (size_t i = 0; i < updateSize; ++i)
      {
        if (updateIndices[i] == StateMemberRoll ||
            updateIndices[i] == StateMemberPitch ||
            updateIndices[i] == StateMemberYaw)
        {
          sigmaDiff(i) = angles::normalize_angle(sigmaDiff(i));
          sigmaStateDiff(i) = angles::normalize_angle(sigmaStateDiff(i));
        }
      }

      predictedMeasCovar.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
      crossCovar.noalias() += covarWeights_[sigmaInd] * (sigmaStateDiff * sigmaDiff.transpose());
    }

    // (3) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_xz * (P_zz + R)^-1
    invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();
    kalmanGainSubset = crossCovar * invInnovCov;

    // (4) Apply the gain to the difference between the actual and predicted measurements: x = x + K(z - z_hat)
    innovationSubset = (measurementSubset - predictedMeasurement);

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll  ||
          updateIndices[i] == StateMemberPitch ||
          updateIndices[i] == StateMemberYaw)
      {
        innovationSubset(i) = angles::normalize_angle(innovationSubset(i));
      }
    }

    FB_DEBUG("Cross covariance is:\n" << crossCovar << "\n");
  }

  void Ukf::correct(const Measurement &measurement)
  {
    std::ostringstream sstream;
    sstream << std::setprecision(16) << measurement.time_;
    std::string timeAsString = sstream.str();
    FB_DEBUG("---------------------- Ukf::correct ----------------------\n" <<
             "State is:\n" << state_ <<
             "\nTopic is:\n" << measurement.topicName_ <<
             "\nTime is:\n" << timeAsString <<
             "\nMeasurement is:\n" << measurement.measurement_ <<
             "\nMeasurement covariance is:\n" << measurement.covariance_ << "\n");
    
    // Prepare the correction for integration into the state and covariance
    std::vector<size_t> updateIndices;
    getUpdateIndices(measurement, updateIndices);
    size_t updateSize = updateIndices.size();
    Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx
    Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, updateSize);             // K
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
    Eigen::MatrixXd predictedMeasCovar(updateSize, updateSize);
    Eigen::MatrixXd invInnovCov;  // TODO - DETERMINE THE SIZE
    innovationSubset.setZero();
    kalmanGainSubset.setZero();
    predictedMeasCovar.setZero();
    measurementCovarianceSubset.setZero();
    prepareCorrect(measurement, updateIndices, innovationSubset, measurementCovarianceSubset,
                   kalmanGainSubset, invInnovCov, predictedMeasCovar);

    if (measurement.isSpeedDependentHeadingGainEnabled_)
    {
      const double lowSpeed = measurement.headingGainLowSpeedMps_;
      const double highSpeed = measurement.headingGainHighSpeedMps_;
      const double lowSpeedGain = std::fmin(1.0, std::fmax(0.0, measurement.headingGainAtLowSpeed_));
      const double speed = std::hypot(state_(StateMemberVx), state_(StateMemberVy));
      double headingGainScale = 1.0;

      if (highSpeed > lowSpeed)
      {
        const double speedAlpha = std::fmin(1.0, std::fmax(0.0, (speed - lowSpeed) / (highSpeed - lowSpeed)));
        headingGainScale = lowSpeedGain + (1.0 - lowSpeedGain) * speedAlpha;
      }
      else if (speed <= lowSpeed)
      {
        headingGainScale = lowSpeedGain;
      }

      kalmanGainSubset.row(StateMemberYaw) *= headingGainScale;
      kalmanGainSubset.row(StateMemberVyaw) *= headingGainScale;
    }

    double sqMahalanobis = getSquaredMahalanobisDistance(innovationSubset, invInnovCov);
    FB_DEBUG("Squared Mahalanobis is: " << sqMahalanobis << "\n" <<
              "Threshold is: " << measurement.mahalanobisThresh_*measurement.mahalanobisThresh_ << "\n" <<
              "Innovation is: " << innovationSubset << "\n" <<
              "Innovation covariance is:\n" << invInnovCov << "\n");
    if ( measurement.publishMahalanobisDistance_)
    {
     //  Useful if inspecting a measurement from a particular source. Or on a specific dimension.
      std::map<std::string, double>& mDistMap = measurementMapPeriodicMaxSquaredMahalanobisDistance_;
      if(auto search = mDistMap.find(measurement.topicName_); search == mDistMap.end())
      {
        mDistMap[measurement.topicName_] = sqMahalanobis;
      }
      double &mDistReference = mDistMap.at(measurement.topicName_);
      // we filter for the max per each periodic update
      mDistReference = std::max(mDistReference, sqMahalanobis);
    }
    // (5) Check Mahalanobis distance of innovation
    if (checkMahalanobisThreshold(sqMahalanobis, measurement.mahalanobisThresh_))
    {
      state_.noalias() += kalmanGainSubset * innovationSubset;

      // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
      estimateErrorCovariance_.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

      wrapStateAngles();

      // Mark that we need to re-compute sigma points for successive corrections
      uncorrected_ = false;

      FB_DEBUG("Predicated measurement covariance is:\n" << predictedMeasCovar <<
               "\nKalman gain subset is:\n" << kalmanGainSubset <<
               "\nInnovation:\n" << innovationSubset <<
               "\nCorrected full state is:\n" << state_ <<
               "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
               "\n\n---------------------- /Ukf::correct ----------------------\n");
    }
    else if (saveRejectedMeasurementTopics_)
    {
      rejectedMeasurementTopics_.push_back(measurement.topicName_);
    }
  }

  void Ukf::predict(const double referenceTime, const double delta)
  {
    std::ostringstream sstream;
    sstream << std::setprecision(16) << referenceTime;
    std::string timeAsString = sstream.str();
    FB_DEBUG("---------------------- Ukf::predict ----------------------\n" <<
             "delta is " << delta <<
             "end time is " << timeAsString << "\n" <<
             "\nstate is " << state_ << "\n");

    prepareControl(referenceTime, delta);

    generateSigmaPoints();

    double roll_sum_x {};
    double roll_sum_y {};
    double pitch_sum_x {};
    double pitch_sum_y {};
    double yaw_sum_x {};
    double yaw_sum_y {};

    // Sum the weighted sigma points to generate a new state prediction
    state_.setZero();
    for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
    {
      // Apply the state transition function to this sigma point
      projectSigmaPoint(sigmaPoints_[sigmaInd], delta);
      state_.noalias() += stateWeights_[sigmaInd] * sigmaPoints_[sigmaInd];

      // Euler angle averaging requires special care
      roll_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPoints_[sigmaInd](StateMemberRoll));
      roll_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPoints_[sigmaInd](StateMemberRoll));
      pitch_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPoints_[sigmaInd](StateMemberPitch));
      pitch_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPoints_[sigmaInd](StateMemberPitch));
      yaw_sum_x += stateWeights_[sigmaInd] * ::cos(sigmaPoints_[sigmaInd](StateMemberYaw));
      yaw_sum_y += stateWeights_[sigmaInd] * ::sin(sigmaPoints_[sigmaInd](StateMemberYaw));
    }

    // Recover average Euler angles
    state_(StateMemberRoll) = ::atan2(roll_sum_y, roll_sum_x);
    state_(StateMemberPitch) = ::atan2(pitch_sum_y, pitch_sum_x);
    state_(StateMemberYaw) = ::atan2(yaw_sum_y, yaw_sum_x);

    // Now use the sigma points and the predicted state to compute a predicted covariance
    estimateErrorCovariance_.setZero();
    Eigen::VectorXd sigmaDiff(STATE_SIZE);
    for (size_t sigmaInd = 0; sigmaInd < sigmaPoints_.size(); ++sigmaInd)
    {
      sigmaDiff = (sigmaPoints_[sigmaInd] - state_);

      sigmaDiff(StateMemberRoll) = angles::normalize_angle(sigmaDiff(StateMemberRoll));
      sigmaDiff(StateMemberPitch) = angles::normalize_angle(sigmaDiff(StateMemberPitch));
      sigmaDiff(StateMemberYaw) = angles::normalize_angle(sigmaDiff(StateMemberYaw));

      estimateErrorCovariance_.noalias() += covarWeights_[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
    }

    // Not strictly in the theoretical UKF formulation, but necessary here
    // to ensure that we actually incorporate the processNoiseCovariance_
    Eigen::MatrixXd *processNoiseCovariance = &processNoiseCovariance_;

    if (useDynamicProcessNoiseCovariance_)
    {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }

    estimateErrorCovariance_.noalias() += delta * (*processNoiseCovariance);

    // Keep the angles bounded
    wrapStateAngles();

    // Mark that we can keep these sigma points
    uncorrected_ = true;

    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\nPredicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /Ukf::predict ----------------------\n");
  }

  void Ukf::generateSigmaPoints()
  {
    // Take the square root of a small fraction of the estimateErrorCovariance_ using LL' decomposition
    weightedCovarSqrt_ = ((static_cast<double>(STATE_SIZE) + lambda_) * estimateErrorCovariance_).llt().matrixL();

    // Compute sigma points

    // First sigma point is the current state
    sigmaPoints_[0] = state_;

    // Next STATE_SIZE sigma points are state + weightedCovarSqrt_[ith column]
    // STATE_SIZE sigma points after that are state - weightedCovarSqrt_[ith column]
    for (size_t sigmaInd = 0; sigmaInd < STATE_SIZE; ++sigmaInd)
    {
      sigmaPoints_[sigmaInd + 1] = state_ + weightedCovarSqrt_.col(sigmaInd);
      sigmaPoints_[sigmaInd + 1 + STATE_SIZE] = state_ - weightedCovarSqrt_.col(sigmaInd);
    }
  }

  void Ukf::projectSigmaPoint(Eigen::VectorXd& sigmaPoint, double delta)
  {
    double roll = sigmaPoint(StateMemberRoll);
    double pitch = sigmaPoint(StateMemberPitch);
    double yaw = sigmaPoint(StateMemberYaw);

    // We'll need these trig calculations a lot.
    double sp = ::sin(pitch);
    double cp = ::cos(pitch);
    double cpi = 1.0 / cp;
    double tp = sp * cpi;

    double sr = ::sin(roll);
    double cr = ::cos(roll);

    double sy = ::sin(yaw);
    double cy = ::cos(yaw);

    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cr * delta;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = -sr * delta;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    sigmaPoint.applyOnTheLeft(transferFunction_);
  }
}  // namespace RobotLocalization
