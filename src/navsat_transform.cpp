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

#include "robot_localization/navsat_transform.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/filter_utilities.h"
#include "robot_localization/navsat_conversions.h"
#include "robot_localization/ros_filter_utilities.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcException.h>

#include <string>

namespace RobotLocalization
{
  NavSatTransform::NavSatTransform(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    use_nav_pvt_(false),
    ned_to_enu_nav_pvt_(false),
    broadcast_cartesian_transform_(false),
    broadcast_cartesian_transform_as_parent_frame_(false),
    gps_updated_(false),
    has_transform_gps_(false),
    has_transform_imu_(false),
    has_transform_odom_(false),
    odom_updated_(false),
    publish_gps_(false),
    transform_good_(false),
    use_manual_datum_(false),
    use_odometry_yaw_(false),
    use_local_cartesian_(false),
    publish_transform_(false),
    zero_altitude_(false),
    current_good_gps_count_(0),
    current_delayed_gps_count_(0),
    magnetic_declination_(0.0),
    ins_timeout_ms_(static_cast<uint32_t>(2.5 * 1000.0 / 30.0)),  // Set for 30Hz and 1 missed message
    yaw_offset_(0.0),
    base_link_frame_id_("base_link"),
    gps_frame_id_(""),
    utm_zone_(0),
    world_frame_id_("odom"),
    transform_timeout_(ros::Duration(0)),
    tf_listener_(tf_buffer_),
    tf_time_offset_(ros::Duration(0))
  {
    ROS_INFO("Waiting for valid clock time...");
    ros::Time::waitForValid();
    ROS_INFO("Valid clock time received. Starting node.");

    latest_cartesian_covariance_.resize(POSE_SIZE, POSE_SIZE);
    latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);

    double frequency;
    double delay = 0.0;
    double transform_timeout = 0.0;

    // Load the parameters we need
    nh_priv.param("use_nav_pvt", use_nav_pvt_, false);  // By default - don't use this
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_cartesian_transform", broadcast_cartesian_transform_, false);
    nh_priv.param("broadcast_cartesian_transform_as_parent_frame",
                  broadcast_cartesian_transform_as_parent_frame_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, false);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("wait_for_datum", use_manual_datum_, false);
    nh_priv.param("use_local_cartesian", use_local_cartesian_, false);
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.param("delay", delay, 0.0);
    nh_priv.param("origin_measurement_delay", origin_measurement_delay_, 0);
    nh_priv.param("origin_measurement_qty_to_avg", origin_measurement_qty_to_avg_, 1);
    nh_priv.param("transform_timeout", transform_timeout, 0.0);
    nh_priv.param("cartesian_frame_id", cartesian_frame_id_, std::string(use_local_cartesian_ ? "local_enu" : "utm"));
    transform_timeout_.fromSec(transform_timeout);

    // Check for deprecated parameters
    if (nh_priv.getParam("broadcast_utm_transform", broadcast_cartesian_transform_))
    {
      ROS_WARN("navsat_transform, Parameter 'broadcast_utm_transform' has been deprecated. Please use"
               "'broadcast_cartesian_transform' instead.");
    }
    if (nh_priv.getParam("broadcast_utm_transform_as_parent_frame", broadcast_cartesian_transform_as_parent_frame_))
    {
      ROS_WARN("navsat_transform, Parameter 'broadcast_utm_transform_as_parent_frame' has been deprecated. Please use"
               "'broadcast_cartesian_transform_as_parent_frame' instead.");
    }

    // Check if tf warnings should be suppressed
    nh.getParam("/silent_tf_failure", tf_silent_failure_);

    // Subscribe to the messages and services we need
    datum_srv_ = nh.advertiseService("datum", &NavSatTransform::datumCallback, this);

    to_ll_srv_ = nh.advertiseService("toLL", &NavSatTransform::toLLCallback, this);
    from_ll_srv_ = nh.advertiseService("fromLL", &NavSatTransform::fromLLCallback, this);
    set_utm_zone_srv_ = nh.advertiseService("setUTMZone", &NavSatTransform::setUTMZoneCallback, this);

    if (use_manual_datum_ && nh_priv.hasParam("datum"))
    {
      XmlRpc::XmlRpcValue datum_config;

      try
      {
        double datum_lat;
        double datum_lon;
        double datum_yaw;

        nh_priv.getParam("datum", datum_config);

        // Handle datum specification. Users should always specify a baseLinkFrameId_ in the
        // datum config, but we had a release where it wasn't used, so we'll maintain compatibility.
        ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datum_config.size() >= 3);

        if (datum_config.size() > 3)
        {
          ROS_WARN_STREAM("Deprecated datum parameter configuration detected. Only the first three parameters "
              "(latitude, longitude, yaw) will be used. frame_ids will be derived from odometry and navsat inputs.");
        }

        std::ostringstream ostr;
        ostr << std::setprecision(20) << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
        std::istringstream istr(ostr.str());
        istr >> datum_lat >> datum_lon >> datum_yaw;

        // Try to resolve tf_prefix
        std::string tf_prefix = "";
        std::string tf_prefix_path = "";
        if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
        {
          nh_priv.getParam(tf_prefix_path, tf_prefix);
        }

        // Append the tf prefix in a tf2-friendly manner
        FilterUtilities::appendPrefix(tf_prefix, world_frame_id_);
        FilterUtilities::appendPrefix(tf_prefix, base_link_frame_id_);

        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = datum_lat;
        request.geo_pose.position.longitude = datum_lon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datum_yaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datum_config.getType() << ")");
      }
    }

    if(use_nav_pvt_)
    {
      gps_nav_pvt_sub_ = nh.subscribe("ublox/ubx_nav_pvt", 1, &NavSatTransform::gpsNavPVTCallback, this);
      nh_priv.param("gps_frame", gps_frame_id_, std::string(""));  // Default to none if not set
      nh_priv.param("world_frame", world_frame_id_, std::string(""));  // Default to none if not set
      nh_priv.param("base_link_frame", base_link_frame_id_, std::string(""));  // Default to none if not set
      // Whether we're publshing the world_frame->base_link_frame transform
      nh_priv.param("publish_tf", publish_transform_, false);
      // NED to ENU transform
      nh_priv.param("ned_to_enu_nav_pvt", ned_to_enu_nav_pvt_, false);
      // Transform future dating
      double offsetTmp;
      nh_priv.param("transform_time_offset", offsetTmp, 0.0);
      tf_time_offset_.fromSec(offsetTmp);
      // Set up the second subscription for INS data
      gps_esf_ins_sub_ = nh.subscribe("ublox/ubx_esf_ins", 1, &NavSatTransform::gpsEsfINSCallback, this);
      int ins_timeout_ms_int = static_cast<int>(ins_timeout_ms_);
      nh_priv.param("ins_timeout_ms", ins_timeout_ms_int, ins_timeout_ms_int);
      ins_timeout_ms_ = static_cast<uint32_t>(ins_timeout_ms_int);
    }
    else
    {
      // Only needed if not using NavPVT as that message substitutes for both of these
      odom_sub_        = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
      gps_sub_         = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);
    }

    if (!use_odometry_yaw_ && !use_manual_datum_)
    {
      imu_sub_ = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);
    }

    gps_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);

    if (publish_gps_)
    {
      filtered_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps/filtered", 10);
    }

    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    ros::Duration start_delay(delay);
    start_delay.sleep();

    periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &NavSatTransform::periodicUpdate, this);
  }

  NavSatTransform::~NavSatTransform()
  {
  }

//  void NavSatTransform::run()
  void NavSatTransform::periodicUpdate(const ros::TimerEvent& event)
  {
    if (!use_nav_pvt_)
    {
      if (!transform_good_)
      {
        computeTransform();

        if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_)
        {
          // Once we have the transform, we don't need the IMU
          imu_sub_.shutdown();
        }
      }
      else
      {
        nav_msgs::Odometry gps_odom;
        if (prepareGpsOdometry(gps_odom))
        {
          gps_odom_pub_.publish(gps_odom);
        }

        if (publish_gps_)
        {
          sensor_msgs::NavSatFix odom_gps;
          if (prepareFilteredGps(odom_gps))
          {
            filtered_gps_pub_.publish(odom_gps);
          }
        }
      }
    }
  }

  void NavSatTransform::computeTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->cartesian_frame transform before
    // 2. We've received the data we need
    if (!transform_good_ &&
        has_transform_odom_ &&
        has_transform_gps_ &&
        has_transform_imu_)
    {
      // The cartesian pose we have is given at the location of the GPS sensor on the robot. We need to get the
      // cartesian pose of the robot's origin.
      tf2::Transform transform_cartesian_pose_corrected;
      if (!use_manual_datum_)
      {
        getRobotOriginCartesianPose(transform_cartesian_pose_, transform_cartesian_pose_corrected, ros::Time(0));
      }
      else
      {
        transform_cartesian_pose_corrected = transform_cartesian_pose_;
      }

      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf2::Matrix3x3 mat(transform_orientation_);

      // Convert to RPY
      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);

      /* The IMU's heading was likely originally reported w.r.t. magnetic north.
       * However, all the nodes in robot_localization assume that orientation data,
       * including that reported by IMUs, is reported in an ENU frame, with a 0 yaw
       * value being reported when facing east and increasing counter-clockwise (i.e.,
       * towards north). To make the world frame ENU aligned, where X is east
       * and Y is north, we have to take into account three additional considerations:
       *   1. The IMU may have its non-ENU frame data transformed to ENU, but there's
       *      a possibility that its data has not been corrected for magnetic
       *      declination. We need to account for this. A positive magnetic
       *      declination is counter-clockwise in an ENU frame. Therefore, if
       *      we have a magnetic declination of N radians, then when the sensor
       *      is facing a heading of N, it reports 0. Therefore, we need to add
       *      the declination angle.
       *   2. To account for any other offsets that may not be accounted for by the
       *      IMU driver or any interim processing node, we expose a yaw offset that
       *      lets users work with navsat_transform_node.
       *   3. UTM grid isn't aligned with True East\North. To account for the difference
       *      we need to add meridian convergence angle when using UTM. This value will be
       *      0.0 when use_local_cartesian is TRUE.
       */
      imu_yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magnetic_declination_ <<
                      ", user-specified offset of " << yaw_offset_ <<
                      " and meridian convergence of " << utm_meridian_convergence_ << "." <<
                      " Transform heading factor is now " << imu_yaw);

      // Convert to tf-friendly structures
      tf2::Quaternion imu_quat;
      imu_quat.setRPY(0.0, 0.0, imu_yaw);

      // The transform order will be orig_odom_pos * orig_cartesian_pos_inverse * cur_cartesian_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf2::Transform cartesian_pose_with_orientation;
      cartesian_pose_with_orientation.setOrigin(transform_cartesian_pose_corrected.getOrigin());
      cartesian_pose_with_orientation.setRotation(imu_quat);

      // Remove roll and pitch from odometry pose
      // Must be done because roll and pitch is removed from cartesian_pose_with_orientation
      double odom_roll, odom_pitch, odom_yaw;
      tf2::Matrix3x3(transform_world_pose_.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
      tf2::Quaternion odom_quat;
      odom_quat.setRPY(0.0, 0.0, odom_yaw);
      tf2::Transform transform_world_pose_yaw_only(transform_world_pose_);
      transform_world_pose_yaw_only.setRotation(odom_quat);

      cartesian_world_transform_.mult(transform_world_pose_yaw_only, cartesian_pose_with_orientation.inverse());

      cartesian_world_trans_inverse_ = cartesian_world_transform_.inverse();

      ROS_INFO_STREAM("Transform world frame pose is: " << transform_world_pose_);
      ROS_INFO_STREAM("World frame->cartesian transform is " << cartesian_world_transform_);

      transform_good_ = true;

      // Send out the (static) Cartesian transform in case anyone else would like to use it.
      if (broadcast_cartesian_transform_)
      {
        geometry_msgs::TransformStamped cartesian_transform_stamped;
        cartesian_transform_stamped.header.stamp = ros::Time::now();
        cartesian_transform_stamped.header.frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                       cartesian_frame_id_ : world_frame_id_);
        cartesian_transform_stamped.child_frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                      world_frame_id_ : cartesian_frame_id_);
        cartesian_transform_stamped.transform = (broadcast_cartesian_transform_as_parent_frame_ ?
                                             tf2::toMsg(cartesian_world_trans_inverse_) :
                                             tf2::toMsg(cartesian_world_transform_));
        cartesian_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                           0.0 : cartesian_transform_stamped.transform.translation.z);
        cartesian_broadcaster_.sendTransform(cartesian_transform_stamped);
      }
    }
  }

  bool NavSatTransform::datumCallback(robot_localization::SetDatum::Request& request,
                                      robot_localization::SetDatum::Response&)
  {
    // If we get a service call with a manual datum, even if we already computed the transform using the robot's
    // initial pose, then we want to assume that we are using a datum from now on, and we want other methods to
    // not attempt to transform the values we are specifying here.
    use_manual_datum_ = true;

    transform_good_ = false;

    sensor_msgs::NavSatFix *fix = new sensor_msgs::NavSatFix();
    fix->latitude = request.geo_pose.position.latitude;
    fix->longitude = request.geo_pose.position.longitude;
    fix->altitude = request.geo_pose.position.altitude;
    fix->header.stamp = ros::Time::now();
    fix->position_covariance[0] = 0.1;
    fix->position_covariance[4] = 0.1;
    fix->position_covariance[8] = 0.1;
    fix->position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;
    sensor_msgs::NavSatFixConstPtr fix_ptr(fix);
    setTransformGps(fix_ptr);

    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 0;
    odom->pose.pose.position.y = 0;
    odom->pose.pose.position.z = 0;
    odom->header.frame_id = world_frame_id_;
    odom->child_frame_id = base_link_frame_id_;
    nav_msgs::OdometryConstPtr odom_ptr(odom);
    setTransformOdometry(odom_ptr);

    sensor_msgs::Imu *imu = new sensor_msgs::Imu();
    imu->orientation = request.geo_pose.orientation;
    imu->header.frame_id = base_link_frame_id_;
    sensor_msgs::ImuConstPtr imu_ptr(imu);
    imuCallback(imu_ptr);

    return true;
  }

  bool NavSatTransform::toLLCallback(robot_localization::ToLL::Request& request,
                                     robot_localization::ToLL::Response& response)
  {
    if (!transform_good_)
    {
      ROS_ERROR("No transform available (yet)");
      return false;
    }
    tf2::Vector3 point;
    tf2::fromMsg(request.map_point, point);
    mapToLL(point, response.ll_point.latitude, response.ll_point.longitude, response.ll_point.altitude);

    return true;
  }

  bool NavSatTransform::fromLLCallback(robot_localization::FromLL::Request& request,
                                       robot_localization::FromLL::Response& response)
  {
    double altitude = request.ll_point.altitude;
    double longitude = request.ll_point.longitude;
    double latitude = request.ll_point.latitude;

    tf2::Transform cartesian_pose;

    double cartesian_x;
    double cartesian_y;
    double cartesian_z;

    if (use_local_cartesian_)
    {
      gps_local_cartesian_.Forward(latitude, longitude, altitude, cartesian_x, cartesian_y, cartesian_z);
    }
    else
    {
      int zone_tmp;
      bool nortp_tmp;
      try
      {
        GeographicLib::UTMUPS::Forward(latitude, longitude, zone_tmp, nortp_tmp, cartesian_x, cartesian_y, utm_zone_);
      }
      catch (const GeographicLib::GeographicErr& e)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
        return false;
      }
    }

    cartesian_pose.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, altitude));

    nav_msgs::Odometry gps_odom;

    if (!transform_good_)
    {
      ROS_ERROR("No transform available (yet)");
      return false;
    }

    response.map_point = cartesianToMap(cartesian_pose).pose.pose.position;

    return true;
  }

  bool NavSatTransform::setUTMZoneCallback(robot_localization::SetUTMZone::Request& request,
                                           robot_localization::SetUTMZone::Response& response)
  {
    double x_unused;
    double y_unused;
    int prec_unused;
    GeographicLib::MGRS::Reverse(request.utm_zone, utm_zone_, northp_, x_unused, y_unused, prec_unused, true);
    ROS_INFO("UTM zone set to %d %s", utm_zone_, northp_ ? "north" : "south");
    return true;
  }

  nav_msgs::Odometry NavSatTransform::cartesianToMap(const tf2::Transform& cartesian_pose) const
  {
    nav_msgs::Odometry gps_odom{};

    tf2::Transform transformed_cartesian_gps{};

    transformed_cartesian_gps.mult(cartesian_world_transform_, cartesian_pose);
    transformed_cartesian_gps.setRotation(tf2::Quaternion::getIdentity());

    // Set header information stamp because we would like to know the robot's position at that timestamp
    gps_odom.header.frame_id = world_frame_id_;
    gps_odom.header.stamp = gps_update_time_;

    // Now fill out the message. Set the orientation to the identity.
    tf2::toMsg(transformed_cartesian_gps, gps_odom.pose.pose);
    gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

    return gps_odom;
  }

  void NavSatTransform::mapToLL(const tf2::Vector3& point, double& latitude, double& longitude, double& altitude) const
  {
    tf2::Transform odom_as_cartesian{};

    tf2::Transform pose{};
    pose.setOrigin(point);
    pose.setRotation(tf2::Quaternion::getIdentity());

    odom_as_cartesian.mult(cartesian_world_trans_inverse_, pose);
    odom_as_cartesian.setRotation(tf2::Quaternion::getIdentity());

    if (use_local_cartesian_)
    {
      double altitude_tmp = 0.0;
      gps_local_cartesian_.Reverse(odom_as_cartesian.getOrigin().getX(),
                                   odom_as_cartesian.getOrigin().getY(),
                                   0.0,
                                   latitude,
                                   longitude,
                                   altitude_tmp);
      altitude = odom_as_cartesian.getOrigin().getZ();
    }
    else
    {
      GeographicLib::UTMUPS::Reverse(utm_zone_,
                                     northp_,
                                     odom_as_cartesian.getOrigin().getX(),
                                     odom_as_cartesian.getOrigin().getY(),
                                     latitude,
                                     longitude);
      altitude = odom_as_cartesian.getOrigin().getZ();
    }
  }

  void NavSatTransform::getRobotOriginCartesianPose(const tf2::Transform &gps_cartesian_pose,
                                                    tf2::Transform &robot_cartesian_pose,
                                                    const ros::Time &transform_time)
  {
    robot_cartesian_pose.setIdentity();

    // Get linear offset from origin for the GPS
    tf2::Transform offset;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 ros::Duration(transform_timeout_),
                                                                 offset,
                                                                 tf_silent_failure_);

    if (can_transform)
    {
      // Get the orientation we'll use for our Cartesian->world transform
      tf2::Quaternion cartesian_orientation = transform_orientation_;
      tf2::Matrix3x3 mat(cartesian_orientation);

      // Add the offsets
      double roll;
      double pitch;
      double yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
      cartesian_orientation.setRPY(roll, pitch, yaw);

      // Rotate the GPS linear offset by the orientation
      // Zero out the orientation, because the GPS orientation is meaningless, and if it's non-zero, it will make the
      // the computation of robot_cartesian_pose erroneous.
      offset.setOrigin(tf2::quatRotate(cartesian_orientation, offset.getOrigin()));
      offset.setRotation(tf2::Quaternion::getIdentity());

      // Update the initial pose
      robot_cartesian_pose = offset.inverse() * gps_cartesian_pose;
    }
    else
    {
      if (gps_frame_id_ != "")
      {
        ROS_WARN_STREAM_ONCE("Unable to obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
          " transform. Will assume navsat device is mounted at robot's origin");
      }

      robot_cartesian_pose = gps_cartesian_pose;
    }
  }

  bool NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time)
  {
    robot_odom_pose.setIdentity();

    // Remove the offset from base_link
    tf2::Transform gps_offset_rotated;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 transform_timeout_,
                                                                 gps_offset_rotated,
                                                                 tf_silent_failure_);

    if (can_transform)
    {
      tf2::Transform robot_orientation;
      can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                              world_frame_id_,
                                                              base_link_frame_id_,
                                                              transform_time,
                                                              transform_timeout_,
                                                              robot_orientation,
                                                              tf_silent_failure_);

      if (can_transform)
      {
        // Zero out rotation because we don't care about the orientation of the
        // GPS receiver relative to base_link
        gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
        gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
        robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << world_frame_id_ << "->" << base_link_frame_id_ <<
          " transform. Will not remove offset of navsat device from robot's origin.");
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
        " transform. Will not remove offset of navsat device from robot's origin.");
    }
    return can_transform;
  }

  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    gps_frame_id_ = msg->header.frame_id;

    if (gps_frame_id_.empty())
    {
      ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
        "origin.");
    }

    // Make sure the GPS data is usable
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                     !std::isnan(msg->altitude) &&
                     !std::isnan(msg->latitude) &&
                     !std::isnan(msg->longitude));

    if (good_gps)
    {
      sensor_msgs::NavSatFix gps_meas;
      if (transform_good_ || use_manual_datum_)
      {
        ROS_INFO_STREAM_ONCE("Begun using GPS fix data for cartesian coordinates.");
        gps_meas = *msg;
      }
      else if (!has_transform_gps_)
      {
        // check for `has_transform_gps_` because we want to set the gps origin once
        if(++current_delayed_gps_count_ < origin_measurement_delay_)
        {
          return;
        }

        origin_llh_[0].push_back(msg->latitude);
        origin_llh_[1].push_back(msg->longitude);
        origin_llh_[2].push_back(msg->altitude);

        if (++current_good_gps_count_ < origin_measurement_qty_to_avg_ )
        {
          return;
        }
        // we now have enough good gps measurements to calculate the origin
        sensor_msgs::NavSatFix gps_centroid = *msg;
        double n = origin_llh_[0].size();
        gps_centroid.latitude = (1.0/n)*std::accumulate(origin_llh_[0].begin(), origin_llh_[0].end(), 0.0);
        gps_centroid.longitude = (1.0/n)*std::accumulate(origin_llh_[1].begin(), origin_llh_[1].end(), 0.0);
        gps_centroid.altitude = (1.0/n)*std::accumulate(origin_llh_[2].begin(), origin_llh_[2].end(), 0.0);
        gps_meas = gps_centroid;
        // If we haven't computed the transform yet, then
        // store this message as the initial GPS data to use
        // first if already tells us -- !transform_good_ && !use_manual_datum_
        setTransformGps(boost::make_shared<sensor_msgs::NavSatFix>(gps_meas));
      }
      else
      {
        // !transform_good_ and has_transform_gps_
        ROS_WARN_STREAM("Missing transform_good_ yet has_transform_gps_.");
        return;
      }


      double cartesian_x = 0.0;
      double cartesian_y = 0.0;
      double cartesian_z = 0.0;
      if (use_local_cartesian_)
      {
        gps_local_cartesian_.Forward(gps_meas.latitude, gps_meas.longitude, gps_meas.altitude,
                                     cartesian_x, cartesian_y, cartesian_z);
      }
      else
      {
        // Transform to UTM using the fixed utm_zone_
        int zone_tmp;
        bool northp_tmp;
        try
        {
          GeographicLib::UTMUPS::Forward(gps_meas.latitude, gps_meas.longitude,
                                        zone_tmp, northp_tmp, cartesian_x, cartesian_y, utm_zone_);
        }
        catch (const GeographicLib::GeographicErr& e)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
          return;
        }
      }
      latest_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, gps_meas.altitude));
      latest_cartesian_covariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latest_cartesian_covariance_(i, j) = gps_meas.position_covariance[POSITION_SIZE * i + j];
        }
      }

      gps_update_time_ = gps_meas.header.stamp;
      gps_updated_ = true;
    }
    else if (!has_transform_gps_)
    {
      ROS_WARN_THROTTLE(15, "GNSS data used for origin is being reset due to a bad GNSS measurement.");
      // gps not good so we reset data used for origin used by geographic lib
      // check for has_transform_gps_ so we do not reset these variables after has_transform_gps_==true
      // - to avoid changing downstream gps/odometry solutions 
      current_good_gps_count_ = 0;
      current_delayed_gps_count_ = 0;
      origin_llh_[0].clear();
      origin_llh_[1].clear();
      origin_llh_[2].clear();
    }
  }

  void NavSatTransform::gpsEsfINSCallback(const ublox_msgs::EsfINSConstPtr& msg)
  {
    gps_esf_ins_ = *msg;
  }

  void NavSatTransform::gpsNavPVTCallback(const ublox_msgs::NavPVTConstPtr& msg)
  {
    // gps_frame_id_ set manually if using NavPVT
    if (gps_frame_id_.empty())
    {
      ROS_WARN_STREAM_ONCE("GPS frame ID not set for NavPVT. Will assume navsat device is mounted at robot's "
        "origin.");
    }

    // By the end of this function call, the following should be done:
    //  1) Publish from gps_odom_pub_ as the filter output (act as though this is the filter node)
    //  2) Publish from filtered_gps_pub_ (if set to true)
    // To do this, need to do the following:
    //  1) Capture the GNSS datum
    //  2) Assume odometry hasn't moved such that GNSS datum is at 0
    //  3) Capture orientation (fake publish the IMU message to capture this)
    //  4) Create necessary transforms
    // Make sure the GPS data is usable
    bool good_gps = (msg->fixType != ublox_msgs::NavPVT::FIX_TYPE_NO_FIX &&
                     msg->fixType != ublox_msgs::NavPVT::FIX_TYPE_TIME_ONLY &&
                     !std::isnan(msg->height) &&
                     !std::isnan(msg->lat) &&
                     !std::isnan(msg->lon));

    if (good_gps)
    {
      sensor_msgs::NavSatFix gps_meas;
      // Convert NavPVT time to ROS timestamp
      // The time in nanoseconds from the NavPVT message can be between -1e9 and 1e9
      //  The ros time uses only unsigned values, so a negative nano seconds must be
      //  converted to a positive value
      if (msg->nano < 0) {
        gps_meas.header.stamp.sec = uBloxTimeToUtcSeconds(msg) - 1;
        gps_meas.header.stamp.nsec = (uint32_t)(msg->nano + 1e9);
      }
      else {
        gps_meas.header.stamp.sec = uBloxTimeToUtcSeconds(msg);
        gps_meas.header.stamp.nsec = (uint32_t)(msg->nano);
      }
      // Handle covariance (ENU = hAcc, hAcc, vAcc) in meters^2
      gps_meas.position_covariance[0] = pow(msg->hAcc * 1e-3, 2);
      gps_meas.position_covariance[4] = pow(msg->hAcc * 1e-3, 2);
      gps_meas.position_covariance[8] = pow(msg->vAcc * 1e-3, 2);
      gps_meas.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      // Either status is GBAS or FIX (0). Since is good GPS, it's not NO_FIX
      gps_meas.status.status = ((msg->flags & ublox_msgs::NavPVT::FLAGS_DIFF_SOLN) != 0) ?
        sensor_msgs::NavSatStatus::STATUS_GBAS_FIX :
        sensor_msgs::NavSatStatus::STATUS_FIX;
      // Measurement is in gps_frame
      gps_meas.header.frame_id = gps_frame_id_;

      // Set declination
      magnetic_declination_ = msg->magDec * 1e-2 * PI / 180.0;

      if (transform_good_ || use_manual_datum_)
      {
        ROS_INFO_STREAM_ONCE("Begun using GPS Nav PVT data for cartesian coordinates, velocity, and heading.");
        gps_meas.latitude = static_cast<double>(msg->lat) * 1e-7;
        gps_meas.longitude = static_cast<double>(msg->lon) * 1e-7;
        gps_meas.altitude = static_cast<double>(msg->height) * 1e-3;
      }
      else if (!has_transform_gps_)
      {
        // check for `has_transform_gps_` because we want to set the gps origin once
        if(++current_delayed_gps_count_ < origin_measurement_delay_)
        {
          return;
        }

        origin_llh_[0].push_back(static_cast<double>(msg->lat) * 1e-7);  // Decimal degrees
        origin_llh_[1].push_back(static_cast<double>(msg->lon) * 1e-7);  // Decimal degrees
        origin_llh_[2].push_back(static_cast<double>(msg->height) * 1e-3);  // Transform to meters

        if (++current_good_gps_count_ < origin_measurement_qty_to_avg_ )
        {
          return;
        }
        // We now have enough good gps measurements to calculate the origin
        // Use the NavSatFix message since already set up for doubles
        double n = origin_llh_[0].size();
        gps_meas.latitude = (1.0/n)*std::accumulate(origin_llh_[0].begin(), origin_llh_[0].end(), 0.0);
        gps_meas.longitude = (1.0/n)*std::accumulate(origin_llh_[1].begin(), origin_llh_[1].end(), 0.0);
        gps_meas.altitude = (1.0/n)*std::accumulate(origin_llh_[2].begin(), origin_llh_[2].end(), 0.0);

        // If we haven't computed the transform yet, then
        // store this message as the initial GPS data to use
        // first if already tells us -- !transform_good_ && !use_manual_datum_
        // Have enough information to set a manual datum.
        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = gps_meas.latitude;
        request.geo_pose.position.longitude = gps_meas.longitude;
        request.geo_pose.position.altitude = gps_meas.altitude;
        tf2::Quaternion quat;
        double nav_pvt_heading_rad = msg->headVeh * 1e-5 * PI / 180.0;  // Vehicle heading
        double enu_heading_rad = nav_pvt_heading_rad;
        if(ned_to_enu_nav_pvt_)
        {
          enu_heading_rad = PI - nav_pvt_heading_rad;
        }
        quat.setRPY(0.0, 0.0, enu_heading_rad);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
        // Override this. We have the information necessary, so want to compute and use the typical methods.
        use_manual_datum_ = false;
      }
      else
      {
        // !transform_good_ and has_transform_gps_
        ROS_WARN_STREAM("Missing transform_good_ yet has_transform_gps_.");
        return;
      }

      double cartesian_x = 0.0;
      double cartesian_y = 0.0;
      double cartesian_z = 0.0;
      if (use_local_cartesian_)
      {
        gps_local_cartesian_.Forward(gps_meas.latitude, gps_meas.longitude, gps_meas.altitude,
                                     cartesian_x, cartesian_y, cartesian_z);
      }
      else
      {
        // Transform to UTM using the fixed utm_zone_
        int zone_tmp;
        bool northp_tmp;
        try
        {
          GeographicLib::UTMUPS::Forward(gps_meas.latitude, gps_meas.longitude,
                                        zone_tmp, northp_tmp, cartesian_x, cartesian_y, utm_zone_);
        }
        catch (const GeographicLib::GeographicErr& e)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, e.what());
          return;
        }
      }
      latest_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, gps_meas.altitude));
      latest_cartesian_covariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latest_cartesian_covariance_(i, j) = gps_meas.position_covariance[POSITION_SIZE * i + j];
        }
      }

      // State that GPS data was updated
      gps_update_time_ = gps_meas.header.stamp;
      gps_updated_ = true;

      // Similar to periodic update, but now all information comes in on 1 message, so do this on message reception
      if (!transform_good_)
      {
        computeTransform();

        if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_)
        {
          // Once we have the transform, we don't need the IMU
          imu_sub_.shutdown();
        }
      }
      else
      {
        nav_msgs::Odometry gps_odom;

        tf2::Quaternion orientation_quat;
        double nav_pvt_heading_rad = msg->headVeh * 1e-5 * PI / 180.0;  // Vehicle heading
        double enu_heading_rad = nav_pvt_heading_rad;
        if(ned_to_enu_nav_pvt_)
        {
          enu_heading_rad = PI - nav_pvt_heading_rad;
        }
        orientation_quat.setRPY(0.0, 0.0, enu_heading_rad);

        // Handle the transform if required
        if (publish_transform_)
        {
          // World (map) to Odom is a static transform
          static tf2_ros::StaticTransformBroadcaster odom_broadcaster;
          static_transform_stamped_map_odom_.header.stamp = gps_meas.header.stamp;
          static_transform_stamped_map_odom_.header.frame_id = "map";
          static_transform_stamped_map_odom_.child_frame_id = "odom";
          static_transform_stamped_map_odom_.transform.translation.x = 0;
          static_transform_stamped_map_odom_.transform.translation.y = 0;
          static_transform_stamped_map_odom_.transform.translation.z = 0;
          tf2::Quaternion quat;
          quat.setRPY(0, 0, 0);
          static_transform_stamped_map_odom_.transform.rotation.x = quat.x();
          static_transform_stamped_map_odom_.transform.rotation.y = quat.y();
          static_transform_stamped_map_odom_.transform.rotation.z = quat.z();
          static_transform_stamped_map_odom_.transform.rotation.w = quat.w();
          odom_broadcaster.sendTransform(static_transform_stamped_map_odom_);


          // Odom to world frame is effectively GPS sensor moved to robot base_link in map frame.
          // Can calculate this in cartesian frame and transform to world frame (map frame).
          tf2::Transform transform_cartesian_gps_pose;
          tf2::Transform transformed_cartesian_gps_pose;
          transform_cartesian_gps_pose.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->height));
          transform_cartesian_gps_pose.setRotation(tf2::Quaternion::getIdentity());
          getRobotOriginCartesianPose(transform_cartesian_gps_pose, transformed_cartesian_gps_pose, gps_odom.header.stamp);
          // Now transform the transformed_cartesian_gps_pose to map
          gps_odom = cartesianToMap(transformed_cartesian_gps_pose);
          // Orientation
          gps_odom.pose.pose.orientation = tf2::toMsg(orientation_quat);
          // Now, gps_odom is in the correct frame for the offsets and orientation.
          //  Use this information to create the odom to base_link transformation
          transform_stamped_odom_base_footprint_.header.frame_id = "odom";
          transform_stamped_odom_base_footprint_.header.stamp = gps_meas.header.stamp + tf_time_offset_;
          transform_stamped_odom_base_footprint_.child_frame_id = "base_footprint";

          transform_stamped_odom_base_footprint_.transform.translation.x = gps_odom.pose.pose.position.x;
          transform_stamped_odom_base_footprint_.transform.translation.y = gps_odom.pose.pose.position.y;
          transform_stamped_odom_base_footprint_.transform.translation.z = gps_odom.pose.pose.position.z;

          transform_stamped_odom_base_footprint_.transform.rotation.w = gps_odom.pose.pose.orientation.w;
          transform_stamped_odom_base_footprint_.transform.rotation.x = gps_odom.pose.pose.orientation.x;
          transform_stamped_odom_base_footprint_.transform.rotation.y = gps_odom.pose.pose.orientation.y;
          transform_stamped_odom_base_footprint_.transform.rotation.z = gps_odom.pose.pose.orientation.z;

          base_footprint_broadcaster_.sendTransform(transform_stamped_odom_base_footprint_);

        }

        if (prepareGpsOdometry(gps_odom))
        {
          // Include the rest of the information
          gps_odom.header.frame_id = world_frame_id_;
          gps_odom.child_frame_id = base_link_frame_id_;  // Typically handled by the filter
          
          // Check whether velocity should be negative in child frame
          if((abs(msg->heading - msg->headVeh) * 1e-5) > 90.0)
          {
            // Vehicle pointed in opposite direction than the vehicle speed - it's moving backwards
            gps_odom.twist.twist.linear.x = -msg->gSpeed * 1e-3;  // Ground speed in m/s
          }
          else
          {
            gps_odom.twist.twist.linear.x = msg->gSpeed * 1e-3;  // Ground speed in m/s
          }
          // If the efsINS message is up to date with valid angular rates, use those.
          // Unsigned int32s, so not using abs to avoid losing precision or causing a negative issue
          // If one has rolled over, then a single point will be zeroed, then go back to normal.
          uint32_t itow_diff = (msg->iTOW > gps_esf_ins_.iTOW) ?
            msg->iTOW - gps_esf_ins_.iTOW :
            gps_esf_ins_.iTOW - msg->iTOW;
          if(itow_diff < ins_timeout_ms_)
          {
            // X - yAngRate due to incorrect internal transform
            if((gps_esf_ins_.bitfield0 & ublox_msgs::EsfINS::BITFIELD0_X_ANG_RATE_VALID) > 0)
            {
              gps_odom.twist.twist.angular.x = (ned_to_enu_nav_pvt_) ?
                static_cast<float>(gps_esf_ins_.yAngRate) / 1e3 :
                static_cast<float>(gps_esf_ins_.xAngRate) / 1e3;
              // TODO: Guess for now
              gps_odom.twist.covariance[21] = pow(0.1, 2);  // (0.1 rad/s)^2 - don't know rotation rate
            }
            else
            {
              gps_odom.twist.covariance[21] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
            }
            // Y - xAngRate due to incorrect internal transform
            if((gps_esf_ins_.bitfield0 & ublox_msgs::EsfINS::BITFIELD0_Y_ANG_RATE_VALID) > 0)
            {
              gps_odom.twist.twist.angular.y = (ned_to_enu_nav_pvt_) ?
                static_cast<float>(gps_esf_ins_.xAngRate) / 1e3 :
                static_cast<float>(gps_esf_ins_.yAngRate) / 1e3;
              // TODO: Guess for now
              gps_odom.twist.covariance[28] = pow(0.1, 2);  // (0.1 rad/s)^2 - don't know rotation rate
            }
            else
            {
              gps_odom.twist.covariance[28] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
            }
            // Z - inverted due to incorrect internal transform
            if((gps_esf_ins_.bitfield0 & ublox_msgs::EsfINS::BITFIELD0_Z_ANG_RATE_VALID) > 0)
            {
              gps_odom.twist.twist.angular.z = (ned_to_enu_nav_pvt_) ?
                -static_cast<float>(gps_esf_ins_.zAngRate) / 1e3 :
                static_cast<float>(gps_esf_ins_.zAngRate) / 1e3;
              // TODO: Guess for now
              gps_odom.twist.covariance[35] = pow(0.1, 2);  // (0.1 rad/s)^2 - don't know rotation rate
            }
            else
            {
              gps_odom.twist.covariance[35] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
            }
          }
          gps_odom.twist.covariance[0] = pow(msg->sAcc * 1e-3, 2);  // Speed accuracy in (m/s)^2
          gps_odom.twist.covariance[21] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
          gps_odom.twist.covariance[28] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
          gps_odom.twist.covariance[35] = pow(TAU, 2);  // (2 rad/s)^2 - don't know rotation rate
          // Orientation and orientation covariance
          gps_odom.pose.pose.orientation = tf2::toMsg(orientation_quat);
          gps_odom.pose.covariance[35] = pow(msg->headAcc * 1e-5 * PI / 180.0, 2);  // Heading accuracy in deg^2

          // Publish as though this were the filter
          gps_odom_pub_.publish(gps_odom);

          if (publish_gps_)
          {
            filtered_gps_pub_.publish(gps_meas);
          }
        }
      }
    }
    else if (!has_transform_gps_)
    {
      ROS_WARN_THROTTLE(15, "GNSS data used for origin is being reset due to a bad GNSS measurement.");
      // gps not good so we reset data used for origin used by geographic lib
      // check for has_transform_gps_ so we do not reset these variables after has_transform_gps_==true
      // - to avoid changing downstream gps/odometry solutions 
      current_good_gps_count_ = 0;
      current_delayed_gps_count_ = 0;
      origin_llh_[0].clear();
      origin_llh_[1].clear();
      origin_llh_[2].clear();
    }
  }

  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // We need the baseLinkFrameId_ from the odometry message, so
    // we need to wait until we receive it.
    if (has_transform_odom_)
    {
      /* This method only gets called if we don't yet have the
       * IMU data (the subscriber gets shut down once we compute
       * the transform), so we can assumed that every IMU message
       * that comes here is meant to be used for that purpose. */
      tf2::fromMsg(msg->orientation, transform_orientation_);

      // Correct for the IMU's orientation w.r.t. base_link
      tf2::Transform target_frame_trans;
      bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                   base_link_frame_id_,
                                                                   msg->header.frame_id,
                                                                   msg->header.stamp,
                                                                   transform_timeout_,
                                                                   target_frame_trans,
                                                                   tf_silent_failure_);

      if (can_transform)
      {
        double roll_offset = 0;
        double pitch_offset = 0;
        double yaw_offset = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        RosFilterUtilities::quatToRPY(target_frame_trans.getRotation(), roll_offset, pitch_offset, yaw_offset);
        RosFilterUtilities::quatToRPY(transform_orientation_, roll, pitch, yaw);

        ROS_DEBUG_STREAM("Initial orientation is " << transform_orientation_);

        // Apply the offset (making sure to bound them), and throw them in a vector
        tf2::Vector3 rpy_angles(FilterUtilities::clampRotation(roll - roll_offset),
                                FilterUtilities::clampRotation(pitch - pitch_offset),
                                FilterUtilities::clampRotation(yaw - yaw_offset));

        // Now we need to rotate the roll and pitch by the yaw offset value.
        // Imagine a case where an IMU is mounted facing sideways. In that case
        // pitch for the IMU's world frame is roll for the robot.
        tf2::Matrix3x3 mat;
        mat.setRPY(0.0, 0.0, yaw_offset);
        rpy_angles = mat * rpy_angles;
        transform_orientation_.setRPY(rpy_angles.getX(), rpy_angles.getY(), rpy_angles.getZ());

        ROS_DEBUG_STREAM("Initial corrected orientation roll, pitch, yaw is (" <<
                         rpy_angles.getX() << ", " << rpy_angles.getY() << ", " << rpy_angles.getZ() << ")");

        has_transform_imu_ = true;
      }
    }
  }

  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    world_frame_id_ = msg->header.frame_id;
    base_link_frame_id_ = msg->child_frame_id;

    if (!transform_good_ && !use_manual_datum_)
    {
      setTransformOdometry(msg);
    }

    tf2::fromMsg(msg->pose.pose, latest_world_pose_);
    latest_odom_covariance_.setZero();
    for (size_t row = 0; row < POSE_SIZE; ++row)
    {
      for (size_t col = 0; col < POSE_SIZE; ++col)
      {
        latest_odom_covariance_(row, col) = msg->pose.covariance[row * POSE_SIZE + col];
      }
    }

    odom_update_time_ = msg->header.stamp;
    odom_updated_ = true;
  }


  bool NavSatTransform::prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps)
  {
    bool new_data = false;

    if (transform_good_ && odom_updated_)
    {
      mapToLL(latest_world_pose_.getOrigin(), filtered_gps.latitude, filtered_gps.longitude, filtered_gps.altitude);

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(cartesian_world_trans_inverse_.getRotation());
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latest_odom_covariance_ = rot_6d * latest_odom_covariance_.eval() * rot_6d.transpose();

      // Copy the measurement's covariance matrix back
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          filtered_gps.position_covariance[POSITION_SIZE * i + j] = latest_odom_covariance_(i, j);
        }
      }

      filtered_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      filtered_gps.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      filtered_gps.header.frame_id = base_link_frame_id_;
      filtered_gps.header.stamp = odom_update_time_;

      // Mark this GPS as used
      odom_updated_ = false;
      new_data = true;
    }

    return new_data;
  }

  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gps_odom)
  {
    bool new_data = false;

    // Only need updated odom if not using NavPVT. Otherwise, relevant information handled separately.
    if (transform_good_ && gps_updated_ && (odom_updated_ || use_nav_pvt_))
    {
      gps_odom = cartesianToMap(latest_cartesian_pose_);

      tf2::Transform transformed_cartesian_gps;
      tf2::fromMsg(gps_odom.pose.pose, transformed_cartesian_gps);

      // Want the pose of the vehicle origin, not the GPS
      tf2::Transform transformed_cartesian_robot;
      bool is_transformed = getRobotOriginWorldPose(transformed_cartesian_gps, transformed_cartesian_robot, gps_odom.header.stamp);
      if (is_transformed)
      {
        // Rotate the covariance as well
        tf2::Matrix3x3 rot(cartesian_world_transform_.getRotation());
        Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
        rot_6d.setIdentity();

        for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
        {
          rot_6d(rInd, 0) = rot.getRow(rInd).getX();
          rot_6d(rInd, 1) = rot.getRow(rInd).getY();
          rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
          rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
          rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
          rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
        }

        // Rotate the covariance
        latest_cartesian_covariance_ = rot_6d * latest_cartesian_covariance_.eval() * rot_6d.transpose();

        // Now fill out the message. Set the orientation to the identity.
        tf2::toMsg(transformed_cartesian_robot, gps_odom.pose.pose);
        gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < POSE_SIZE; i++)
        {
          for (size_t j = 0; j < POSE_SIZE; j++)
          {
            gps_odom.pose.covariance[POSE_SIZE * i + j] = latest_cartesian_covariance_(i, j);
          }
        }

        // Mark this GPS as used
        gps_updated_ = false;
        new_data = true;
      }
    }

    return new_data;
  }

  void NavSatTransform::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double cartesian_x = 0;
    double cartesian_y = 0;
    double cartesian_z = 0;
    if (use_local_cartesian_)
    {
      const double hae_altitude = 0.0;
      gps_local_cartesian_.Reset(msg->latitude, msg->longitude, hae_altitude);
      gps_local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, cartesian_x, cartesian_y, cartesian_z);

      // UTM meridian convergence is not meaningful when using local cartesian, so set it to 0.0
      utm_meridian_convergence_ = 0.0;
    }
    else
    {
      double k_tmp;
      double utm_meridian_convergence_degrees;
      GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone_, northp_,
                                     cartesian_x, cartesian_y, utm_meridian_convergence_degrees, k_tmp);
      utm_meridian_convergence_ = utm_meridian_convergence_degrees * NavsatConversions::RADIANS_PER_DEGREE;
    }

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << std::setprecision(16) << msg->latitude << ", " <<
                    msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum " << ((use_local_cartesian_)? "Local Cartesian" : "UTM") <<
                    " coordinate is (" << std::fixed << cartesian_x << ", " << cartesian_y << ") zone " << utm_zone_);

    transform_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
    transform_cartesian_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
  }

  void NavSatTransform::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg)
  {
    tf2::fromMsg(msg->pose.pose, transform_world_pose_);
    has_transform_odom_ = true;

    ROS_INFO_STREAM_ONCE("Initial odometry pose is " << transform_world_pose_);

    // Users can optionally use the (potentially fused) heading from
    // the odometry source, which may have multiple fused sources of
    // heading data, and so would act as a better heading for the
    // Cartesian->world_frame transform.
    if (!transform_good_ && use_odometry_yaw_ && !use_manual_datum_)
    {
      sensor_msgs::Imu *imu = new sensor_msgs::Imu();
      imu->orientation = msg->pose.pose.orientation;
      imu->header.frame_id = msg->child_frame_id;
      imu->header.stamp = msg->header.stamp;
      sensor_msgs::ImuConstPtr imuPtr(imu);
      imuCallback(imuPtr);
    }
  }

}  // namespace RobotLocalization

