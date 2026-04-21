/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// 1. ROS 2 Core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> // Replaces actionlib

// 2. TF2 (Replaces tf and tf_conversions)
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_kdl/tf2_kdl.hpp>           // Replaces tf_kdl.h
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 3. KDL & Parser (Stay mostly the same, but use Humble vendor paths)
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

// 4. Messages and Actions (Note the /msg/ or /action/ subfolder)
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/point_head.hpp>            // Replaces PointHeadAction.h
#include <control_msgs/srv/query_trajectory_state.hpp>   // Replaces QueryTrajectoryState.h
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

// 5. System and C++ Standard (Boost is mostly replaced by std)
#include <memory>  // Replaces boost/scoped_ptr.hpp with std::unique_ptr
#include <cmath>
#include <functional> // for std::bind
#include <urdf/model.h>

class ControlHead
{
private:
  // Typedefs for clarity
  using PointHead = control_msgs::action::PointHead;
  using PHAS = rclcpp_action::Server<PointHead>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PointHead>;

  // Node Pointer
  rclcpp::Node::SharedPtr node_;

  const std::string action_name_;
  std::string root_;
  std::string tip_;
  std::string pan_link_;
  std::string default_pointing_frame_;
  std::string pointing_frame_;
  tf2::Vector3 pointing_axis_;
  std::vector<std::string> joint_names_;

  // ROS 2 Interfaces
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::Client<control_msgs::srv::QueryTrajectoryState>::SharedPtr cli_query_traj_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Action Server
  PHAS::SharedPtr action_server_;
  bool has_active_goal_;
  std::shared_ptr<GoalHandle> active_goal_; // Use shared_ptr for GoalHandles in ROS 2
  double success_angle_threshold_;

  // Kinematics (KDL remains mostly the same)
  KDL::Tree tree_;
  KDL::Chain chain_;
  tf2::Vector3 target_in_root_; // Replaced tf::Point with tf2::Vector3
  tf2::Vector3 desired_pointing_axis_in_frame_;
  double goal_error_;

  // Solvers (Replaced boost::scoped_ptr with std::unique_ptr)
  std::unique_ptr<KDL::ChainFkSolverPos> pose_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  // TF2 (Buffer and Listener are separate in ROS 2)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  urdf::Model urdf_model_;

  // Pointer to last state (Note the change in Smart Pointer naming)
  control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr last_controller_state_;

public:
  ControlHead(rclcpp::Node::SharedPtr node)
      : node_(node),
        action_name_("point_head_action"),
        has_active_goal_(false)
  {
      // 1. Parameters (Replaces pnh_.param)
      node_->declare_parameter("pan_link", "head_pan_link");
      node_->declare_parameter("default_pointing_frame", "head_tilt_link");
      node_->declare_parameter("success_angle_threshold", 0.1);

      pan_link_ = node_->get_parameter("pan_link").as_string();
      default_pointing_frame_ = node_->get_parameter("default_pointing_frame").as_string();
      success_angle_threshold_ = node_->get_parameter("success_angle_threshold").as_double();

      // Cleaning frame names (Slash at start is not used in ROS 2 frames)
      if (!pan_link_.empty() && pan_link_[0] == '/') pan_link_.erase(0, 1);
      if (!default_pointing_frame_.empty() && default_pointing_frame_[0] == '/') 
          default_pointing_frame_.erase(0, 1);

      // 2. Publisher, Subscription, and Client
      pub_controller_command_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 2);
      
      sub_controller_state_ = node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
          "state", 1, std::bind(&ControlHead::controllerStateCB, this, std::placeholders::_1));

      cli_query_traj_ = node_->create_client<control_msgs::srv::QueryTrajectoryState>("query_state");

      // 3. Robot Description (ROS 2 uses parameters, not searchParam)
      std::string robot_desc_string;
      if (node_->has_parameter("robot_description")) {
          robot_desc_string = node_->get_parameter("robot_description").as_string();
      } else {
          RCLCPP_ERROR(node_->get_logger(), "robot_description parameter not found!");
      }

      if (!robot_desc_string.empty()) {
          if (!kdl_parser::treeFromString(robot_desc_string, tree_)) {
              RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree");
          }
          if (!urdf_model_.initString(robot_desc_string)) {
              RCLCPP_ERROR(node_->get_logger(), "Failed to parse urdf string.");
          }
      }

      // 4. Action Server (The big change)
      this->action_server_ = rclcpp_action::create_server<control_msgs::action::PointHead>(
          node_,
          action_name_,
          std::bind(&ControlHead::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&ControlHead::handle_cancel, this, std::placeholders::_1),
          std::bind(&ControlHead::handle_accepted, this, std::placeholders::_1)
      );

      // 5. Watchdog Timer (Uses std::chrono)
      using namespace std::chrono_literals;
      watchdog_timer_ = node_->create_wall_timer(1s, std::bind(&ControlHead::watchdog, this));

      RCLCPP_INFO(node_->get_logger(), "Head Action Server started.");
  }

  rclcpp_action::GoalResponse handle_goal
  (
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::PointHead::Goal> goal
  ) 
  {
      
      // Stage 1: handle_goal (Decide whether to accept or reject)
      // In ROS 2, if the tree is not ready, we can reject the goal immediately
      if (root_.empty())
      {
          std::string err_msg;
          // Search for the parent of pan_link_ in the TF buffer
          if (tf_buffer_->_frameExists(pan_link_))
          {
              try {
                  // In TF2, we get the frame metadata to find the parent
                  root_ = tf_buffer_->_getParent(pan_link_);
              } catch (const tf2::TransformException &ex) {
                  RCLCPP_ERROR(node_->get_logger(), "TF2 Error: %s", ex.what());
              }
          }

          if (root_.empty())
          {
              RCLCPP_ERROR(node_->get_logger(), "Could not get parent of %s in the TF tree", pan_link_.c_str());
              return rclcpp_action::GoalResponse::REJECT;
          }
          
          // Remove leading slash for ROS 2 compatibility
          if (root_[0] == '/') root_.erase(0, 1);
      }

      RCLCPP_INFO(node_->get_logger(), "Received point head goal request");


      // Stage 2: Process pointing frame
      pointing_frame_ = goal->pointing_frame;
      if (pointing_frame_.empty())
      {
          RCLCPP_WARN(
            node_->get_logger(), 
            "Pointing frame not specified, using %s [1, 0, 0] by default.", 
            default_pointing_frame_.c_str()
          );
          pointing_frame_ = default_pointing_frame_;
          pointing_axis_ = tf2::Vector3(1.0, 0.0, 0.0);
      }
      else
      {
          // Remove leading slash for ROS 2
          if (pointing_frame_[0] == '/') pointing_frame_.erase(0, 1);

          try
          {
              // Check if transform exists (Replaces waitForTransform)
              // In ROS 2, we check if the transform is available in the buffer
              if (!tf_buffer_->canTransform(pan_link_, pointing_frame_, goal->target.header.stamp, 
                                            tf2::durationFromSec(1.0))) // Smaller timeout for callbacks
              {
                  RCLCPP_ERROR(node_->get_logger(), "Transform from %s to %s not available.", 
                              pan_link_.c_str(), pointing_frame_.c_str());
                  // In handle_accepted, you would abort; in handle_goal, you would reject.
                  return rclcpp_action::GoalResponse::REJECT; 
              }

              // Convert pointing axis (Replaces vector3MsgToTF)
              tf2::fromMsg(goal->pointing_axis, pointing_axis_);

              if (pointing_axis_.length() < 0.1)
              {
                  if (pointing_frame_.find("optical_frame") != std::string::npos)
                  {
                      RCLCPP_WARN(node_->get_logger(), "Pointing axis zero-length. Using [0, 0, 1] for optical frame.");
                      pointing_axis_ = tf2::Vector3(0, 0, 1);
                  }
                  else
                  {
                      RCLCPP_WARN(node_->get_logger(), "Pointing axis zero-length. Using [1, 0, 0] for non-optical frame.");
                      pointing_axis_ = tf2::Vector3(1, 0, 0);
                  }
              }
              else
              {
                  pointing_axis_.normalize();
              }
          }
          catch (const tf2::TransformException &ex)
          {
              RCLCPP_ERROR(node_->get_logger(), "Transform failure: %s", ex.what());
              return rclcpp_action::GoalResponse::REJECT;
          }
      }


      const auto & target = goal->target; // goal is the shared_ptr from handle_goal arguments
      try
      {
          // Check if the transform is available (Non-blocking or very short timeout)
          // We use tf2::durationFromSec(0.1) instead of 5.0 to keep the callback responsive.
          if (!tf_buffer_->canTransform(root_, target.header.frame_id, target.header.stamp, 
                                        tf2::durationFromSec(0.1)))
          {
              RCLCPP_ERROR(node_->get_logger(), "Could not transform from %s to %s at requested time.", 
                          target.header.frame_id.c_str(), root_.c_str());
              return rclcpp_action::GoalResponse::REJECT;
          }

          // Transform the Point (New ROS 2 syntax)
          // tf2_buffer->transform handles the msg types directly
          geometry_msgs::msg::PointStamped target_in_root_msg;
          target_in_root_msg = tf_buffer_->transform(target, root_);

          // 3. Convert message to tf2::Vector3 for KDL/Math use
          tf2::fromMsg(target_in_root_msg.point, target_in_root_);

          RCLCPP_DEBUG(node_->get_logger(), "Target point in root frame (%s): (%f, %f, %f)", 
                      root_.c_str(), target_in_root_.x(), target_in_root_.y(), target_in_root_.z());
      }
      catch (const tf2::TransformException &ex)
      {
          RCLCPP_ERROR(node_->get_logger(), "Transform failure: %s", ex.what());
          return rclcpp_action::GoalResponse::REJECT;
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void goalCB(GoalHandle gh)
  {
    // Before we do anything, we need to know that name of the pan_link's parent, which we will treat as the root.
    if (root_.empty())
    {
      for (int i = 0; i < 10; ++i)
      {
        try
        {
          tfl_.getParent(pan_link_, ros::Time(), root_);
          break;
        }
        catch (const tf::TransformException &ex) {}
        ros::Duration(0.5).sleep();
      }
      if (root_.empty())
      {
        ROS_ERROR("Could not get parent of %s in the TF tree", pan_link_.c_str());
        gh.setRejected();
        return;
      }
    }
    if (root_[0] == '/') root_.erase(0, 1);

    ROS_DEBUG("Got point head goal!");

    // Process pointing frame and axis
    const geometry_msgs::PointStamped &target = gh.getGoal()->target;
    pointing_frame_ = gh.getGoal()->pointing_frame;
    if (pointing_frame_.length() == 0)
    {
      ROS_WARN("Pointing frame not specified, using %s [1, 0, 0] by default.", default_pointing_frame_.c_str());
      pointing_frame_ = default_pointing_frame_;
      pointing_axis_ = tf::Vector3(1.0, 0.0, 0.0);
    }
    else
    {
      if (pointing_frame_[0] == '/') pointing_frame_.erase(0, 1);
      bool ret1 = false;
      try
      {
        ret1 = tfl_.waitForTransform(pan_link_, pointing_frame_, target.header.stamp,
                                     ros::Duration(5.0), ros::Duration(0.01));

        tf::vector3MsgToTF(gh.getGoal()->pointing_axis, pointing_axis_);
        if (pointing_axis_.length() < 0.1)
        {
          size_t found = pointing_frame_.find("optical_frame");
          if (found != std::string::npos)
          {
            ROS_WARN("Pointing axis appears to be zero-length. Using [0, 0, 1] as default for an optical frame.");
            pointing_axis_ = tf::Vector3(0, 0, 1);
          }
          else
          {
            ROS_WARN("Pointing axis appears to be zero-length. Using [1, 0, 0] as default for a non-optical frame.");
            pointing_axis_ = tf::Vector3(1, 0, 0);
          }
        }
        else
        {
          pointing_axis_.normalize();
        }
      }
      catch (const tf::TransformException &ex)
      {
        ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
        gh.setRejected();
        return;
      }
    }

    //Put the target point in the root frame (usually torso_lift_link).
    bool ret1 = false;
    try
    {
      std::string error_msg;
      ret1 = tfl_.waitForTransform(root_.c_str(), target.header.frame_id, target.header.stamp,
                                       ros::Duration(5.0), ros::Duration(0.01), &error_msg);

      geometry_msgs::PointStamped target_in_root_msg;
      tfl_.transformPoint(root_.c_str(), target, target_in_root_msg );
      tf::pointMsgToTF(target_in_root_msg.point, target_in_root_);
      ROS_DEBUG_STREAM("Target point in base frame: (" << target_in_root_[0] << ", " << target_in_root_[1] << ", " << target_in_root_[2] << ")");
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      gh.setRejected();
      return;
    }

    if (tip_.compare(pointing_frame_) != 0)
    {
      bool success = tree_.getChain(root_.c_str(), pointing_frame_.c_str(), chain_);
      if (!success)
      {
        ROS_ERROR("Couldn't create chain from %s to %s.", root_.c_str(), pointing_frame_.c_str());
        gh.setRejected();
        return;
      }
      tip_ = pointing_frame_;

      pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
      jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
      joint_names_.resize(chain_.getNrOfJoints());
    }

    const unsigned int joints = chain_.getNrOfJoints();

    KDL::JntArray jnt_pos(joints), jnt_eff(joints);
    KDL::Jacobian jacobian(joints);

    control_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now() + ros::Duration(0.01);
    if (!cli_query_traj_.call(traj_state))
    {
      ROS_ERROR("Service call to query controller trajectory failed.");
      gh.setRejected();
      return;
    }
    if (traj_state.response.name.size() != joints)
    {
      ROS_ERROR("Number of joints mismatch: urdf chain vs. trajectory controller state.");
      gh.setRejected();
      return;
    }
    std::vector<urdf::JointLimits> limits_(joints);

    // Get initial joint positions and joint limits.
    for (unsigned int i = 0; i < joints; ++i)
    {
      joint_names_[i] = traj_state.response.name[i];
      limits_[i] = *(urdf_model_.joints_[joint_names_[i].c_str()]->limits);
      ROS_DEBUG("Joint %d %s: %f, limits: %f %f", i, traj_state.response.name[i].c_str(), traj_state.response.position[i], limits_[i].lower, limits_[i].upper);
      jnt_pos(i) = 0;
    }

    int count = 0;
    int limit_flips = 0;
    float correction_angle = 2*M_PI;
    float correction_delta = 2*M_PI;
    const int MAX_ITERATIONS = 15;
    while( ros::ok() &&
           fabs(correction_delta) > 0.001 &&
           count < MAX_ITERATIONS) //limit the iterations
    {
      //get the pose and jacobian for the current joint positions
      KDL::Frame pose;
      pose_solver_->JntToCart(jnt_pos, pose);
      jac_solver_->JntToJac(jnt_pos, jacobian);

      tf::Transform frame_in_root;
      tf::poseKDLToTF(pose, frame_in_root);

      tf::Vector3 axis_in_frame = pointing_axis_.normalized();
      tf::Vector3 target_from_frame = (target_in_root_ - frame_in_root.getOrigin()).normalized();
      tf::Vector3 current_in_frame = frame_in_root.getBasis().inverse()*target_from_frame;
      float prev_correction = correction_angle;
      correction_angle = current_in_frame.angle(axis_in_frame);
      correction_delta = correction_angle - prev_correction;

      ROS_DEBUG("At step %d, joint poses are %.4f and %.4f, angle error is %f radians", count, jnt_pos(0), jnt_pos(1), correction_angle);
      goal_error_ = correction_angle; //expected error after this iteration
      if ( correction_angle < 0.5*success_angle_threshold_ )
      {
        ROS_DEBUG_STREAM("Accepting solution as estimated error is: " << correction_angle*180.0/M_PI << "degrees and stopping condition is half of " << success_angle_threshold_*180.0/M_PI);
        break;
      }
      tf::Vector3 correction_axis = frame_in_root.getBasis()*(axis_in_frame.cross(current_in_frame).normalized());
      tf::Transform correction_tf(tf::Quaternion(correction_axis, 0.5*correction_angle), tf::Vector3(0,0,0));
      KDL::Frame correction_kdl;
      tf::transformTFToKDL(correction_tf, correction_kdl);

      // We apply a "wrench" proportional to the desired correction
      KDL::Frame identity_kdl;
      KDL::Twist twist = diff(correction_kdl, identity_kdl);
      KDL::Wrench wrench_desi;
      for (unsigned int i=0; i<6; ++i)
        wrench_desi(i) = -1.0*twist(i);

      // Converts the "wrench" into "joint corrections" with a jacbobian-transpose
      for (unsigned int i = 0; i < joints; ++i)
      {
        jnt_eff(i) = 0;
        for (unsigned int j=0; j<6; ++j)
          jnt_eff(i) += (jacobian(j,i) * wrench_desi(j));
        jnt_pos(i) += jnt_eff(i);
      }

     // account for pan_link joint limit in back.
     // if(jnt_pos(0) < limits_[0].lower && limit_flips++ == 0){ jnt_pos(0) += 1.5*M_PI; }
     // if(jnt_pos(0) > limits_[0].upper && limit_flips++ == 0){ jnt_pos(0) -= 1.5*M_PI; }

      // Move both joins to a position between the upper and lower joint limits
      jnt_pos(0) = std::max(limits_[0].lower, jnt_pos(0));
      jnt_pos(0) = std::min(limits_[0].upper, jnt_pos(0));
      jnt_pos(1) = std::max(limits_[1].lower, jnt_pos(1));
      jnt_pos(1) = std::min(limits_[1].upper, jnt_pos(1));

      count++;

      if (limit_flips > 1)
      {
        ROS_ERROR("Goal is out of joint limits, trying to point there anyway... \n");
        break;
      }
    }
    ROS_DEBUG_STREAM("Iterative solver took " << count << " steps. Expected error: " << correction_angle << " radians");
    if ( count == MAX_ITERATIONS )
      ROS_WARN("Aborted because maximum number of iterations was reached");    

    std::vector<double> q_goal(joints);

    //saturate joint positions considering the joint limits
    for(unsigned int i = 0; i < joints; i++)
    {
      jnt_pos(i) = std::max(limits_[i].lower, jnt_pos(i));
      jnt_pos(i) = std::min(limits_[i].upper, jnt_pos(i));
      q_goal[i] = jnt_pos(i);
      ROS_DEBUG("Joint %d %s: %f", i, joint_names_[i].c_str(), jnt_pos(i));
    }

    //re-compute desired pointing axis from desired joint positions 
    //(to take the case in which joint limits have been enforced into account)
    KDL::Frame pose;
    pose_solver_->JntToCart(jnt_pos, pose);

    tf::Transform frame_in_root;
    tf::poseKDLToTF(pose, frame_in_root);
    tf::Vector3 target_from_frame = target_in_root_ - frame_in_root.getOrigin();
    target_from_frame.normalize();
    ROS_DEBUG_STREAM("BEFORE applying joint limits => desired pointing axis = (" <<
                     pointing_axis_[0] << ", " <<
                     pointing_axis_[1] << ", " <<
                     pointing_axis_[2] << ")");
    desired_pointing_axis_in_frame_ = frame_in_root.getBasis().inverse()*target_from_frame;
    ROS_DEBUG_STREAM("AFTER applying joint limits => desired pointing axis = (" <<
                     desired_pointing_axis_in_frame_[0] << ", " <<
                     desired_pointing_axis_in_frame_[1] << ", " <<
                     desired_pointing_axis_in_frame_[2] << ")");

    //the goal will end when the angular error of the pointing axis
    //is lower than goal_error_. This variable is assigned with the maximum
    //between the ros param success_angle_threshold_ and the estimated error
    //from the iterative solver last iteration, i.e. goal_error_ current value
    goal_error_ = std::max(goal_error_, success_angle_threshold_);
    ROS_DEBUG_STREAM("the goal will terminate when error is: " << goal_error_*180.0/M_PI << " degrees => " << goal_error_ << " radians");

    if (has_active_goal_)
    {
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Computes the duration of the movement.
    ros::Duration min_duration(0.01);

    if (gh.getGoal()->min_duration > min_duration)
        min_duration = gh.getGoal()->min_duration;

    // Determines if we need to increase the duration of the movement in order to enforce a maximum velocity.
    if (gh.getGoal()->max_velocity > 0)
    {
      // compute the largest required rotation among all the joints
      double largest_rotation = 0;
      for(unsigned int i = 0; i < joints; i++)
      {
        double required_rotation = fabs(q_goal[i] - traj_state.response.position[i]);
        if ( required_rotation > largest_rotation )
          largest_rotation = required_rotation;
      }

      ros::Duration limit_from_velocity(largest_rotation / gh.getGoal()->max_velocity);
      if (limit_from_velocity > min_duration)
        min_duration = limit_from_velocity;
    }

    // Computes the command to send to the trajectory controller.
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = traj_state.request.time;

    traj.joint_names.push_back(traj_state.response.name[0]);
    traj.joint_names.push_back(traj_state.response.name[1]);

    traj.points.resize(1);
    traj.points[0].positions = q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    traj.points[0].time_from_start = ros::Duration(min_duration);


    pub_controller_command_.publish(traj);
  }

  void watchdog()
  {
    const rclcpp::Time now = node_->get_clock()->now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_ && active_goal_)
    {
      bool should_abort = false;
      std::string reason;

      if (!last_controller_state_)
      {
        should_abort = true;
        reason = "Aborting goal because we have never heard a controller state message.";
      }
      else
      {
        // Calculate duration between now and the last message timestamp
        rclcpp::Duration diff = now - last_controller_state_->header.stamp;
        if (diff > rclcpp::Duration(std::chrono::seconds(5)))
        {
          should_abort = true;
          // Use .seconds() to get a double value for logging
          reason = "Aborting goal because we haven't heard from the controller in " + 
                  std::to_string(diff.seconds()) + " seconds";
        }
      }

      if (should_abort)
      {
        RCLCPP_WARN(node_->get_logger(), "%s", reason.c_str());

        // 1. Stops the controller
        auto empty_msg = trajectory_msgs::msg::JointTrajectory();
        empty_msg.joint_names = joint_names_;
        pub_controller_command_->publish(empty_msg);

        // 2. Marks the current goal as aborted in ROS 2
        auto result = std::make_shared<control_msgs::action::PointHead::Result>();
        active_goal_->abort(result);

        // 3. Reset state
        has_active_goal_ = false;
        active_goal_ = nullptr;
      }
    }
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
      RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

      if (has_active_goal_ && active_goal_ == goal_handle)
      {
          // 1. Stop the controller by sending an empty trajectory
          auto empty_msg = trajectory_msgs::msg::JointTrajectory();
          empty_msg.joint_names = joint_names_;
          // In ROS 2, we use .publish() on the SharedPtr
          pub_controller_command_->publish(empty_msg);

          // 2. Reset state
          has_active_goal_ = false;
          // active_goal_ = nullptr; // Optional: wait until result is sent
          
          return rclcpp_action::CancelResponse::ACCEPT;
      }

      return rclcpp_action::CancelResponse::REJECT;
  }

  void controllerStateCB(const control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr msg)
  {
    last_controller_state_ = msg;
    // Get the clock from the node pointer
    const rclcpp::Time now = node_->get_clock()->now();

    if (!has_active_goal_ || !active_goal_)
        return;

    try
    {     
        // 1. Load Joint Positions into KDL
        KDL::JntArray jnt_pos(msg->joint_names.size());
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            jnt_pos(i) = msg->actual.positions[i];
            RCLCPP_DEBUG(node_->get_logger(), "current state of joint %zu: %f", i, jnt_pos(i));
        }

        // 2. Kinematics Solver
        KDL::Frame pose;
        pose_solver_->JntToCart(jnt_pos, pose);

        // 3. KDL to TF2 Conversion
        // Use tf2_kdl instead of the old tf_conversions
        tf2::Transform frame_in_root = tf2::KDLToTransform(pose);

        tf2::Vector3 axis_in_frame = pointing_axis_.normalized();
        
        // target_in_root_ was converted to tf2::Vector3 in the class definition
        tf2::Vector3 target_from_frame = target_in_root_ - frame_in_root.getOrigin();
        target_from_frame.normalize();

        // 4. Calculate error using TF2 math
        tf2::Vector3 current_in_frame = frame_in_root.getBasis().transpose() * target_from_frame;

        // 5. Publish Feedback
        auto feedback = std::make_shared<control_msgs::action::PointHead::Feedback>();
        feedback->pointing_angle_error = current_in_frame.angle(desired_pointing_axis_in_frame_);

        RCLCPP_DEBUG(node_->get_logger(), "current error is: %f radians", feedback->pointing_angle_error);
        active_goal_->publish_feedback(feedback);

        // 6. Check for Success
        if (feedback->pointing_angle_error <= goal_error_)
        {        
            RCLCPP_DEBUG(node_->get_logger(), "goal succeeded with error: %f radians", feedback->pointing_angle_error);
            
            auto result = std::make_shared<control_msgs::action::PointHead::Result>();
            // Result is currently empty for PointHead, but required by the API
            active_goal_->succeed(result);
            
            has_active_goal_ = false;
            active_goal_ = nullptr;
        }
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform: %s", ex.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_head_action");
  ros::NodeHandle node;
  ControlHead ch(node);
  ros::spin();
  return 0;
}
