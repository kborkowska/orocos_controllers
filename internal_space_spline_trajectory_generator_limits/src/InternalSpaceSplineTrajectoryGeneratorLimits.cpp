/*
 * Copyright (c) 2010-2015 Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * InternalSpaceSplineTrajectoryGeneratorLimits.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <rtt/Component.hpp>
#include <string>
#include <exception>
#include "InternalSpaceSplineTrajectoryGeneratorLimits.h"

#include "rtt_rosclock/rtt_rosclock.h"

InternalSpaceSplineTrajectoryGeneratorLimits::InternalSpaceSplineTrajectoryGeneratorLimits(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      last_point_not_set_(false),
      trajectory_active_(false),
      trajectory_ptr_(0),
      number_of_joints_(0),
      port_trajectory_in_("trajectoryPtr_INPORT"),
      port_is_trajectory_feasible_("IsTrajectoryFeasible_OUTPORT"),
      port_internal_space_position_command_out_("JointPositionCommand_OUTPORT",
                                                false),
      port_generator_active_out_("GeneratorActive_OUTPORT", false),
      port_internal_space_position_measurement_in_("JointPosition_INPORT"),
      port_is_synchronised_in_("IsSynchronised_INPORT"),
      ns_interval_(0),
      research_mode_(true) {
  this->ports()->addPort(port_trajectory_in_);
  this->ports()->addPort(port_internal_space_position_command_out_);
  this->ports()->addPort(port_internal_space_position_measurement_in_);
  this->ports()->addPort(port_generator_active_out_);
  this->ports()->addPort(port_is_synchronised_in_);
  this->ports()->addPort(port_is_trajectory_feasible_);

  this->addProperty("number_of_joints", number_of_joints_);
  this->addProperty("ns_interval", ns_interval_);
  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
  return;
}

InternalSpaceSplineTrajectoryGeneratorLimits::~InternalSpaceSplineTrajectoryGeneratorLimits() {
  return;
}

bool InternalSpaceSplineTrajectoryGeneratorLimits::configureHook() {
  RTT::Logger::In in("InternalSpaceSplineTrajectoryGeneratorLimits::configureHook");

  if (number_of_joints_ <= 0) {
    RTT::Logger::log(RTT::Logger::Error) << "wrong number of joints"
        << RTT::endlog();
    return false;
  }

  vel_profile_.resize(number_of_joints_);
  active_points_.resize(number_of_joints_);

  max_velocities_.resize(number_of_joints_);
  max_accelerations_.resize(number_of_joints_);

  old_velocities_.resize(number_of_joints_);
  end_times_.resize(number_of_joints_);

  des_jnt_pos_.resize(number_of_joints_);
  port_internal_space_position_command_out_.setDataSample(des_jnt_pos_);

  std_msgs::Int16 smpl_ = std_msgs::Int16();
  port_is_trajectory_feasible_.setDataSample(smpl_);

  ns_higher_bound_ = ns_interval_ * 1.1;
  ns_higher_increment_ = ns_interval_ * 1.05;
  ns_lower_bound_ = ns_interval_ * 0.9;
  ns_lower_increment_ = ns_interval_ * 0.95;

  return true;
}

bool InternalSpaceSplineTrajectoryGeneratorLimits::startHook() {

  std::cout<<max_velocities_[0]<<std::endl;

  bool is_synchronised = true;

  if (port_internal_space_position_measurement_in_.read(setpoint_)
      == RTT::NoData) {
    return false;
  }

  port_internal_space_position_measurement_in_.getDataSample(setpoint_);
  if (setpoint_.size() != number_of_joints_) {
    RTT::Logger::log(RTT::Logger::Error) << "wrong data sample on port "
        << port_internal_space_position_measurement_in_.getName()
        << RTT::endlog();
    return false;
  }

  port_is_synchronised_in_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }

  port_generator_active_out_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;

  last_time_ = rtt_rosclock::host_now();
  update_hook_iter_ = 0;

  return true;
}

void InternalSpaceSplineTrajectoryGeneratorLimits::stopHook() {
  port_generator_active_out_.write(false);
}

void InternalSpaceSplineTrajectoryGeneratorLimits::updateHook() {
  update_hook_iter_++;
  port_generator_active_out_.write(true);

  trajectory_msgs::JointTrajectoryConstPtr trj_ptr_tmp;
  if (port_trajectory_in_.read(trj_ptr_tmp) == RTT::NewData) {
    std::cout<<"lol"<<std::endl;
    trajectory_ = trj_ptr_tmp;
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    for(auto& ov: old_velocities_){
      ov= 0.0;
    }
    for(auto& et: end_times_){
      et= 0.0;
    }
    for(auto&& ap: active_points_){
      ap= false;
    }
    last_point_not_set_ = true;
    trajectory_active_ = true;
    std::cout<<"lol 10"<<std::endl;
//   std::cout << std::endl<< "InternalSpaceSplineTrajectoryGeneratorLimits new trj" << std::endl<< std::endl<< std::endl;
  }

// std::cout << "InternalSpaceSplineTrajectoryGeneratorLimits" << std::endl;

  ros::Time now = rtt_rosclock::host_now();
  if ((ns_higher_bound_ > 0)
      && (now - last_time_) >= ros::Duration(0, ns_higher_bound_)) {
    //  std::cout << now - last_time_ << std::endl;
    last_time_ += ros::Duration(0, ns_higher_increment_);
    now = last_time_;
  } else if ((ns_lower_bound_ > 0)
      && ((now - last_time_) <= ros::Duration(0, ns_lower_bound_))
      && (update_hook_iter_ > 1)) {
    // std::cout << now - last_time_ << std::endl;
    last_time_ += ros::Duration(0, ns_lower_increment_);
    now = last_time_;
  }
  last_time_ = now;

  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    //std::cout<<"lol 1"<<trajectory_active_<<trajectory_<<std::endl;

    double t = (now - trajectory_->header.stamp).toSec();
    if(std::any_of(active_points_.begin(), active_points_.end(),[](bool ap){return ap;})){
      std::cout<<"lol 2"<<std::endl;
      for(unsigned int i = 0; i < number_of_joints_; i++){
        if(end_times_[i]>t){
          setpoint_(i) = vel_profile_[i].Pos(t);
        } else {
          active_points_[i]=false;
        }
      }
    } else if(trajectory_ptr_< trajectory_->points.size()){
      std::cout<<"lol 3"<<std::endl;
      for(unsigned int i = 0; i < number_of_joints_; i++){
        if(vel_profile_[i].SetProfileTrapezoidal(
          old_point_(i),
          trajectory_->points[trajectory_ptr_].positions[i],
          old_velocities_[i],
          trajectory_->points[trajectory_ptr_].velocities[i],
          max_velocities_[i],
          max_accelerations_[i],
          t,
          lowerLimits_[i],
          upperLimits_[i],
          research_mode_)
        ){
          end_times_[i] = vel_profile_[i].Duration()+t;
          active_points_[i]=true;
        } else {
          errMsg_ = vel_profile_[i].getErrorMsg();
          std::cout<<"i:"<<i<<" errMsg:"<<errMsg_<<std::endl;
          RTT::Logger::log(RTT::Logger::Debug) << errMsg_ << RTT::endlog();
          errNo_ = std_msgs::Int16();
          errNo_.data = -1;
          port_is_trajectory_feasible_.write(errNo_);
          std::cout<<"wrote: errNo_:"<<errNo_<<std::endl;
          trajectory_ = trajectory_msgs::JointTrajectoryConstPtr();
          break;
        }
      }
      ++trajectory_ptr_;
    } else {
      std::cout<<"lol 5"<<std::endl;
      for (unsigned int i = 0; i < number_of_joints_; i++) {
        setpoint_(i) = trajectory_->points[trajectory_->points.size() - 1]
            .positions[i];
      }
      for(auto&& ap: active_points_){
        ap= false;
      }
      trajectory_ = trajectory_msgs::JointTrajectoryConstPtr();
    }
  }
  //std::cout<<setpoint_<<std::endl;
  //std::cout<<"update iter:"<<update_hook_iter_<<std::endl;
  port_internal_space_position_command_out_.write(setpoint_);
}

ORO_CREATE_COMPONENT(InternalSpaceSplineTrajectoryGeneratorLimits)