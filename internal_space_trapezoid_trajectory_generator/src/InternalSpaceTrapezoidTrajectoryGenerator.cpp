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
 * InternalSpaceTrapezoidTrajectoryGenerator.cpp
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#include <iostream>
#include <fstream>
#include <ctime>
#include <stdlib.h> 
#include <cstdlib>

#include <rtt/Component.hpp>
#include <string>
#include <exception>
#include "InternalSpaceTrapezoidTrajectoryGenerator.h"

#include "rtt_rosclock/rtt_rosclock.h"

InternalSpaceTrapezoidTrajectoryGenerator::InternalSpaceTrapezoidTrajectoryGenerator(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      //last_point_not_set_(false),
      trajectory_active_(false),
      trajectory_point_index_(0),
      number_of_joints_(0),
      port_trajectory_in_("trajectoryPtr_INPORT"),
      port_generator_result_("GeneratorResult_OUTPORT"),
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
  this->ports()->addPort(port_generator_result_);

  this->addProperty("number_of_joints", number_of_joints_);
  this->addProperty("ns_interval", ns_interval_);
  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
  return;
}

InternalSpaceTrapezoidTrajectoryGenerator::~InternalSpaceTrapezoidTrajectoryGenerator() {
  return;
}

bool InternalSpaceTrapezoidTrajectoryGenerator::configureHook() {
  RTT::Logger::In in("InternalSpaceTrapezoidTrajectoryGenerator::configureHook");

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

  trapezoidal_trajectory_msgs::TrapezoidGeneratorResult smpl_ = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult();
  port_generator_result_.setDataSample(smpl_);

  ns_higher_bound_ = ns_interval_ * 1.1;
  ns_higher_increment_ = ns_interval_ * 1.05;
  ns_lower_bound_ = ns_interval_ * 0.9;
  ns_lower_increment_ = ns_interval_ * 0.95;

  trajectory_active_ = false;

  return true;
}

bool InternalSpaceTrapezoidTrajectoryGenerator::startHook() {

  //std::cout<<max_velocities_[0]<<std::endl;

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
  //last_point_not_set_ = false;
  trajectory_active_ = false;

  last_time_ = rtt_rosclock::host_now();
  update_hook_iter_ = 0;

  return true;
}

void InternalSpaceTrapezoidTrajectoryGenerator::stopHook() {
  port_generator_active_out_.write(false);
}

void InternalSpaceTrapezoidTrajectoryGenerator::saveDataToFile() const{
  time_t now = time(0);
  tm *ltm = localtime(&now);
  const char* home = getenv("HOME");
  std::string path(home);
  path += "/IRPOS_results/trapezoid_generator_results/"+std::to_string(ltm->tm_mday)+
                                                        std::to_string(ltm->tm_mon)+
                                                        std::to_string(ltm->tm_year)+
                                                        std::to_string(1 + ltm->tm_hour)+
                                                        std::to_string(1 + ltm->tm_min)+
                                                        std::to_string(1 + ltm->tm_sec)+"/";
  std::string command = "mkdir -pZ "+path;
  std::system(command.c_str());
  std::ofstream myfile;

  //save points from msg
  myfile.open(path+"user_setpoints.txt");
  for(int i = 0; i<trajectory_.points.size();++i){
    for(int k =0; k<number_of_joints_; ++k){
      myfile << std::to_string(trajectory_.points[i].positions[k])+ " ";
    }
    myfile << "\n";
  }
  myfile.close();

  myfile.open(path+"setpoints.txt");
  for(int i = 0; i<setpoint_results_.size();++i){
    for(int k =0; k<number_of_joints_; ++k){
      myfile << std::to_string(setpoint_results_[i](k))+ " ";
    }
    myfile << "\n";
  }
  myfile.close();

  myfile.open(path+"results.txt");
  for(int i = 0; i<jnt_results_.size();++i){
    for(int k =0; k<number_of_joints_; ++k){
      myfile << std::to_string(jnt_results_[i](k))+ " ";
    }
    myfile << "\n";
  }
  myfile.close();
}

trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal InternalSpaceTrapezoidTrajectoryGenerator::getNewGoalAndInitData(){

  trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal trj_ptr_tmp;
  if (port_trajectory_in_.read(trj_ptr_tmp) == RTT::NewData) {
    //init trajectory data
    trajectory_ = trj_ptr_tmp.trajectory;
    trajectory_point_index_ = -1;
    old_point_ = setpoint_;
    //check wether research mode is on
    //if so init tabels used in this setting
    research_mode_ = trj_ptr_tmp.research_mode;
    if(research_mode_){
        for(auto& ov: old_velocities_){
        ov= 0.0;
      }
      for(auto& et: end_times_){
        et= 0.0;
      }
      for(auto&& ap: active_points_){
        ap= false;
      }
      max_velocities_ = trj_ptr_tmp.max_velocities;
      max_accelerations_ = trj_ptr_tmp.max_accelerations;
    } else {
      phase_end_time_ = 0.0;
    }
    //init data used for establishing when the movement is over
    //last_point_not_set_ = true;
    trajectory_active_ = true;
    std::cout<<"[GEN]got new goal"<<std::endl;
    //new_point_ = true;
  }
  //if needed a pointer 
  return trj_ptr_tmp;
}

void InternalSpaceTrapezoidTrajectoryGenerator::adjustTimeFrames(ros::Time now) {
  if ((ns_higher_bound_ > 0)
      && (now - last_time_) >= ros::Duration(0, ns_higher_bound_))
  {
    last_time_ += ros::Duration(0, ns_higher_increment_);
    now = last_time_;
  } else if ((ns_lower_bound_ > 0)
      && ((now - last_time_) <= ros::Duration(0, ns_lower_bound_))
      && (update_hook_iter_ > 1)) 
  {
    last_time_ += ros::Duration(0, ns_lower_increment_);
    now = last_time_;
  }
  last_time_ = now;
}


void InternalSpaceTrapezoidTrajectoryGenerator::sendPositions(bool saveData){

  if(saveData){
    port_internal_space_position_measurement_in_.read(des_jnt_pos_);
    jnt_results_.push_back(des_jnt_pos_);
    //std::cout<<setpoint_<<std::endl;
    //std::cout<<"[GEN]pos:"<<setpoint_(0)<<" time:"<<std::to_string(now.toSec())<<std::endl;
    setpoint_results_.push_back(setpoint_);
  }

  port_internal_space_position_command_out_.write(setpoint_);

}

inline bool InternalSpaceTrapezoidTrajectoryGenerator::isAnyJointStillInMotion(){
  return std::any_of(active_points_.begin(), active_points_.end(),
                                                  [](bool ap){return ap;});
}

inline bool InternalSpaceTrapezoidTrajectoryGenerator::isTheLastTrajectoryPointReached(){
  return trajectory_point_index_< trajectory_.points.size();
}

bool InternalSpaceTrapezoidTrajectoryGenerator::setVelocityProfiles(double t, bool is_velocity_based_){
  int result;
  ++trajectory_point_index_;
  for(unsigned int i = 0; i < number_of_joints_; i++){
    if(is_velocity_based_){
      result = vel_profile_[i].SetProfileVelocity(
                                  t,
                                  old_point_(i),
                                  trajectory_.points[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.points[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i],
                                  lowerLimits_[i],
                                  upperLimits_[i],
                                  research_mode_);
      if(vel_profile_[i].Duration() != 0.0){
        end_times_[i] = vel_profile_[i].Duration()+t;
        active_points_[i]=true;
      }
    } else {
      result = vel_profile_[i].SetProfileDuration(
                                  t,
                                  phase_end_time_-t,
                                  old_point_(i),
                                  trajectory_.points[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.points[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i],
                                  lowerLimits_[i],
                                  upperLimits_[i],
                                  research_mode_);
    }
    //vel_profile_[i].printCoeffs();
    if(result != trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::SUCCESSFUL){
      trapezoidal_trajectory_msgs::TrapezoidGeneratorResult res = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult();
      res.error_string = vel_profile_[i].getErrorMsg() + " in joint :" + std::to_string(i);
      res.error_code = result;
      RTT::Logger::log(RTT::Logger::Debug) << res.error_string <<std::endl<<"On joint no:"<<i<< RTT::endlog();
      std::cout<<result<<std::endl;
      std::cout<<"[GEN]about to sent a failure msg"<<std::endl;
      port_generator_result_.write(res);
      return false;
    }
  }
  for(unsigned int i = 0; i < number_of_joints_; i++){
    old_point_(i) = trajectory_.points[trajectory_point_index_].positions[i];
    old_velocities_[i] = trajectory_.points[trajectory_point_index_].velocities[i];
  }
  return true;
}

void InternalSpaceTrapezoidTrajectoryGenerator::setLastPointsAndSendSuccesMsg(){
  for (unsigned int i = 0; i < number_of_joints_; i++) {
    setpoint_(i) = trajectory_.points[trajectory_.points.size() - 1]
        .positions[i];
  }
  for(auto&& ap: active_points_){
    ap= false;
  }
  trapezoidal_trajectory_msgs::TrapezoidGeneratorResult res = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult();
  res.error_string = " ";
  res.error_code = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::SUCCESSFUL;
  port_generator_result_.write(res);
}

void InternalSpaceTrapezoidTrajectoryGenerator::generatePositions(double t) {
  for(unsigned int i = 0; i < number_of_joints_; i++){
    if(end_times_[i]>t && active_points_[i]==true){
      setpoint_(i) = vel_profile_[i].Pos(t);
    } else {
      active_points_[i]=false;
    }
  }
}

void InternalSpaceTrapezoidTrajectoryGenerator::updateHookWithVelocityBasedProfiles(ros::Time now, bool save_data) {
  if(!(trajectory_active_ && (trajectory_.header.stamp < now))){
    //std::cout<<"[GEN]trajectory inactive"<<std::endl;
    return;
  }
  std::cout<<"[GEN]trajectory is active"<<std::endl;
  double t = now.toSec();
  if(!isTheLastTrajectoryPointReached() && !isAnyJointStillInMotion()){
    std::cout<<"[GEN] setting profiles"<<std::endl;
    if(!setVelocityProfiles(t, save_data)){
      std::cout<<"[GEN] setting profiles failed"<<std::endl;
      trajectory_active_=false;
      if(save_data)
        saveDataToFile();
    }
  } else {
    std::cout<<"[GEN] setting last point"<<std::endl;
    setLastPointsAndSendSuccesMsg();
    trajectory_active_=false;
    if(save_data)
      saveDataToFile();
  }
  std::cout<<"[GEN]done the big if"<<std::endl;
  if(trajectory_active_){
    std::cout<<"[GEN]started generating positions"<<std::endl;
    generatePositions(t);
    std::cout<<"[GEN]ended generating positions"<<std::endl;
  }
}

inline bool InternalSpaceTrapezoidTrajectoryGenerator::phaseTimeHasPassed(double t){
  return (phase_end_time_ <= t);
}

bool InternalSpaceTrapezoidTrajectoryGenerator::calculatePhaseDuration(double t){
  double duration_longest = 0.0;
  double duration;
  for(unsigned int i = 0; i < number_of_joints_; i++){
    duration = vel_profile_[i].calculateDuration(
                                  old_point_(i),
                                  trajectory_.points[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.points[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i]);
    if(duration > duration_longest){
      duration_longest = duration;
    }
  }
  phase_end_time_ = duration_longest + t;
}

void InternalSpaceTrapezoidTrajectoryGenerator::updateHookWithDurationBasedProfiles(ros::Time now, bool save_data) {
  if(!(trajectory_active_ && (trajectory_.header.stamp < now))){
    return;
  }

  double t = now.toSec();
  if(!isTheLastTrajectoryPointReached()){
    if(phaseTimeHasPassed(t)){
      calculatePhaseDuration(t);
      if(!setVelocityProfiles(t, save_data)){
        trajectory_active_=false;
        if(save_data)
          saveDataToFile();
      }
    }
  } else {
    setLastPointsAndSendSuccesMsg();
    trajectory_active_=false;
    if(save_data)
      saveDataToFile();
  }

  if(trajectory_active_){
    generatePositions(t);
  }

}

void InternalSpaceTrapezoidTrajectoryGenerator::updateHook() {
  update_hook_iter_++;
  //std::cout<<"[GEN]in generator"<<std::endl;
  port_generator_active_out_.write(true);
  //std::cout<<"[GEN]starting getNewGoalAndInitData"<<std::endl;
  getNewGoalAndInitData();
  //std::cout<<"[GEN]starting adjustTimeFrames"<<std::endl;
  ros::Time now = rtt_rosclock::host_now();
  adjustTimeFrames(now);

  if(research_mode_){
    //std::cout<<"[GEN]starting updateHookWithVelocityBasedProfiles"<<std::endl;
    updateHookWithVelocityBasedProfiles(now, true);
    //std::cout<<"[GEN]ended updateHookWithVelocityBasedProfiles"<<std::endl;
  } else {
    std::cout<<"[GEN]starting updateHookWithDurationBasedProfiles"<<std::endl;
    updateHookWithDurationBasedProfiles(now, false);
    std::cout<<"[GEN]ended updateHookWithDurationBasedProfiles"<<std::endl;
  }

  //std::cout<<"[GEN]started sending positions"<<std::endl;
  sendPositions(research_mode_);
  //lostd::cout<<"[GEN]ended sending positions"<<std::endl;

}

ORO_CREATE_COMPONENT(InternalSpaceTrapezoidTrajectoryGenerator)