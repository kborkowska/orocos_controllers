/*
 * Copyright (c) 2010-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * InterrnalSpaceTrajectoryAction.cpp
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz
 */

#include "InternalSpaceTrapezoidTrajectoryAction.h"
#include <rtt/Component.hpp>
#include <string>

#include "rtt_rosclock/rtt_rosclock.h"
#include "eigen_conversions/eigen_msg.h"

#include <std_msgs/Float64.h>

InternalSpaceTrapezoidTrajectoryAction::InternalSpaceTrapezoidTrajectoryAction(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      //RTT::Property<int> numberOfJoints_prop_
      numberOfJoints_prop_("number_of_joints", "", 0),
      //RTT::InputPort<trajectory_msgs::JointTrajectory> command_port_
      command_port_("command") {
  // Add action server ports to this task's root service
  //rtt_actionlib::RTTActionServer<trapezoidal_trajectory_msgs::TrapezoidTrajectoryAction> as_
  as_.addPorts(this->provides());

  // Bind action server goal and cancel callbacks (see below)
  as_.registerGoalCallback(
      boost::bind(&InternalSpaceTrapezoidTrajectoryAction::goalCB, this, _1));
  as_.registerCancelCallback(
      boost::bind(&InternalSpaceTrapezoidTrajectoryAction::cancelCB, this, _1));

  this->addPort("trajectoryPtr", trajectory_ptr_port_);
  this->addPort("JointPosition", port_joint_position_);
  this->addPort("JointPositionCommand", port_joint_position_command_);
  this->addPort("GeneratorResult", port_generator_result_);
  this->addEventPort(
      command_port_,
      boost::bind(&InternalSpaceTrapezoidTrajectoryAction::commandCB, this));
  this->addProperty("joint_names", jointNames_);
  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
}

InternalSpaceTrapezoidTrajectoryAction::~InternalSpaceTrapezoidTrajectoryAction() {
}

bool InternalSpaceTrapezoidTrajectoryAction::configureHook() {
// configuration of a hook that is called in updateHook every
// time quant given in irp6_ui/irp6_bringup/config/irp6-p-inside.ops
// currently (13.11.2017) set to 0.01 sec (100Hz)


  //check if there are any joints asigned
  if (jointNames_.size() <= 0) {
    return false;
  }

  // asign number of joints according to size of name table size
  numberOfJoints_ = jointNames_.size();

  //trapezoidal_trajectory_msgs::TrapezoidTrajectoryFeedback feedback_
  //describes parameters for comparison
  feedback_.actual.positions.resize(numberOfJoints_);
  feedback_.desired.positions.resize(numberOfJoints_);
  feedback_.error.positions.resize(numberOfJoints_);
  feedback_.joint_names.resize(numberOfJoints_);

  //velocity profile, describes velocities at given  points
  //vel_profile_.resize(number_of_joints_);

  //adds names of joints to feedback_ table
  for (int i = 0; i < jointNames_.size(); i++) {
    feedback_.joint_names.push_back(jointNames_[i]);
  }

  //remapTable is used to remap joints between a given goal table
  //and corresponding spots in jointNames_ table (in goalCB)
  remapTable_.resize(numberOfJoints_);

  //checks wether limits have been loaded properly
  if (lowerLimits_.size() != numberOfJoints_
      || upperLimits_.size() != numberOfJoints_) {
    RTT::Logger::log(RTT::Logger::Error) << "Limits not loaded"
        << RTT::endlog();
    return false;
  }

  //this.resArray

  return true;
}

bool InternalSpaceTrapezoidTrajectoryAction::startHook() {
  //rtt_actionlib::RTTActionServer<trapezoidal_trajectory_msgs::TrapezoidTrajectoryAction> as_
  as_.start();
  goal_active_ = false;

  return true;
}

bool InternalSpaceTrapezoidTrajectoryAction::positionViolationOccurs(Goal g){
  for (int i = 0; i < numberOfJoints_; i++) {
    for (int j = 0; j < g->goal_tolerance.size(); j++) {
      if (g->goal_tolerance[j].name == g->trajectory.joint_names[i]) {
        if (joint_position_[remapTable_[i]] + g->goal_tolerance[j].position
              < g->trajectory.points[g->trajectory.points.size() - 1].positions[i]
            || joint_position_[remapTable_[i]]- g->goal_tolerance[j].position
              > g->trajectory.points[g->trajectory.points.size() - 1].positions[i]) 
        {
          RTT::Logger::log(RTT::Logger::Debug) << g->goal_tolerance[j].name
                                               << " violated with position "
                                               << joint_position_[remapTable_[i]]
                                               << RTT::endlog();
          return true;
        }
      }
    }
  }
  return false;
}

void InternalSpaceTrapezoidTrajectoryAction::updateHook() {
  //checking for joint position for feedback publishing

  bool joint_position_data = true;
  if (port_joint_position_.read(joint_position_) == RTT::NoData) {
    joint_position_data = false;
  }
  //what the generator ha given as a setpoint for this iteration
  port_joint_position_command_.read(desired_joint_position_);
  //end goal

  Goal g = activeGoal_.getGoal();
  //msgs for comunication with generator
  trapezoidal_trajectory_msgs::TrapezoidTrajectoryResult res;
  trapezoidal_trajectory_msgs::TrapezoidGeneratorResult genMsg = 
    trapezoidal_trajectory_msgs::TrapezoidGeneratorResult();

  if (port_generator_result_.read(genMsg) == RTT::NewData) {
    std::cout<<"got data form generator"<<std::endl;
    if(genMsg.error_code == 0){
      if (positionViolationOccurs(g)) {
        res.result.error_code =
            trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::GOAL_TOLERANCE_VIOLATED;
        activeGoal_.setAborted(res, "");
      } else {
        res.result.error_code = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::SUCCESSFUL;
        activeGoal_.setSucceeded(res, "");
      }
    } else {
      res.result.error_code = genMsg.error_code;
      res.result.error_string = genMsg.error_string;
      activeGoal_.setAborted(res, "");
    }
    goal_active_ = false;
  }
  // if the goal is still unreached and the component was able to
  // recieve data about joint position feedback is send to irpos api
  if (goal_active_ && joint_position_data) {
    ros::Time now = rtt_rosclock::host_now();
    if(pathToleranceBreached(g)){
      res.result.error_code =
            trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::PATH_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res, "");
      goal_active_ = false;
    } else {
      // fealing feedback msg
      for (int i = 0; i < numberOfJoints_; i++) {
        feedback_.actual.positions[i] = joint_position_[i];
        feedback_.desired.positions[i] = desired_joint_position_[i];
        feedback_.error.positions[i] = joint_position_[i]
            - desired_joint_position_[i];
      }
      feedback_.header.stamp = rtt_rosclock::host_now();
      activeGoal_.publishFeedback(feedback_);
    }
  }
}

bool InternalSpaceTrapezoidTrajectoryAction::pathToleranceBreached(Goal g){

  if(g->research_mode){
    return false;
  }
  for (int i = 0; i < g->path_tolerance.size(); i++) {
    for (int j = 0; j < jointNames_.size(); j++) {
      if (jointNames_[j] == g->path_tolerance[i].name) {
        if (fabs(joint_position_[j] - desired_joint_position_[j])
            > g->path_tolerance[i].position) {
          RTT::Logger::log(RTT::Logger::Error) << "Path tolerance violated"
              << RTT::endlog();
          return true;
        }
      }
    }
  }
  return false;
}


bool InternalSpaceTrapezoidTrajectoryAction::fillRemapTable(Goal g){

    unsigned int j;
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
      for (j = 0; j < g->trajectory.joint_names.size(); j++) {
        if (g->trajectory.joint_names[j] == jointNames_[i]) {
          remapTable_[i] = j;
          break;
        }
      }
      if (j == g->trajectory.joint_names.size()) {
        RTT::Logger::log(RTT::Logger::Error)
            << "Trajectory contains invalid joint" << RTT::endlog();
        return false;
      }
    }
    return true;
}

bool InternalSpaceTrapezoidTrajectoryAction::trajectoryHasInvalidPoints(Goal g){
  for (unsigned int i = 0; i < numberOfJoints_; i++) {
    for (int j = 0; j < g->trajectory.points.size(); j++) {
      const double joint_position = g->trajectory.points[j].positions[i];
      if (joint_position > upperLimits_[remapTable_[i]] || 
          joint_position < lowerLimits_[remapTable_[i]]) 
      {
        RTT::Logger::log(RTT::Logger::Debug) << 
            "Invalid goal [" << i << "]: " << 
            upperLimits_[remapTable_[i]] << ">" << 
            joint_position << ">" << 
            lowerLimits_[remapTable_[i]] << RTT::endlog();
        return true;
      }
    }
  }
  return false;
}

bool InternalSpaceTrapezoidTrajectoryAction::remapJointsAndLimits(
                trajectory_msgs::JointTrajectory* trj_ptr, Goal g, double* max_vel, double* max_acc){

  trj_ptr->header = g->trajectory.header;
  trj_ptr->points.resize(g->trajectory.points.size());


  if(!g->duration_mode){
    //check wether right ammount of vel and acc limitations was given
    if(g->max_velocities.size() != numberOfJoints_ ||
       g->max_accelerations.size() != numberOfJoints_){
      RTT::Logger::log(RTT::Logger::Debug) << 
            "not enough limit values for velocites for"<<
            " accelerations for research mode";
      return false;
    } else {
      //remap max_vel and max_acc tables
      for (unsigned int j = 0; j < numberOfJoints_;j++) {
        max_vel[j] = g->max_velocities[remapTable_[j]];
        max_acc[j] = g->max_accelerations[remapTable_[j]];
      }
    }
  }
  for (unsigned int i = 0; i < g->trajectory.points.size(); i++) {
    trj_ptr->points[i].positions.resize(
        g->trajectory.points[i].positions.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].positions.size();
        j++) {
      trj_ptr->points[i].positions[j] =
          g->trajectory.points[i].positions[remapTable_[j]];
    }

    trj_ptr->points[i].velocities.resize(
        g->trajectory.points[i].velocities.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size();
        j++) {
      trj_ptr->points[i].velocities[j] =
          g->trajectory.points[i].velocities[remapTable_[j]];
    }

    trj_ptr->points[i].accelerations.resize(
        g->trajectory.points[i].accelerations.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].accelerations.size();
        j++) {
      trj_ptr->points[i].accelerations[j] = g->trajectory.points[i]
          .accelerations[remapTable_[j]];
    }

    trj_ptr->points[i].time_from_start = g->trajectory.points[i]
        .time_from_start;
  }

  return true;
}

bool InternalSpaceTrapezoidTrajectoryAction::getPeersReady(){
  bool peers_ready_ = true;

  RTT::TaskContext::PeerList peers = this->getPeerList();
  for (size_t i = 0; i < peers.size(); i++) {
    RTT::Logger::log(RTT::Logger::Debug) << "Starting peer : " << peers[i]
        << RTT::endlog();
    peers_ready_ = peers_ready_ && this->getPeer(peers[i])->start();
  }
  return peers_ready_;
}

void InternalSpaceTrapezoidTrajectoryAction::goalCB(GoalHandle gh) {
  if (!goal_active_) {
    trajectory_msgs::JointTrajectory* trj_ptr =
        new trajectory_msgs::JointTrajectory;
    Goal g = gh.getGoal();

    trapezoidal_trajectory_msgs::TrapezoidTrajectoryResult res;

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain "
        << g->trajectory.points.size() << " points" << RTT::endlog();

    if(!fillRemapTable(g)){
      res.result.error_code =
          trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::INVALID_JOINTS;
      gh.setRejected(res, "");
      return;
    }
    std::cout<<"filled remap table"<<std::endl;
    // Sprawdzenie ograniczeń w jointach INVALID_GOAL
    if(trajectoryHasInvalidPoints(g)){
      res.result.error_code = trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::INVALID_GOAL;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }
    std::cout<<"trajectory has no invalid points"<<std::endl;
    double max_vel[numberOfJoints_];
    double max_acc[numberOfJoints_];
    // Remap joints
    if(!remapJointsAndLimits(trj_ptr, g, max_vel, max_acc)){
      res.result.error_code = trapezoidal_trajectory_msgs::
                        TrapezoidGeneratorResult::
                        INVALID_LIMIT_ARRAY;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }
    std::cout<<"remaped joints and set limits"<<std::endl;
    // Sprawdzenie czasu w nagłówku OLD_HEADER_TIMESTAMP
    if (g->trajectory.header.stamp < rtt_rosclock::host_now()) {
      RTT::Logger::log(RTT::Logger::Debug) << "Old header timestamp"
          << RTT::endlog();
      res.result.error_code =
          trapezoidal_trajectory_msgs::TrapezoidGeneratorResult::OLD_HEADER_TIMESTAMP;
      gh.setRejected(res, "");
    }

    activeGoal_ = gh;
    goal_active_ = true;

    if (getPeersReady()) {
      trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal trj_cptr =
          trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal();
      std::cout<<"got peers ready"<<std::endl;
      trj_cptr.trajectory = *trj_ptr;
      trj_cptr.save_data = g->save_data;
      trj_cptr.research_mode = g->research_mode;
      trj_cptr.duration_mode = g->duration_mode;
      for (unsigned int j = 0; j < numberOfJoints_;j++) {
        trj_cptr.max_velocities.push_back(max_vel[j]);
        trj_cptr.max_accelerations.push_back(max_acc[j]);
      }
      trajectory_ptr_port_.write(trj_cptr);
      std::cout<<"send data to generator"<<std::endl;
      gh.setAccepted();
      goal_active_ = true;
    } else {
      gh.setRejected();
      goal_active_ = false;
    }
  } else {
    gh.setRejected();
  }
  std::cout<<"finished goal pocessing"<<std::endl;
}

void InternalSpaceTrapezoidTrajectoryAction::cancelCB(GoalHandle gh) {
  goal_active_ = false;
}

void InternalSpaceTrapezoidTrajectoryAction::commandCB() {
}

/*bool InternalSpaceTrapezoidTrajectoryAction::check_trajectory_limits(JointTrajectoryPoint point, int remapTable_index){

}*/

ORO_CREATE_COMPONENT(InternalSpaceTrapezoidTrajectoryAction)

