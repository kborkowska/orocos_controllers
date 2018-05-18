/*
 * Copyright (c) 2010, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * InternalSpaceTrapezoidTrajectoryGenerator.h
 *
 * Generator for both the motor and joint spline interpolation
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef InternalSpaceTrapezoidTrajectoryGenerator_H_
#define InternalSpaceTrapezoidTrajectoryGenerator_H_

#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Int16.h>
#include <string>
#include <vector>

#include <trapezoidal_trajectory_msgs/TrapezoidGeneratorResult.h>
#include <trapezoidal_trajectory_msgs/TrapezoidGeneratorGoal.h>

#include "velocityprofile_trapezoid.hpp"

class InternalSpaceTrapezoidTrajectoryGenerator : public RTT::TaskContext {
 public:
  explicit InternalSpaceTrapezoidTrajectoryGenerator(const std::string& name);
  virtual ~InternalSpaceTrapezoidTrajectoryGenerator();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 protected:
  RTT::InputPort<trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal> port_trajectory_in_;

  RTT::OutputPort<Eigen::VectorXd> port_internal_space_position_command_out_;
  RTT::OutputPort<trapezoidal_trajectory_msgs::TrapezoidGeneratorResult> port_generator_result_;
  RTT::InputPort<Eigen::VectorXd> port_internal_space_position_measurement_in_;
  RTT::OutputPort<bool> port_generator_active_out_;
  RTT::InputPort<bool> port_is_synchronised_in_;

 private:
  virtual void saveDataToFile() const;
  trapezoidal_trajectory_msgs::TrapezoidGeneratorGoal getNewGoalAndInitData(void);
  void adjustTimeFrames(ros::Time now);
  void sendPositions();
  bool isAnyJointStillInMotion();
  bool isTheLastTrajectoryPointReached();
  bool setVelocityProfiles(double t, bool is_velocity_based_);
  void setLastPointsAndSendSuccesMsg();
  void generatePositions(double t, bool is_velocity_based_);
  void updateHookWithVelocityBasedProfiles(ros::Time now);
  bool phaseTimeHasPassed(double t);
  bool calculatePhaseDuration(double t);
  void updateHookWithDurationBasedProfiles(ros::Time now);
  void prf(double x, std::string name = "[GEN]: ");

  //bool last_point_not_set_;
  bool trajectory_active_;
  std::vector<bool> active_points_; 
  std::vector<KDL::VelocityProfile_Trapezoid> vel_profile_;

  Eigen::VectorXd des_jnt_pos_, setpoint_, old_point_;

  std::vector <Eigen::VectorXd> setpoint_results_, jnt_results_;

  trajectory_msgs::JointTrajectory trajectory_;
  int trajectory_point_index_;

  ros::Time last_time_;
  int update_hook_iter_;

  int ns_higher_bound_;
  int ns_higher_increment_;
  int ns_lower_bound_;
  int ns_lower_increment_;

  // properties
  int number_of_joints_;
  int ns_interval_;
  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;
  std::vector <double> old_velocities_, end_times_, max_velocities_, max_accelerations_;

  double phase_end_time_;

  bool research_mode_;
  bool save_data_;

  std::string errMsg_;
  std_msgs::Int16 errNo_;
};

#endif  // InternalSpaceTrapezoidTrajectoryGenerator_H_

