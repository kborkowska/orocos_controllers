/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#include <limits>
#include <assert.h>

#include "velocityprofile_spline.hpp"

namespace KDL {

static inline void generatePowers(int n, double x, double* powers) {
  powers[0] = 1.0;
  for (int i = 1; i <= n; i++) {
    powers[i] = powers[i - 1] * x;
  }
  return;
}


VelocityProfile_Spline::VelocityProfile_Spline() {
  fillPropertiesWithZeros();
  return;
}

VelocityProfile_Spline::VelocityProfile_Spline(
    const VelocityProfile_Spline &p) {
  duration_ = p.duration_;
  ap_time_ = p.ap_time_;
  dp_time_ = p.dp_time_;
  bg_time_ = p.bg_time_;

  ap_coeff_[0] = p.ap_coeff_[0];
  ap_coeff_[1] = p.ap_coeff_[1];
  ap_coeff_[2] = p.ap_coeff_[2];

  cvp_coeff_[0] = p.cvp_coeff_[0];
  cvp_coeff_[1] = p.cvp_coeff_[1];
  cvp_coeff_[2] = p.cvp_coeff_[2];

  dp_coeff_[0] = p.dp_coeff_[0];
  dp_coeff_[1] = p.dp_coeff_[1];
  dp_coeff_[2] = p.dp_coeff_[2];

  error_msg_ = p.error_msg_;

  return;
}

void VelocityProfile_Spline::fillPropertiesWithZeros(){
  duration_ = 0.0;
  ap_time_ = 0.0;
  dp_time_ = 0.0;
  bg_time_ = 0.0;

  ap_coeff_[0] = 0.0;
  ap_coeff_[1] = 0.0;
  ap_coeff_[2] = 0.0;

  cvp_coeff_[0] = 0.0;
  cvp_coeff_[1] = 0.0;
  cvp_coeff_[2] = 0.0;

  dp_coeff_[0] = 0.0;
  dp_coeff_[1] = 0.0;
  dp_coeff_[2] = 0.0;

  error_msg_ = " ";
}

std::string VelocityProfile_Spline::getErrorMsg() const{
  return error_msg_;
}

bool VelocityProfile_Spline::trajectoryIsFeasible(
      double distance, double vel1, double vel2,
      double vel_max, double acc_max,
      double lower_limit, double upper_limit, bool research_mode){

  int sign = distance/(abs(distance));

  // max_vel that is zero makes it impossible to calculate duration 
  // only possible when displacemat is zero, but it is assumed that this function
  // shuold not be used then (no displacemet -> no trajectory to speak of)
  if (vel_max == 0.0){
    error_msg_ = "Max velocity is zero. vel_max="+std::to_string(vel_max)+";";
  }
  // check wether boundary velocities are greater then max velocity
  // needed in research mode wich assumes max velocity is a physical
  // limit rather then velocity to be established in constant phase
  else if((abs(vel1) > vel_max ||
           abs(vel2) > vel_max) && 
           research_mode){
    error_msg_ = "Boundary velocity greater then vel_max. vel1="+std::to_string(vel1)+
                 "; vel2="+std::to_string(vel2)+"; vel_max="+std::to_string(vel_max)+";";
  }
  // check wether it is possible to achieve boundary velocities
  // when one of them is greater then max_velocity 
  // (here treated as const velocit rather then reachable limit, hence
  // in research mode even those cases won't be allowed)
  else if((sign*vel1 - vel_max > 0) || 
          (sign*vel2 - vel_max > 0)){
    error_msg_ = "Boundary velocity error. vel1="+std::to_string(vel1)+
                 "; vel2="+std::to_string(vel2)+"; vel_max="+std::to_string(vel_max)+";";
  }
  // check wether displacement is too small for given acceleration
  // page 71 in Trajectory Planning for Automatic Machines and Robots
  // by Luigi Biagiotti & Claudio Melchiorri 
  else if(acc_max*distance < (abs(vel1*vel1-vel2*vel2)/2)){
    error_msg_ = "Displacement is too small for given acceleration. vel1=" +std::to_string(vel1)+
                 "; vel2="+std::to_string(vel2)+"; acc_max="+std::to_string(acc_max)+
                 "; displacement="+std::to_string(distance)+ ";";
  }

  if(!error_msg_.compare(" ")){
    return false;
  } else {
    return true;
  }
}

bool VelocityProfile_Spline::calculateCoeffs(
  double pos1, double pos2, double vel1, double vel2, double vel_max){
  fillPropertiesWithZeros();
  if(duration_<= 0.0){
    error_msg_ = "Duration less, or equal zero with nonzero distance. duration_="+std::to_string(duration_)+
                 "; displacement="+std::to_string((pos2-pos1))+";";
    return false;
  }
  if(ap_time_>0){
    ap_coeff_[0] = pos1 - vel1*bg_time_ + (vel_max-vel1)*(bg_time_*bg_time_)/(2*ap_time_);
    ap_coeff_[1] = vel1 - (vel_max-vel1)*bg_time_/ap_time_;
    ap_coeff_[2] = (vel_max-vel1)/(2*ap_time_);
  }
  cvp_coeff_[0] = pos1 + (vel1-vel_max)*ap_time_/2 - vel_max*bg_time_;
  cvp_coeff_[1] = vel_max;
  cvp_coeff_[2] = 0.0;
  if(dp_time_>0){
    dp_coeff_[0] = pos2 - vel2*(bg_time_+duration_) - (vel_max-vel2)*((bg_time_+duration_)*(bg_time_+duration_))/(2*dp_time_);
    dp_coeff_[1] = vel2 + (vel_max-vel2)*(bg_time_+duration_)/dp_time_;
    dp_coeff_[2] = (vel2-vel_max)/(2*dp_time_);
  }
  return true;
}


bool VelocityProfile_Spline::maxVelocityIsReachable(
      double distance, double vel1, double vel2,
      double vel_max, double acc_max){
  if(distance*acc_max < (vel_max*vel_max - (vel1*vel1 + vel2*vel2)/2)){
    return false;
  }
  return true;
}

bool VelocityProfile_Spline::trajectoryBreachesPositionLimits(
      double pos1, double pos2, double lower_limit, double upper_limit){
    double time_a = (-1)*ap_coeff_[1]/(2*ap_coeff_[2]);
    double time_d = (-1)*dp_coeff_[1]/(2*dp_coeff_[2]);
    error_msg_ = " ";
    if(bg_time_ < time_a && 
       time_a < bg_time_+ap_time_){
      error_msg_ = "Position breached further into trajectory. time_a=" +std::to_string(time_a)+ ";";
    }
    if(bg_time_+duration_-dp_time_ < time_d && 
       time_d < bg_time_+duration_){
      if(!error_msg_.compare(" ")){
        error_msg_ = "Position breached further into trajectory. time_d=" +std::to_string(time_d)+ ";";
      } else {
        error_msg_.append(" time_d=" +std::to_string(time_d)+ ";");
      }
    }
    if(!error_msg_.compare(" ")){
      return true;
    } else {
      return false;
    }
  }

VelocityProfile_Spline::~VelocityProfile_Spline() {
  return;
}

void VelocityProfile_Spline::SetProfile(double pos1, double pos2) {
  assert(("Function SetProfile is unusable when dealing"
           "with trapezoidal velocity profile.", 0));
  return;
}

void VelocityProfile_Spline::SetProfileDuration(double pos1, double pos2,
                                                         double duration){
  assert(("Function SetProfileDuration is unusable due to lack of values"
           "for maximum velocity and acceleration", 0));
  return;
}



bool VelocityProfile_Spline::SetProfileTrapezoidal(
      double pos1, double pos2, double vel_max, double acc_max,
      double lower_limit, double upper_limit, bool research_mode){
  SetProfileTrapezoidal(pos1, pos2, 0.0, 0.0,vel_max, acc_max, 0.0, lower_limit, upper_limit, research_mode);
}



bool VelocityProfile_Spline::SetProfileTrapezoidal(
      double pos1, double pos2, double vel1, double vel2,
      double vel_max, double acc_max, 
      double lower_limit, double upper_limit, bool research_mode){
  SetProfileTrapezoidal(pos1, pos2, vel1, vel2, vel_max, acc_max, 0.0, lower_limit, upper_limit,research_mode);
}



bool VelocityProfile_Spline::SetProfileTrapezoidal(
      double pos1, double pos2, double vel1, double vel2,
      double vel_max, double acc_max, double time1,
      double lower_limit, double upper_limit, bool research_mode){

  error_msg_ = " ";
  double distance = pos2-pos1;
  int sign = distance/(abs(distance));
  bg_time_ = time1;

  if(distance == 0.0){
    fillPropertiesWithZeros();
    return true;
  }

  // if by accident the given max velocity is lower then 0,
  // and therefore making it impossible to compute dration
  // the program assumes it suppose to be an absolute value
  vel_max = abs(vel_max);

  if(!trajectoryIsFeasible(distance, vel1, vel2, vel_max, acc_max, lower_limit, upper_limit, research_mode)){
    return false;
  }

  //calculate duration of constant phase
  std::cout<<"vel_max:"<<vel_max<<" acc_max:"<<acc_max<<std::endl;
  duration_ = distance/vel_max + (vel_max/(2*acc_max))*(((1- sign*vel1/vel_max)*(1- sign*vel1/vel_max))+((1- sign*vel2/vel_max)*(1- sign*vel2/vel_max)));
  ap_time_ = abs(sign*vel_max-vel1)/acc_max;
  dp_time_ = abs(sign*vel_max-vel2)/acc_max;
  // check if, despite possibility of reching the goal, it will be done with smaller velocity
  if(!maxVelocityIsReachable(distance, vel1, vel2, vel_max, acc_max)){
    if(research_mode){
      error_msg_ = "Max velocity unreachable. vel1=" +std::to_string(vel1)+
                   "; vel2="+std::to_string(vel2)+"; vel_max="+std::to_string(vel_max)+
                   "; acc_max="+std::to_string(acc_max)+"; displacement="+std::to_string(distance)+";";
      return false;
    } else {
      vel_max=sqrt(distance*acc_max + (vel1*vel1 + vel2*vel2)/2);
      ap_time_=(vel_max-vel1)/acc_max;
      dp_time_=(vel_max-vel2)/acc_max;
      duration_=ap_time_+dp_time_;
    }
  }
  // change velocity according to the movement direction
  vel_max = sign*vel_max;

  if(!calculateCoeffs(pos1, pos2, vel1, vel2, vel_max)){
    return false;
  }

  if(trajectoryBreachesPositionLimits(pos1, pos2, lower_limit, upper_limit)){
    return false;
  }

  return true;

}



double VelocityProfile_Spline::Duration() const {
  return duration_;
}


double VelocityProfile_Spline::Pos(double time) const {
  double t[3];
  double position;
  generatePowers(2, time, t);

  if(time < bg_time_+ap_time_){
    position = t[0] * ap_coeff_[0] + t[1] * ap_coeff_[1] + t[2] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){
    position = t[0] * cvp_coeff_[0] + t[1] * cvp_coeff_[1] + t[2] * cvp_coeff_[2];
  }else{
    position = t[0] * dp_coeff_[0] + t[1] * dp_coeff_[1] + t[2] * dp_coeff_[2];
  }

  return position;
}

double VelocityProfile_Spline::Vel(double time) const {
  double t[2];
  double velocity;
  generatePowers(1, time, t);

  if(time < bg_time_+ap_time_){ 
    velocity = t[0] * ap_coeff_[1] + 2.0 * t[1] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){
    velocity = t[0] * cvp_coeff_[1] + 2.0 * t[1] * cvp_coeff_[2];
  }else{
    velocity = t[0] * dp_coeff_[1] + 2.0 * t[1] * dp_coeff_[2];
  }

  return velocity;
}

double VelocityProfile_Spline::Acc(double time) const {
  double t[1];
  double acceleration;
  generatePowers(0, time, t);

  if(time < bg_time_+ap_time_){
    acceleration = 2.0 * t[0] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){ 
    acceleration = 2.0 * t[0] * cvp_coeff_[2];
  }else{
    acceleration = 2.0 * t[0] * dp_coeff_[2];
  }

  return acceleration;
}

void VelocityProfile_Spline::Write(std::ostream& os) const {
  os << "coefficients : [ " << ap_coeff_[0] << " " << ap_coeff_[1] << " " << ap_coeff_[2]
      << " " << cvp_coeff_[0] << " " << cvp_coeff_[1]<< " " << cvp_coeff_[2]
      << " " << dp_coeff_[0] << " " << dp_coeff_[1]<< " " << dp_coeff_[2] << " ]";
  return;
}

VelocityProfile* VelocityProfile_Spline::Clone() const {
  return new VelocityProfile_Spline(*this);
}

}  // namespace KDL