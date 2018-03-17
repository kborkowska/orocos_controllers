#ifndef VELOCITYPROFILE_SPLINE_H
#define VELOCITYPROFILE_SPLINE_H

#include "kdl/velocityprofile.hpp"

namespace KDL
{
	/**
	 * \brief A spline VelocityProfile trajectory interpolation.
	 * @ingroup Motion
	 */
class VelocityProfile_Spline : public VelocityProfile
{
public:
    VelocityProfile_Spline();
    VelocityProfile_Spline(const VelocityProfile_Spline &p);

	virtual ~VelocityProfile_Spline();
	
    virtual void SetProfile(double pos1, double pos2);

    virtual void SetProfileDuration(
      double pos1, double pos2, double duration);
    /**
     * Generate trapezoid interpolation coeffcients.
     *
     * @param pos1 begin position.
     * @param pos2 end position.
     * @param vel max velocity.
     * @param acc max acceleration.
     */
    virtual bool SetProfileTrapezoidal(
      double pos1, double pos2, double vel_max, double acc_max,
      double lower_limit, double upper_limit, bool research_mode);
    /**
     * Generate trapezoid interpolation coeffcients.
     *
     * @param pos1 begin position.
     * @param pos2 end position.
     * @param vel1 initial velocity.
     * @param vel2 final velocity.
     * @param vel_max max acceleration.
     * @param acc_max max acceleration.
     * @param time1 time of the movement beggining.
     */
    virtual bool SetProfileTrapezoidal(
      double pos1, double pos2, double vel1, double vel2,
      double vel_max, double acc_max, double time1,
      double lower_limit, double upper_limit, bool research_mode);
        /**
     * Generate trapezoid interpolation coeffcients.
     *
     * @param pos1 begin position.
     * @param pos2 end position.
     * @param vel1 initial velocity.
     * @param vel2 final velocity.
     * @param vel_max max acceleration.
     * @param acc_max max acceleration.
     */
    virtual bool SetProfileTrapezoidal(
      double pos1, double pos2, double vel1, double vel2,
      double vel_max, double acc_max,
      double lower_limit, double upper_limit, bool research_mode);

    virtual double Duration() const;
    virtual double Pos(double time) const;
    virtual double Vel(double time) const;
    virtual double Acc(double time) const;
    virtual void Write(std::ostream& os) const;
    virtual VelocityProfile* Clone() const;
    virtual std::string getErrorMsg() const;
private:

/*
        ______________     ___ v_max
       /|            |\
      / |            | \
     /  |            |  \  ___ v_final
    /   |                  ___ v_initial
    +---+------------+--+
    |at |            |dt|
    |      duration     |
    */
    double ap_time_; //acceleration time (at)
    double dp_time_; //deceleration time (dt)
    double bg_time_; //begining time
    double duration_; //movement duration
    double ap_coeff_[3]; //acceleration phase coeffs
    double cvp_coeff_[3]; //constant velocity phase coeffs
    double dp_coeff_[3]; // decceleration phase coeffs 
    std::string error_msg_;

    virtual bool trajectoryIsFeasible(
      double distance, double vel1, double vel2,
      double vel_max, double acc_max,
      double lower_limit, double upper_limit, bool research_mode);
    virtual void fillPropertiesWithZeros();
    virtual bool calculateCoeffs(double pos1, double pos2, double vel1, double vel2, double vel_max);
    virtual bool maxVelocityIsReachable(double distance, double vel1, double vel2,
                                        double vel_max, double acc_max);
    virtual bool trajectoryBreachesPositionLimits(
      double pos1, double pos2, double lower_limit, double upper_limit);
};
}
#endif // VELOCITYPROFILE_CUBICSPLINE_H
