#include "kdl_ros_control/kdl_planner.h"
#include <cmath>

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, double _trajRadius, int _choice)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_ = _trajRadius;
    choice_ = _choice;
}

/*CIRCULAR TRAJECTORY CONSTRUCTOR DEFINITION*/
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

/*TRAPEZOIDAL VELOCITY PROFILE*/
void KDLPlanner::trapezoidal_vel(double time, double &s, double &dot_s, double &ddot_s)
{
  double ddot_traj_s = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

  if(time <= accDuration_)
  {
    s = 0.5*ddot_traj_s*std::pow(time,2);
    dot_s = ddot_traj_s*time;
    ddot_s = ddot_traj_s;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = ddot_traj_s*accDuration_*(time-accDuration_/2);
    dot_s = ddot_traj_s*accDuration_;
    ddot_s = 0.0;
  }
  else
  {
    s = 1 - 0.5*ddot_traj_s*std::pow(trajDuration_-time,2);
    dot_s = ddot_traj_s*(trajDuration_-time);
    ddot_s = -ddot_traj_s;
  }
}

/*CUBIC POLYNOMIAL VELOCITY PROFILE*/
void KDLPlanner::cubic_polinomial(double time, double &s, double &dot_s,double &ddot_s)
{
  double a_0 = 0.0;
  double a_1 = 0.0;
  double a_2 = 3/std::pow(trajDuration_,2);
  double a_3 = -2/(std::pow(trajDuration_,3));

  s = a_3*std::pow(time,3)+a_2*std::pow(time,2)+a_1*time+a_0;          // s -> position
  dot_s = 3*a_3*std::pow(time,2)+2*a_2*time+a_1;                       // dot_s -> velocity
  ddot_s = 6*a_3*time+2*a_2;                                           // ddot_s -> acceleration
}

/*LINEAR TRAJECTORY*/
trajectory_point KDLPlanner::linear_trajectory(double &s, double &dot_s,double &ddot_s){
  trajectory_point traj_lin;

  traj_lin.pos=(1-s)*trajInit_+s*trajEnd_;
  traj_lin.vel=(-trajInit_+trajEnd_)*dot_s;
  traj_lin.acc=(-trajInit_+trajEnd_)*ddot_s;

  //std::cout<<"pos: "<<traj_lin.pos[1]<<" "<<traj_lin.pos[2]<<" "<<traj_lin.pos[3] <<std::endl;
  return traj_lin;
}

/*CIRCULAR TRAJECTORY*/
trajectory_point KDLPlanner::circular_trajectory(double &s, double &dot_s,double &ddot_s){
  trajectory_point traj_cir;

  // POSITION
  traj_cir.pos[0] = trajInit_[0];                                                                       // x
  traj_cir.pos[1] = (trajInit_[1] + trajRadius_) - trajRadius_*cos(2*M_PI*s);                           // y
  traj_cir.pos[2] = trajInit_[2] - trajRadius_*sin(2*M_PI*s);                                           // z

  // VELOCITY
  traj_cir.vel[0] = 0.0;                                                                                // x_dot
  traj_cir.vel[1] = trajRadius_*(2*M_PI)*dot_s*sin(2*M_PI*s);                                           // y_dot
  traj_cir.vel[2] = -trajRadius_*(2*M_PI)*dot_s*cos(2*M_PI*s);                                          // z_dot

  // ACCELERATION
  traj_cir.acc[0] = 0.0;                                                                                // x_ddot
  traj_cir.acc[1] = trajRadius_*(2*M_PI)*(dot_s*dot_s*2*M_PI*cos(2*M_PI*s)+ddot_s*sin(2*M_PI*s));       // y_ddot
  traj_cir.acc[2] = -trajRadius_*(2*M_PI)*(-dot_s*dot_s*2*M_PI*sin(2*M_PI*s)+ddot_s*cos(2*M_PI*s));     // z_ddot

  //std::cout<<"pos: "<<traj_cir.pos[1]<<" "<<traj_cir.pos[2]<<" "<<traj_cir.pos[3] <<std::endl;
  return traj_cir;  
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  if(choice_==1){
    double s_time, dot_s_time, ddot_s_time;
    trapezoidal_vel(time,s_time,dot_s_time,ddot_s_time);

    //std::cout<<"time: "<<time<<" pos: "<<s_time<<" vel: " <<dot_s_time<<" acc: "<<ddot_s_time<<std::endl;
    return linear_trajectory(s_time,dot_s_time,ddot_s_time);
  }
  if(choice_==2){
    double s_time, dot_s_time, ddot_s_time;
    cubic_polinomial(time,s_time,dot_s_time,ddot_s_time);

    //std::cout<<"time: "<<time<<" pos: "<<s_time<<" vel: " <<dot_s_time<<" acc: "<<ddot_s_time<<std::endl;
    return linear_trajectory(s_time,dot_s_time,ddot_s_time);
  }
  if(choice_==3){
    double s_time, dot_s_time, ddot_s_time;
    trapezoidal_vel(time,s_time,dot_s_time,ddot_s_time);

    //std::cout<<"time: "<<time<<" pos: "<<s_time<<" vel: " <<dot_s_time<<" acc: "<<ddot_s_time<<std::endl;
    return circular_trajectory(s_time,dot_s_time,ddot_s_time);
  }
  if(choice_==4){
    double s_time, dot_s_time, ddot_s_time;
    cubic_polinomial(time,s_time,dot_s_time,ddot_s_time);

    //std::cout<<"time: "<<time<<" pos: "<<s_time<<" vel: " <<dot_s_time<<" acc: "<<ddot_s_time<<std::endl;
    return circular_trajectory(s_time,dot_s_time,ddot_s_time);
  }
}