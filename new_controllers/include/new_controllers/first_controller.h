// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector> 
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace new_controllers {

class FirstController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  ros::Publisher gripper_flag_pub;

  /***
   * gripper_flag: 0 - do nothing
   * gripper_flag: 1 - grasp
   * gripper_flag: 2 - release
   * ***/
  int gripper_flag;
  
 private:
 // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

  /***
   * gripper_status: filling this array makes sure that the gripper action is completed 
   * for a sudden amount of time
   * ***/
  Eigen::Matrix<double, Eigen::Dynamic, 1>  gripper_status;

  /**
   * trajectory_state = 0: homing above peg
   * trajectory_state = 1: going down to pick-up peg
   * trajectory_state = 2: homing above hole
   * trajectory_state = 3: going to to place peg
   * */
  int trajectory_state; 
  
  /**
   * accuracy_flag = 0: wait before continuing to next flag
   * accuracy_flag = 1: accurate enough to continue trajectory
   * **/
  int accuracy_flag;

  /**
   * control_state = 0: initialising
   * control_state = 1: joint space PD control
   * control_state = 2: Cartesian space TB+TF impedance control
   * control_state = 3: draining kinetic energy
   * control_state = 4: compliance
   * */
  int control_state; 
  int ko_modulation_counter, kt_modulation_counter, b_modulation_counter; // counts how long the stiffness is modulated
  size_t Njoints, optimisation_length; 
  bool use_TB, dataPrint, use_modulated_TF, use_dynamic_injection, fail, drained;
  std::string torque_path, Hv0_path, qi_path, t_flag_path, qdot_path, tauc_gravity_path,
  dataAnalysis_tau_TB_path, dataAnalysis_tau_TF_path, dataAnalysis_dq_path, 
  dataAnalysis_q_path, dataAnalysis_tau_measured_path, dataAnalysis_tau_desired_path,
  dataAnalysis_xyz_ref_path;
  double kt, ko, b; //impedance control paramaters
  double bdrain; //drainage damping
  double epsP, epsE; //Power and Energy margins for collision detection/energy tanks
  double Ek_drained; //maximum kinetic energy of the robot when it is assumed to be drained
  double Ek; //current total kinetic energy of the robot
  double kp, kd; //PD join space control parameters
  double alpha; //lowpassfilter constant
  double a4; double a7; double d1; double d3; double d5; double dF; double dGripper;
  double accuracy_thr; //accuracy necessary before grabbing/releasing peg
  double kt_modulation_factor, ko_modulation_factor, ktmax, komax, b_modulation_factor, bmax; //adjusting modulated impedance
  ros::WallTime t1, t2; //timer between collision detection and energy draining
  Eigen::Matrix<double, 7, 1> Etank, Etank_margin; 
  Eigen::Matrix<double, 7, 2> x; //low-pass filter state
  Eigen::Matrix<double, 3, 1> w4, r4, w5, r5, w7, r7;
  Eigen::Matrix<double, 3, 3> I33;
  Eigen::Matrix<double, 3, 3> Ko;
  Eigen::Matrix<double, 3, 3> Kt;
  Eigen::Matrix<double, 3, 3> Go;
  Eigen::Matrix<double, 3, 3> Gt;
  Eigen::Matrix<double, 4, 4> Hv0, Hv0_optimised;
  Eigen::Matrix<double, 7, 7> B;
  Eigen::Matrix<double, 6, 1> T100, T211, T322, T433, T544, T655, T766;
  Eigen::Matrix<double, 6, 1> T210, T320, T430, T540, T650, T760;
  Eigen::Matrix<double, 4, 4> H10_0, H20_0, H30_0, H40_0, H50_0, H60_0, H70_0; 
  Eigen::Matrix<double, 7, 1> qi; //initial desired joint positions
  Eigen::Matrix<double, 7, 1> P_meas; //measured power
  Eigen::Matrix<double, 6, 1> t_flag; //flags concerning the trajectory phases
  std::ifstream inFile;
  double num;
  size_t update_calls; //Task-Based and trajectory indices
  size_t gripper_calls; //how often the gripper is called to do something
  std::vector<double> tau_TB_index, Hv0_index, qi_index, t_flag_index, qdot_index, 
  tauc_gravity_index;
  Eigen::Matrix<double, 7, Eigen::Dynamic> tau_TB_mat, qdot_mat, tauc_gravity_mat;
  Eigen::Matrix<double, 7, Eigen::Dynamic> P_opt; //Power consumption based on optimisation
  std::ofstream dataAnalysis_tau_TB, dataAnalysis_tau_TF, dataAnalysis_dq, 
  dataAnalysis_q, dataAnalysis_tau_measured, dataAnalysis_tau_desired,
  dataAnalysis_xyz_ref;

  struct Brockett_params {
    Eigen::Matrix<double, 6, 1> Twist;
    Eigen::Matrix<double,4, 4> H0;
  } Brockett_p [7]; //this structure is 7 items long

  Eigen::Matrix<double,7,1> LowPassFilter(const Eigen::Matrix<double, 7, 1>& u);
  double trace(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> As(const 
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
  Eigen::Matrix<double, 6, 6> Adjoint(const Eigen::Matrix<double, 4, 4>& Hmat);
  
  struct Hn0_struct {
    Eigen::Matrix<double,4, 4> H0;
  } Hn0_matrices, Hi0;

  struct Hn0_struct Brockett(const Eigen::Matrix<double, 7, 1>& q,
  struct Brockett_params *Brockett_str, size_t nBrockett, size_t n);
  Eigen::Matrix<double, 4, 4> matrixExponential(
    const Eigen::Matrix<double, 6, 1>& T,double q_i);

  struct Hv0_struct {
    Eigen::Matrix<double,4, 4> H;
  };

  Hv0_struct *Hv0_matrices;

};
}
