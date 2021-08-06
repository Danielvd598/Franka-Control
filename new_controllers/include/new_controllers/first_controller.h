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

  /**
   * trajectory_state = 0: homing above peg
   * trajectory_state = 1: going down to pick-up peg
   * trajectory_state = 2: homing above hole
   * trajectory_state = 3: going to to place peg
   * */
  int trajectory_state; 
  
  /**
   * control_state = 0: initialising
   * control_state = 1: joint space PD control
   * control_state = 2: Cartesian space TB+TF impedance control
   * */
  int control_state; 
  int modulation_counter; // counts how long the stiffness is modulated
  size_t nDoF; 
  bool use_optimisation, TaskBased, dataPrint, use_modulated_TF, use_cyclic;
  std::string torque_path, Hv0_path, qi_path, t_flag_path, dataAnalysis_tau_TB_path,
  dataAnalysis_tau_TF_path, dataAnalysis_dq_path, dataAnalysis_q_path;
  double kt, ko, b; //impedance control paramaters
  double cycle_wait_period; //how long the programm should wait before repeating task [ms]
  double xd, yd, zd, phi, psi, theta; // desired configuration
  double kp, kd; //PD join space control parameters
  double a4; double a7; double d1; double d3; double d5; double dF; double dGripper;
  double accuracy_thr; //accuracy necessary before grabbing/releasing peg
  double modulation_factor; //adjusting the slope of the modulated stiffness
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
  Eigen::Matrix<double, 4, 1> t_flag; //flags concerning the trajectory phases
  std::ifstream inFile;
  double num;
  size_t update_calls;
  std::vector<double> tau_TB_index, Hv0_index, qi_index, t_flag_index;
  Eigen::Matrix<double, 7, Eigen::Dynamic> tau_TB_mat;
  std::ofstream dataAnalysis_tau_TB, dataAnalysis_tau_TF, dataAnalysis_dq, 
  dataAnalysis_q;

  struct Brockett_params {
    Eigen::Matrix<double, 6, 1> Twist;
    Eigen::Matrix<double,4, 4> H0;
  } Brockett_p [7]; //this structure is 7 items long

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
