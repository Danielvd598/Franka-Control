// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector> 

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

//impedance parameters
  size_t nDoF;
  double k;
  double b;
  double xd;
  double yd;
  double zd;
  double phi;
  double psi;
  double theta;
  double a4; double a7; double d1; double d3; double d5; double dF;
  Eigen::Matrix<double, 3, 3> I33;
  Eigen::Matrix<double, 3, 3> Ko;
  Eigen::Matrix<double, 3, 3> Kt;
  Eigen::Matrix<double, 3, 3> Go;
  Eigen::Matrix<double, 3, 3> Gt;
  Eigen::Matrix<double, 4, 4> Hv0;
  Eigen::Matrix<double, 7, 7> B;
  Eigen::Matrix<double, 6, 1> T100, T211, T322, T433, T544, T655, T766;
  Eigen::Matrix<double, 4, 4> H10_0, H20_0, H30_0, H40_0, H50_0, H60_0, H70_0; 

  struct Brockett_params {
    Eigen::Matrix<double, 6, 1> Twist;
    Eigen::Matrix<double,4, 4> H0;
  } Brockett_p [7];

  double trace(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> As(const 
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix);
  Eigen::Matrix<double, 6, 6> Adjoint(const Eigen::Matrix<double, 4, 4>& Hmat);
  
  /*struct Hn0_struct {
    int i;
  };*/
  
  struct Hn0_struct {
    Eigen::Matrix<double,4, 4> H0;
  } Hn0_matrices [1];
  struct Hn0_struct Brockett(const Eigen::Matrix<double, 7, 1>& q,
  struct Brockett_params *Brockett_str, size_t nBrockett);
  Eigen::Matrix<double, 4, 4> matrixExponential(
    const Eigen::Matrix<double, 6, 1>& T,double q_i);


};
}
