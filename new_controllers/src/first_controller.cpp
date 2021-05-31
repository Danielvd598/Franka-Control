#include <new_controllers/first_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace new_controllers {

bool FirstController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FirstController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
    franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  node_handle.getParam("/first_controller/k",k);
  node_handle.getParam("/first_controller/b",b);
  node_handle.getParam("/first_controller/xd",xd);
  node_handle.getParam("/first_controller/yd",yd);
  node_handle.getParam("/first_controller/zd",zd);
  node_handle.getParam("/first_controller/phi",phi);
  node_handle.getParam("/first_controller/psi",psi);
  node_handle.getParam("/first_controller/theta",theta);
  return true;
}

void FirstController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

  I33 << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

  I33 << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  Ko << k, 0, 0,
       0, k, 0,
       0, 0, k;
  /*Ko << 0, 0, 0,
       0, 0, 0,
       0, 0, 0;*/
  Kt << k, 0, 0,
       0, k, 0,
       0, 0, k;
  Go = 0.5*trace(Ko)*I33 - Ko;
  Gt = 0.5*trace(Kt)*I33 - Kt;
  B << b, 0, 0, 0, 0, 0, 0,
       0, b, 0, 0, 0, 0, 0,
       0, 0, b, 0, 0, 0, 0,
       0, 0, 0, b, 0, 0, 0,
       0, 0, 0, 0, b, 0, 0,
       0, 0, 0, 0, 0, b, 0,
       0, 0, 0, 0, 0, 0, b;

  Hv0 << cos(theta)*cos(psi), -cos(phi)*sin(theta) + sin(psi)*sin(phi)*cos(theta), 
  sin(theta)*sin(phi) + cos(theta)*sin(psi)*cos(phi), xd,
         cos(psi)*sin(theta), cos(theta)*cos(phi) + sin(theta)*sin(phi)*sin(psi), 
  -sin(phi)*cos(theta) + cos(phi)*sin(psi)*sin(theta), yd,
         -sin(psi), sin(phi)*cos(psi), cos(psi)*cos(phi), zd,
         0, 0, 0, 1;
}

void FirstController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data()); 
  Eigen::Map<Eigen::Matrix<double, 4, 4> > Hn0(robot_state.O_T_EE.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();

  Eigen::VectorXd pn0(3), pnv(3), mn(3), fn(3), Wn(6), W0(6);
  Eigen::MatrixXd H0n(4,4), Hnv(4,4), Rn0(3,3), Rnv(3,3), Rvn(3,3), pnv_skew(3,3),
   mn_skew(3,3), fn_skew(3,3);


  Hn0 = Hv0;
  Hn0(0,3) = 0.3;
  Hn0(2,3) = 0.5;
  pn0 << Hn0(0,3), Hn0(1,3), Hn0(2,3);
  //  std::cout << "pn0: " << pn0 << std::endl;
  Rn0 << Hn0(0,0), Hn0(0,1), Hn0(0,2),
         Hn0(1,0), Hn0(1,1), Hn0(1,2),
         Hn0(2,0), Hn0(2,1), Hn0(2,2);
  H0n = Hn0.inverse();

  Rnv << Hnv(0,0), Hnv(0,1), Hnv(0,2),
         Hnv(1,0), Hnv(1,1), Hnv(1,2),
         Hnv(2,0), Hnv(2,1), Hnv(2,2);
  pnv << Hnv(0,3), Hnv(1,3), Hnv(2,3);
  Rvn = Rnv.inverse();
  pnv_skew << 0, -pnv(2), pnv(1),
            pnv(2), 0, -pnv(0),
            -pnv(1), pnv(0), 0;
  
  mn_skew = -2*As(Go*Rnv) - As(Gt*Rvn*pnv_skew*pnv_skew*Rnv);
  fn_skew = -Rvn*As(Gt*pnv_skew)*Rnv - As(Gt*Rvn*pnv_skew*Rnv);
  mn << mn_skew(2,1), mn_skew(0,2), mn_skew(1,0);
  fn << fn_skew(2,1), fn_skew(0,2), fn_skew(1,0);
  Wn << mn, fn;
  W0 = Adjoint(H0n).transpose() * Wn;
  tau_d = jacobian.transpose() * W0 - (B * dq);
  

  /*desired_force_torque(2) = 0;
  tau_d << jacobian.transpose() * desired_force_torque;*/

  tau_cmd = tau_d;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }
 
}

Eigen::Matrix<double, 7, 1> FirstController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

// function to find the trace of a matrix
 double FirstController::trace(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& matrix){
  double trace = 0;
  for(int i=0;i<matrix.rows();i++){
    for(int j=0;j<matrix.rows();j++){
      if(i==j){
        trace = trace + matrix(i,j);
      }
    }
  }
   return trace;
 }
 // function to find anti symmetric part of a square matrix
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> FirstController::As(const 
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& matrix){
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Asmatrix;
    Asmatrix = 0.5 * (matrix - matrix.transpose());
   return Asmatrix;
}

//function to find the adjoint of a homogeneous matrix
Eigen::Matrix<double, 6, 6> FirstController::Adjoint(const 
Eigen::Matrix<double, 4, 4>& Hmat){
Eigen::Matrix<double, 6, 6> AdH;
Eigen::Matrix<double, 3, 3> p;
Eigen::Matrix<double, 3, 3> R;  
Eigen::Matrix<double, 3, 3> pR;  

p << 0, -Hmat(2,3), Hmat(1,3),
     Hmat(2,3), 0, -Hmat(0,3),
     -Hmat(1,3), Hmat(0,3), 0;
R <<  Hmat(0,0), Hmat(0,1), Hmat(0,2),
      Hmat(1,0), Hmat(1,1), Hmat(1,2),
      Hmat(2,0), Hmat(2,1), Hmat(2,2);
pR = p*R;


AdH << Hmat(0,0), Hmat(0,1), Hmat(0,2), 0, 0, 0,
       Hmat(1,0), Hmat(1,1), Hmat(1,2), 0, 0, 0,
       Hmat(2,0), Hmat(2,1), Hmat(2,2), 0, 0, 0,
       pR(0,0), pR(0,1), pR(0,2), Hmat(0,0), Hmat(0,1), Hmat(0,2),
       pR(1,0), pR(1,1), pR(1,2), Hmat(1,0), Hmat(1,1), Hmat(1,2),
       pR(2,0), pR(2,1), pR(2,2), Hmat(2,0), Hmat(2,1), Hmat(2,2);
return AdH;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(new_controllers::FirstController,
                       controller_interface::ControllerBase)
