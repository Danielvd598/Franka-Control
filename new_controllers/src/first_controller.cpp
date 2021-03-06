#include <new_controllers/first_controller.h>

#include <cmath>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>
#include <std_msgs/Int16.h>

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

  node_handle.getParam("/first_controller/use_optimisation",use_optimisation);
  node_handle.getParam("/first_controller/use_modulated_TF",use_modulated_TF);
  node_handle.getParam("/first_controller/use_cyclic",use_cyclic);
  node_handle.getParam("/first_controller/TaskBased",TaskBased);
  node_handle.getParam("/first_controller/kt",kt);
  node_handle.getParam("/first_controller/ko",ko);
  node_handle.getParam("/first_controller/b",b);
  node_handle.getParam("/first_controller/xd",xd);
  node_handle.getParam("/first_controller/yd",yd);
  node_handle.getParam("/first_controller/zd",zd);
  node_handle.getParam("/first_controller/phi",phi);
  node_handle.getParam("/first_controller/psi",psi);
  node_handle.getParam("/first_controller/theta",theta);
  node_handle.getParam("/first_controller/accuracy_thr",accuracy_thr);
  node_handle.getParam("/first_controller/modulation_factor",modulation_factor);
  node_handle.getParam("/first_controller/torque_path", torque_path);
  node_handle.getParam("/first_controller/Hv0_path", Hv0_path);
  node_handle.getParam("/first_controller/qi_path", qi_path);
  node_handle.getParam("/first_controller/t_flag_path", t_flag_path);
  node_handle.getParam("/first_controller/kp", kp);
  node_handle.getParam("/first_controller/kd", kd);
  node_handle.getParam("/first_controller/cycle_wait_period", cycle_wait_period);
  node_handle.getParam("/first_controller/dataPrint", dataPrint);
  node_handle.getParam("/first_controller/dataAnalysis_tau_TB_path", dataAnalysis_tau_TB_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_TF_path", dataAnalysis_tau_TF_path);
  node_handle.getParam("/first_controller/dataAnalysis_dq_path", dataAnalysis_dq_path);
  node_handle.getParam("/first_controller/dataAnalysis_q_path", dataAnalysis_q_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_measured_path", dataAnalysis_tau_measured_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_desired_path", dataAnalysis_tau_desired_path);


  control_state = 0;
  nDoF = 7;
  I33 << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
  Ko << ko, 0, 0,
       0, ko, 0,
       0, 0, ko;
  Kt << kt, 0, 0,
       0, kt, 0,
       0, 0, kt;
  Go = 0.5*trace(Ko)*I33 - Ko;
  Gt = 0.5*trace(Kt)*I33 - Kt;
  B << b, 0, 0, 0, 0, 0, 0,
       0, b, 0, 0, 0, 0, 0,
       0, 0, b, 0, 0, 0, 0,
       0, 0, 0, b, 0, 0, 0,
       0, 0, 0, 0, b, 0, 0,
       0, 0, 0, 0, 0, b, 0,
       0, 0, 0, 0, 0, 0, b;
  a4=0.0825; a7=0.088; d1=0.333; d3=0.316; d5=0.384; dF=0.107; dGripper=0.102;
  T100 << 0, 0, 1, 0, 0, 0; T211 << 0, 1, 0, 0, 0, 0; T322 << 0, -1, 0, 0, 0, 0;
  w4 << 0, -1 ,0; r4 << a4, 0, 0; w5 << 0, 1, 0; r5 << -a4, 0, 0;
  w7 << 0, -1, 0; r7 << a7, 0, 0;
  T433 << w4, r4.cross(w4); T544 << w5, r5.cross(w5); T655 << 0, -1, 0, 0, 0, 0;
  T766 << w7, r7.cross(w7);
  H10_0 << 1, 0, 0, 0, 
           0, 1, 0, 0,
           0, 0, 1, d1,
           0, 0, 0, 1;
  H20_0 << 1, 0, 0, 0, 
           0, 0, 1, 0,
           0, -1, 0, d1,
           0, 0, 0, 1;
  H30_0 << 1, 0, 0, 0, 
           0, 1, 0, 0,
           0, 0, 1, d1+d3,
           0, 0, 0, 1;
  H40_0 << 1, 0, 0, a4, 
           0, 0, -1, 0,
           0, 1, 0, d1+d3,
           0, 0, 0, 1;
  H50_0 << 1, 0, 0, 0, 
           0, 1, 0, 0,
           0, 0, 1, d1+d3+d5,
           0, 0, 0, 1;           
  H60_0 << 1, 0, 0, 0, 
           0, 0, -1, 0,
           0, 1, 0, d1+d3+d5,
           0, 0, 0, 1;
  H70_0 << 0.7071, 0.7071, 0, a7, 
           0.7071, -0.7071, 0, 0,
           0, 0, -1, d1+d3+d5-dF-dGripper,
           0, 0, 0, 1;

  //fill Brockett structure with Twist and H0 matrices
  Brockett_p[0].Twist = T100; Brockett_p[1].Twist = T211; 
  Brockett_p[2].Twist = T322; Brockett_p[3].Twist = T433;
  Brockett_p[4].Twist = T544; Brockett_p[5].Twist = T655;
  Brockett_p[6].Twist = T766;
  Brockett_p[0].H0 = H10_0; Brockett_p[1].H0 = H20_0;
  Brockett_p[2].H0 = H30_0; Brockett_p[3].H0 = H40_0;
  Brockett_p[4].H0 = H50_0; Brockett_p[5].H0 = H60_0;
  Brockett_p[6].H0 = H70_0;

  //read torque data
  if(use_optimisation){
    inFile.open(torque_path);
    if(!inFile) {
      std::cerr << "Unable to open torque txt file!";
      exit(1);
    }
    num = 0.0;
    while (inFile >> num){
      tau_TB_index.push_back(num);
    }
    for(size_t i=0;i<tau_TB_index.size()/nDoF;i++){
      tau_TB_mat.conservativeResize(7,i+1);
      tau_TB_mat.col(i) << tau_TB_index[i], tau_TB_index[tau_TB_index.size()/nDoF+i], 
                          tau_TB_index[2*tau_TB_index.size()/nDoF+i],
                          tau_TB_index[3*tau_TB_index.size()/nDoF+i], 
                          tau_TB_index[4*tau_TB_index.size()/nDoF+i], 
                          tau_TB_index[5*tau_TB_index.size()/nDoF+i],
                          tau_TB_index[6*tau_TB_index.size()/nDoF+i]; 
    } 
    inFile.close();

    //read desired configuration data
    inFile.open(Hv0_path);
    if(!inFile) {
      std::cerr << "Unable to open end-effector trajectory txt file";
      exit(1);
    }
    num = 0.0;
    while (inFile >> num){
      Hv0_index.push_back(num);
    }
    inFile.close();

    inFile.open(qi_path);
    if(!inFile) {
      std::cerr << "Unable to open initial joint configuration txt file!";
      exit(1);
    }
    num = 0.0;
    while(inFile >> num){
      qi_index.push_back(num);
    }
    inFile.close();
    qi << qi_index[0],qi_index[1],qi_index[2],qi_index[3],qi_index[4],qi_index[5],qi_index[6];

    Hv0_matrices = new Hv0_struct [Hv0_index.size()/12]; //create new structure with initialized size
    for(size_t i=0;i<Hv0_index.size()/12;i++){
      Hv0_optimised << Hv0_index[i], 
                      Hv0_index[Hv0_index.size()/12+i], 
                      Hv0_index[2*Hv0_index.size()/12+i], 
                      Hv0_index[9*Hv0_index.size()/12+i], //Hv0(1,4)
                      Hv0_index[3*Hv0_index.size()/12+i],
                      Hv0_index[4*Hv0_index.size()/12+i],
                      Hv0_index[5*Hv0_index.size()/12+i],
                      Hv0_index[10*Hv0_index.size()/12+i], //Hv0(2,4)
                      Hv0_index[6*Hv0_index.size()/12+i],
                      Hv0_index[7*Hv0_index.size()/12+i],
                      Hv0_index[8*Hv0_index.size()/12+i],
                      Hv0_index[11*Hv0_index.size()/12+i], //Hv0(3,4)
                      0,0,0,1;
      Hv0_matrices[i].H = Hv0_optimised;
    } 
  } else{
      Hv0 << cos(theta)*cos(psi), -cos(phi)*sin(theta) + sin(psi)*sin(phi)*cos(theta), 
       sin(theta)*sin(phi) + cos(theta)*sin(psi)*cos(phi), xd,
             cos(psi)*sin(theta), cos(theta)*cos(phi) + sin(theta)*sin(phi)*sin(psi), 
      -sin(phi)*cos(theta) + cos(phi)*sin(psi)*sin(theta), yd,
             -sin(psi), sin(phi)*cos(psi), cos(psi)*cos(phi), zd,
              0, 0, 0, 1;
  }

  inFile.open(t_flag_path);
    if(!inFile) {
      std::cerr << "Unable to open t_flag txt file!";
      exit(1);
    }
    num = 0.0;
    while(inFile >> num){
      t_flag_index.push_back(num);
    }
  inFile.close();
  t_flag << t_flag_index[0], t_flag_index[1], t_flag_index[2], t_flag_index[3];

  trajectory_state = 0;
  modulation_counter = 0;
  update_calls = 0;  
  gripper_flag = 0;
  gripper_flag_pub = node_handle.advertise<std_msgs::Int16>("gripper_flag",10);
  optimisation_length = tau_TB_mat.size()/nDoF;
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
  control_state = 1; //start bringing robot to starting condition

  if(dataPrint){
    if (!dataAnalysis_tau_TB.is_open())
    {
      dataAnalysis_tau_TB.open(dataAnalysis_tau_TB_path); //open file to write data to
    }
    if (!dataAnalysis_tau_TF.is_open())
    {
      dataAnalysis_tau_TF.open(dataAnalysis_tau_TF_path); //open file to write data to
    }
    if (!dataAnalysis_dq.is_open())
    {
      dataAnalysis_dq.open(dataAnalysis_dq_path); //open file to write data to
    }
    if (!dataAnalysis_q.is_open())
    {
      dataAnalysis_q.open(dataAnalysis_q_path); //open file to write data to
    }
    if (!dataAnalysis_tau_measured.is_open())
    {
      dataAnalysis_tau_measured.open(dataAnalysis_tau_measured_path); //open file to write data to
    }
    if (!dataAnalysis_tau_desired.is_open())
    {
      dataAnalysis_tau_desired.open(dataAnalysis_tau_desired_path); //open file to write data to
    }
  }
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
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
  Eigen::VectorXd tau_d(7),tau_TF(7),tau_TB(7),desired_force_torque(6), tau_cmd(7), 
  tau_ext(7);
  desired_force_torque.setZero();

  Eigen::VectorXd pn0(3), pnv(3), mn(3), fn(3), Wn(6), W0(6);
  Eigen::MatrixXd H0n(4,4), Hnv(4,4), Rn0(3,3), Rnv(3,3), Rvn(3,3), pnv_skew(3,3),
   mn_skew(3,3), fn_skew(3,3), GeoJac(6,7);

  //Define all twists for the Geometric Jacobian
  Eigen::VectorXd T1(6), T2(6), T3(6), T4(6), T5(6), T6(6), T7(6);

 //assign twist via for loop because I don't know how to get struct from function
  for(size_t i=0;i<nDoF;i++){
      switch (i)
      {
      case 0:
        T1 = Brockett_p[i].Twist;
        break;
      case 1:
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T2 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 2:
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T3 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 3: 
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T4 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 4:
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T5 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 5:
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T6 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 6:
        Hi0 = Brockett(q,Brockett_p,nDoF,i-1);
        T7 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      default:
        break;
      }
    }

  if(update_calls<optimisation_length){
    tau_TB = tau_TB_mat.col(update_calls);  //determine the Task-Based torque
    if(use_optimisation){ //only overwrite if we want a Task-Free feed-back behaviour
      Hv0 = Hv0_matrices[update_calls].H;
    }
  } else { //no TB torque
    tau_TB << 0,0,0,0,0,0,0;
    if(use_optimisation) { //if you still want to use the optimisation use the last matrix as reference
      Hv0 = Hv0_matrices[optimisation_length - 1].H;
    }
  }
  if (!TaskBased)
  {
    tau_TB << 0,0,0,0,0,0,0;
  }
  
  GeoJac << T1, T2, T3, T4, T5, T6, T7;
  pn0 << Hn0(0,3), Hn0(1,3), Hn0(2,3);
  Rn0 << Hn0(0,0), Hn0(0,1), Hn0(0,2),
         Hn0(1,0), Hn0(1,1), Hn0(1,2),
         Hn0(2,0), Hn0(2,1), Hn0(2,2);
  H0n = Hn0.inverse();
  Hnv = Hv0.inverse()*Hn0;

  Rnv << Hnv(0,0), Hnv(0,1), Hnv(0,2),
         Hnv(1,0), Hnv(1,1), Hnv(1,2),
         Hnv(2,0), Hnv(2,1), Hnv(2,2);
  pnv << Hnv(0,3), Hnv(1,3), Hnv(2,3);
  Rvn = Rnv.inverse();
  pnv_skew << 0, -pnv(2), pnv(1),
            pnv(2), 0, -pnv(0),
            -pnv(1), pnv(0), 0;

  if(use_modulated_TF){
    if ((dq.array() < 0.1).all()){
      if(modulation_counter < 3000) { //put an upper limit on the maximum stiffness
        modulation_counter++;
      }
      Ko << ko+modulation_factor*modulation_counter, 0, 0,
            0, ko+modulation_factor*modulation_counter, 0,
            0, 0, ko+modulation_factor*modulation_counter;
      Kt << kt+modulation_factor*modulation_counter, 0, 0,
            0, kt+modulation_factor*modulation_counter, 0,
            0, 0, kt+modulation_factor*modulation_counter;
    Go = 0.5*trace(Ko)*I33 - Ko;
    Gt = 0.5*trace(Kt)*I33 - Kt;
    } else { //reset to original stiffness and reset counter
        modulation_counter = 0;
        Ko << ko, 0, 0,
              0, ko, 0,
              0, 0, ko;
        Kt << kt, 0, 0,
              0, kt, 0,
              0, 0, kt;
        Go = 0.5*trace(Ko)*I33 - Ko;
        Gt = 0.5*trace(Kt)*I33 - Kt;
    }
  }
  
  mn_skew = -2*As(Go*Rnv) - As(Gt*Rvn*pnv_skew*pnv_skew*Rnv);
  fn_skew = -Rvn*As(Gt*pnv_skew)*Rnv - As(Gt*Rvn*pnv_skew*Rnv);
  fn << fn_skew(2,1), fn_skew(0,2), fn_skew(1,0);
  mn << mn_skew(2,1), mn_skew(0,2), mn_skew(1,0);
  Wn << mn, fn;
  W0 = Adjoint(H0n).transpose() * Wn;
  tau_TF = GeoJac.transpose() * W0 - (B * dq);

  //final control torque
  tau_d =  tau_TF + tau_TB;

  // don't want to heavy torques on the last joint
  if(tau_d[6] > 2){
    tau_d[6] = 2;
  }
  if(tau_d[6] < -2){
    tau_d[6] = -2; 
  }

  //determine control_state
  if(use_optimisation){
    if(control_state!=2){
      for (size_t i=0;i<q.size();i++)
      {
        double error = qi[i]-q[i];
        if(std::abs(error)>0.0005){
          std::cout << "\n Error is not small enough, error: \n" << 
          error << "\n joint #: \n" << i << std::endl;
          control_state = 1;
          break;
        }
        else {
          std::cout << "\n Within limits, starting trajectory! \n" << std::endl;
          control_state = 2;
        }
      }
    }
  } else control_state = 2;

  //set torque command based on the control state
  if(control_state==2){
    tau_cmd = tau_d; //Cartesian impedance control
  } 
  if(control_state==1){
    Eigen::Matrix<double, 7, 1> P_action;
    for (size_t i=0;i<nDoF; i++)
    {
      P_action[i] = std::abs(kp/dq[i]);
      tau_cmd[i] = P_action[i] * (qi[i]-q[i]) + kd*(-dq[i]); //Joint space control
    }
    std::cout << "joint error: \n" << qi-q << std::endl;
  }
 
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  //export data for analysis
  if (dataPrint)
  {
    if(dataAnalysis_tau_TB.is_open() && dataAnalysis_tau_TF.is_open()
    && dataAnalysis_dq.is_open() && dataAnalysis_q.is_open() && 
    dataAnalysis_tau_measured.is_open() && dataAnalysis_tau_desired.is_open()){ 
      dataAnalysis_tau_TB << tau_TB << std::endl;
      dataAnalysis_tau_TF << tau_TF << std::endl;
      dataAnalysis_dq << dq << std::endl;
      dataAnalysis_q << q << std::endl;
      dataAnalysis_tau_measured << tau_measured << std::endl;
      dataAnalysis_tau_desired << tau_J_d + gravity << std::endl;
      } else std::cout << "Unable to open output txt files!";
  }

  // determine gripper flag value
  if (use_optimisation){
    if (update_calls <= t_flag[0]){
      gripper_flag = 2; //make sure the gripper is open before grabbing
    }
    if (update_calls > t_flag[1]*1000 && update_calls < t_flag[2]*1000 && 
    std::abs(Hnv(0,3)) < accuracy_thr && std::abs(Hnv(1,3)) < accuracy_thr && 
    std::abs(Hnv(2,3)) < accuracy_thr)
    {
      gripper_flag = 1; 
      ROS_INFO("grabbing!");
    } 
    if (update_calls > t_flag[3]*1000 && std::abs(Hnv(0,3)) < accuracy_thr
    && std::abs(Hnv(1,3)) < accuracy_thr && std::abs(Hnv(2,3)) < accuracy_thr){
      gripper_flag = 2;
      ROS_INFO("releasing!");
    }
    if (update_calls > (t_flag[3]*1000 + cycle_wait_period) && gripper_flag == 2 && use_cyclic){
      ROS_INFO("completed task, returning to initial position and repeating task");
      control_state = 1; 
      update_calls = 0;
    }
  }
  std_msgs::Int16 gripper_flag_msg;
  gripper_flag_msg.data = gripper_flag;
  gripper_flag_pub.publish(gripper_flag_msg);

  //std::cout << "tau_cmd: \n" << tau_cmd << std::endl;
  //std::cout << "tau_d: \n" << tau_d << std::endl;
  //std::cout << "tau_J_d: \n" << tau_J_d << std::endl;
  //std::cout << "tau_measured: \n" << tau_measured-gravity << std::endl;
  //std::cout << "Hv0: \n" << Hv0 << std::endl;
  //std::cout << "q: \n" << q << std::endl;
  //std::cout << "Hn0: \n" << Hn0 << std::endl;
  //std::cout << "Geometric Jacobian: \n" << GeoJac << std::endl;
  //std::cout << "Wn: \n" << Wn << std::endl;
  //std::cout << "W0: \n" << W0 << std::endl;
  std::cout << "Hnv: \n" << Hnv << std::endl;
  //std::cout << "tau_TB:\n " << tau_TB << std::endl;
  //std::cout << "update calls:\n " << update_calls << std::endl;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  if(control_state==2){
    update_calls++; //update function calls
  }
}

//function to saturate the torque rate
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

//Brockett's formula to find the homegenous transform from any link 
//to the base frame
struct FirstController::Hn0_struct FirstController::Brockett(const 
    Eigen::Matrix<double, 7, 1>& q, struct Brockett_params *Brockett_str, 
    size_t nBrockett, size_t n){
      T100 = Brockett_str[0].Twist;
      T210 = Adjoint(Brockett_str[0].H0)*Brockett_str[1].Twist;
      T320 = Adjoint(Brockett_str[1].H0)*Brockett_str[2].Twist;
      T430 = Adjoint(Brockett_str[2].H0)*Brockett_str[3].Twist;
      T540 = Adjoint(Brockett_str[3].H0)*Brockett_str[4].Twist;
      T650 = Adjoint(Brockett_str[4].H0)*Brockett_str[5].Twist;
      T760 = Adjoint(Brockett_str[5].H0)*Brockett_str[6].Twist;

      switch (n) {
        case 0:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          Brockett_str[0].H0;
          return Hn0_matrices;
          break;
        case 1:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          Brockett_str[1].H0;
          return Hn0_matrices;
          break;
        case 2:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          matrixExponential(T320,q(2))*
          Brockett_str[2].H0;
          return Hn0_matrices;
          break;
        case 3:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          matrixExponential(T320,q(2))*
          matrixExponential(T430,q(3))*
          Brockett_str[3].H0;
          return Hn0_matrices;
          break;
        case 4:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          matrixExponential(T320,q(2))*
          matrixExponential(T430,q(3))*
          matrixExponential(T540,q(4))*
          Brockett_str[4].H0;
          return Hn0_matrices;
          break;
        case 5:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          matrixExponential(T320,q(2))*
          matrixExponential(T430,q(3))*
          matrixExponential(T540,q(4))*
          matrixExponential(T650,q(5))*
          Brockett_str[5].H0;
          return Hn0_matrices;
          break;
        case 6:
          Hn0_matrices.H0 =
          matrixExponential(T100,q(0))*
          matrixExponential(T210,q(1))*
          matrixExponential(T320,q(2))*
          matrixExponential(T430,q(3))*
          matrixExponential(T540,q(4))*
          matrixExponential(T650,q(5))*
          matrixExponential(T760,q(6))*
          Brockett_str[6].H0;
          return Hn0_matrices;
          break;
        default:
          break;
    }
    }

//calculate the matrix exponential using Rodriquez and Mossi
Eigen::Matrix<double, 4, 4> FirstController::matrixExponential(const 
Eigen::Matrix<double, 6, 1>& T, double q_i){
  Eigen::Matrix<double, 4, 4> e;
  Eigen::Vector3d w, v, evec;
  Eigen::Matrix3d w_tilde, emat;
  w << T(0), T(1), T(2);
  v << T(3), T(4), T(5);
  w_tilde  << 0, -w(2), w(1),
             w(2), 0, -w(0),
            -w(1), w(0), 0;
  emat = I33 + w_tilde*sin(q_i) + w_tilde*w_tilde*(1-cos(q_i));
  evec = (1/(w.norm()*w.norm())) * ((I33 - (I33 + w_tilde*sin(q_i) + 
  w_tilde*w_tilde*(1-cos(q_i))))*w.cross(v)) + w*v.transpose()*w;
  e << emat(0,0), emat(0,1), emat(0,2), evec(0), 
       emat(1,0), emat(1,1), emat(1,2), evec(1),
       emat(2,0), emat(2,1), emat(2,2), evec(2),
       0, 0, 0, 1; 
  return e;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(new_controllers::FirstController,
                       controller_interface::ControllerBase)
