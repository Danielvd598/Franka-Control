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

  node_handle.getParam("/first_controller/use_modulated_TF",use_modulated_TF);
  node_handle.getParam("/first_controller/use_dynamic_injection",use_dynamic_injection);
  node_handle.getParam("/first_controller/use_TB",use_TB);
  node_handle.getParam("/first_controller/kt",kt);
  node_handle.getParam("/first_controller/ko",ko);
  node_handle.getParam("/first_controller/b",b);
  node_handle.getParam("/first_controller/bdrain",bdrain);
  node_handle.getParam("/first_controller/alpha",alpha);
  node_handle.getParam("/first_controller/accuracy_thr",accuracy_thr);
  node_handle.getParam("/first_controller/ko_modulation_factor",ko_modulation_factor);
  node_handle.getParam("/first_controller/kt_modulation_factor",kt_modulation_factor);
  node_handle.getParam("/first_controller/ktmax",ktmax);
  node_handle.getParam("/first_controller/komax",komax);
  node_handle.getParam("/first_controller/b_modulation_factor",b_modulation_factor);
  node_handle.getParam("/first_controller/bmax",bmax);
  node_handle.getParam("/first_controller/epsE",epsE);
  node_handle.getParam("/first_controller/epsP",epsP);
  node_handle.getParam("/first_controller/Ek_drained",Ek_drained);
  node_handle.getParam("/first_controller/torque_path", torque_path);
  node_handle.getParam("/first_controller/Hv0_path", Hv0_path);
  node_handle.getParam("/first_controller/qi_path", qi_path);
  node_handle.getParam("/first_controller/t_flag_path", t_flag_path);
  node_handle.getParam("/first_controller/qdot_path", qdot_path);
  node_handle.getParam("/first_controller/tauc_gravity_path", tauc_gravity_path);
  node_handle.getParam("/first_controller/kp", kp);
  node_handle.getParam("/first_controller/kd", kd);
  node_handle.getParam("/first_controller/dataPrint", dataPrint);
  node_handle.getParam("/first_controller/dataAnalysis_tau_TB_path", dataAnalysis_tau_TB_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_TF_path", dataAnalysis_tau_TF_path);
  node_handle.getParam("/first_controller/dataAnalysis_dq_path", dataAnalysis_dq_path);
  node_handle.getParam("/first_controller/dataAnalysis_q_path", dataAnalysis_q_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_measured_path", dataAnalysis_tau_measured_path);
  node_handle.getParam("/first_controller/dataAnalysis_tau_desired_path", dataAnalysis_tau_desired_path);
  node_handle.getParam("/first_controller/dataAnalysis_xyz_ref_path", dataAnalysis_xyz_ref_path);
  node_handle.getParam("/first_controller/dataAnalysis_accuracy_thr_path", dataAnalysis_accuracy_thr_path);


  control_state = 0;
  Njoints = 7;
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

  // read the optimisation data
  // read task-based torque data
  inFile.open(torque_path);
  if(!inFile) {
    std::cerr << "Unable to open torque txt file!";
    exit(1);
  }
  num = 0.0;
  while (inFile >> num){
    tau_TB_index.push_back(num);
  }
  // determine the total length of the optimisation
  optimisation_length = tau_TB_index.size()/Njoints;
  tau_TB_mat.conservativeResize(7,optimisation_length);
  ROS_INFO("total optimisation length [ms]: %d", optimisation_length);
  for(size_t i=0;i<optimisation_length;i++){   
    tau_TB_mat.col(i) << tau_TB_index[i], tau_TB_index[optimisation_length+i], 
                          tau_TB_index[2*optimisation_length+i],
                          tau_TB_index[3*optimisation_length+i], 
                          tau_TB_index[4*optimisation_length+i], 
                          tau_TB_index[5*optimisation_length+i],
                          tau_TB_index[6*optimisation_length+i]; 
  } 
  inFile.close();

  // read task-based torque data including gravity
  inFile.open(tauc_gravity_path);
  if(!inFile) {
    std::cerr << "Unable to open torque gravity txt file!";
    exit(1);
  }
  num = 0.0;
  while (inFile >> num){
    tauc_gravity_index.push_back(num);
  }
  tauc_gravity_mat.conservativeResize(7,optimisation_length);
  for(size_t i=0;i<optimisation_length;i++){   
    tauc_gravity_mat.col(i) << tauc_gravity_index[i], tauc_gravity_index[optimisation_length+i], 
                          tauc_gravity_index[2*optimisation_length+i],
                          tauc_gravity_index[3*optimisation_length+i], 
                          tauc_gravity_index[4*optimisation_length+i], 
                          tauc_gravity_index[5*optimisation_length+i],
                          tauc_gravity_index[6*optimisation_length+i]; 
  } 
  inFile.close();

  //read optimised joint velocities
  inFile.open(qdot_path);
  if(!inFile) {
    std::cerr << "Unable to open qdot txt file!";
    exit(1);
  }
  num = 0.0;
  while (inFile >> num){
    qdot_index.push_back(num);
  }
  qdot_mat.conservativeResize(7,optimisation_length);
  for(size_t i=0;i<(optimisation_length-1)*Njoints;i=i+Njoints){
    qdot_mat.col(i/Njoints) << qdot_index[i],qdot_index[i+1], 
                        qdot_index[i+2],
                        qdot_index[i+3], 
                        qdot_index[i+4], 
                        qdot_index[i+5],
                        qdot_index[i+6]; 
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

  Hv0_matrices = new Hv0_struct [optimisation_length]; //create new structure with initialized size
  for(size_t i=0;i<optimisation_length;i++){
    Hv0_optimised << Hv0_index[i], 
                    Hv0_index[optimisation_length+i], 
                    Hv0_index[2*optimisation_length+i], 
                    Hv0_index[9*optimisation_length+i], //Hv0(1,4)
                    Hv0_index[3*optimisation_length+i],
                    Hv0_index[4*optimisation_length+i],
                    Hv0_index[5*optimisation_length+i],
                    Hv0_index[10*optimisation_length+i], //Hv0(2,4)
                    Hv0_index[6*optimisation_length+i],
                    Hv0_index[7*optimisation_length+i],
                    Hv0_index[8*optimisation_length+i],
                    Hv0_index[11*optimisation_length+i], //Hv0(3,4)
                    0,0,0,1;
    Hv0_matrices[i].H = Hv0_optimised;
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
  t_flag << t_flag_index[0], t_flag_index[1], t_flag_index[2], t_flag_index[3],
            t_flag_index[4], t_flag_index[5], t_flag_index[6];

  trajectory_state = 0;
  ko_modulation_counter = 0;
  kt_modulation_counter = 0;
  update_calls = 0;
  gripper_calls = 0;
  gripper_flag = 0;
  fail = 0;
  drained = 0;
  x << 0,0,0,0,0,0,0,0,0,0,0,0,0,0; //initialize filter
  gripper_flag_pub = node_handle.advertise<std_msgs::Int16>("gripper_flag",10);
  P_opt.conservativeResize(7,optimisation_length);
  for (size_t i=0;i<Njoints;i++){
    for (size_t j=0;j<optimisation_length;j++){
      P_opt(i,j) = std::abs(tauc_gravity_mat(i,j) * qdot_mat(i,j)); // THIS IS INCORRECT NEEDS STATES 
    }
  }

  Etank = 0.001 * P_opt.rowwise().sum();
  if(use_dynamic_injection){
    Etank = Etank*epsE;
  } else Etank = Etank + epsE*Etank;
  ROS_INFO("Initial energy tanks level: [%f %f %f %f %f %f %f] \n",
           Etank(0),Etank(1),Etank(2),Etank(3),Etank(4),Etank(5),Etank(6));
  ROS_INFO("The trajectory flags are at t = [%f %f %f %f %f %f %f] \n",
           t_flag(0),t_flag(1),t_flag(2),t_flag(3),t_flag(4),t_flag(5),t_flag(6));
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
    if (!dataAnalysis_xyz_ref.is_open())
    {
      dataAnalysis_xyz_ref.open(dataAnalysis_xyz_ref_path); //open file to write data to
    }
    if (!dataAnalysis_accuracy_thr.is_open())
    {
      dataAnalysis_accuracy_thr.open(dataAnalysis_accuracy_thr_path); //open file to write data to
    }
  }
}

void FirstController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49>  mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());

  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data()); 
  Eigen::Map<Eigen::Matrix<double, 4, 4> > Hn0(robot_state.O_T_EE.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
  Eigen::VectorXd tau_d(7),tau_TF(7),tau_TB(7),desired_force_torque(6), tau_cmd(7), 
  tau_ext(7), dq_ref(7);
  desired_force_torque.setZero();

  Eigen::VectorXd pn0(3), pnv(3), mn(3), fn(3), Wn(6), W0(6);
  Eigen::MatrixXd H0n(4,4), Hnv(4,4), Rn0(3,3), Rnv(3,3), Rvn(3,3), pnv_skew(3,3),
   mn_skew(3,3), fn_skew(3,3), GeoJac(6,7);

  //Define all twists for the Geometric Jacobian
  Eigen::VectorXd T1(6), T2(6), T3(6), T4(6), T5(6), T6(6), T7(6);

 //assign twist via for loop because I don't know how to get struct from function
  for(size_t i=0;i<Njoints;i++){
      switch (i)
      {
      case 0:
        T1 = Brockett_p[i].Twist;
        break;
      case 1:
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T2 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 2:
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T3 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 3: 
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T4 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 4:
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T5 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 5:
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T6 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      case 6:
        Hi0 = Brockett(q,Brockett_p,Njoints,i-1);
        T7 = Adjoint(Hi0.H0) * Brockett_p[i].Twist;
        break;
      default:
        break;
      }
    }

  if(update_calls<optimisation_length){
    tau_TB = tau_TB_mat.col(update_calls);  //determine the Task-Based torque
    Hv0 = Hv0_matrices[update_calls].H;
    dq_ref = qdot_mat.col(update_calls);
  } else { //no TB torque
    tau_TB << 0,0,0,0,0,0,0;
    Hv0 = Hv0_matrices[optimisation_length - 1].H;
  }

  if (!use_TB)
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

  Eigen::Matrix<double,7,1> dq_ref_filt = LowPassFilter(dq_ref);

//only modulated stiffness when doing cartesian control, otherwise the torque commands
//will have sudden jumps
  if(use_modulated_TF && control_state == 2 && gripper_flag == 0 
  ){
    if(update_calls <= static_cast<int>(t_flag[1]*1000) 
      && update_calls >= static_cast<int>(t_flag[0]*1000)){
        Kt << kt + ((update_calls-(t_flag[0]*1000))/((t_flag[1]-t_flag[0])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(0,3)),3)), 0 ,0,
              0, kt + ((update_calls-(t_flag[0]*1000))/((t_flag[1]-t_flag[0])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(1,3)),3)), 0,
              0, 0, kt + ((update_calls-(t_flag[0]*1000))/((t_flag[1]-t_flag[0])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(2,3)),3));
    } 
    else if(update_calls <= static_cast<int>(t_flag[5]*1000) 
      && update_calls >= static_cast<int>(t_flag[4]*1000)){
        Kt << kt + ((update_calls-(t_flag[4]*1000))/((t_flag[5]-t_flag[4])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(0,3)),3)), 0 ,0,
              0, kt + ((update_calls-(t_flag[4]*1000))/((t_flag[5]-t_flag[4])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(1,3)),3)), 0,
              0, 0, kt + ((update_calls-(t_flag[4]*1000))/((t_flag[5]-t_flag[4])*1000))*(ktmax-kt)/(1 + kt_modulation_factor*pow(std::abs(Hnv(2,3)),3));
    }
    else Kt << kt, 0, 0,
               0, kt, 0,
               0, 0, kt;
    Gt = 0.5*trace(Kt)*I33 - Kt;
    ROS_INFO_THROTTLE(0.1,"The stiffness!\n Kt: %f, %f, %f \n Ko: %f, %f, %f",
                      Kt(0,0),Kt(1,1), Kt(2,2),Ko(0,0),Ko(1,1), Ko(2,2));
  }

  
  mn_skew = -2*As(Go*Rnv) - As(Gt*Rvn*pnv_skew*pnv_skew*Rnv);
  fn_skew = -Rvn*As(Gt*pnv_skew)*Rnv - As(Gt*Rvn*pnv_skew*Rnv);
  fn << fn_skew(2,1), fn_skew(0,2), fn_skew(1,0);
  mn << mn_skew(2,1), mn_skew(0,2), mn_skew(1,0);
  Wn << mn, fn;
  W0 = Adjoint(H0n).transpose() * Wn;
  tau_TF = GeoJac.transpose() * W0 - (B * (dq - dq_ref));

  //final control torque
  tau_d =  tau_TF + tau_TB;

  // don't want to heavy torques on the last joint
  if(tau_d[6] > 2){
    tau_d[6] = 2;
  }
  if(tau_d[6] < -2){
    tau_d[6] = -2; 
  }

  // determine gripper flag value
  gripper_flag = 0; //doing nothing 
  if (update_calls > 1 && update_calls <= t_flag[0]*1000 && gripper_status.size() < 500){
    gripper_flag = 2; //make sure the gripper is open before grabbing
    ROS_INFO_THROTTLE(0.1,"Opening the gripper before grabbing action!");
    gripper_status.conservativeResize(gripper_calls+1,1);
    gripper_status.row(gripper_calls) << 1;
  } 
  if (update_calls == static_cast<int>(t_flag[1]*1000) && 
      std::abs(Hnv(0,3)) < accuracy_thr && std::abs(Hnv(1,3)) < accuracy_thr && 
      std::abs(Hnv(2,3)) < accuracy_thr && gripper_status.size() < 1000) {
    gripper_flag = 1; 
    ROS_INFO_THROTTLE(0.1,"grabbing!");
    gripper_status.conservativeResize(gripper_calls+1,1);
    gripper_status.row(gripper_calls) << 2;
  } 
  if (update_calls ==  static_cast<int>(t_flag[6]*1000) && 
  std::abs(Hnv(0,3)) < accuracy_thr && std::abs(Hnv(1,3)) < accuracy_thr 
  && std::abs(Hnv(2,3)) < accuracy_thr && gripper_status.size() < 1500){
    gripper_flag = 2;
    ROS_INFO_THROTTLE(0.1,"releasing!");
    gripper_status.conservativeResize(gripper_calls+1,1);
    gripper_status.row(gripper_calls) << 3;
  } 

  std_msgs::Int16 gripper_flag_msg;
  gripper_flag_msg.data = gripper_flag;

  // only publish gripper flag if releasing and grabbing, sleep for 1 seconds to complete action
  if (gripper_flag != 0) {
      gripper_flag_pub.publish(gripper_flag_msg);
      gripper_calls++;
  }

  // // Safety
  if (control_state==2 && gripper_flag == 0){// only update when performing trajectory
    for (size_t i=0;i<Njoints;i++){
      P_meas(i) = tau_measured(i) * dq(i); // calculate measured mechanical power
      if(use_dynamic_injection){
        Etank(i) = Etank(i) + P_opt(i,update_calls);   // fill the tanks
      } 
      // empty the tanks based on measured torque and joint velocities
      Etank(i) = Etank(i) - P_meas(i);
      //check if a tank is empty
      if(Etank(i) <= 0 && !fail){
        fail = true;
        ROS_WARN_THROTTLE(0.1,"Energy tank of joint %f is empty!",(i+1));
        t1 = ros::WallTime::now();
      }
    }
  }
  if(fail){
    Ek = dq.transpose()*mass*dq; //calculate current kinetic energy
    if (Ek < Ek_drained && !drained){
      drained = true;
      t2 = ros::WallTime::now();
      double drainage_time = (t2 - t1).toSec();
      ROS_WARN_THROTTLE(0.1,"Energy drained, system is compliant!");
      ROS_INFO_STREAM("\nDrainage time (s): " << drainage_time);
    }
  }

  //determine control_state
  if(control_state!=2 && control_state!=3 && control_state!=4){ //only do at start
    for (size_t i=0;i<q.size();i++)
    {
      double error = qi[i]-q[i];
      if(std::abs(error)>0.001){
        ROS_WARN_THROTTLE(0.1,"\n Error is not small enough:\n error: %f \n joint: %f",error,i+1);
        control_state = 1;
        break;
      }
      else if(std::abs(error)<=0.001 && i == q.size()-1){
        ROS_INFO("\n Within limits, starting trajectory!");
        control_state = 2;
      }
    }
  }
  if(fail && !drained){
    control_state = 3;
  } else if(fail && drained){
    control_state = 4;
  }


  //set torque command based on the control state
  if(control_state==2){
    tau_cmd = tau_d; //Cartesian impedance control
  } 
  if(control_state==1){
    Eigen::Matrix<double, 7, 1> P_action;
    for (size_t i=0;i<Njoints; i++)
    {
      P_action[i] = std::abs(kp/dq[i]);
      tau_cmd[i] = P_action[i] * (qi[i]-q[i]) + kd*(-dq[i]); //Joint space control
    }
  }
  if(control_state==3){ 
    ROS_WARN_THROTTLE(0.1,"Trajectory failed, draining the kinetic energy!");
    tau_cmd = -bdrain * dq;// draining the kinetic energy of the system
  }
  if(control_state==4 || gripper_flag != 0){ //if gripper is moving don't send torque commands
    tau_cmd << 0,0,0,0,0,0,0; //gravity is compensated internally in the Franka
  }

  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  //export data for analysis
  if (dataPrint)
  {
    if(dataAnalysis_tau_TB.is_open() && dataAnalysis_tau_TF.is_open()
    && dataAnalysis_dq.is_open() && dataAnalysis_q.is_open() && 
    dataAnalysis_tau_measured.is_open() && dataAnalysis_tau_desired.is_open() &&
    dataAnalysis_xyz_ref.is_open()){ 
      if(gripper_flag == 0){
      dataAnalysis_tau_TB << tau_TB << std::endl;
      dataAnalysis_tau_TF << tau_TF << std::endl;
      dataAnalysis_dq << dq << std::endl;
      dataAnalysis_q << q << std::endl;
      dataAnalysis_tau_measured << tau_measured << std::endl;
      dataAnalysis_tau_desired << tau_J_d + gravity << std::endl;
      if(accuracy_flag == 0){//print such that I know where optimised/realtime is disrupted
        dataAnalysis_accuracy_thr << update_calls << std::endl;
      }
      dataAnalysis_xyz_ref << Hv0(0,3) << "\n" << Hv0(1,3) << "\n" << Hv0(2,3) << std::endl;
      }
    } else std::cerr << "Unable to open output txt files!";
  }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  for (size_t i=1; i<Njoints; i=i+5){ //only for high accuracy parts
    if (update_calls == static_cast<int>(t_flag[i]*1000) && 
        (std::abs(Hnv(0,3)) > accuracy_thr || std::abs(Hnv(1,3)) > accuracy_thr ||
         std::abs(Hnv(2,3)) > accuracy_thr)){
        ROS_INFO_THROTTLE(0.1,"Accuracy is not sufficient enough at high accuracy points!");
        accuracy_flag = 0; 
        break;
    }
    else accuracy_flag = 1;
  }


  if(control_state == 2){
    ROS_INFO_THROTTLE(0.1,"Update calls: %d", update_calls);
    ROS_INFO_THROTTLE(0.1,"pnv: \n[%f \n %f \n %f]", Hnv(0,3),Hnv(1,3),Hnv(2,3));

  }
  //ROS_INFO_THROTTLE(0.1,"pn0: \n[%f \n %f \n %f]", Hn0(0,3),Hn0(1,3),Hn0(2,3));

  if(control_state==2 && gripper_flag == 0 && accuracy_flag == 1){
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

Eigen::Matrix<double,7,1> FirstController::LowPassFilter(const Eigen::Matrix<double,7,1>& u){
  Eigen::Matrix<double,7,1> u_filt;
  for(size_t i;i<Njoints;i++){
    x(i,1) = (1-alpha)*x(i,0) + alpha*u(i,0);
    u_filt << x(0,0), x(1,0), x(2,0), x(3,0), x(4,0), x(5,0), x(6,0);
  }
  return u_filt;
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
