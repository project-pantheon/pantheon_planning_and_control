#include <src/nodes/IBVS_random_controller_node.h>

using namespace std;

IBVSRandomNode::IBVSRandomNode(ros::NodeHandle& nh, const std::string& yaml_short_file)
  : MavGUI(nh), nh_(nh), first_trajectory_cmd_(false), SHERPA_planner_(yaml_short_file), 
    ang_vel_ref(SHERPA_planner_.odometry.angular_velocity_B), trees_array(21, Eigen::Vector2d(0,0))

{

  odom_sub_ = nh_.subscribe( SHERPA_planner_.odometry_topic, 1, &IBVSRandomNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay() );
  cmd_pose_sub_ = nh_.subscribe( SHERPA_planner_.traj_cmd_topic, 1, &IBVSRandomNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay() );
  ackrmann_cmd_sub_ = nh_.subscribe( SHERPA_planner_.command_topic, 1, &IBVSRandomNode::AkrmCommandsCallback, this, ros::TransportHints().tcpNoDelay() );
  trajectory_pts_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>( SHERPA_planner_.traj_topic, 1);
  lyapunov_sub_ = nh.subscribe( SHERPA_planner_.lyapunov_topic, 1, &IBVSRandomNode::LyapunovCallback, this, ros::TransportHints().tcpNoDelay() );

  updateObstacles_serv_ = nh.advertiseService("updateObstacles", &IBVSRandomNode::updateObstacles, this);

  std::cerr << "\n" << FBLU("Initializing short term Controller from:") << " " << yaml_short_file << "\n";
  SHERPA_planner_.InitializeController();

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions.resize( ( ACADO_N )*3 );
  trajectory_pts_.points.push_back(pt);
  trajectory_pts_.joint_names.push_back("sherpa_base_link");

  this->init3DObjRendering( ros::package::getPath("rvb_mpc") );

  int iter = 0;
  while(true){
    if (exists( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" ) ){
        iter++;
    } else {
      logFileStream.open( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" );
      break;
    }
  }

}

IBVSRandomNode::~IBVSRandomNode(){
  logFileStream.close();
}

void IBVSRandomNode::writeLogData(){

  logFileStream << ros::Time::now().toSec() << " " << SHERPA_planner_.odometry.position_W.x() << " " << SHERPA_planner_.odometry.position_W.y() << " " << 
                   SHERPA_planner_.odometry.getYaw() << " " << SHERPA_planner_.solve_time << "\n"; //  orientation_W_B.w() << " " << stnl_controller.odometry.orientation_W_B.x() << " " << stnl_controller.odometry.orientation_W_B.y() << " " << stnl_controller.odometry.orientation_W_B.z() << " " <<
                   /*trajectory_point.position_W.x() << " " << trajectory_point.position_W.y() << " " << trajectory_point.position_W.z() << " " << trajectory_point.getYaw() << " " <<
                   _target_pos3f[0] << " " << _target_pos3f[1] << " " << _target_pos3f[2] << " " << _target_vel3f[0] << " " << _target_vel3f[1] << " " << _target_vel3f[2] << " " << *_t_delay << " " <<
                   _vert_obst1_[0] << " " << _vert_obst1_[1] << " " << _vert_obst2_[0] << " " << _vert_obst2_[1] << " " << _horiz_obst_[0] << " " << _horiz_obst_[1] << " " <<
                   stnl_controller.pT_W_.x() << " " << stnl_controller.pT_W_.y() << " " << stnl_controller.pT_W_.z() << " " << stnl_controller.camera_instrinsics_.x() << " " << stnl_controller.camera_instrinsics_.y() <<  " " <<
                   stnl_controller.iter << " " << stnl_controller.solve_time << "\n";*/

} 

void IBVSRandomNode::resetSolver(){
  SHERPA_planner_.InitializeController();
}

void IBVSRandomNode::LyapunovCallback(const std_msgs::Float32ConstPtr& lyapunov_msg){
  _lyapunov_cost = lyapunov_msg->data;
}

void IBVSRandomNode::AkrmCommandsCallback(const geometry_msgs::TwistConstPtr& akrm_cmd_msg){
  _v = akrm_cmd_msg->linear.x;
  _phi = akrm_cmd_msg->angular.z;
}

void IBVSRandomNode::CommandPoseCallback(const nav_msgs::OdometryConstPtr& cmd_pose_msg)
{
  ROS_INFO_ONCE("Optimal IBVS controller got first command message.");
  SHERPA_planner_.setCommandPose(*cmd_pose_msg);
  
  if(first_trajectory_cmd_)
    return;
  
  first_trajectory_cmd_ = true;  

  return;
}

void IBVSRandomNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  
  ROS_INFO_ONCE("Optimal IBVS controller got first odometry message.");

  SHERPA_planner_.setOdometry(odom_msg);
  _current_orientation = utils::quaternionFromMsg(odom_msg->pose.pose.orientation); 
  _current_yaw_orientation = utils::yawFromQuaternion(_current_orientation);
  _current_yaw_orientation_deg = _current_yaw_orientation * 180.0 / M_PI;
  _current_odom_position = utils::vector3FromPointMsg(odom_msg->pose.pose.position);  

  if( trees_received )
    computeClosestTrees();

  // Check Is the first trajectory msg has been received
  if(!first_trajectory_cmd_)
    return;
  
  SHERPA_planner_.calculateRollPitchYawRateThrustCommands(trajectory_pts_);
  trajectory_pts_.header.stamp. ros::Time::now();
  trajectory_pts_pub_.publish(trajectory_pts_);

  writeLogData();
  return;
}

void IBVSRandomNode::computeClosestTrees(){

  // TO IMPLEMENT
}

void IBVSRandomNode::getStaticObstacle(){
  
  // TO IMPLEMENT
}

void IBVSRandomNode::setDynamicObstacle(){

  SHERPA_planner_.obst7_ = Eigen::Vector2d(_dyn_obst_vec2f[0], _dyn_obst_vec2f[1]);
  SHERPA_planner_.InitializeController();
  std::cout << FGRN("dynamic_obstacle succesfully set to: ") << SHERPA_planner_.obst7_.transpose() << "\n";
}

bool IBVSRandomNode::updateObstacles(
            rvb_mpc::Obstacles::Request& req, 
            rvb_mpc::Obstacles::Response& res)
{
  try{
    ROS_INFO("updateObstacles");
    SHERPA_planner_.obst1_ = Eigen::Vector2d(req.obst1_x, req.obst1_x);
    SHERPA_planner_.obst2_ = Eigen::Vector2d(req.obst2_x, req.obst2_x);
    SHERPA_planner_.obst3_ = Eigen::Vector2d(req.obst3_x, req.obst3_x);
    SHERPA_planner_.obst4_ = Eigen::Vector2d(req.obst4_x, req.obst4_x);
    SHERPA_planner_.obst5_ = Eigen::Vector2d(req.obst5_x, req.obst5_x);
    SHERPA_planner_.obst6_ = Eigen::Vector2d(req.obst6_x, req.obst6_x);
    SHERPA_planner_.InitializeController();
    res.result=true;
  }catch(...){
    ROS_WARN("IBVS: Fail uploading new obstacles.");
    res.result=false;
  }

    return 1;
}

static void error_callback(int error, const char* description) {
  fprintf(stderr, "Error %d: %s\n", error, description);
}

int main(int argc, char** argv) {

  if(argc < 3) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_short_term_file>" << "\n";
        std::exit(1);
  }

  std::string yaml_short_filename = argv[1];

  // Setup window
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
    return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  #if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  #endif
  GLFWwindow* window = glfwCreateWindow(1000, 1000, "sherpa_planner_app", NULL, NULL);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  gl3wInit();

  // Setup ImGui binding
  ImGui_ImplGlfwGL3_Init(window, true);

  bool show_test_window = false;
  ImVec4 clear_color = ImColor(114, 144, 154);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if(argc < 2) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_file.txt>" << "\n";
        std::exit(1);
  }

  ros::init(argc, argv, "sherpa_planner_gui_node");
  ros::NodeHandle nh("~");

  IBVSRandomNode IBVS_node(nh, yaml_short_filename);
  bool show_gnomic_GUI = true;

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    ImGui_ImplGlfwGL3_NewFrame();

    if (show_test_window) {
      ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
      ImGui::ShowTestWindow(&show_test_window);
    }

    if(show_gnomic_GUI) {
      ImGui::SetNextWindowPos(ImVec2(650,650), ImGuiCond_FirstUseEver);
      IBVS_node.showGUI(&show_gnomic_GUI);
    }

    // Rendering
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui::Render();
    glfwSwapBuffers(window);

    ros::spinOnce();
  }

  // Cleanup
  ImGui_ImplGlfwGL3_Shutdown();
  glfwTerminate();

  ros::spin();

  return 0;
}