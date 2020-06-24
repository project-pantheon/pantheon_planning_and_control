#include <src/nodes/IBVS_random_controller_node.h>

using namespace std;

IBVSRandomNode::IBVSRandomNode(ros::NodeHandle& nh, const std::string& yaml_short_file)
  : MavGUI(nh), nh_(nh), first_trajectory_cmd_(false), SHERPA_planner_(yaml_short_file), 
    ang_vel_ref(SHERPA_planner_.odometry.angular_velocity_B)
{

  odom_sub_ = nh_.subscribe( SHERPA_planner_.odometry_topic, 1, &IBVSRandomNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay() );
  cmd_pose_sub_ = nh_.subscribe( SHERPA_planner_.traj_cmd_topic, 1, &IBVSRandomNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay() );
  ackrmann_cmd_sub_ = nh_.subscribe( SHERPA_planner_.command_topic, 1, &IBVSRandomNode::AkrmCommandsCallback, this, ros::TransportHints().tcpNoDelay() );
  trajectory_pts_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>( SHERPA_planner_.traj_topic, 1);
  lyapunov_sub_ = nh_.subscribe( SHERPA_planner_.lyapunov_topic, 1, &IBVSRandomNode::LyapunovCallback, this, ros::TransportHints().tcpNoDelay() );
  nav_obsts_sub_ = nh_.subscribe( "/ekf_slam_node_with_neighbourhood/navigation_obstacles", 1, &IBVSRandomNode::navigationObstaclesCallback, this, ros::TransportHints().tcpNoDelay() );

  std::cerr << "\n" << FBLU("Initializing short term Controller from:") << " " << yaml_short_file << "\n";
  SHERPA_planner_.InitializeController();

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions.resize( ( ACADO_N )*3 );
  trajectory_pts_.points.push_back(pt);
  trajectory_pts_.joint_names.push_back("sherpa_base_link");

  this->init3DObjRendering( ros::package::getPath("pantheon_planning_and_control") );

}

IBVSRandomNode::~IBVSRandomNode(){}



void IBVSRandomNode::resetSolver(){
  SHERPA_planner_.InitializeController();
}

void IBVSRandomNode::navigationObstaclesCallback(const pantheon_2d_slam::navigationObstaclesConstPtr& navobstsmsg){
  
  int num_static_obstacles = navobstsmsg->staticObstacles.size();
  if( !num_static_obstacles )
    return;

  int iter = 0;
  for( geometry_msgs::Point pt : navobstsmsg->staticObstacles ){
    static_obstacles[iter] << pt.x, pt.y;
    SHERPA_planner_.static_obstacles[iter] << pt.x, pt.y;
    iter++;
  } 

  if( navobstsmsg->dynamicObstacles.size() ){
    geometry_msgs::Point dynamic_obstacle = navobstsmsg->dynamicObstacles[0];
    _dyn_obst_vec2f[0] = dynamic_obstacle.x;
    _dyn_obst_vec2f[1] = dynamic_obstacle.y;
    SHERPA_planner_.static_obstacles[6] << dynamic_obstacle.x, dynamic_obstacle.y;
  }

  SHERPA_planner_.UpdateObstacles();
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

  // Check Is the first trajectory msg has been received
  if(!first_trajectory_cmd_)
    return;
  
  SHERPA_planner_.calculateRollPitchYawRateThrustCommands(trajectory_pts_);
  trajectory_pts_.header.stamp. ros::Time::now();
  trajectory_pts_pub_.publish(trajectory_pts_);

  return;
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
  GLFWwindow* window = glfwCreateWindow(1000, 1000, "mav_gui_app", NULL, NULL);

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

  ros::init(argc, argv, "mav_gnomic_gui_node");
  ros::NodeHandle nh("~");

  IBVSRandomNode IBVS_node(nh, yaml_short_filename);
  bool show_gnomic_GUI = true;

  // Main loop
  while (ros::ok()) {
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