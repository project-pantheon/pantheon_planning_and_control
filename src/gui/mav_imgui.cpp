#include "mav_imgui.h"

MavGUI::MavGUI(ros::NodeHandle nh) : BaseGUI(nh), Obst1_(4,-2,0) ,
                                     Obst2_(7,-2,0), Obst3_(10,-2,0) ,
                                     Obst4_(4,-6.5,0) , Obst5_(7,-6.5,0) , Obst6_(10,-6.5,0) {

  _des_pos_vec3f_t[0] = 0.f;
  _des_pos_vec3f_t[1] = 0.f;
  _des_orientationf_t = 0.f;

  _des_pos_vec3f_w[0] = 0.f;
  _des_pos_vec3f_w[1] = 0.f;
  _des_orientationf_w = 0.f;

  _K_values[0] = 1.f;
  _K_values[1] = 6.f;
  _K_values[2] = 3.f;

  _dyn_obst_vec2f[0] = 0.f;
  _dyn_obst_vec2f[1] = 0.f;

  _gui_ros_time = ros::Time::now();

  _set_control_gains = _base_nh.serviceClient<rm3_ackermann_controller::SetKvalues>("/set_k");
  _activate_controller = _base_nh.serviceClient<rm3_ackermann_controller::ActivateController>("/activate_controller");

  avatarImg = cv::Mat(cv::Size(640,640), CV_8UC3);
  camera = std::make_shared<Camera>( glm::vec3(0.f, 0.f, 9.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.f, 0.f );
  std::cout << FGRN("Camera Correctly Initialized\n\n");
}


void MavGUI::init3DObjRendering(std::string&& package_path_str){

  char vs_path[200], fs_path[200], model_path_tree[200], model_path_sherpa[200], model_path_dyn_obst[200];

  strcpy(vs_path, package_path_str.c_str());
  strcat(vs_path, "/src/assimp_loader/assets/shaders/modelTextured.vs");

  strcpy(fs_path, package_path_str.c_str());
  strcat(fs_path, "/src/assimp_loader/assets/shaders/modelTextured.fs");

  strcpy(model_path_tree, package_path_str.c_str());
  strcat(model_path_tree, "/src/assimp_loader/assets/low_poly_tree/low_poly_tree.obj");
  strcpy(model_path_sherpa, package_path_str.c_str());
  strcat(model_path_sherpa, "/src/assimp_loader/assets/sherpa/box.obj");
  strcpy(model_path_dyn_obst, package_path_str.c_str());
  strcat(model_path_dyn_obst, "/src/assimp_loader/assets/dyn_obst/box.obj");

  shader =  std::make_shared<Shader>( vs_path, fs_path );
  std::cout << FGRN("Shader Correctly Initialized\n\n");

  tree_model = std::make_shared<Model>(model_path_tree);
  sherpa_model = std::make_shared<Model>(model_path_sherpa);
  dyn_obst_model = std::make_shared<Model>(model_path_dyn_obst);
  std::cout << FGRN("Model Correctly Initialized\n\n");

}

void MavGUI::processAvatar(){

    camera = std::make_shared<Camera>( glm::vec3(_current_odom_position(0), _current_odom_position(1), 9.0f),
                                       glm::vec3(0.0f, 1.0f, 0.0f), -90.f, 0.f );

    ImVec4 clear_color = ImColor(34, 43, 46);
    //ImVec4 clear_color = ImColor(255, 255, 255);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, 640, 640);
    // don't forget to enable shader before setting uniforms
    shader->use();

    // view/projection transformations
    glm::mat4 projection = glm::perspective((float)476, (float)640 / (float)640, 0.1f, 250.0f);

    glm::mat4 view = camera->GetViewMatrix();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);

    // render the loaded model
    glm::mat4 currmodel1 = glm::mat4(1.0f);
    currmodel1 = glm::translate(currmodel1, glm::vec3(4, -2, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel1);
    tree_model->Draw(*shader);

    glm::mat4 currmodel2 = glm::mat4(1.0f);
    currmodel2 = glm::translate(currmodel2, glm::vec3(7, -2, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel2);
    tree_model->Draw(*shader);

    glm::mat4 currmodel3 = glm::mat4(1.0f);
    currmodel3 = glm::translate(currmodel3, glm::vec3(10, -2, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel3);
    tree_model->Draw(*shader);

    glm::mat4 currmodel4 = glm::mat4(1.0f);
    currmodel4 = glm::translate(currmodel4, glm::vec3(4, -6.5, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel4);
    tree_model->Draw(*shader);

    glm::mat4 currmodel5 = glm::mat4(1.0f);
    currmodel5 = glm::translate(currmodel5, glm::vec3(7, -6.5, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel5);
    tree_model->Draw(*shader);

    glm::mat4 currmodel6 = glm::mat4(1.0f);
    currmodel6 = glm::translate(currmodel6, glm::vec3(10, -6.5, 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel6);
    tree_model->Draw(*shader);

    glm::mat4 currmodel8 = glm::mat4(1.0f);
    currmodel8 = glm::translate(currmodel8, glm::vec3(_dyn_obst_vec2f[0], _dyn_obst_vec2f[1], 0)); // translate it down so it's at the center of the scene
    shader->setMat4("model", currmodel8);
    dyn_obst_model->Draw(*shader);

    float angle = 2 * std::acos(_current_orientation.w());
    float norm_fact = std::sqrt(1 - _current_orientation.w()*_current_orientation.w());
    glm::vec3 rot_axis(_current_orientation.x() / norm_fact,
                       _current_orientation.y() / norm_fact,
                       _current_orientation.z() / norm_fact);

    glm::mat4 currmodel7 = glm::mat4(1.0f);
    currmodel7 = glm::translate(currmodel7, glm::vec3(_current_odom_position(0), _current_odom_position(1), 0)); // translate it down so it's at the center of the scene
    currmodel7 = glm::rotate(currmodel7, angle, rot_axis);
    shader->setMat4("model", currmodel7);
    sherpa_model->Draw(*shader);


    glReadPixels ( 0, 0, 640, 640, GL_BGR,
                   GL_UNSIGNED_BYTE, ( GLubyte * ) avatarImg.data );
    cv::flip(avatarImg, avatarImg, 0);
    cv::cvtColor(avatarImg, avatarImg, CV_RGB2BGR);
    
    cv::resize(avatarImg, avatarImg_res, cv::Size(avatarImg.cols/2, avatarImg.rows/2) );
    cv::transpose(avatarImg_res, avatarImg_res);
    cv::flip(avatarImg_res, avatarImg_res, 1);

    int N = trajectory_pts_.points[0].positions.size()/3;
    std::vector<Eigen::Vector2f> pt( N, Eigen::Vector2f(0,0) );
    for(unsigned int iter = 0; iter < N; ++iter){
      pt[iter] = Eigen::Vector2f( - trajectory_pts_.points[0].positions[iter * 3 + 1], - trajectory_pts_.points[0].positions[iter * 3] ) - 
                Eigen::Vector2f( - _current_odom_position(1), - _current_odom_position(0));
      pt[iter] = 180*pt[iter]/9 + Eigen::Vector2f(320/2,320/2); 
      cv::circle(avatarImg_res, cv::Point2i(pt[iter](0), pt[iter](1)), 5, cv::Scalar(255,0,0), 3);
    }

    for(unsigned int iter = 0; iter < N - 1; ++iter)
      cv::line(avatarImg_res, cv::Point2i(pt[iter](0), pt[iter](1)), cv::Point2i(pt[iter+1](0), pt[iter+1](1)), cv::Scalar(0,0,255),3);

}


void MavGUI::updateDesiredState() {

  nav_msgs::Odometry cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.pose.pose.position.x = _des_pos_vec3f_t[0];
  cmd_msg.pose.pose.position.y = _des_pos_vec3f_t[1];
  cmd_msg.pose.pose.position.z = 0;

  Eigen::Quaterniond q( Eigen::AngleAxisd(_des_orientationf_t, Eigen::Vector3d::UnitZ() ) );
  cmd_msg.pose.pose.orientation = utils::fromEigenQuaternionrToQuaternion(q);
  _cmd_pub.publish( cmd_msg );
}

void MavGUI::sendWaypoint() {

  geometry_msgs::Point pt_msg;
  pt_msg.x = _des_pos_vec3f_w[0];
  pt_msg.y = _des_pos_vec3f_w[1];
  pt_msg.z = _des_orientationf_w;
  if(_sendingWaypoint)
    _waypoint_pub.publish(pt_msg);
}

void MavGUI::activatePublisher(const std::string &cmd_publisher_name, const std::string &waypoint_publisher_name) {
  _cmd_pub = _base_nh.advertise<nav_msgs::Odometry>(cmd_publisher_name, 1, this);
  _waypoint_pub = _base_nh.advertise<geometry_msgs::Point>(waypoint_publisher_name, 1, this);
}

void MavGUI::activateController(){

  rm3_ackermann_controller::ActivateController srvCall;
  srvCall.request.is_active = true;
  _activate_controller.call(srvCall);

  std::cout << FBLU("SherpaPlannerGUI: ") << srvCall.response.result << "\n";

}

void MavGUI::disactivateController(){
  
  rm3_ackermann_controller::ActivateController srvCall;
  srvCall.request.is_active = false;
  _activate_controller.call(srvCall);

  std::cout << FBLU("SherpaPlannerGUI: ") << srvCall.response.result << "\n";
}



void MavGUI::showGUI(bool *p_open) {

  processAvatar();
  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_MenuBar;
 
  ImGui::SetNextWindowSize(ImVec2(700, 800), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("SherpaPlennerGUI", p_open, window_flags)) {
    // Early out if the window is collapsed, as an optimization.
    ImGui::End();
    return;
  }
  
  /// MAIN WINDOW CONTENT
  ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.0f, 1.0f), "SherpaPlannerGUI");
  ImGui::Text("Activate callback and publisher with Gazebo simulator open before sending the desired goal.");
  ImGui::Spacing();
  static char urdf_model_name[64] = "sherpa";
  ImGui::InputText(" ", urdf_model_name, 64);
  ImGui::SameLine();
  if(ImGui::Button("Reset Model")) resetGazeboScene(urdf_model_name);
  ImGui::Spacing(); 
  ImGui::Separator();
  ImGui::Spacing(); 
  static char cmd_pub[64] = "/command/pose";
  static char waypoint_pub[64] = "/waypoint";
  ImGui::InputText("cmd_pub", cmd_pub, 64);
  ImGui::InputText("waypoint_pub", waypoint_pub, 64);
  if (ImGui::Button("Activate Publisher")) {
    std::cerr << "activating publishers: " << cmd_pub << " " << waypoint_pub << "\n";
    activatePublisher(cmd_pub, waypoint_pub);
  }

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(2, "state_time");
  ImGui::Text("Desired State (Trajectory)");
  ImGui::DragFloat2("x y [meters]", _des_pos_vec3f_t, 0.01f, -20.0f, 20.0f);
  ImGui::DragFloat("yaw [radians]", &_des_orientationf_t, 0.01f, -M_PI, M_PI);
  if (ImGui::Button("Update Desired State")) 
    updateDesiredState();


  ImGui::NextColumn();
  ImGui::Text("Desired State (Waypoint)");
  ImGui::DragFloat2("x y [meters] ", _des_pos_vec3f_w, 0.01f, -20.0f, 200.0f);
  ImGui::DragFloat("yaw [radians] ", &_des_orientationf_w, 0.01f, -M_PI, M_PI);
  if (ImGui::Button("Start Sending"))
    _sendingWaypoint = true;
  ImGui::SameLine();
  if (ImGui::Button("Stop Sending"))
    _sendingWaypoint = false;
    
  sendWaypoint();
  
  // Plotting Telemetry
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(1); // 4-ways, with border
  ImGui::Text("Current State:");
  ImGui::Columns(4, "mycolumns"); // 4-ways, with border
  addDataPlot(_x_values, _x_min, _x_max, _current_odom_position(0));
  addDataPlot(_y_values, _y_min, _y_max, _current_odom_position(1));
  addDataPlot(_z_values, _z_min, _z_max, _current_odom_position(2));
  addDataPlot(_yaw_values, _yaw_min, _yaw_max, _current_yaw_orientation_deg);
  ImGui::Separator();
  ImGui::Text("x[m]"); ImGui::NextColumn();
  ImGui::Text("y[m]"); ImGui::NextColumn();
  ImGui::Text("z[m]"); ImGui::NextColumn();
  ImGui::Text("yaw[deg]"); ImGui::NextColumn();
  ImGui::Separator();
  ImGui::Text("%f", _current_odom_position(0)); ImGui::NextColumn();
  ImGui::Text("%f", _current_odom_position(1)); ImGui::NextColumn();
  ImGui::Text("%f", _current_odom_position(2)); ImGui::NextColumn();
  ImGui::Text("%f", _current_yaw_orientation_deg);  ImGui::NextColumn();

  ImGui::PlotLinesWithTarget("",_x_values, IM_ARRAYSIZE(_x_values), _des_pos_vec3f_w[0],
                             0,"x", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_y_values, IM_ARRAYSIZE(_y_values), _des_pos_vec3f_w[1],
                             0, "y", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_z_values, IM_ARRAYSIZE(_z_values), 0,
                             0, "z", _z_min, _z_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_yaw_values, IM_ARRAYSIZE(_yaw_values), _des_orientationf_w,
                             0, "yaw", FLT_MAX, FLT_MAX, ImVec2(0,40));

  // Plotting Ackermann Commands and Lyapunov Cost Function
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(1); // 4-ways, with border
  ImGui::Text("Commands and Lyapunov Cost Function:");
  ImGui::Columns(3, "mycolumns"); 
  addDataPlot(_v_values, _v_min, _v_max, _v);
  addDataPlot(_phi_values, _phi_min, _phi_max, _phi);
  addDataPlot(_lyapunov_values, _lyapunov_min, _lyapunov_max, _lyapunov_cost);
  ImGui::Separator();
  ImGui::Text("v[m/s]"); ImGui::NextColumn();
  ImGui::Text("phi[rad]"); ImGui::NextColumn();
  ImGui::Text("Lyapunov"); ImGui::NextColumn();
  ImGui::Separator();
  ImGui::Text("%f", _v); ImGui::NextColumn();
  ImGui::Text("%f", _phi); ImGui::NextColumn();
  ImGui::Text("%f", _lyapunov_cost); ImGui::NextColumn();

  // ImGui::PlotHistogram("Histogram", _v_values, IM_ARRAYSIZE(_v_values), 0, NULL, _v_min, _v_max, ImVec2(0,80)); ImGui::NextColumn();

  ImGui::PlotLinesSaturation("",_v_values, IM_ARRAYSIZE(_v_values), _v_sup, _v_sdown,
                    0, "v", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesSaturation("",_phi_values, IM_ARRAYSIZE(_phi_values), _phi_sup, _phi_sdown, 
                    0, "phi", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_lyapunov_values, IM_ARRAYSIZE(_lyapunov_values), 0,
                    "lyapunov", _lyapunov_min, _lyapunov_max, ImVec2(0,40)); ImGui::NextColumn();





  ImGui::Columns(1);
  ImGui::Spacing();
  ImGui::Separator();

  // Show Here Auxiliar GUIs
  // if(_show_gazebo_gui) {
  //   showGazeboGUI(&_show_gazebo_gui);
  // }

  // Turn the RGB pixel data into an OpenGL texture:
  glDeleteTextures(1, &my_avatar_texture);
  glGenTextures(1, &my_avatar_texture);
  glBindTexture(GL_TEXTURE_2D, my_avatar_texture);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  ImGui::Columns(2, "Current Image and UAV Avatar");
  ImGui::Text("UAV Avatar");
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, avatarImg_res.cols, avatarImg_res.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, avatarImg_res.data);
  ImGui::Image((void*)(intptr_t)my_avatar_texture, ImVec2(avatarImg_res.cols, avatarImg_res.rows));

  ImGui::NextColumn();
  ImGui::Text("Control law gains");
  ImGui::Text("0.2 0.4 3.5 Pose Regulation");
  ImGui::Text("0.5 2 3 Traj. Tracking");
  ImGui::DragFloat3(" K1 K2 K3 ", _K_values, 0.01f, -20.0f, 200.0f);
  if (ImGui::Button("Send gains"))
    changeControlLawGains();
  
  ImGui::Spacing();
  ImGui::Text("Ackermann Controller");
  if (ImGui::Button("Activate"))
    activateController();
  ImGui::SameLine();
  if (ImGui::Button("Disactivate"))
    disactivateController();

  ImGui::Spacing();
  ImGui::Text("Dynamic Obstacle");
  ImGui::DragFloat2(" x y ", _dyn_obst_vec2f, 0.01f, -20.0f, 20.0f);
  if (ImGui::Button("set Dyn. Obstacle"))
    setDynamicObstacle();  

  ImGui::Spacing();
  ImGui::Text("Read Static Obstacles");
  if (ImGui::Button("get Static Obstacle"))
    getStaticObstacle();  

}



void MavGUI::changeControlLawGains(){

  rm3_ackermann_controller::SetKvalues srvCall;
  srvCall.request.k1 = _K_values[0];
  srvCall.request.k2 = _K_values[1];
  srvCall.request.k3 = _K_values[2];
  _set_control_gains.call(srvCall);

  std::cout << FBLU("SherpaPlannerGUI: ") << srvCall.response.result << "\n";

}