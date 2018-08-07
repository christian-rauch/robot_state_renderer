#include <StateRenderer.hpp>
#include <thread>

#define WINDOW_NAME "StateRenderer"
#define DISP_FREE_VIS "free_view_vis"
#define DISP_ROBO_VIS "robot_view_vis"
#define DISP_ROBO "robot_view"

StateRenderer::StateRenderer(const std::string default_urdf_path, const bool advertise_service) {

    if(advertise_service) {
        n = std::make_shared<ros::NodeHandle>();
    }

    const std::string urdf_path = ros::param::param("~urdf_path", default_urdf_path);
    robot.parseURDF(urdf_path);
    robot.loadLinkMeshes();
    robot.loadJointNames();
    robot.generateMeshColours(false, true);

    pangolin::CreateWindowAndBind(WINDOW_NAME, 640, 240);
    glEnable(GL_DEPTH_TEST);
    // draw a empty (black) frame
    pangolin::FinishFrame();

    shader.AddShader(pangolin::GlSlVertexShader, LabelVertexShader);
    shader.AddShader(pangolin::GlSlFragmentShader, LabelFragmentShader);
    shader.Link();

    robot.renderSetup();

    if(advertise_service) {
        service = n->advertiseService("render", &StateRenderer::render, this);
    }

    pangolin::GetBoundWindow()->RemoveCurrent();
}

StateRenderer::~StateRenderer() {
    pangolin::GetBoundWindow()->RemoveCurrent();
    pangolin::DestroyWindow(WINDOW_NAME);
    pangolin::QuitAll();
}

StateRenderer& StateRenderer::visualise_setup() {
  mutex.lock();
  pangolin::BindToContext(WINDOW_NAME);

  view_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.01,100),
      pangolin::ModelViewLookAt(-1,0,1, 0,0,0, pangolin::AxisZ)
  );

  pangolin::Display(DISP_FREE_VIS)
          .SetAspect(640.0/480.0)
          .SetHandler(new pangolin::Handler3D(view_cam));

  pangolin::Display("dual")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqual)
        .AddDisplay(pangolin::Display(DISP_FREE_VIS))
        .AddDisplay(pangolin::Display(DISP_ROBO_VIS));

  pangolin::GetBoundWindow()->RemoveCurrent();
  mutex.unlock();

  return *this;
}

void StateRenderer::visualise_loop() {
  while(!pangolin::ShouldQuit()) {
      mutex.lock();
      pangolin::BindToContext(WINDOW_NAME);

      // visualisation
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      /// free view

      pangolin::Display(DISP_FREE_VIS).Activate(view_cam);

      shader.Bind();
      shader.SetUniform("MVP", view_cam.GetProjectionModelViewMatrix());
      shader.Unbind();

      pangolin::glDrawAxis(1);    // robot pose in world
      const pangolin::OpenGlMatrix T_wc = robot.T_wr*robot.T_cr.Inverse();    // camera pose
      pangolin::glDrawAxis(T_wc, 0.5);  // camera pose

      robot.render(shader, true);

      /// robot view

      const int w = camera_info.width;
      const int h = camera_info.height;

      pangolin::Display(DISP_ROBO_VIS).SetAspect(w/double(h));
      pangolin::Display(DISP_ROBO_VIS).Activate(robot_cam);

      pangolin::glDrawAxis(1);
      pangolin::glDrawRectPerimeter(-1,-1,1,1);
      pangolin::glDrawAxis(T_wc, 0.5);

      shader.Bind();
      shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
      shader.Unbind();

      robot.render(shader, true);

      pangolin::FinishFrame();

      pangolin::GetBoundWindow()->RemoveCurrent();
      mutex.unlock();

      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void StateRenderer::spin() {
  pangolin::BindToContext(WINDOW_NAME);
  while(!pangolin::ShouldQuit() && ros::ok()) {
      ros::spinOnce();
      mutex.lock();
      pangolin::BindToContext(WINDOW_NAME);
      pangolin::FinishFrame();
      pangolin::GetBoundWindow()->RemoveCurrent();
      mutex.unlock();
  }
  pangolin::GetBoundWindow()->RemoveCurrent();
}

bool StateRenderer::render(robot_state_renderer::RenderRobotStateRequest &req, robot_state_renderer::RenderRobotStateResponse &res) {

    // set state

    // check dimensions
    if(req.state.name.size()!=req.state.position.size()) {
        throw std::runtime_error("number of joint names ("+std::to_string(req.state.name.size())+") and values ("+std::to_string(req.state.position.size())+") mismatch");
    }

    // camera pose, T_wc
    Eigen::Isometry3d cam_pose;
    tf::poseMsgToEigen(req.camera_pose, cam_pose);

    // robot pose, T_wr
    Eigen::Isometry3d robot_pose;
    tf::poseMsgToEigen(req.robot_pose, robot_pose);

    mutex.lock();
    pangolin::BindToContext(WINDOW_NAME);

    const pangolin::OpenGlMatrix T_wc(cam_pose.matrix());
    robot.T_wr = pangolin::OpenGlMatrix(robot_pose.matrix());
    robot.T_cr = T_wc.Inverse()*robot.T_wr;

    // clear and set joint values
    robot.joints.clear();
    for(uint i = 0; i<req.state.name.size(); i++) {
        robot.joints[req.state.name[i]] = float(req.state.position[i]);
    }

    robot.updateFrames();

    for(const std::pair<std::string, uint> &kv : robot.link_label_id) {
        res.link_names.push_back(kv.first);
        // transformation from root to link
        const pangolin::OpenGlMatrix T_rl = RobotModel::MatrixFromFrame(robot.getFramePose(kv.first));
        const pangolin::OpenGlMatrix T_cl = robot.T_cr * T_rl;
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(Eigen::Affine3d(T_cl), pose);
        res.link_poses.push_back(pose);
    }

    camera_info = req.camera_info;

    // render

    const int w = int(camera_info.width);
    const int h = int(camera_info.height);
    const double fu = camera_info.K[0];
    const double fv = camera_info.K[4];
    const double cx = camera_info.K[2];
    const double cy = camera_info.K[5];

    // check camera parameter
    if(w*h==0)        ROS_WARN_STREAM("invalid camera dimension");
    if(!(fu*fv>0.0))  ROS_WARN_STREAM("invalid focal length");
    if(!(cx*cy>0.0))  ROS_WARN_STREAM("invalid camera centre");

    const double z_near = 0.0001;
    const double z_far = 1000;

    // camera coordinate system
    // http://www.songho.ca/opengl/gl_projectionmatrix.html#perspective
    // x - Right, y - Down, z - Front
    robot_cam.SetProjectionMatrix(pangolin::ProjectionMatrixRDF_TopLeft(
        w, h, fu, fv, cx, cy, z_near, z_far));
    robot_cam.SetModelViewMatrix(T_wc.Inverse());

    pangolin::Display(DISP_ROBO).SetAspect(w/double(h));
    pangolin::Viewport offscreen_view(0,0,w,h);

    color_buffer.Reinitialise(w,h);
    depth_buffer.Reinitialise(w,h);
    pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);

    fbo_buffer.Bind();
    offscreen_view.Activate();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader.Bind();
    shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
    shader.Unbind();

    robot.render(shader, false);

    glFlush();

    // read depth in OpenGL coordinates
    cv::Mat_<float> depth_gl(h, w);
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth_gl.data);

    // read green channel as label
    cv::Mat_<uint8_t> label(h, w);
    glReadPixels(0, 0, w, h, GL_GREEN, GL_UNSIGNED_BYTE, label.data);

    fbo_buffer.Unbind();

    pangolin::GetBoundWindow()->RemoveCurrent();
    mutex.unlock();

    cv::Mat_<cv::Vec3d> points(h, w);
    for(int y(0); y<h; y++) {
        for(int x(0); x<w; x++) {
            if(depth_gl(y,x)<1.0f) {
                offscreen_view.GetCamCoordinates(robot_cam, x, y, double(depth_gl(y,x)), points(y,x)[0], points(y,x)[1], points(y,x)[2]);
            }
        }
    }

    // transform 2D pixel coordinates from OpenGL (RUB) to camera frame (RDF)
    // rotate by pi around x-axis
    cv::flip(points, points, 0);
    cv::flip(label, label, 0);

    res.point_cloud.height = uint(h);
    res.point_cloud.width  = uint(w);
    res.point_cloud.header = req.camera_info.header;
    res.point_cloud.is_dense = false;
    res.point_cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(res.point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> iter_xyz(res.point_cloud, "x");
    for(int y(0); y<h; y++) {
        for(int x(0); x<w; x++) {
            iter_xyz[0] = float(points(y,x)[0]);
            iter_xyz[1] = float(points(y,x)[1]);
            iter_xyz[2] = float(points(y,x)[2]);
            ++iter_xyz;
        }
    }

    // mask
    cv_bridge::CvImage mask_img(req.camera_info.header, "mono8", label);
    mask_img.toImageMsg(res.mask);

    // points
    cv_bridge::CvImage points_img(req.camera_info.header, "64FC3", points);
    points_img.toImageMsg(res.points);

    return true;
}
