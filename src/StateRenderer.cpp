#include <StateRenderer.hpp>
#include <thread>

StateRenderer::StateRenderer(const std::string default_urdf_path, const bool provide_service, const bool visualise) {
    std::cout << "cnstr..." << std::endl;
    n = std::make_shared<ros::NodeHandle>();

    const std::string urdf_path = ros::param::param("~urdf_path", default_urdf_path);
    robot.parseURDF(urdf_path);
    robot.loadLinkMeshes();
    robot.loadJointNames();
    robot.generateMeshColours(false, true);

    std::cout << "... window" << std::endl;
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    std::cout << ".. shader" << std::endl;
    shader.AddShader(pangolin::GlSlVertexShader, LabelVertexShader);
    shader.AddShader(pangolin::GlSlFragmentShader, LabelFragmentShader);
    shader.Link();

    std::cout << "... robot" << std::endl;
    robot.renderSetup();

    if(provide_service) {
        service = n->advertiseService("render", &StateRenderer::render, this);
    }

    std::cout << "setup done" << std::endl;
    std::unique_lock<std::mutex> lck_setup(mtx_setup_done);
    cv_setup_done.notify_all();

    // visualisation and offscreen rendering in constructor to avoid issues
    // with restoring the pangolin context

//    has_request.store(false);

    pangolin::OpenGlRenderState view_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.01,100),
        pangolin::ModelViewLookAt(-1,0,1, 0,0,0, pangolin::AxisZ)
    );
    pangolin::View &view_display = pangolin::Display("free view")
            .SetAspect(640.0/480.0)
            .SetHandler(new pangolin::Handler3D(view_cam));

    pangolin::View &robot_display = pangolin::Display("robot view")
            .SetHandler(new pangolin::Handler3D(robot_cam));

    pangolin::Display("multi")
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutEqual)
          .AddDisplay(view_display)
          .AddDisplay(robot_display);

    glEnable(GL_DEPTH_TEST);

    ros::Rate r(60);

    render_call = std::make_shared<robot_state_renderer::RenderRobotState>();

    while(!pangolin::ShouldQuit()) {
        if(visualise) {
        // visualisation
//        std::cout << "visualise ... " << std::endl;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /// free view

        view_display.Activate(view_cam);

        mutex.lock();

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

        robot_display.SetAspect(w/double(h));
        robot_display.Activate(robot_cam);

        pangolin::glDrawAxis(1);
        pangolin::glDrawRectPerimeter(-1,-1,1,1);
        pangolin::glDrawAxis(T_wc, 0.5);

        shader.Bind();
        shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
        shader.Unbind();

        robot.render(shader, true);

        mutex.unlock();

//        std::cout << "... vis done" << std::endl;

        } // visualise

        std::unique_lock<std::mutex> lck_req(mtx_req);
        if(cond_req.wait_for(lck_req, std::chrono::milliseconds(1))==std::cv_status::no_timeout) {
//        if(has_request2) {
            std::cout << "render ... " << std::endl;
            auto &req = render_call->request;
            auto &res = render_call->response;

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

            pangolin::View &robot_display = pangolin::Display("robot view")
                    .SetHandler(new pangolin::Handler3D(robot_cam));

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

            robot_display.SetAspect(w/double(h));
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

            mutex.unlock();

            // read depth in OpenGL coordinates
            cv::Mat_<float> depth_gl(h, w);
            glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth_gl.data);

            // read green channel as label
            cv::Mat_<uint8_t> label(h, w);
            glReadPixels(0, 0, w, h, GL_GREEN, GL_UNSIGNED_BYTE, label.data);

            fbo_buffer.Unbind();

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

            std::cout << "notify.." << std::endl;
//            std::unique_lock<std::mutex> lck_srv(srv_mutex);
            cond_res.notify_all();
//            has_request.store(false);
            std::cout << "... done" << std::endl;
        }
//        std::cout << "render done ... " << std::endl;

        pangolin::FinishFrame();
//        std::cout << "pango done ... " << std::endl;

//        ros::spinOnce();
//        std::cout << "spinned ... " << std::endl;
//        r.sleep();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        std::cout << "loop done ... " << std::endl;
    }
}

StateRenderer::~StateRenderer() {
    pangolin::DestroyWindow(window_name);
}

void StateRenderer::run(const bool visualise) {
//  pangolin::OpenGlRenderState view_cam(
//      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.01,100),
//      pangolin::ModelViewLookAt(-1,0,1, 0,0,0, pangolin::AxisZ)
//  );
//  pangolin::View &view_display = pangolin::Display("free view")
//          .SetAspect(640.0/480.0)
//          .SetHandler(new pangolin::Handler3D(view_cam));

//  pangolin::View &robot_display = pangolin::Display("robot view")
//          .SetHandler(new pangolin::Handler3D(robot_cam));

//  pangolin::Display("multi")
//        .SetBounds(0.0, 1.0, 0.0, 1.0)
//        .SetLayout(pangolin::LayoutEqual)
//        .AddDisplay(view_display)
//        .AddDisplay(robot_display);

//  glEnable(GL_DEPTH_TEST);

//  ros::Rate r(60);

//  while(!pangolin::ShouldQuit()) {
//      if(visualise) {
//      // visualisation
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//      /// free view

//      view_display.Activate(view_cam);

//      mutex.lock();

//      shader.Bind();
//      shader.SetUniform("MVP", view_cam.GetProjectionModelViewMatrix());
//      shader.Unbind();

//      pangolin::glDrawAxis(1);    // robot pose in world
//      const pangolin::OpenGlMatrix T_wc = robot.T_wr*robot.T_cr.Inverse();    // camera pose
//      pangolin::glDrawAxis(T_wc, 0.5);  // camera pose

//      robot.render(shader, true);

//      /// robot view

//      const int w = camera_info.width;
//      const int h = camera_info.height;

//      robot_display.SetAspect(w/double(h));
//      robot_display.Activate(robot_cam);

//      pangolin::glDrawAxis(1);
//      pangolin::glDrawRectPerimeter(-1,-1,1,1);
//      pangolin::glDrawAxis(T_wc, 0.5);

//      shader.Bind();
//      shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
//      shader.Unbind();

//      robot.render(shader, true);

//      mutex.unlock();

//      } // visualise

//      pangolin::FinishFrame();

//      ros::spinOnce();
//      r.sleep();
//  }
}

bool StateRenderer::render(std::shared_ptr<robot_state_renderer::RenderRobotState> &srv) {
    std::cout << "manual call ..." << std::endl;
    // wait for construction
//    std::unique_lock<std::mutex> lck_setup(mtx_setup_done);
    std::cout << " wait constructed ..." << std::endl;
//    cv_setup_done.wait(lck_setup);

    std::cout << " ... constructed " << std::endl;

    render_call = srv;
    cond_req.notify_all();
//    has_request2 = true;

    // wait for result
    std::unique_lock<std::mutex> lck_res(mtx_res);
    std::cout << "waiting result ... " << std::endl;
    cond_res.wait(lck_res);
    std::cout << "... done" << std::endl;
    return true;
}

bool StateRenderer::render(robot_state_renderer::RenderRobotStateRequest &req, robot_state_renderer::RenderRobotStateResponse &res) {

    std::cout << "ROS call ..." << std::endl;
    // wait for construction
  //    std::unique_lock<std::mutex> lck_setup(mtx_setup_done);
    std::cout << " wait constructed ..." << std::endl;
  //    cv_setup_done.wait(lck_setup);

    std::cout << " ... constructed " << std::endl;

    render_call->request = req;
    cond_req.notify_all();
  //    has_request.store(true);
//    has_request2 = true;

    // wait for result
    std::cout << "waiting ... " << std::endl;
    std::unique_lock<std::mutex> lck_res(mtx_res);
    cond_res.wait(lck_res);
    res = render_call->response;
    std::cout << "... done" << std::endl;
    return true;

//    // set state

//    // check dimensions
//    if(req.state.name.size()!=req.state.position.size()) {
//        throw std::runtime_error("number of joint names ("+std::to_string(req.state.name.size())+") and values ("+std::to_string(req.state.position.size())+") mismatch");
//    }

//    // camera pose, T_wc
//    Eigen::Isometry3d cam_pose;
//    tf::poseMsgToEigen(req.camera_pose, cam_pose);

//    // robot pose, T_wr
//    Eigen::Isometry3d robot_pose;
//    tf::poseMsgToEigen(req.robot_pose, robot_pose);

//    mutex.lock();

//    const pangolin::OpenGlMatrix T_wc(cam_pose.matrix());
//    robot.T_wr = pangolin::OpenGlMatrix(robot_pose.matrix());
//    robot.T_cr = T_wc.Inverse()*robot.T_wr;

//    // clear and set joint values
//    robot.joints.clear();
//    for(uint i = 0; i<req.state.name.size(); i++) {
//        robot.joints[req.state.name[i]] = float(req.state.position[i]);
//    }

//    robot.updateFrames();

//    for(const std::pair<std::string, uint> &kv : robot.link_label_id) {
//        res.link_names.push_back(kv.first);
//        // transformation from root to link
//        const pangolin::OpenGlMatrix T_rl = RobotModel::MatrixFromFrame(robot.getFramePose(kv.first));
//        const pangolin::OpenGlMatrix T_cl = robot.T_cr * T_rl;
//        geometry_msgs::Pose pose;
//        tf::poseEigenToMsg(Eigen::Affine3d(T_cl), pose);
//        res.link_poses.push_back(pose);
//    }

//    camera_info = req.camera_info;

//    // render

//    pangolin::View &robot_display = pangolin::Display("robot view")
//            .SetHandler(new pangolin::Handler3D(robot_cam));

//    const int w = int(camera_info.width);
//    const int h = int(camera_info.height);
//    const double fu = camera_info.K[0];
//    const double fv = camera_info.K[4];
//    const double cx = camera_info.K[2];
//    const double cy = camera_info.K[5];

//    // check camera parameter
//    if(w*h==0)        ROS_WARN_STREAM("invalid camera dimension");
//    if(!(fu*fv>0.0))  ROS_WARN_STREAM("invalid focal length");
//    if(!(cx*cy>0.0))  ROS_WARN_STREAM("invalid camera centre");

//    const double z_near = 0.0001;
//    const double z_far = 1000;

//    // camera coordinate system
//    // http://www.songho.ca/opengl/gl_projectionmatrix.html#perspective
//    // x - Right, y - Down, z - Front
//    robot_cam.SetProjectionMatrix(pangolin::ProjectionMatrixRDF_TopLeft(
//        w, h, fu, fv, cx, cy, z_near, z_far));
//    robot_cam.SetModelViewMatrix(T_wc.Inverse());

//    robot_display.SetAspect(w/double(h));
//    pangolin::Viewport offscreen_view(0,0,w,h);

//    color_buffer.Reinitialise(w,h);
//    depth_buffer.Reinitialise(w,h);
//    pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);

//    fbo_buffer.Bind();
//    offscreen_view.Activate();

//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    shader.Bind();
//    shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
//    shader.Unbind();

//    robot.render(shader, false);

//    glFlush();

//    mutex.unlock();

//    // read depth in OpenGL coordinates
//    cv::Mat_<float> depth_gl(h, w);
//    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth_gl.data);

//    // read green channel as label
//    cv::Mat_<uint8_t> label(h, w);
//    glReadPixels(0, 0, w, h, GL_GREEN, GL_UNSIGNED_BYTE, label.data);

//    fbo_buffer.Unbind();

//    cv::Mat_<cv::Vec3d> points(h, w);
//    for(int y(0); y<h; y++) {
//        for(int x(0); x<w; x++) {
//            if(depth_gl(y,x)<1.0f) {
//                offscreen_view.GetCamCoordinates(robot_cam, x, y, double(depth_gl(y,x)), points(y,x)[0], points(y,x)[1], points(y,x)[2]);
//            }
//        }
//    }

//    // transform 2D pixel coordinates from OpenGL (RUB) to camera frame (RDF)
//    // rotate by pi around x-axis
//    cv::flip(points, points, 0);
//    cv::flip(label, label, 0);

//    res.point_cloud.height = uint(h);
//    res.point_cloud.width  = uint(w);
//    res.point_cloud.header = req.camera_info.header;
//    res.point_cloud.is_dense = false;
//    res.point_cloud.is_bigendian = false;

//    sensor_msgs::PointCloud2Modifier pcd_modifier(res.point_cloud);
//    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
//    sensor_msgs::PointCloud2Iterator<float> iter_xyz(res.point_cloud, "x");
//    for(int y(0); y<h; y++) {
//        for(int x(0); x<w; x++) {
//            iter_xyz[0] = float(points(y,x)[0]);
//            iter_xyz[1] = float(points(y,x)[1]);
//            iter_xyz[2] = float(points(y,x)[2]);
//            ++iter_xyz;
//        }
//    }

//    // mask
//    cv_bridge::CvImage mask_img(req.camera_info.header, "mono8", label);
//    mask_img.toImageMsg(res.mask);

//    // points
//    cv_bridge::CvImage points_img(req.camera_info.header, "64FC3", points);
//    points_img.toImageMsg(res.points);

//    return true;
}
