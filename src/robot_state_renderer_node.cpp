#include <ros/ros.h>
#include <robot_state_renderer/RenderRobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>
#include <RobotModel.hpp>
#include <pangolin/pangolin.h>


static const std::string LabelVertexShader = \
        "#version 300 es\n"
        "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
        "uniform mat4 MVP;\n"
        "uniform mat4 M; // ModelMatrix\n"
        "void main(){\n"
        "    gl_Position =  MVP * M * vec4(vertexPosition_modelspace,1);\n"
        "}\n"
;

static const std::string LabelFragmentShader = \
        "#version 300 es\n"
        "out mediump vec4 color;\n"
        "uniform mediump vec4 label_colour;\n"
        "void main(){\n"
        "  color = label_colour;\n"
        "}\n"
;

class StateRenderer {
public:
    StateRenderer(int argc, char *argv[]) {

        // transform from OpenGL to camera frame
        // http://www.songho.ca/opengl/gl_projectionmatrix.html#perspective
        G << 1, 0, 0, 0,
             0,-1, 0, 0,
             0, 0,-1, 0,
             0, 0, 0, 1;

        ros::init(argc, argv, "StateRenderer");
        n = std::make_shared<ros::NodeHandle>();

        std::string urdf_path;
        ros::param::param<std::string>("~urdf_path", urdf_path, argv[1]);
        robot.parseURDF(urdf_path);
        robot.loadLinkMeshes();
        robot.loadJointNames();
        robot.generateMeshColours(false, true);

        pangolin::CreateWindowAndBind("StateRenderer",640,480);

        pangolin::OpenGlRenderState view_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.01,100),
            pangolin::ModelViewLookAt(0,-1,1, 0,0,0, pangolin::AxisY)
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

        shader.AddShader(pangolin::GlSlVertexShader, LabelVertexShader);
        shader.AddShader(pangolin::GlSlFragmentShader, LabelFragmentShader);
        shader.Link();

        robot.renderSetup();

        service = n->advertiseService("render", &StateRenderer::render, this);

        while(!pangolin::ShouldQuit()) {
            // visualisation
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

            pangolin::FinishFrame();

            ros::spinOnce();
        }
    }

    ~StateRenderer() { }

    bool render(robot_state_renderer::RenderRobotStateRequest &req, robot_state_renderer::RenderRobotStateResponse &res) {

        // set state

        // check dimensions
        if(req.state.name.size()!=req.state.position.size()) {
            throw std::runtime_error("number of joint names ("+std::to_string(req.state.name.size())+") and values ("+std::to_string(req.state.position.size())+") mismatch");
        }

        // camera pose
        Eigen::Isometry3d cam_pose;
        tf::poseMsgToEigen(req.camera_pose, cam_pose);

        // robot pose
        Eigen::Isometry3d robot_pose;
        tf::poseMsgToEigen(req.robot_pose, robot_pose);

        mutex.lock();

        robot.T_cr = pangolin::OpenGlMatrix(cam_pose.inverse().matrix());
        robot.T_wr = pangolin::OpenGlMatrix(robot_pose.inverse().matrix());
        const pangolin::OpenGlMatrix T_wc = robot.T_wr*robot.T_cr.Inverse();    // camera pose

        // clear and set joint values
        robot.joints.clear();
        for(uint i = 0; i<req.state.name.size(); i++) {
            robot.joints[req.state.name[i]] = float(req.state.position[i]);
        }

        robot.updateFrames();

        camera_info = req.camera_info;

        // render

        pangolin::View &robot_display = pangolin::Display("robot view")
                .SetHandler(new pangolin::Handler3D(robot_cam));

        const int w = camera_info.width;
        const int h = camera_info.height;
        const double fu = camera_info.K[0];
        const double fv = camera_info.K[4];
        const double cx = camera_info.K[2];
        const double cy = camera_info.K[5];

        const double z_near = 0.0001;
        const double z_far = 1000;

        robot_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            w, h, fu, fv, cx, cy, z_near, z_far));
        robot_cam.SetModelViewMatrix(G*T_wc.Inverse());

        robot_display.SetAspect(w/double(h));

        color_buffer.Reinitialise(w,h);
        depth_buffer.Reinitialise(w,h);
        pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);

        fbo_buffer.Bind();
        glViewport(0,0,w,h);
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
        for(double y(0); y<h; y++) {
            for(double x(0); x<w; x++) {
                if(depth_gl(y,x)<1.0) {
                    robot_display.GetCamCoordinates(robot_cam, x, y, depth_gl(y,x), points(y,x)[0], points(y,x)[1], points(y,x)[2]);
                }
            }
        }

        // from OpenGL to camera frame: rotate by pi around x-axis
        // - flip around x-axis (rotate pi around x-axis)
        // - inverse z
        cv::flip(points, points, 0);
        cv::flip(label, label, 0);
        cv::Mat zs;
        cv::extractChannel(points, zs, 2);
        zs *= -1;
        cv::insertChannel(zs, points, 2);

        // mask
        cv_bridge::CvImage mask_img(req.state.header, "mono8", label);
        mask_img.toImageMsg(res.mask);

        // points
        cv_bridge::CvImage points_img(req.state.header, "64FC3", points);
        points_img.toImageMsg(res.points);

        return true;
    }

private:
    RobotModel robot;
    pangolin::GlSlProgram shader;
    std::shared_ptr<ros::NodeHandle> n;
    ros::ServiceServer service;
    std::mutex mutex;

    pangolin::GlTexture color_buffer;
    pangolin::GlRenderBuffer depth_buffer;

    Eigen::Matrix4d G;

    pangolin::OpenGlRenderState robot_cam;

    sensor_msgs::CameraInfo camera_info;
};

int main(int argc, char *argv[]) {
    StateRenderer(argc, argv);
}
