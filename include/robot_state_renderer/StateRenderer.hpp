#ifndef STATERENDERER_HPP
#define STATERENDERER_HPP

#include <ros/ros.h>
#include <robot_state_renderer/RenderRobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>
#include "RobotModel.hpp"
#include <pangolin/pangolin.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>

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
    StateRenderer(const std::string default_urdf_path = std::string(), const bool advertise_service = true, const bool default_offscreen = false);

    ~StateRenderer();

    /**
     * @brief visualise visualise the requested state
     * @param period_ms delay (in milliseconds) between visualisation, small values will decrease the offscreen rendering speed
     */
    void visualise(const uint period_ms = 10);

    /**
     * @brief spin run the ROS event loop without visualisation
     */
    void spin();

    bool render(robot_state_renderer::RenderRobotStateRequest &req, robot_state_renderer::RenderRobotStateResponse &res);

private:
    RobotModel robot;
    pangolin::GlSlProgram shader;
    std::shared_ptr<ros::NodeHandle> n;
    ros::ServiceServer service;
    std::mutex mutex;

    pangolin::GlTexture color_buffer;
    pangolin::GlRenderBuffer depth_buffer;

    cv::Mat_<float> depth_gl;
    cv::Mat_<uint8_t> label;

    pangolin::OpenGlRenderState robot_cam;

    image_geometry::PinholeCameraModel camera_model;

    pangolin::OpenGlRenderState view_cam;

    std::unique_ptr<pangolin::Handler3D> view_cam_handler = nullptr;

    bool is_setup = false;
};

#endif // STATERENDERER_HPP
