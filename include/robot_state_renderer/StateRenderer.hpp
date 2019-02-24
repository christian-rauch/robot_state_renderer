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
    /**
     * @brief StateRenderer construct the renderer
     * The StateRenderer can be used as stand-alone library or as dedicated
     * server process. As library, the render method 'render' can directly be
     * called to benefit from shared memory. As server process, it will advertise
     * a ROS service and listen for incomming render requests.
     * If used as stand-alone library within another process, it's recommended
     * to not advertise the render service and visualise the rendered state.
     * @param urdf_path path to URDF file with robot definition
     * @param advertise_service set to true to advertise the render service (default: false)
     * If the state is visualised in another thread of a  process that already
     * handles ROS events, this needs to be set to false.
     * @param visualise set to true to show the rendered state (default: false)
     */
    StateRenderer(const std::string urdf_path = std::string(), const bool advertise_service = false, const bool visualise = false);

    ~StateRenderer();

    /**
     * @brief run process incomming render requests
     * This either shows the visualised state and processes one request per rendered
     * state or runs the ROS single-threaded event loop without visualisation.
     * This method will return immediately if 'advertise_service' and 'visualise'
     * are both set to false.
     * @param period_ms delay (in milliseconds) between visualisation, small values will decrease the offscreen rendering speed
     */
    void run(const uint period_ms = 10);

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

    std::map<std::string, std::shared_ptr<Mesh>> mesh_cache;

    std::vector<std::tuple<std::shared_ptr<Mesh>&, std::string, Eigen::Isometry3d>> objects;

    bool is_setup = false;

    const bool visualise;

    const bool advertise_service;
};

#endif // STATERENDERER_HPP
