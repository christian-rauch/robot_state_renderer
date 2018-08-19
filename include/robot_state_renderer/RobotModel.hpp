#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <urdf_model/model.h>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <set>

#include "Mesh.hpp"
#include "MeshLoader.hpp"

typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePrt;

class RobotModel {
private:
    ModelInterfacePrt urdf_model;

    KDL::Tree robot_tree;

    std::string root_frame;

    std::string mesh_package_path;

    std::set<std::string> skipMeshes;

    // link poses in world (OpenGL) frame
    // this stores poses of all links, not only those with mesh
    std::map<std::string, KDL::Frame> frame_poses_gl;

    // link poses in camera frame
    // this stores poses of all links, not only those with mesh
    std::map<std::string, KDL::Frame> frame_poses;

    std::map<std::string, KDL::Frame> frame_origins;

public:
    RobotModel() { }

    RobotModel(const std::string &urdf_path);

    void parseURDF(const std::string &urdf_path);

    /**
     * @brief loadLinkMeshes load all meshes defined in URDF
     */
    void loadLinkMeshes();

    /**
     * @brief loadJointNames initialise the joints and the robot pose
     */
    void loadJointNames();

    /**
     * @brief updateFrames do FK for all links
     */
    void updateFrames();

    /**
     * @brief renderSetup initialise OpenGL buffer and upload mesh data
     */
    void renderSetup();

    void generateMeshColours(const bool single_colour=true, const bool gray=false);

    /**
     * @brief render render the whole robot with the provided share
     * @param shader shader (texture, colour, ...)
     * @param link_colour true: use the original link colour, false: use label colour
     */
    void render(pangolin::GlSlProgram &shader, const bool link_colour = true, std::map<std::string, pangolin::Image<uint8_t>> *mesh_masks = NULL);

    void addSkip(const std::string& link) { skipMeshes.insert(link); }

    void resetSkip() { skipMeshes.clear(); }

    bool hasFrame(const std::string &frame) const;

    KDL::Frame getFramePose(const std::string& frame);

    pangolin::OpenGlMatrix getFramePoseMatrix(const std::string& frame) {
        return MatrixFromFrame(getFramePose(frame));
    }

    static pangolin::OpenGlMatrix MatrixFromFrame(const KDL::Frame &frame_pose);

    static KDL::Frame FrameFromMatrix(const pangolin::OpenGlMatrix &M);

    KDL::Frame& getLinkPoseInCameraFrame(const std::string& link_name) {
        return frame_poses.at(link_name);
    }

    const std::string& getRootFrame() const {
        return root_frame;
    }

    std::map<std::string, MeshPtr> link_meshes;

    std::map<std::string, pangolin::Colour> link_label_colours;

    std::map<std::string, pangolin::Colour> link_label_colours_gray;

    std::map<std::string, pangolin::Colour> link_label_colours_rgb;

    // original link colour as given by the URDF
    std::map<std::string, pangolin::Colour> link_colours;

    std::map<std::string, uint> link_label_id;

    std::map<std::string, float> joints;    // current joint positions

    pangolin::OpenGlMatrix T_wr;    // robot pose in world

    pangolin::OpenGlMatrix T_cr;    // robot pose in camera

    std::string camera_frame_name;
};

#endif // ROBOTMODEL_HPP
