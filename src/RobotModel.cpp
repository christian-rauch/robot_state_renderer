#include "RobotModel.hpp"

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser.hpp>

#include <experimental/filesystem>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <GL/glew.h>

#define PACKAGE_PATH_URI_SCHEME "package://"

namespace fs = std::experimental::filesystem;

// find the file "package.xml" for obtaining the root folder of our mesh
std::string getROSPackagePath(const std::string &start_path,
                              const std::string ros_package_path_file = "package.xml")
{
    fs::path fpath = fs::canonical(start_path);

    // search backwards for package path, e.g. directory that contains the package file
    while(fpath.has_parent_path() && !fs::is_regular_file(fpath / ros_package_path_file)) {
        // go one step backward closer to root
        fpath = fpath.parent_path();
    }

    std::string package_path = "";
    if(!fs::is_regular_file(fpath / ros_package_path_file)) {
        // package path not found, use relative path
        package_path = fs::canonical(start_path).parent_path().native()
                + fs::path::preferred_separator;
    }
    else {
        // store package path with trailing directory seperator
        package_path = fpath.parent_path().native() + fs::path::preferred_separator;
    }
    return package_path;
}

RobotModel::RobotModel(const std::string &urdf_path) {
    parseURDF(urdf_path);
}

void RobotModel::parseURDF(const std::string &urdf_path) {
    // read URDF and create kinematic tree
    urdf_model = urdf::parseURDFFile(urdf_path);
    if(urdf_model!=NULL) {
        kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree);
    }
    else {
        std::cerr<<"no robot model"<<std::endl;
    }

    // get location of urdf package for mesh path
    std::string urdf_dir = urdf_path.substr(0, urdf_path.find_last_of('/'));
    mesh_package_path = getROSPackagePath(urdf_dir);
}

void RobotModel::loadLinkMeshes() {
    // get root frame
    root_frame = urdf_model->getRoot()->name;

    std::vector<boost::shared_ptr<urdf::Link>> links;
    urdf_model->getLinks(links);

    // load mesh for each link
    for(boost::shared_ptr<urdf::Link> l : links) {
        if(l->visual!=NULL) {
            //
            if(l->visual->geometry->type==urdf::Geometry::MESH) {
                std::string mesh_path = dynamic_cast<urdf::Mesh*>(&*l->visual->geometry)->filename;

                if(mesh_path.find(PACKAGE_PATH_URI_SCHEME) != std::string::npos) {
                    // we need to replace "package://" by full path
                    boost::algorithm::replace_first(mesh_path, PACKAGE_PATH_URI_SCHEME, mesh_package_path);
                }
                else {
                    // prepend full path
                    mesh_path = mesh_package_path + mesh_path;
                }

                link_meshes[l->name] = MeshLoader::getMesh(mesh_path);
            }

            if(l->visual->material!=NULL){
                const urdf::Color colour = l->visual->material->color;
                link_colours[l->name].r = colour.r;
                link_colours[l->name].g = colour.g;
                link_colours[l->name].b = colour.b;
                link_colours[l->name].a = colour.a;
            }
        }
    }
}

void RobotModel::loadJointNames() {
    T_wr.SetIdentity();
    // initialise joint map
    for(auto j : urdf_model->joints_) {
        joints[j.first] = 0;
    }
}

void RobotModel::updateFrames() {
    // camera pose in world frame
    // chose camera pose from FK or from external
    const KDL::Frame camera_pose = hasFrame(camera_frame_name) ? getFramePose(camera_frame_name) : FrameFromMatrix(T_cr);

    // get pose of each link
    for(const auto& s : robot_tree.getSegments()) {
        const std::string link_name = s.first;

        const KDL::Frame link_pose = getFramePose(link_name);
        frame_poses[link_name] = camera_pose.Inverse()*link_pose;

        const pangolin::OpenGlMatrix M = MatrixFromFrame(link_pose);
        frame_poses_gl[link_name] = M;
    }
}

void RobotModel::renderSetup() {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        it->second->renderSetup();
    }
}

void RobotModel::generateMeshColours(const bool single_colour, const bool gray) {
    pangolin::ColourWheel colours(1.0, 1.0, 1.0);
    uint i = 0;
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        if(single_colour) {
            link_label_colours[it->first] = pangolin::Colour::White();
        }
        else {
            // unique colours per link
            i++;
            link_label_id[it->first] = i;
            // set unique gray level
            link_label_colours_gray[it->first] = pangolin::Colour(i/255.0f, i/255.0f, i/255.0f, 1.0);
            // set unique rgb colour
            link_label_colours_rgb[it->first] = colours.GetUniqueColour();
        }
    }
    link_label_colours = gray ? link_label_colours_gray : link_label_colours_rgb;
//    std::cout<<"labels: "<<i<<std::endl;
}

bool RobotModel::hasFrame(const std::string &frame) const {
    return (robot_tree.getSegment(frame)!=robot_tree.getSegments().cend());
}

KDL::Frame RobotModel::getFramePose(const std::string& frame) {
    // kineamtic chain from root to frame
    KDL::Chain chain;
    if(!robot_tree.getChain(root_frame, frame, chain)) {
        throw std::runtime_error("frame '"+frame+"' is not part of the kinematic chain");
    }
    KDL::ChainFkSolverPos_recursive fk(chain);

    // get list of joints in chain (from root to tip)
    std::vector<float> joint_values;
    for(KDL::Segment segm : chain.segments) {
        const std::string jname = segm.getJoint().getName();
        // only add values of not NONE joints
        if(segm.getJoint().getType()!=KDL::Joint::None) {
            if(joints.count(jname)) {
                joint_values.push_back(joints.at(jname));
            }
            else {
                // set not provided joint values to 0
                joint_values.push_back(0);
            }
        }
    }

    assert(joint_values.size()==chain.getNrOfJoints());

    // populate chain joint values
    KDL::JntArray j(chain.getNrOfJoints());
    for(uint i(0); i<chain.getNrOfJoints(); i++) {
        j(i) = joint_values[i];
    }

    // get pose of frame
    KDL::Frame frame_pose;
    fk.JntToCart(j, frame_pose);

    return frame_pose;
}

pangolin::OpenGlMatrix RobotModel::MatrixFromFrame(const KDL::Frame &frame_pose) {
    // model matrix
    pangolin::OpenGlMatrix M;
    M.SetIdentity();
    // translation
    M(0, 3) = frame_pose.p.x();
    M(1, 3) = frame_pose.p.y();
    M(2, 3) = frame_pose.p.z();
    // rotation
    M(0,0) = frame_pose.M(0,0);
    M(0,1) = frame_pose.M(0,1);
    M(0,2) = frame_pose.M(0,2);
    M(1,0) = frame_pose.M(1,0);
    M(1,1) = frame_pose.M(1,1);
    M(1,2) = frame_pose.M(1,2);
    M(2,0) = frame_pose.M(2,0);
    M(2,1) = frame_pose.M(2,1);
    M(2,2) = frame_pose.M(2,2);

    return M;
}

KDL::Frame RobotModel::FrameFromMatrix(const pangolin::OpenGlMatrix &M) {
    KDL::Frame frame_pose;
    // translation
    frame_pose.p.x(M(0, 3));
    frame_pose.p.y(M(1, 3));
    frame_pose.p.z(M(2, 3));
    // rotation
    frame_pose.M(0,0) = M(0,0);
    frame_pose.M(0,1) = M(0,1);
    frame_pose.M(0,2) = M(0,2);
    frame_pose.M(1,0) = M(1,0);
    frame_pose.M(1,1) = M(1,1);
    frame_pose.M(1,2) = M(1,2);
    frame_pose.M(2,0) = M(2,0);
    frame_pose.M(2,1) = M(2,1);
    frame_pose.M(2,2) = M(2,2);
    return frame_pose;
}

void RobotModel::render(pangolin::GlSlProgram &shader, const bool link_colour, std::map<std::string, pangolin::Image<uint8_t>> *mesh_masks) {
    // get viewport dimensions
    int w,h;
    if(mesh_masks!=NULL) {
        GLint vp_dim[4];
        glGetIntegerv(GL_VIEWPORT, vp_dim);
        w = vp_dim[2];
        h = vp_dim[3];
    }

    // get pose of each link and render mesh
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        const std::string link_name = it->first;

        // skip meshes that should not be rendered
        if(skipMeshes.count(link_name))
            continue;

        const pangolin::OpenGlMatrix M = frame_poses_gl[link_name];

        // apply frame transformation to shader
        shader.Bind();
        shader.SetUniform("M", T_wr*M);
        if(link_colour) {
            shader.SetUniform("label_colour", link_colours[it->first]);
        }
        else {
            if(!link_label_colours.empty()) {
                shader.SetUniform("label_colour", link_label_colours[it->first]);
            }
            else {
                throw std::runtime_error("requested to render label colours, but none are given");
            }
        }
        shader.Unbind();

        it->second->render(shader);

        if(mesh_masks!=NULL) {
            // render and export each link individually
            glFlush();

            (*mesh_masks)[link_name].Alloc(w, h, w);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0,0,w,h, GL_ALPHA, GL_UNSIGNED_BYTE, (*mesh_masks)[link_name].ptr );

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }
    }
}
