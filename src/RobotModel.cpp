#include "RobotModel.hpp"

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>

#include <experimental/filesystem>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <GL/glew.h>

#define PACKAGE_PATH_URI_SCHEME "package://"

#include <ros/package.h>

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

void RobotModel::createKinematicTree() {
    if(urdf_model==nullptr) {
        throw std::runtime_error("no robot model");
    }

    kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree);
    T_wr.SetIdentity();
}

void RobotModel::parseURDFString(const std::string &urdf_string) {
    // parse URDF string
    urdf_model = urdf::parseURDF(urdf_string);
    createKinematicTree();

    mesh_package_path = std::string();
}

void RobotModel::parseURDF(const std::string &urdf_path) {
    // read URDF file and create kinematic tree
    urdf_model = urdf::parseURDFFile(urdf_path);
    createKinematicTree();

    // get location of urdf package for mesh path
    mesh_package_path = getROSPackagePath(urdf_path.substr(0, urdf_path.find_last_of('/')));
}

void RobotModel::loadLinkMeshes() {
    // get root frame
    root_frame = urdf_model->getRoot()->name;

    std::vector<urdf::LinkSharedPtr> links;
    urdf_model->getLinks(links);

    // load mesh for each link
    for(const urdf::LinkSharedPtr & l : links) {
        for(size_t i=0; i<l->visual_array.size(); i++) {
            const auto & vis = l->visual_array[i];
            if(vis->geometry->type==urdf::Geometry::MESH) {
                std::string mesh_path = dynamic_cast<urdf::Mesh*>(&*vis->geometry)->filename;

                if(mesh_path.find(PACKAGE_PATH_URI_SCHEME) != std::string::npos) {
                    // given as ROS package url
                    // we need to replace "package://" by package path
                    std::string mod_url = mesh_path;
                    mod_url.erase(0, strlen(PACKAGE_PATH_URI_SCHEME));
                    const size_t pos = mod_url.find("/");
                    const std::string package_path = ros::package::getPath(mod_url.substr(0, pos));
                    mod_url.erase(0, pos);

                    if(package_path.size()!=0) {
                        // found ROS package in workspace
                        mesh_path = package_path + mod_url;
                    }
                    else {
                        // manually replace ROS package path
                        const size_t pos = mesh_path.find(PACKAGE_PATH_URI_SCHEME);
                        mesh_path.replace(pos, std::string(PACKAGE_PATH_URI_SCHEME).length(), mesh_package_path);
                    }
                }
                else {
                    // given as relative path, prepend base path
                    mesh_path = mesh_package_path + mesh_path;
                }

                link_meshes[l->name].push_back(MeshLoader::getMesh(mesh_path));
            }
            else {
                // convert primitive shape to mesh
                // https://github.com/ros-planning/geometric_shapes/blob/melodic-devel/src/mesh_operations.cpp
                MeshPtr mesh = std::make_unique<Mesh>();
                switch (vis->geometry->type) {
                // Sphere
                case urdf::Geometry::SPHERE: {
                    const urdf::SphereSharedPtr sphere = std::dynamic_pointer_cast<urdf::Sphere>(vis->geometry);

                    const float r = float(sphere->radius);
                    const float pi = float(M_PI);
                    const unsigned int seg = std::max<unsigned int>(6, 0.5 + 2.0 * pi * r / 0.01);
                    const unsigned int ring = std::max<unsigned int>(6, 2.0 * r / 0.01);

                    float phi, phid;
                    phid = pi * 2.0f / seg;
                    phi = 0.0;

                    float theta, thetad;
                    thetad = pi / (ring + 1);
                    theta = 0;

                    for (unsigned int i = 0; i < ring; ++i) {
                        float theta_ = theta + thetad * (i + 1);
                        for (unsigned int j = 0; j < seg; ++j) {
                            mesh->vertices.push_back({r * sin(theta_) * cos(phi + j * phid), r * sin(theta_) * sin(phi + j * phid), r * cos(theta_)});
                        }
                    }
                    mesh->vertices.push_back({0.0, 0.0, r});
                    mesh->vertices.push_back({0.0, 0.0, -r});

                    for (unsigned int i = 0; i < ring - 1; ++i) {
                        for (unsigned int j = 0; j < seg; ++j) {
                            unsigned int a, b, c, d;
                            a = i * seg + j;
                            b = (j == seg - 1) ? (i * seg) : (i * seg + j + 1);
                            c = (i + 1) * seg + j;
                            d = (j == seg - 1) ? ((i + 1) * seg) : ((i + 1) * seg + j + 1);
                            mesh->faces.push_back({a, c, b});
                            mesh->faces.push_back({b, c, d});
                        }
                    }

                    for (unsigned int j = 0; j < seg; ++j) {
                        unsigned int a, b;
                        a = j;
                        b = (j == seg - 1) ? 0 : (j + 1);
                        mesh->faces.push_back({ring * seg, a, b});

                        a = (ring - 1) * seg + j;
                        b = (j == seg - 1) ? (ring - 1) * seg : ((ring - 1) * seg + j + 1);
                        mesh->faces.push_back({a, ring * seg + 1, b});
                    }
                    break;
                }

                // Box
                case urdf::Geometry::BOX: {
                    const urdf::BoxSharedPtr box = std::dynamic_pointer_cast<urdf::Box>(vis->geometry);

                    const float x = float(box->dim.x / 2.0);
                    const float y = float(box->dim.y / 2.0);
                    const float z = float(box->dim.z / 2.0);

                    mesh->vertices.push_back({-x, -y, -z});
                    mesh->vertices.push_back({x, -y, -z});
                    mesh->vertices.push_back({x, -y, z});
                    mesh->vertices.push_back({-x, -y, z});
                    mesh->vertices.push_back({-x, y, z});
                    mesh->vertices.push_back({-x, y, -z});
                    mesh->vertices.push_back({x, y, z});
                    mesh->vertices.push_back({x, y, -z});

                    mesh->faces = { {0, 1, 2}, {2, 3, 0}, {4, 3, 2}, {2, 6, 4}, {7, 6, 2}, {2, 1, 7},
                                    {3, 4, 5}, {5, 0, 3}, {0, 5, 7}, {7, 1, 0}, {7, 5, 4}, {4, 6, 7} };
                    break;
                }

                // Cylinder
                case urdf::Geometry::CYLINDER: {
                    const urdf::CylinderSharedPtr cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(vis->geometry);

                    static const unsigned int tot_for_unit_cylinder = 100;
                    const float r = float(cylinder->radius);
                    const float h = float(cylinder->length);

                    const float pi = float(M_PI);
                    const unsigned int tot = std::max<unsigned int>(6, ceil(tot_for_unit_cylinder * r));
                    const float phid = pi * 2 / tot;

                    const float circle_edge = phid * r;
                    const unsigned int h_num = ceil(std::abs(h) / circle_edge);

                    float phi = 0;
                    const float hd = h / h_num;

                    for (unsigned int i = 0; i < tot; ++i) {
                        mesh->vertices.push_back({r * cos(phi + phid * i), r * sin(phi + phid * i), h / 2});
                    }

                    for (unsigned int i = 0; i < h_num - 1; ++i) {
                        for (unsigned int j = 0; j < tot; ++j) {
                            mesh->vertices.push_back({r * cos(phi + phid * j), r * sin(phi + phid * j), h / 2 - (i + 1) * hd});
                        }
                    }

                    for (unsigned int i = 0; i < tot; ++i) {
                        mesh->vertices.push_back({r * cos(phi + phid * i), r * sin(phi + phid * i), -h / 2});
                    }

                    mesh->vertices.push_back({0, 0, h / 2});
                    mesh->vertices.push_back({0, 0, -h / 2});

                    for (unsigned int i = 0; i < tot; ++i) {
                        mesh->faces.push_back({(h_num + 1) * tot, i, (i == tot - 1) ? 0 : (i + 1)});
                    }

                    for (unsigned int i = 0; i < tot; ++i) {
                        mesh->faces.push_back({(h_num + 1) * tot + 1, h_num * tot + ((i == tot - 1) ? 0 : (i + 1)), h_num * tot + i});
                    }

                    for (unsigned int i = 0; i < h_num; ++i) {
                        for (unsigned int j = 0; j < tot; ++j) {
                            int a, b, c, d;
                            a = j;
                            b = (j == tot - 1) ? 0 : (j + 1);
                            c = j + tot;
                            d = (j == tot - 1) ? tot : (j + 1 + tot);

                            uint start = i * tot;
                            mesh->faces.push_back({start + b, start + a, start + c});
                            mesh->faces.push_back({start + b, start + c, start + d});
                        }
                    }
                    break;
                }

                // Mesh, already processed in if block
                case urdf::Geometry::MESH: break;
                }

                link_meshes[l->name].push_back(std::move(mesh));
            }

            if(vis->material!=NULL){
                // workaround: only the colour of the first visual is recognised
                // apply the first visual's colours to all visuals
                const urdf::Color colour = l->visual_array[0]->material->color;
                link_colours[l->name].emplace_back(colour.r, colour.g, colour.b, colour.a);
            }

            const urdf::Pose pose = vis->origin;
            visual_origins[l->name].push_back(KDL::Frame(KDL::Rotation::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w), KDL::Vector(pose.position.x, pose.position.y, pose.position.z)));
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
        frame_poses_gl[link_name] = link_pose;
    }
}

void RobotModel::renderSetup() {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        for(auto & m : it->second) {
            m->renderSetup();
        }
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
                std::cout << "Setting joint '" << jname << "' to 0." << std::endl;
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

        // skip frames without transformation
        if(!frame_poses_gl.count(link_name))
            continue;

        for(size_t imesh=0; imesh<it->second.size(); imesh++) {
          const pangolin::OpenGlMatrix M = MatrixFromFrame(frame_poses_gl.at(link_name)*visual_origins.at(link_name)[imesh]);

          // apply frame transformation to shader
          shader.Bind();
          shader.SetUniform("M", T_wr*M);
          if(link_colour) {
              if(link_meshes[it->first][imesh]->hasColour()) {
                  const auto c = link_meshes[it->first][imesh]->colour[0];
                  shader.SetUniform("label_colour", pangolin::Colour(c[0], c[1], c[2]));
              }
              else {
                  shader.SetUniform("label_colour", link_colours[it->first][imesh]);
              }
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

          (it->second)[imesh]->render(shader);

          if(mesh_masks!=NULL) {
              // render and export each link individually
              glFlush();

              (*mesh_masks)[link_name].Alloc(w, h, w);
              glReadBuffer(GL_BACK);
              glPixelStorei(GL_PACK_ALIGNMENT, 1);
              glReadPixels(0,0,w,h, GL_ALPHA, GL_UNSIGNED_BYTE, (*mesh_masks)[link_name].ptr );

              glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
          }
        } // imesh
    }
}
