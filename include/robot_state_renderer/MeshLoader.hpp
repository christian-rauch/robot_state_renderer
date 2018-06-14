#ifndef MESHLOADER_HPP
#define MESHLOADER_HPP

#include <string>
#include <memory>

#include "Mesh.hpp"

//#include <Eigen/Core>

//typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Point3DList;
//typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> Point2DList;
//typedef Eigen::Matrix<uint, Eigen::Dynamic, 3, Eigen::RowMajor> Index3DList;



//typedef std::shared_ptr<Mesh> MeshPtr;
typedef std::unique_ptr<Mesh> MeshPtr;

namespace MeshLoader {

MeshPtr getMesh(const std::string &path);

}


#endif // MESHLOADER_HPP
