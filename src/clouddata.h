#ifndef CLOUDDATAH_H
#define CLOUDDATAH_H

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>



class CloudData{

public:

    std::string file_path;
    std::string file_name;

    std::string type_cloud;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PolygonMesh cloud_mesh;

    CloudData( std::string file_path );

};
#endif // CLOUDDATAH_H
