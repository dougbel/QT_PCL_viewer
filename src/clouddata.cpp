#include "clouddata.h"

CloudData::CloudData( std::string path_str )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::string filename_str;

    int return_status = -1;

    if (boost::algorithm::iends_with( path_str , ".pcd") )
        return_status = pcl::io::loadPCDFile( path_str, *cloud_tmp );
    else if ( boost::algorithm::iends_with( path_str , ".ply") )
        return_status = pcl::io::loadPLYFile( path_str, *cloud_tmp );


    if (return_status != 0)
    {
        throw "Error reading point cloud";
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud (*cloud_tmp, *cloud_);
    else
    {
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
    }


    boost::filesystem::path pathObj(path_str);
    filename_str = pathObj.filename().string();

    this->file_path = path_str;
    this->file_name = filename_str;
    this->cloud = cloud_;

}

