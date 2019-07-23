#include "clouddata.h"

CloudData::CloudData( std::string path_str )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PolygonMesh mesh_;

    std::string filename_str;
    std::string type_cloud_;

    int return_status = -1;

    if (boost::algorithm::iends_with( path_str , ".pcd") ){
        type_cloud_   = "pcd";
        return_status = pcl::io::loadPCDFile( path_str, *cloud_tmp );

    }
    else if ( boost::algorithm::iends_with( path_str , ".ply") ) {
        type_cloud_   = "ply";
        return_status = pcl::io::loadPLYFile( path_str, *cloud_tmp );

        pcl::io::loadPolygonFilePLY(path_str, mesh_ );

        return_status = return_status || mesh_.cloud.width==0 ;
    }



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

    this->file_path  = path_str;
    this->file_name  = filename_str;
    this->cloud      = cloud_;
    this->cloud_mesh = mesh_;
    this->type_cloud = type_cloud_;

}

