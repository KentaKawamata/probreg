//fuck
#include <iostream>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


int main(int argc, char* argv[]) {

    std::string filepath = "/mnt/container-data/ply_data/";
    int count=0;

    namespace fs = boost::filesystem;
    if(!fs::exists(filepath) || !fs::is_directory(filepath)) {
        std::cout << "file path misstake !!" << std::endl;
        exit(0);
    }

    fs::directory_iterator last;
    for(fs::directory_iterator pos(filepath); pos!=last; ++pos) {
        ++count;
    }

    std::cout << "files num : " << std::to_string(count) << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i=0; i<count; i++){

        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        std::string filename = filepath + std::to_string(i) + ".ply";
        std::cout << "READ FILE NAME : " << filename << std::endl;
        pcl::io::loadPLYFile(filename, *cloud);


        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");
        passX.setFilterLimits(-2.0, 2.0);
        passX.setFilterLimitsNegative(false);
        passX.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(cloud);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(0.0, 2.55);
        passY.setFilterLimitsNegative(false);
        passY.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passZ;
        passZ.setInputCloud(cloud);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(0.0, 2.0);
        passZ.setFilterLimitsNegative(false);
        passZ.filter(*cloud);

        std::string save_filename = "/mnt/container-data/remove_plane/" + std::to_string(i) + ".ply"; 
        std::cout << "SAVE FILE NAME : " << save_filename << std::endl;
        pcl::io::savePLYFileASCII(save_filename, *cloud);

    }

    return 0;
}
