
#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include <sys/stat.h>

#include <pcl/io/boost.h>
#include <boost/program_options.hpp>

#include <fstream>

void convertPCDtoASCII(std::string &in_file, std::string &out_file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1) //* load the file
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return; // (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from "
              << in_file
              << " with the following fields: "
              << std::endl;

    std::vector<pcl::PCLPointField> fields;
    std::vector<int> fields_sizes;
    size_t fsize = 0;
    size_t data_size = 0;
    size_t nri = 0;
    pcl::getFields(*cloud, fields);
    for (size_t i = 0; i < fields.size(); i++)
    {
        std::cout << fields[i] << std::endl;
    }
    
    cloud->erase(cloud->end()-1);

    // Compute the total size of the fields
    std::cout << "data_size size: " << data_size << std::endl;
    pcl::io::savePCDFileASCII(out_file.data(), *cloud);
}

int main(int argc, char const *argv[])
{
    std::string in(argv[1]), out(argv[2]);
    convertPCDtoASCII(in, out);
    return 0;
}