#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <sstream>
#include <dirent.h>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include <sys/stat.h>

#include <pcl/io/boost.h>
#include <boost/program_options.hpp>

#include <fstream>
void copyAnnoFile(std::string &in_file, std::string &out_file)
{
    std::cout << "copy " << in_file << " to " << out_file << std::endl;
    std::ifstream in(in_file, std::ios::binary);
    std::ofstream out(out_file, std::ios::binary);
    out << in.rdbuf();
    out.flush();
    out.close();
    in.close();
}

void convertPCDtoBin(std::string &in_file, std::string &out_file, float h)
{
    std::cout << "convert " << in_file << " to " << out_file << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1) //* load the file
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return; // (-1);
    }
    std::ofstream out(out_file, std::ios::binary);
    char buf[sizeof(float) * 4];
    if (out.good())
    {
        char *ptr = buf;
        for (auto point = cloud->points.begin(); point != cloud->points.end(); point++)
        {
            point->z += h;
            memcpy(ptr, &(point->x), sizeof(float));
            memcpy(ptr + sizeof(float), &(point->y), sizeof(float));
            memcpy(ptr + sizeof(float) * 2, &(point->z), sizeof(float));
            memcpy(ptr + sizeof(float) * 3, &(point->intensity), sizeof(float));
            out.write(buf, sizeof(float) * 4);
        }
        out.flush();
        out.close();
        std::cout << "read " << cloud->points.size() << " points, write to " << out_file << std::endl;
    }
    else
    {
        std::cerr << "cannot read this file " << out_file << std::endl;
    }
}

void read_filelists(const std::string &dir_path, std::vector<std::string> &out_filelsits, std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    if (dir == NULL)
    {
        std::cout << "read failed" << std::endl;
        std::cerr << "no such file or directory: " << dir_path << std::endl;
        return;
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')
            continue;
        if (type.size() <= 0)
        {
            out_filelsits.push_back(ptr->d_name);
            std::cout << ptr->d_name << std::endl;
        }
        else
        {
            if (tmp_file.size() < type.size())
                continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(), type.size());
            if (tmp_cut_type == type)
            {
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

std::string path_join(const std::string &parent, const std::string &child)
{
    return std::string(parent).append("/").append(child);
}

std::string next_file(int start = 0)
{
    static int count = start;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << count;
    count++;
    return ss.str();
}

void create_folders(std::vector<std::string> folders)
{
    for (auto f : folders)
    {
        std::cout << f << std::endl;
    }
}

int main(int argc, char const *argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: "
                     "bin2ascii [training|testing]"
                  << std::endl;
        exit(2);
    }

    std::string in_folder = "/root/compdata/";
    std::string out_folder = "/Datasets/";
    if (strcmp(argv[1], "training") == 0)
    {
        std::string in_files[] = {
            "part1", "part10", "part12",
            "part13", "part14", "part16", "part17",
            "part3", "part4", "part5"};
        float height[] = {
            8.8862, 8.67158, 7.29706,
            6.56072, 7.5917, 8.26836, 8.83257,
            8.0304, 6.3609, 6.63761};
        std::cout << argv[1] << std::endl;
        out_folder.append("training");
        in_folder.append("lidarData");
        std::vector<std::string> src_ann_list;
        std::vector<std::string> src_pcd_list;
        auto h = height;
        // for (auto in : in_files)
        for (int i = 0; i < 10; i++)
        {
            std::string in = in_files[i];
            std::cout << in << std::endl;

            auto in_path = path_join(in_folder, in);
            auto src_ann = path_join(in_path, "annotation");
            auto src_pcd = path_join(in_path, "pcd");
            auto out_pcd = path_join(out_folder, "velodyne");
            auto out_ann = path_join(out_folder, "label_2");
            create_folders({out_pcd, out_ann});
            read_filelists(src_pcd, src_pcd_list, "pcd");
            read_filelists(src_ann, src_ann_list, "txt");

            std::cout << "pcd list size is " << src_pcd_list.size()
                      << "\tanno list size is " << src_ann_list.size() << std::endl;

            if (src_pcd_list.size() == src_ann_list.size())
            {
                std::cout << "start convert, please waiting" << std::endl;
                for (size_t i = 0; i < src_pcd_list.size(); i++)
                {
                    std::string file_name = src_pcd_list[i].substr(0, src_pcd_list[i].length() - 4);
                    auto src = path_join(src_ann, file_name).append(".txt");
                    std::string count;
                    if (argc == 3)
                    {
                        count = next_file(atoi(argv[2]));
                    }
                    else
                    {
                        count = next_file();
                    }

                    auto des = path_join(out_ann, count);
                    des.append(".txt");
                    copyAnnoFile(src, des);

                    src = path_join(src_pcd, file_name).append(".pcd");
                    des = path_join(out_pcd, count);
                    des.append(".bin");
                    convertPCDtoBin(src, des, h[i] - 2.2);
                }
            }
        }
    }

    else if (strcmp(argv[1], "testing") == 0)
    {
        std::cout << argv[1] << std::endl;
        out_folder.append("testing");
        in_folder.append("test_pcd");
        std::vector<std::string> src_pcd_list;
        read_filelists(in_folder, src_pcd_list, "pcd");
        std::string count;
        for (size_t i = 0; i < src_pcd_list.size(); i++)
        {
            count = next_file();
            auto src = path_join(in_folder, src_pcd_list[i]);
            auto des = path_join(out_folder, "velodyne");
            des = path_join(des, count);
            des.append(".bin");
            convertPCDtoBin(src, des, 0);
        }
    }
    else
    {
        std::cerr << "input arguments error" << std::endl;
        return 1;
    }

    return 0;
}
