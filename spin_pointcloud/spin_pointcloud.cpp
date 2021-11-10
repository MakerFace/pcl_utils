#include <CSF.h>
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "Cfg.h"
#include <vector>
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/common/transforms.h>
#include <math.h>

struct callback_args{
    // PointXYZI point;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void MouseCallback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;

    if (event.getPointIndex() == -1)
    {
        return;
    }

    pcl::PointXYZ currentPoint;
    event.getPoint(currentPoint.x, currentPoint.y, currentPoint.z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clickedPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	std::cout << currentPoint.x << " " << currentPoint.y << " " << currentPoint.z << std::endl;


}

void PointCloudToCSF(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CSF &csf)
{
	std::vector<std::vector<float>> points;
	std::vector<float> point(3);
	for (auto it : cloud->points)
	{
		point[0] = it.x;
		point[1] = it.y;
		point[2] = it.z;
		points.push_back(point);
	}
	csf.setPointCloud(points);
	return;
}

int main(int argc, char **argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(argv[1], *cloud);
	std::cout << argv[2] << std::endl;
	Cfg cfg;
	std::string slop_smooth;
	cfg.readConfigFile(argv[2], "slop_smooth", slop_smooth);
	bool ss = false;
	if (slop_smooth == "true" || slop_smooth == "True")
	{
		ss = true;
	}
	else if (slop_smooth == "false" || slop_smooth == "False")
	{
		ss = false;
	}
	else
	{
		if (atoi(slop_smooth.c_str()) == 0)
		{
			ss = false;
		}
		else
		{
			ss = true;
		}
	}

	std::string class_threshold;
	cfg.readConfigFile(argv[2], "class_threshold", class_threshold);
	std::string cloth_resolution;
	cfg.readConfigFile(argv[2], "cloth_resolution", cloth_resolution);
	std::string iterations;
	cfg.readConfigFile(argv[2], "iterations", iterations);
	std::string rigidness;
	cfg.readConfigFile(argv[2], "rigidness", rigidness);
	std::string time_step;
	cfg.readConfigFile(argv[2], "time_step", time_step);
	std::string terr_pointClouds_filepath;
	cfg.readConfigFile(argv[2], "terr_pointClouds_filepath", terr_pointClouds_filepath);

	CSF csf;

	PointCloudToCSF(cloud, csf);

	csf.params.bSloopSmooth = ss;
	csf.params.class_threshold = atof(class_threshold.c_str());
	csf.params.cloth_resolution = atof(cloth_resolution.c_str());
	csf.params.interations = atoi(iterations.c_str());
	csf.params.rigidness = atoi(rigidness.c_str());
	csf.params.time_step = atof(time_step.c_str());

	std::vector<int> groundIndexes, offGroundIndexes;
	csf.do_filtering(groundIndexes, offGroundIndexes, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	// Mandatory-设置目标几何形状
	seg.setModelType(pcl::SACMODEL_PLANE);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置误差容忍范围，也就是我说过的阈值
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);

	for (auto it : groundIndexes)
	{
		inliers->indices.push_back(it);
		cloudTemp->points.push_back(cloud->points[it]);
	}


    // auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PointCloud"));
    // viewer->addPointCloud<pcl::PointXYZ>(cloudTemp, "Calibration");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Calibration");
    // viewer->setCameraPosition(0, 0, -2, 0, -1, 0);

	// struct callback_args cb_args;
    // cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    // viewer->registerPointPickingCallback(MouseCallback, (void*)&cb_args);


	// viewer->spin();

	seg.segment(*inliers, *coefficients);

	float a, b, c, d;

	a = coefficients->values[0];
	b = coefficients->values[1];
	c = coefficients->values[2];
	d = coefficients->values[3];

	std::cout << a << " " << b << " " << c << " " << d << std::endl;

	Eigen::Vector3f n = Eigen::Vector3f();

	n << a, b, c;

	n.normalize();

	Eigen::Vector3f e = Eigen::Vector3f(0, 0, 1);

	float theta = acos(n.dot(e));

	std::cout << "theta: " << theta * 180 / 3.1415926 << std::endl;
	Eigen::Matrix3f t;
	// t(0, 0) = pow()
	// pcl::getTransformationFromTwoUnitVectorsAndOrigin();

	return 0;
}
// 34.2469 -1.49357 -8.25707
// 44.9157 3.18541 -8.25862
// 27.9862 2.67811 -8.22851