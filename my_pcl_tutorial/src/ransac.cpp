#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

using namespace std;

struct points {
	float x;
	float y;
	float z;
	float rgb;
};

int main(int argc, char *argv[]) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("Cup_high_view.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("Cup_Table_view_smoothed.pcd", *model) == -1) {
		PCL_ERROR("Couldn't read file Cup_Table_view_smoothed.pcd \n");
		return (-1);
	}

	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);

	vector<points> pclp;
	vector<points> samples;
	int pointcloud_size = cloud_filtered->width * cloud_filtered->height;
	cout << "Number of points in the cloud: " << pointcloud_size << endl;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it =
			cloud_filtered->begin(); it != cloud_filtered->end(); it++) {

		points poinT;
		poinT.x = it->x;
		poinT.y = it->y;
		poinT.z = it->z;
		poinT.rgb = it->rgb;
		pclp.push_back(poinT);

	}
	for (int maxSample = 0; maxSample < 20; maxSample++) {
		int number = rand() % pointcloud_size;
		samples.push_back(pclp.at(number));
		cout << "The number is: " << number << endl;
	}
	for (vector<points>::iterator it = samples.begin(); it != samples.end();
			it++) {
		cout << "The samples are: " << "X: " << it->x << ", " << " Y: " << it->y
				<< ", " << " Z: " << it->z << ", " << " RGB: " << it->rgb
				<< endl;
	}
	cout << "Size: " << samples.size() << endl;

	return 0;
}
