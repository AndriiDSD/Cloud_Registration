#include <stdio.h>
#include <string.h>

#include <boost/make_shared.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/point_representation.h>



#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

//#include <pcl/features/normal_3d.h>

//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
//#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void pairAlign (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
	if(downsample)
	{
		voxelGrid.setLeafSize(0.05, 0.05, 0.05);
		voxelGrid.setinputCloud(cloud_src);
		voxelGrid.filter(*src);	

		voxelGrid.setInputCloud(cloud_tgt);
		voxelGrid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt= cloud_tgt;
	}
}

int main(int argc, char **argv)
{
	if(argc!=3)
	{
		std::cout << "Usage: " << argv[0] << " file1.pcl "<< "file2.pcl "<<std::endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud1_ptr) != 0)
	{
		std::cout << "Could not open "<< argv[1]<< std::endl;
		return -1;
	}


	//remove NAN points from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud1_ptr,*cloud1_ptr, indices);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *cloud2_ptr) != 0)
	{
		std::cout << "Could not open "<< argv[2]<< std::endl;
		return -1;
	}

	std::vector<int> indices2;
        pcl::removeNaNFromPointCloud(*cloud2_ptr,*cloud2_ptr, indices2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	//for(int i=0; i<=1; i++)
	//{
	source = cloud1_ptr;
	target = cloud2_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	pairAlign (source, target, temp, pairTransform, true);

	pcl::transformPointCloud (*temp, *result, GlobalTransform);

	GlobalTransform = GlobalTransform * pairTransform;
		
	//}

	int viewID1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, viewID1);
	viewer->setBackgroundColor(0.3,0.3,0.3, viewID1);
	viewer->addText("First", 10, 10, "v1 text", viewID1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud1_ptr, 0, 0, 255);	
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud1_ptr,rgb, "sample cloud1", viewID1);


	//viewer->showCloud(cloud1_ptr);
	int viewID2(1);
	viewer->createViewPort (0.5, 0.0, 1.0, 0.5, viewID2);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, viewID2);
	viewer->addText ("Second", 10, 10, "v2 text", viewID2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>  rgb2(cloud2_ptr,0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud2_ptr,rgb2,"sample cloud2", viewID2);


	int viewID3(2);
	viewer->createViewPort (0.0, 0.5, 1.0, 1.0, viewID3);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, viewID3);
	viewer->addText ("Third", 10, 10, "v3 text", viewID3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>  rgb3(cloud1_ptr,255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud1_ptr,rgb3,"sample cloud3", viewID3);
	//pcl::io::savePCDFile ("out.pcd", *result, true);

	while (!viewer->wasStopped())
	{
		// Do nothing but wait.
		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;
}

















