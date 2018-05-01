#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree_search.h>

float ThirdValuePCA(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // Tested and working
    // To fix : warning from PCL
    // To fix : adapt to XYZRGBL
    pcl::PCA<pcl::PointXYZ> pca(cloud);
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    std::cout << "First eigenvalue : " << eigenvalues(0) << std::endl;
    std::cout << "Second eigenvalue : " << eigenvalues(1) << std::endl;
    std::cout << "Third eigenvalue : " << eigenvalues(2) << std::endl;
    return(eigenvalues(2));
}

void load_raw_file(std::string filename, pcl::PointCloud<pcl::PointXYZRGBL> &pc)
{
    std::string filename_data = std::string(filename)+std::string(".txt");
    std::string filename_labels = std::string(filename)+std::string(".labels");
  
    // load xyzirgb
    std::ifstream data(filename_data.c_str());
    // load labels
    std::ifstream labels(filename_labels.c_str());

    // Check sucess
    if(data.fail() || labels.fail()){
        std::cerr << "Error when reading the files" << std::endl;
        exit(1);
    }
    
    float x, y, z, i; 
    int r, g, b, l;
    int unlabeled = 0;
    // Read the file.
    while(data >> x >> y >> z >> i >> r >> g >> b && labels >> l)
    {
      if(l==unlabeled)
      {
        continue;
      }
      // we don't use i
      pcl::PointXYZRGBL point;
      point.x = x;
      point.y = y;
      point.z = z;  
      point.r = r;
      point.g = g;
      point.b = b;
      point.label = l;
      pc.push_back(point);
    }
}

void display_cloud(pcl::PointCloud<pcl::PointXYZRGBL> &pc)
{
  // use this function to test that everything is ok

  for (size_t i = 0; i < pc.points.size(); ++i)
  {
    std::cout << pc.points[i].x << " " << pc.points[i].y << " " << pc.points[i].z << " ";
    // RGB is stored as unsigned char, to display it one have to convert them to int first
    std::cout << (int)pc.points[i].r << " " << (int)pc.points[i].g << " " << (int)pc.points[i].b << " ";
    std::cout << pc.points[i].label << std::endl;
  }
}

void compute_octree(float resolution, const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc)
{
  // Octree and K-NN search demo

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBL> octree(resolution);
  octree.setInputCloud(pc);
  octree.addPointsFromInputCloud();
  
  pcl::PointXYZRGBL searchPoint;

  searchPoint = pc->points[0];

  // Neighbors within voxel search

  std::vector<int> pointIdxVec;

  if (octree.voxelSearch(searchPoint, pointIdxVec))
  {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x 
              << " " << searchPoint.y 
              << " " << searchPoint.z << ")" 
              << std::endl;
              
    for(size_t i = 0; i < pointIdxVec.size(); ++i) 
    {
      std::cout << "    " << pc->points[pointIdxVec[i]].x 
                << " " << pc->points[pointIdxVec[i]].y 
                << " " << pc->points[pointIdxVec[i]].z << std::endl;
    }
  }
}

int main (int argc, char** argv)
{
  if(argc<2) 
  {
    std::cerr << "USAGE : " << argv[0] << " path/to/data/file" << std::endl;
    exit(1);
  }
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBL>);
  std::cout << "Loading point cloud...";
  load_raw_file(argv[1],*pc); 
  std::cout << " Done." << std::endl;
  std::cout << "Point cloud contain " << pc->width << " points after removing unlabeled data." << std::endl;
  std::cout << "Computing Octree...";
  compute_octree(0.5, pc);
  std::cout << " Done." << std::endl;

  //display_cloud(*pc);
  return (0);
}