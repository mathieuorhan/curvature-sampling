#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

float ThirdValuePCA(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    // Tested and working
    // To fix : warning from PCL
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
    if(data.fail() || labels.fail()){
        std::cerr << "Error when reading the files" << std::endl;
        exit(1);
    }
    
    // Don't forget to check that the file open operation succeeds!
    
    float x, y, z, i, l; 
    int r, g, b;
    // Read the file.
    while(data >> x >> y >> z >> i >> r >> g >> b && labels >> l)
    {
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

int main (int argc, char** argv)
{
  if(argc<2) 
  {
    std::cerr << "USAGE : " << argv[0] << " path/to/data/file" << std::endl;
    exit(1);
  }
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBL>);
  load_raw_file(argv[1],*pc); 
  display_cloud(*pc);
  return (0);
}