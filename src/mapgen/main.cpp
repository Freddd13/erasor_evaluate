#include <experimental/filesystem>  // requires gcc version >= 8
#include "mapgen.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "kumo.h"
namespace fs = std::experimental::filesystem;

mapgen mapgenerator;

ros::Publisher cloudPublisher;
ros::Publisher mapPublisher;
ros::Publisher pathPublisher;
nav_msgs::Path path;

using namespace erasor;

std::string sequence;
std::string init_stamp = "";
std::string final_stamp = "";
std::string save_path;
std::string dataset_name;
std::string pose_url;
std::string cloud_bin_dir;

int interval;
int viz_interval;

bool is_large_scale;

float voxelsize;
const double scanPeriod = 0.1;


void saveGlobalMap() {
  std::string original_dir = save_path + dataset_name + "_" + init_stamp + "_to_" + final_stamp + "_w_interval" +
                             std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) + "_original.pcd";

  std::string map_dir = save_path + dataset_name + "_voxel_" + std::to_string(voxelsize) + ".pcd";
  std::cout << "save_dir: " << map_dir << std::endl;

  mapgenerator.saveNaiveMap(original_dir, map_dir);
}


inline bool GetScanID(int &scanID, float &verticalAngle, int &count, int num_lidar_beams) {
  if (num_lidar_beams == 16) {
    scanID = int((verticalAngle + 15) / 2 + 0.5);
    if (scanID > (num_lidar_beams - 1) || scanID < 0) {
      count--;
      return false;
    }
  } else if (num_lidar_beams == 32) {
    scanID = int((verticalAngle + 92.0 / 3.0) * 3.0 / 4.0);
    if (scanID > (num_lidar_beams - 1) || scanID < 0) {
      count--;
      return false;
    }
  } else if (num_lidar_beams == 64) {
    if (verticalAngle >= -8.83) {
      scanID = int((2 - verticalAngle) * 3.0 + 0.5);
    } else {
      scanID = num_lidar_beams / 2 + int((-8.83 - verticalAngle) * 2.0 + 0.5);
    }
    if (verticalAngle > 2 || verticalAngle < -24.33 || scanID > 50 || scanID < 0) {
      count--;
      return false;
    }
  }

  else if (num_lidar_beams == 40) {  // TODO config
    // TODO2 ground scan modify
    if (verticalAngle < -6) {
      scanID = int(round(verticalAngle + 15.6));
    } else if (verticalAngle >= -6 && verticalAngle < 2) {
      scanID = 10 + int(round((verticalAngle + 5.8) / 0.33));
    } else if (verticalAngle >= 2) {
      scanID = 34 + int(round(verticalAngle - 2.3));
    } else {
      count--;
      return false;
    }
  } else {
    printf("wrong scan number\n");
    ROS_BREAK();
    return false;
  }
  return true;
}


void SaveGRLOAMSpecificFilterIndexMapping(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                          pcl::PointCloud<pcl::PointXYZI> &cloud_out, float th1, float th2) {
  CloudXYZI::Ptr cloud_filtered_dist(new CloudXYZI());
  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    float dis = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y +
                cloud_in.points[i].z * cloud_in.points[i].z;
    if (dis < th1 * th1) continue;
    if (dis > th2 * th2) continue;
    if (cloud_in.points[i].x < 0 && abs(cloud_in.points[i].y) < 0.5) continue;

    cloud_filtered_dist->push_back(cloud_in.points[i]);
  }


  ////////////
  int num_lidar_beams = 16;
  std::vector<std::vector<int>> mapping_container_beams(num_lidar_beams);

  // 雷达点云投影
  int cloudSize = cloud_filtered_dist->points.size();
  float startOri = -atan2(cloud_filtered_dist->points[0].y, cloud_filtered_dist->points[0].x);
  float endOri =
      -atan2(cloud_filtered_dist->points[cloudSize - 1].y, cloud_filtered_dist->points[cloudSize - 1].x) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  int count = cloudSize;
  PointXYZI point;
  std::vector<pcl::PointCloud<PointXYZI>> laserCloudScans(num_lidar_beams);


  for (int i = 0; i < cloudSize; ++i) {
    point.x = cloud_filtered_dist->points[i].x;
    point.y = cloud_filtered_dist->points[i].y;
    point.z = cloud_filtered_dist->points[i].z;
    // if (use_semantic_poss_) {
    //   point.x = -cloud_filtered_dist.points[i].y;
    //   point.y = cloud_filtered_dist.points[i].x;
    // }
    //计算点的仰角(根据lidar文档垂直角计算公式),根据仰角排列激光线号，velodyne每两个scan之间间隔2度
    float verticalAngle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scanID = 0;
    if (!(GetScanID(scanID, verticalAngle, count, num_lidar_beams))) {
      continue;
    }

    // 计算点水平方向的转角，并根据其转角占lidar扫描角度的比值计算其时间占比
    float ori = -atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > startOri + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - startOri > M_PI) halfPassed = true;
    } else {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > endOri + M_PI / 2)
        ori -= 2 * M_PI;
    }
    laserCloudScans[scanID].push_back(point);
  }


  // 像slam一样合并
  for (int i = 0; i < num_lidar_beams; ++i) {
    cloud_out += laserCloudScans[i];
  }
}


/*
思路：
根据corr(timestamps.txt)读SLAM给出的位姿和对应全点云,拼接地图即可
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "merger");
  ros::NodeHandle nodeHandler;
  std::cout << "kumo map gen start!" << std::endl;

  float start_timestamp, end_timestamp;
  int start_id, end_id;
  std::string corr_url;

  nodeHandler.param("/map/voxelsize", voxelsize, (float)0.05);
  nodeHandler.param<std::string>("/map/save_path", save_path, "/mnt/d");
  nodeHandler.param<std::string>("/map/dataset_name", dataset_name, "machi");
  nodeHandler.param<std::string>("/map/cloud_bin_dir", cloud_bin_dir, "machi");
  nodeHandler.param<std::string>("/map/poses_url", pose_url, "machi");
  nodeHandler.param<std::string>("/map/corr_url", corr_url, "machi");
  nodeHandler.param<float>("/map/start_timestamp", start_timestamp, 999999999.9);
  nodeHandler.param<float>("/map/end_timestamp", end_timestamp, 999999999.9);

  nodeHandler.param<int>("/map/start_id", start_id, 999999999);
  nodeHandler.param<int>("/map/end_id", end_id, 999999999);

  nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

  mapgenerator.setValue(save_path, voxelsize, dataset_name, "1", "1", interval, is_large_scale);

  save_path = (save_path.back() == '/') ? save_path : (save_path + "/");
  dataset_name = (dataset_name.back() == '/') ? dataset_name : (dataset_name + "/");
  cloud_bin_dir = (cloud_bin_dir.back() == '/') ? cloud_bin_dir : (cloud_bin_dir + "/");


  // 生成地图!!!! tatakai!!!!!!!!
  // 对每个bin, 配对位姿 （去掉最前面几帧！）
  // pose_with_timestamp
  std::ifstream f_read_pose, f_read_corr;
  f_read_pose.open(pose_url, std::fstream::in);
  f_read_corr.open(corr_url, std::fstream::in);
  std::cout << pose_url << std::endl;
  std::cout << corr_url << std::endl;

  /////////////// 读corr corr[timestamp]=filename_int ///////////////////
  std::map<std::string, std::string> corr_data;  // errors
  std::string t_corr, cloud_name_corr;
  while (f_read_corr >> cloud_name_corr >> t_corr) {
    corr_data[t_corr] = cloud_name_corr;  // input them into the map
  }
  f_read_corr.close();
  std::cout << "read map success" << std::endl;
  // for (auto&element : corr_data){
  //   std::cout << element.first << " " << element.second << std::endl;
  // }

  ////////////// 读pose ////////////////////////
  if (f_read_pose.is_open()) {
    printf("begin load pose... \n");
    char line[1024];
    Eigen::Matrix4f pose_eigen4f = Eigen::Matrix4f::Identity();
    while (f_read_pose.getline(line, sizeof(line), '\n')) {
      char *p;
      p = strtok(line, " ");
      std::string timestamp;  //时间
      bool time_init = false;
      std::vector<double> pose;
      while (p != NULL) {
        if (!time_init) {
          time_init = true;
          timestamp = p;
        }
        else {
          pose.push_back(atof(p));
        }
        p = strtok(NULL, " ");
      }
      ///// 读完了,看是否满足要求
      if (atof(timestamp.c_str()) < start_timestamp || atof(timestamp.c_str()) > end_timestamp) {
        continue;
      }

      // 读corr
      if (corr_data.count(timestamp)) {
        std::ostringstream tmp;
        tmp << std::setw(6) << std::setfill('0') << corr_data[timestamp];
        std::string cloud_file_name = cloud_bin_dir + tmp.str() + ".bin";
        // 读完pose timestamp, 找相应的pointcloud bin
        // std::cout << cloud_file_name << std::endl;


        // 这里是bin，不能用pcl的函数
        pcl::PointCloud<pcl::PointXYZI> cloud_raw;
        fstream input(cloud_file_name.c_str(), ios::in | ios::binary);
        
        while (input.good()) {
          pcl::PointXYZI point;
          input.read((char *)&point.x, 3 * sizeof(float));
          input.read((char *)&point.intensity, sizeof(float));
          if (input.eof()) {
            break;
          }
          cloud_raw.push_back(point);
        }
        input.close();


        // 处理和SLAM一致
        CloudXYZI cloud;
        std::vector<int> _;
        pcl::removeNaNFromPointCloud(cloud_raw, cloud_raw, _);
        SaveGRLOAMSpecificFilterIndexMapping(cloud_raw,cloud, 0.5, 80);
        // std::cout << "re after " << cloud.size() << std::endl;

        // for (auto&pt:cloud.points) {
        //   if (!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z) || !pcl_isfinite(pt.intensity)) {
        //     ROS_WARN("1");
        //     continue;
        //   }
        // }

        if (cloud.size()==0) {
          std::cout << cloud_file_name << " nopoints!!!"<< std::endl;
          return -1;
        }

        // for (auto& ps : pose) {
        //   std::cout << ps << " ";
        // }
        // std::cout << std::endl;
        // std::cout << std::endl;

        // 读pose
        pose_eigen4f(0, 0) = pose[0];
        pose_eigen4f(0, 1) = pose[1];
        pose_eigen4f(0, 2) = pose[2];
        pose_eigen4f(0, 3) = pose[3];
        pose_eigen4f(1, 0) = pose[4];
        pose_eigen4f(1, 1) = pose[5];
        pose_eigen4f(1, 2) = pose[6];
        pose_eigen4f(1, 3) = pose[7];
        pose_eigen4f(2, 0) = pose[8];
        pose_eigen4f(2, 1) = pose[9];
        pose_eigen4f(2, 2) = pose[10];
        pose_eigen4f(2, 3) = pose[11];

        mapgenerator.KumoAccumulateCloud(cloud, pose_eigen4f);

        // 送进去！！！！
      } else {
        std::cout << "!!!! no such file !!! " << timestamp << std::endl;
        continue;
      }
    }
    saveGlobalMap();
  }
  f_read_pose.close();

  return 0;
}
