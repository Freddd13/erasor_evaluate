/*** 
 * @Date: 2023-02-04 18:52:20
 * @LastEditors: yxt
 * @LastEditTime: 2023-02-04 19:57:13
 * @Description: 
 */
#include <experimental/filesystem>  // requires gcc version >= 8
#include "kumo.h"
#include "mapgen.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "kumo_process.hpp"

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
std::string slam_map_save_path;
std::string dataset_name;
std::string pose_url;
std::string cloud_bin_dir;

int interval;
int viz_interval;

bool is_large_scale;

float voxelsize;
const double scanPeriod = 0.1;


void saveGlobalMap() {
  std::string original_dir = slam_map_save_path + dataset_name + "_" + init_stamp + "_to_" + final_stamp + "_w_interval" +
                             std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) + "_original.pcd";

  std::string map_dir = slam_map_save_path + dataset_name + "_voxel_" + std::to_string(voxelsize) + ".pcd";
  std::cout << "save_dir: " << map_dir << std::endl;

  mapgenerator.saveNaiveMap(original_dir, map_dir);
}



/*
思路：
读SLAM给出的判断的每帧动态点索引文件，读相应的位姿和其全点云。
由于这里使用了SLAM一样的点云前处理，就不需要index mapping，它处理后可以直接使用动态索引
将SLAM的结果标注到读入全点云的intensity上，拼接地图即可
*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "merger");
  ros::NodeHandle nodeHandler;
  std::cout << "kumo map gen start!" << std::endl;

  float start_timestamp, end_timestamp;
  int start_id, end_id;
  std::string corr_url, index_mapping_dir, slam_dynamic_indices_dir;

  nodeHandler.param("/map/voxelsize", voxelsize, (float)0.05);
  nodeHandler.param<std::string>("/map/slam_map_save_path", slam_map_save_path, "/mnt/d");
  nodeHandler.param<std::string>("/map/dataset_name", dataset_name, "machi");
  nodeHandler.param<std::string>("/map/cloud_bin_dir", cloud_bin_dir, "machi");
  nodeHandler.param<std::string>("/map/poses_url", pose_url, "machi");
  nodeHandler.param<std::string>("/map/corr_url", corr_url, "machi");
  nodeHandler.param<std::string>("/map/slam_dynamic_indices_dir", slam_dynamic_indices_dir, "machi");
  nodeHandler.param<float>("/map/start_timestamp", start_timestamp, 999999999.9);
  nodeHandler.param<float>("/map/end_timestamp", end_timestamp, 999999999.9);

  nodeHandler.param<int>("/map/start_id", start_id, 999999999);
  nodeHandler.param<int>("/map/end_id", end_id, 999999999);

  nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

  mapgenerator.setValue(slam_map_save_path, voxelsize, dataset_name, "1", "1", interval, is_large_scale);

  slam_map_save_path = (slam_map_save_path.back() == '/') ? slam_map_save_path : (slam_map_save_path + "/");
  dataset_name = (dataset_name.back() == '/') ? dataset_name : (dataset_name + "/");
  cloud_bin_dir = (cloud_bin_dir.back() == '/') ? cloud_bin_dir : (cloud_bin_dir + "/");
  slam_dynamic_indices_dir =
      (slam_dynamic_indices_dir.back() == '/') ? slam_dynamic_indices_dir : (slam_dynamic_indices_dir + "/");


  // 生成地图!!!! tatakai!!!!!!!!
  // 对每个bin, 配对位姿 （去掉最前面几帧！）
  // pose_with_timestamp
  std::ifstream f_read_pose, f_read_corr;
  f_read_pose.open(pose_url, std::fstream::in);
  f_read_corr.open(corr_url, std::fstream::in);

  // 将corr 读进一个map
  std::map<std::string, std::string> corr_data;  // errors
  std::string t_corr, cloud_name_corr;
  while (f_read_corr >> cloud_name_corr >> t_corr) {
    corr_data[t_corr] = cloud_name_corr;  // input them into the map
  }
  f_read_corr.close();

  // debug
  std::cout << pose_url << std::endl;
  std::cout << corr_url << std::endl;


  std::cout << "read map success" << std::endl;

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

      // 读入位姿
      std::vector<double> pose;
      while (p != NULL) {
        if (!time_init) {
          time_init = true;
          timestamp = p;
        } else {
          pose.push_back(atof(p));
        }
        p = strtok(NULL, " ");
      }


      ///// 读完了时间+pose,看时间是否满足要求
      if (atof(timestamp.c_str()) < start_timestamp || atof(timestamp.c_str()) > end_timestamp) {
        continue;
      }

      ///// 必须slam有这点云
      std::string url_slam_dynamic_indices = slam_dynamic_indices_dir + timestamp + ".txt";
      if (!fs::exists(url_slam_dynamic_indices)) {
        std::cout << "skip " << timestamp << " for lack of slam data" << std::endl;
        continue;
      }

      // 读corr
      if (corr_data.count(timestamp)) {
        std::ostringstream tmp;
        tmp << std::setw(6) << std::setfill('0') << corr_data[timestamp];
        std::string cloud_file_name = cloud_bin_dir + tmp.str() + ".bin";
        // 读完pose timestamp, 找相应的pointcloud bin
        if (!fs::exists(cloud_file_name)) {
          std::cout << "!!!skip " << timestamp << " for lack of cloud data " << cloud_file_name << std::endl;
          continue;
        }

        // 这里是bin，不能用pcl的函数
        pcl::PointCloud<pcl::PointXYZI> cloud_raw;
        fstream input(cloud_file_name.c_str(), ios::in | ios::binary);

        // 读入全点云
        int index_original = 0;
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
        SaveGRLOAMSpecificFilterIndexMapping(cloud_raw, cloud, 0.5, 80);
        if (cloud.size() == 0) {
          std::cout << cloud_file_name << " nopoints!!!" << std::endl;
          return -1;
        }

        // 读入slam dynamic indices, 并赋值相应intensity
        std::ifstream f_slam_dynamic_indices(url_slam_dynamic_indices);
        std::vector<int> data_slam_dynamic_indices;  //记录了动态索引
        int element;
        int num_dynamic = 0;
        while (f_slam_dynamic_indices >> element) {
          // data_slam_dynamic_indices.push_back(element);
          num_dynamic++;
          cloud.points[element].intensity = 254;//TODO
        }
        // std::cout << " num_dynamic " << num_dynamic << std::endl;


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
