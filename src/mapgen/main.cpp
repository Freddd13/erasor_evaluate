#include <experimental/filesystem>  // requires gcc version >= 8
#include "mapgen.hpp"
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

void saveGlobalMap() {
  std::string original_dir = save_path + sequence + "_" + init_stamp +
                             "_to_" + final_stamp + "_w_interval" +
                             std::to_string(interval) + "_voxel_" +
                             std::to_string(voxelsize) + "_original.pcd";

  std::string map_dir = save_path + "/" + sequence + "_voxel_" +
                        std::to_string(voxelsize) + ".pcd";

  mapgenerator.saveNaiveMap(original_dir, map_dir);
}

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
  nodeHandler.param<float>("/map/start_timestamp", start_timestamp,
                           999999999.9);
  nodeHandler.param<float>("/map/end_timestamp", end_timestamp, 999999999.9);

  nodeHandler.param<int>("/map/start_id", start_id, 999999999);
  nodeHandler.param<int>("/map/end_id", end_id, 999999999);

  nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

  mapgenerator.setValue(save_path, voxelsize, dataset_name, "1", "1", interval,
                        is_large_scale);

  save_path = (save_path.back() == '/') ? save_path : (save_path + "/");
  dataset_name =
      (dataset_name.back() == '/') ? dataset_name : (dataset_name + "/");
  cloud_bin_dir =
      (cloud_bin_dir.back() == '/') ? cloud_bin_dir : (cloud_bin_dir + "/");

  // 生成地图!!!! tatakai!!!!!!!!
  // 对每个bin, 配对位姿 （去掉最前面几帧！）
  // pose_with_timestamp
  std::ifstream f_read_pose, f_read_corr;
  f_read_pose.open(pose_url, std::fstream::in);
  f_read_corr.open(corr_url, std::fstream::in);
  std::cout << pose_url << std::endl;
  std::cout << corr_url << std::endl;

  /////////////// 读corr ///////////////////
  std::map<std::string, std::string> corr_data;  // errors
  std::string t_corr, cloud_name_corr;
  while (f_read_corr >> cloud_name_corr >> t_corr) {
    corr_data[t_corr] = cloud_name_corr;  // input them into the map
  }
  std::cout << "read map success" << std::endl;
  // for (auto&element : corr_data){
  //   std::cout << element.first << " " << element.second << std::endl;
  // }

  ////////////// 读pose ////////////////////////
  if (f_read_pose.is_open() && f_read_corr.is_open()) {
    printf("begin load pose... \n");
    char line[1024];
    Eigen::Matrix4f pose_eigen4f = Eigen::Matrix4f::Identity();
    while (f_read_pose.getline(line, sizeof(line), '\n')) {
      char *p;
      p = strtok(line, " ");
      std::string timestamp = p;  //时间
      p = strtok(line, " ");
      std::vector<double> pose;
      while (p != NULL) {
        pose.push_back(atof(p));
        p = strtok(NULL, " ");
      }
      ///// 读完了,看是否满足要求
      if (atof(timestamp.c_str()) < start_timestamp ||
          atof(timestamp.c_str()) > end_timestamp) {
        continue;
      }

      // 读corr
      if (corr_data.count(timestamp)) {
        std::ostringstream tmp;
        tmp << std::setw(6) << std::setfill('0') << corr_data[timestamp];
        std::string cloud_file_name = cloud_bin_dir + tmp.str() + ".bin";
        // 读点云
        std::cout << cloud_file_name << std::endl;

        // 送进去！！！！
      } else {
        std::cout << "!!!! no such file !!! " << timestamp << std::endl;
        continue;
      }

      // pose_eigen4f(0, 0) = pose[1];
      // pose_eigen4f(0, 1) = pose[2];
      // pose_eigen4f(0, 2) = pose[3];
      // pose_eigen4f(0, 3) = pose[4];
      // pose_eigen4f(1, 0) = pose[5];
      // pose_eigen4f(1, 1) = pose[6];
      // pose_eigen4f(1, 2) = pose[7];
      // pose_eigen4f(1, 3) = pose[8];
      // pose_eigen4f(2, 0) = pose[9];
      // pose_eigen4f(2, 1) = pose[10];
      // pose_eigen4f(2, 2) = pose[11];
      // pose_eigen4f(2, 3) = pose[12];

      // 读完pose timestamp, 找相应的pointcloud bin
    }
  }
  f_read_pose.close();

  // int num = 0;
  // for (const auto &entry : fs::directory_iterator(pose_url)) {
  //   std::cout << entry.path() << std::endl;
  //   num++;
  // }

  // for () {
  //   mapgenerator.KumoAccumulateCloud(msg, path);
  // }
  // saveGlobalMap();

  return 0;
}
