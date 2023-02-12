/***
 * @Date: 2023-02-04 18:52:20
 * @LastEditors: yxt
 * @LastEditTime: 2023-02-12 01:02:36
 * @Description:
 */
#include <experimental/filesystem>  // requires gcc version >= 8
#include "kumo.h"
#include "kumo_process.hpp"
#include "mapgen.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"

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


void KumoSaveFeatureCloud(CloudXYZI::Ptr cloud_full, float leafsize, std::string name) {
  std::string map_dir = slam_map_save_path + dataset_name + "_" + name + "_" + init_stamp + "_to_" + final_stamp + "_voxel_" + std::to_string(leafsize) +
                        ".pcd";
  std::cout << "save_dir: " << map_dir << std::endl;

  CloudXYZI::Ptr cloud_to_save(new CloudXYZI());

  erasor_utils::voxelize_preserving_labels(cloud_full, *cloud_to_save, leafsize);

  cloud_to_save->width = cloud_to_save->points.size();
  cloud_to_save->height = 1;
  std::cout << "[Debug]: " << cloud_to_save->width << ", " << cloud_to_save->height << ", "
            << cloud_to_save->points.size() << std::endl;
  std::cout << "\033[1;32m Saving the map to pcd... at " << map_dir << "\033[0m" << std::endl;


  pcl::io::savePCDFileASCII(map_dir, *cloud_to_save);
  std::cout << "\033[1;32m Complete to save the map!:";
  std::cout << map_dir << "\033[0m" << std::endl;
}


void KumoAccumulateFeatureCloud(pcl::PointCloud<pcl::PointXYZI> &cloud_this,
                                pcl::PointCloud<pcl::PointXYZI> &cloud_full, Eigen::Matrix4f pose) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(cloud_this, *world_transformed, pose);
  cloud_full += *world_transformed;
}

/*
思路：
读SLAM给出的判断的每帧动态点索引文件，读相应的位姿和其全点云。 这里直接从带intensity的bin中读
由于这里使用了SLAM一样的点云前处理，就不需要index mapping，它处理后可以直接使用动态索引.
将slam认为的非动态点云保存下来作为保留地图，由于其intensity来自GT bin，起到了自带标签的效果，可以直接生成结果地图
这和ERASOR给出的点云应该是一致的，可以用来评估pr rr
*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "merger");
  ros::NodeHandle nodeHandler;
  std::cout << "kumo map gen start!" << std::endl;

  float start_timestamp, end_timestamp;
  int start_id, end_id;
  std::string corr_url;
  std::string corner_idx_dir, surf_idx_dir, corner_cloud_dir, surf_cloud_dir, res_corner_cloud_dir, res_surf_cloud_dir;
  bool also_save_ori_fpcloud;
  float corner_leafsize, surf_leafsize;

  nodeHandler.param("/map/voxelsize", voxelsize, (float)0.05);
  nodeHandler.param<std::string>("/map/slam_map_save_path", slam_map_save_path, "/mnt/d");
  nodeHandler.param<std::string>("/map/dataset_name", dataset_name, "machi");
  nodeHandler.param<std::string>("/map/cloud_bin_dir", cloud_bin_dir, "machi");
  nodeHandler.param<std::string>("/map/poses_url", pose_url, "machi");
  nodeHandler.param<std::string>("/map/corr_url", corr_url, "machi");
  nodeHandler.param<std::string>("/map/corner_idx_dir", corner_idx_dir, "machi");
  nodeHandler.param<std::string>("/map/surf_idx_dir", surf_idx_dir, "machi");
  nodeHandler.param<std::string>("/map/res_corner_cloud_dir", res_corner_cloud_dir, "machi");
  nodeHandler.param<std::string>("/map/res_surf_cloud_dir", res_surf_cloud_dir, "machi");
  nodeHandler.param<float>("/map/corner_leafsize", corner_leafsize, 0.2);
  nodeHandler.param<float>("/map/surf_leafsize", surf_leafsize, 0.4);
  nodeHandler.param<bool>("/map/also_save_ori_fpcloud", also_save_ori_fpcloud, false);

  nodeHandler.param<float>("/map/start_timestamp", start_timestamp, 999999999.9);
  nodeHandler.param<float>("/map/end_timestamp", end_timestamp, 999999999.9);

  nodeHandler.param<int>("/map/start_id", start_id, 999999999);
  nodeHandler.param<int>("/map/end_id", end_id, 999999999);

  nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

  mapgenerator.setValue(slam_map_save_path, voxelsize, dataset_name, "1", "1", interval, is_large_scale);

  slam_map_save_path = (slam_map_save_path.back() == '/') ? slam_map_save_path : (slam_map_save_path + "/");
  cloud_bin_dir = (cloud_bin_dir.back() == '/') ? cloud_bin_dir : (cloud_bin_dir + "/");

  corner_idx_dir = (corner_idx_dir.back() == '/') ? corner_idx_dir : (corner_idx_dir + "/");
  surf_idx_dir = (surf_idx_dir.back() == '/') ? surf_idx_dir : (surf_idx_dir + "/");
  res_corner_cloud_dir = (res_corner_cloud_dir.back() == '/') ? res_corner_cloud_dir : (res_corner_cloud_dir + "/");
  res_surf_cloud_dir = (res_surf_cloud_dir.back() == '/') ? res_surf_cloud_dir : (res_surf_cloud_dir + "/");

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

  // 至于GT是用original还是后面的，看结果决定
  // original
  CloudXYZI::Ptr ori_corner_full(new CloudXYZI());
  CloudXYZI::Ptr ori_surf_full(new CloudXYZI());

  // res
  CloudXYZI::Ptr res_corner_full(new CloudXYZI());
  CloudXYZI::Ptr res_surf_full(new CloudXYZI());


  ////////////// 读pose ////////////////////////
  static int num_matched = 0;
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


        // 读入全点云
        // 这里是bin，不能用pcl的函数
        pcl::PointCloud<pcl::PointXYZI> cloud_raw;
        fstream input(cloud_file_name.c_str(), ios::in | ios::binary);
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


        // 检查角面点idx、 相应的结果cloud存在
        std::string url_corner_idx_this = corner_idx_dir + timestamp + ".txt";
        std::string url_corner_rescloud_this = res_corner_cloud_dir + timestamp + ".pcd.bin";
        std::string url_surf_idx_this = surf_idx_dir + timestamp + ".txt";
        std::string url_surf_rescloud_this = res_surf_cloud_dir + timestamp + ".pcd.bin";

        if (!fs::exists(url_corner_idx_this)) {
          std::cout << url_corner_idx_this << " not exists" << std::endl;
          continue;
        }
        if (!fs::exists(url_corner_rescloud_this)) {
          std::cout << url_corner_rescloud_this << " not exists" << std::endl;
          continue;
        }
        if (!fs::exists(url_surf_idx_this)) {
          std::cout << url_surf_idx_this << " not exists" << std::endl;
          continue;
        }
        if (!fs::exists(url_surf_rescloud_this)) {
          std::cout << url_surf_rescloud_this << " not exists" << std::endl;
          continue;
        }
        std::cout << timestamp << " matched" << std::endl;


        // 读入index
        //角SLAM索引
        std::ifstream f_corner_idx_this;
        f_corner_idx_this.open(url_corner_idx_this, std::fstream::in);
        std::vector<int> corner_indices;
        int index;
        while (f_corner_idx_this >> index) {
          corner_indices.push_back(index);
        }
        f_corner_idx_this.close();

        // 面SLAM索引
        std::ifstream f_surf_idx_thid;
        f_surf_idx_thid.open(url_surf_idx_this, std::fstream::in);
        std::vector<int> surf_indices;
        while (f_surf_idx_thid >> index) {
          surf_indices.push_back(index);
        }
        f_surf_idx_thid.close();


        // 形成角面点云
        CloudXYZI::Ptr oricloud_corner_this(new CloudXYZI());
        for (const auto &idx : corner_indices) {
          oricloud_corner_this->push_back(cloud.points[idx]);
        }
        CloudXYZI::Ptr oricloud_surf_this(new CloudXYZI());
        for (const auto &idx : surf_indices) {
          oricloud_surf_this->push_back(cloud.points[idx]);
        }


        // 读取结果点云，为其寻找最近
        CloudXYZI::Ptr rescloud_corner_this(new CloudXYZI());
        CloudXYZI::Ptr rescloud_surf_this(new CloudXYZI());
        pcl::io::loadPCDFile(url_corner_rescloud_this, *rescloud_corner_this);
        pcl::io::loadPCDFile(url_surf_rescloud_this, *rescloud_surf_this);
        erasor_utils::find_nearest_labels(oricloud_corner_this, rescloud_corner_this);
        erasor_utils::find_nearest_labels(oricloud_surf_this, rescloud_surf_this);


        // 合并并下采样结果点云，得到最终的GT或SLAM结果
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

        // 这个是原始的
        if (also_save_ori_fpcloud) {
          KumoAccumulateFeatureCloud(*oricloud_corner_this, *ori_corner_full, pose_eigen4f);
          KumoAccumulateFeatureCloud(*oricloud_surf_this, *ori_surf_full, pose_eigen4f);
        }

        // 这个是地图产生的，也有了label
        KumoAccumulateFeatureCloud(*rescloud_corner_this, *res_corner_full, pose_eigen4f);
        KumoAccumulateFeatureCloud(*rescloud_surf_this, *res_surf_full, pose_eigen4f);

        // 送进去！！！！
      } else {
        std::cout << "!!!! no such file !!! " << timestamp << std::endl;
        continue;
      }
    }
    std::cout << "total matched " << num_matched << " frames" << std::endl;

    if (also_save_ori_fpcloud) {
      KumoSaveFeatureCloud(ori_corner_full, corner_leafsize, "ori_corner");
      KumoSaveFeatureCloud(ori_surf_full, surf_leafsize, "ori_surf");
    }

    KumoSaveFeatureCloud(res_corner_full, corner_leafsize, "res_corner");
    KumoSaveFeatureCloud(res_surf_full, surf_leafsize, "res_surf");
  }
  f_read_pose.close();

  return 0;
}
