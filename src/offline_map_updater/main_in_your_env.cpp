//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <cassert>
#include <cstdlib>
#include <erasor/OfflineMapUpdater.h>
#include "kumo.h"
#include "kumo_process.hpp"


string DATA_DIR;
int INTERVAL, INIT_IDX;
float VOXEL_SIZE;
bool STOP_FOR_EACH_FRAME;
std::string filename = "/staticmap_via_erasor.pcd";

using PointType = pcl::PointXYZI;


vector<float> split_line(string input, char delimiter) {
    vector<float> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

void load_all_poses(string txt, vector<Eigen::Matrix4f >& poses){
    // These poses are already w.r.t body frame!
    // Thus, tf4x4 by pose * corresponding cloud -> map cloud
    cout<<"Target path: "<< txt<<endl;
    poses.clear();
    poses.reserve(2000);
    std::ifstream in(txt);
    std::string line;

    int count = 0;
    while (std::getline(in, line)) {
        if (count == 0){
            count++;
            continue;
        }

        vector<float> pose = split_line(line, ',');

        Eigen::Translation3f ts(pose[2], pose[3], pose[4]);
        Eigen::Quaternionf q(pose[8], pose[5], pose[6], pose[7]);
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity(); // Crucial!
        tf4x4_cam.topLeftCorner<3, 3>(0, 0) = q.toRotationMatrix();
        tf4x4_cam.topRightCorner(3, 1) = ts.vector();

        Eigen::Matrix4f tf4x4_lidar = tf4x4_cam;
        poses.emplace_back(tf4x4_lidar);
        count++;
    }
    std::cout<<"Total "<<count<<" poses are loaded"<<std::endl;
}


void KumoLoadPoses(vector<Eigen::Matrix4f> &poses, vector<std::string> &cloud_urls, double start_timestamp, double end_timestamp,
                   std::string cloud_bin_dir, std::string pose_url, std::string corr_url) {
  // These poses are already w.r.t body frame!
  // Thus, tf4x4 by pose * corresponding cloud -> map cloud
  cout << "pose_url path: " << pose_url << endl;
  cout << "corr_url path: " << corr_url << endl;
  cout << "cloud_bin_dir path: " << cloud_bin_dir << endl;
  poses.clear();
//   poses.reserve(2000);


  /////////////// 读corr corr[timestamp]=filename_int ///////////////////
  std::ifstream f_read_corr;
  f_read_corr.open(corr_url, std::fstream::in);
  std::map<std::string, std::string> corr_data;  // errors
  std::string t_corr, cloud_name_corr;
  while (f_read_corr >> cloud_name_corr >> t_corr) {
    corr_data[t_corr] = cloud_name_corr;  // input them into the map
  }
  f_read_corr.close();


  // 读pose
  std::ifstream f_read_pose;
  int count = 0;
  f_read_pose.open(pose_url, std::fstream::in);
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
        } else {
          pose.push_back(atof(p));
        }
        p = strtok(NULL, " ");
      }
      ///// 读完了,看是否满足要求
      if (atof(timestamp.c_str()) < start_timestamp || atof(timestamp.c_str()) > end_timestamp) {
        continue;
      }

      // 读corr,看是否存在
      if (corr_data.count(timestamp)) {
        std::ostringstream tmp;
        tmp << std::setw(6) << std::setfill('0') << corr_data[timestamp];
        std::string cloud_file_name = cloud_bin_dir + tmp.str() + ".bin";
        // 读完pose timestamp, 找相应的pointcloud bin
        // std::cout << cloud_file_name << std::endl;
        // TODO check if cloud_file_name exists

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

        poses.emplace_back(pose_eigen4f);
        cloud_urls.emplace_back(cloud_file_name);
        count++;

      } else {
        std::cout << "!!!! no such file !!! " << timestamp << std::endl;
        continue;
      }
    }
  }

  std::cout << "Total " << count << " poses are loaded" << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "erasor_in_your_env");
    ros::NodeHandle nh;
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();

    // nh.param<string>("/data_dir", DATA_DIR, "/");
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
    nh.param<int>("/init_idx", INIT_IDX, 0);
    nh.param<int>("/interval", INTERVAL, 2);
    nh.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false);

    // Set ROS visualization publishers
    ros::Publisher NodePublisher = nh.advertise<erasor::node>("/node/combined/optimized", 100);
    ros::Rate loop_rate(10);


    // mine
    std::string pose_url;
    std::string corr_url;
    std::string cloud_bin_dir;

    double start_timestamp;
    double end_timestamp;

    nh.param<double>("/kumo/start_timestamp", start_timestamp, 99999999999.9);
    nh.param<double>("/kumo/end_timestamp", end_timestamp, 99999999999.9);
    nh.param<std::string>("/kumo/pose_url", pose_url, "/mnt/d");
    nh.param<std::string>("/kumo/corr_url", corr_url, "/mnt/d");
    nh.param<std::string>("/kumo/cloud_bin_dir", cloud_bin_dir, "/mnt/d");

    cloud_bin_dir = (cloud_bin_dir.back()=='/') ? cloud_bin_dir : cloud_bin_dir + "/";

    // cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
    // string raw_map_path = DATA_DIR + "/dense_global_map.pcd";
    // string pose_path = DATA_DIR + "/poses_lidar2body.csv";
    // string pcd_dir = DATA_DIR + "/pcds"; //
    // Load raw pointcloud

    vector<Eigen::Matrix4f> poses;
    vector<std::string> cloud_urls;
    KumoLoadPoses(poses, cloud_urls, start_timestamp, end_timestamp, cloud_bin_dir, pose_url, corr_url);
    // load_all_poses(pose_path, poses);


    int N = poses.size();
    assert(N==cloud_urls.size());

    for (int i = INIT_IDX; i < N; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        // 读对应的点云
        std::string cloud_file_name = cloud_urls[i];
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
        
        if (cloud_raw.size() ==0) {
          cout << "empty cloud!!!" << endl;
          return -1;
        }

        // 处理和SLAM一致
        CloudXYZI::Ptr cloud(new CloudXYZI());
        std::vector<int> _;
        pcl::removeNaNFromPointCloud(cloud_raw, cloud_raw, _);
        SaveGRLOAMSpecificFilterIndexMapping(cloud_raw, *cloud, 0.5, 80);


        erasor::node node;
        node.header.seq = i;
        node.odom = erasor_utils::eigen2geoPose(poses[i]);
        node.lidar = erasor_utils::cloud2msg(*cloud);
        NodePublisher.publish(node);
        ros::spinOnce();
        loop_rate.sleep();

        if (STOP_FOR_EACH_FRAME) {
            cout<< "[Debug]: STOP! Press any button to continue" <<endl;
            cin.ignore();
        }
    }

    updater.save_static_map(0.2);

    cout<< "Static map building complete!" << endl;

    return 0;
}