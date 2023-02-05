/*** 
 * @Date: 2023-02-04 10:50:00
 * @LastEditors: yxt
 * @LastEditTime: 2023-02-05 08:15:58
 * @Description: 保证点云前处理与SLAM相同
 */

#pragma once
#include <experimental/filesystem>  // requires gcc version >= 8
#include "kumo.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"


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
    point.intensity = cloud_filtered_dist->points[i].intensity;
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