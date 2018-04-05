/******************************************************************************
 * Copyright 2018 The Hesai Technology. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/




#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include "pandora/pandora.h"


using namespace apollo::drivers::hesai;

void convert_rot(const double array[], double rr[]) {
  cv::Mat rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
  cv::Mat rot;
  cv::Rodrigues(rvec, rot);
  for (int i = 0; i < 9; ++i) {
    rr[i] = rot.at<double>(i);
  }
}

void convert_rot_trans(const double array[], double rr[], double tt[]) {
  convert_rot(array, rr);
  for (int i = 0; i < 3; ++i) {
    tt[i] = array[3 + i];
  }
}

void convert_quater_to_axisd(const std::vector<double> q, double axsid[]) {
  Eigen::Quaternion<double> rotation(q[0], q[1], q[2], q[3]);
  Eigen::AngleAxis<double> aa(rotation);
  Eigen::Vector3d avec = aa.axis() * aa.angle();

  axsid[0] = avec[0];
  axsid[1] = avec[1];
  axsid[2] = avec[2];
}

bool convert_inv_extrinsics(const double extrinsics_src[],
                            double extrinsics_inv[]) {
  double rr[9];
  double tt[3];
  convert_rot_trans(extrinsics_src, rr, tt);
  extrinsics_inv[0] = -extrinsics_src[0];
  extrinsics_inv[1] = -extrinsics_src[1];
  extrinsics_inv[2] = -extrinsics_src[2];
  extrinsics_inv[3] = -rr[0] * tt[0] - rr[3] * tt[1] - rr[6] * tt[2];
  extrinsics_inv[4] = -rr[1] * tt[0] - rr[4] * tt[1] - rr[7] * tt[2];
  extrinsics_inv[5] = -rr[2] * tt[0] - rr[5] * tt[1] - rr[8] * tt[2];
}

void convert_quater_to_trans(const std::vector<double> q,
                             const std::vector<double> tt, double array[]) {
  convert_quater_to_axisd(q, array);
  array[3] = tt[0];
  array[4] = tt[1];
  array[5] = tt[2];
}

void convert_cv_vec(const double array[], cv::Mat &rvec, cv::Mat &tvec) {
  rvec = (cv::Mat_<double>(3, 1) << array[0], array[1], array[2]);
  tvec = (cv::Mat_<double>(3, 1) << array[3], array[4], array[5]);
}

void convert_camera_matrix(const cv::Mat &camera_matrix, double array[]) {
  assert(!camera_matrix.empty() && camera_matrix.rows == 3 &&
         camera_matrix.cols == 3);
  array[0] = camera_matrix.at<double>(0, 0);
  array[1] = camera_matrix.at<double>(1, 1);
  array[2] = camera_matrix.at<double>(0, 2);
  array[3] = camera_matrix.at<double>(1, 2);
}

void convert_camera_matrix(const double array[], cv::Mat &camera_matrix) {
  camera_matrix = (cv::Mat_<double>(3, 3) << array[0], 0.0, array[2], 0.0,
                   array[1], array[3], 0.0, 0.0, 1.0);
}

void convert_camera_dist(const double array[], cv::Mat &camera_dist) {
  camera_dist = (cv::Mat_<double>(5, 1) << array[0], array[1], array[2],
                 array[3], array[4]);
}

void compute_color_map(int color_mode, std::vector<cv::Scalar> &clr_vec) {
  cv::Mat color_map_gray = cv::Mat::zeros(256, 1, CV_8UC1);
  for (int i = 0; i < color_map_gray.rows; ++i) {
    color_map_gray.at<uchar>(i) = 255 - i;
  }
  if (color_mode >= 0) {
    cv::Mat color_map;
    cv::applyColorMap(color_map_gray, color_map, color_mode);
    for (int i = 0; i < color_map.total(); ++i) {
      cv::Vec3b clr = color_map.at<cv::Vec3b>(i);
      clr_vec.push_back(cv::Scalar(clr[0], clr[1], clr[2], 255));
    }
  } else {
    for (int i = 0; i < color_map_gray.total(); ++i) {
      clr_vec.push_back(cv::Scalar(i, i, i, 255));
    }
  }
}

bool project_cloud_to_image(boost::shared_ptr<PPointCloud> cloud,
                            const double intrinsic[], const double distortion[],
                            const double extrinsic[], int render_mode,
                            double min_v, double max_v,
                            const std::vector<cv::Scalar> &clr_vec,
                            int point_size, int thinkness, int point_type,
                            cv::Mat &out_image) {
  if (cloud->empty() || out_image.empty() || clr_vec.empty()) {
    return false;
  }
  point_size = point_size <= 0 ? 1 : point_size;
  int point_count = cloud->points.size();
  std::vector<cv::Point3f> pt3d_vec;
  std::vector<int> inten_vec;
  double rr[9];
  double tt[3];
  convert_rot_trans(extrinsic, rr, tt);
  double xx[3];
  double yy[3];
  float img_pt[2];
  int width = out_image.cols;
  int height = out_image.rows;
  float ratio = 0.5;
  float min_w = -ratio * width;
  float max_w = (1 + ratio) * width;
  float min_h = -ratio * height;
  float max_h = (1 + ratio) * height;
  for (int i = 0; i < point_count; ++i) {
    xx[0] = cloud->points[i].x;
    xx[1] = cloud->points[i].y;
    xx[2] = cloud->points[i].z;
    if (std::isnan(xx[0]) || std::isnan(xx[1]) || std::isnan(xx[2])) {
      continue;
    }
    yy[2] = rr[6] * xx[0] + rr[7] * xx[1] + rr[8] * xx[2] + tt[2];
    if (yy[2] < 1.0) {
      continue;
    }
    yy[2] = 1.0 / yy[2];
    yy[0] = rr[0] * xx[0] + rr[1] * xx[1] + rr[2] * xx[2] + tt[0];
    img_pt[0] = yy[0] * yy[2] * intrinsic[0] + intrinsic[2];
    if (img_pt[0] < min_w || img_pt[0] > max_w) {
      continue;
    }
    yy[1] = rr[3] * xx[0] + rr[4] * xx[1] + rr[5] * xx[2] + tt[1];
    img_pt[1] = yy[1] * yy[2] * intrinsic[1] + intrinsic[3];
    if (img_pt[1] < min_h || img_pt[1] > max_h) {
      continue;
    }
    pt3d_vec.push_back(cv::Point3f(xx[0], xx[1], xx[2]));
    inten_vec.push_back(cloud->points[i].intensity);
  }
  if (pt3d_vec.empty()) {
    return false;
  }

  cv::Mat rvec;
  cv::Mat tvec;
  convert_cv_vec(extrinsic, rvec, tvec);
  cv::Mat camera_k;
  convert_camera_matrix(intrinsic, camera_k);
  cv::Mat camera_d;
  convert_camera_dist(distortion, camera_d);
  std::vector<cv::Point2f> img_pts;
  cv::projectPoints(pt3d_vec, rvec, tvec, camera_k, camera_d, img_pts);

  int clr_count = clr_vec.size();
  double kk = clr_count * 1.0 / (max_v - min_v);
  double bb = -min_v * clr_count / (max_v - min_v);
  for (int i = 0; i < img_pts.size(); ++i) {
    int idx = 0;
    if (render_mode == 1) {
      idx = int(kk * pt3d_vec[i].z + bb);
    } else if (render_mode == 2) {
      double distance = cv::norm(pt3d_vec[i]);
      idx = int(kk * distance + bb);
    } else if (render_mode == 3) {
      idx = int(kk * inten_vec[i] + bb);
    } else {
      idx = int(kk * inten_vec[i] + bb);
    }
    // std::cout << max_v << " " << min_v << " " << kk << " " << bb<< " " <<
    // pt3d_vec[i].z << " " << idx << std::endl;
    idx = idx < 0 ? 0 : idx;
    idx = idx >= clr_count ? clr_count - 1 : idx;
    cv::Scalar clr = clr_vec[idx];
    if (point_type == 1) {
      float x = img_pts[i].x;
      float y = img_pts[i].y;
      cv::line(out_image, cv::Point2f(x + point_size, y),
               cv::Point2f(x - point_size, y), clr, thinkness);
      cv::line(out_image, cv::Point2f(x, y + point_size),
               cv::Point2f(x, y - point_size), clr, thinkness);
    } else {
      cv::circle(out_image, img_pts[i], point_size, clr, thinkness);
    }
  }
  return true;
}

using namespace apollo::drivers::hesai;

class CameraData {
 public:
  boost::shared_ptr<cv::Mat> matp;
  double timestamp;
  int picid;
  bool distortion;
};

class LidarData {
 public:
  boost::shared_ptr<PPointCloud> cld;
  double timestamp;
};

class ProjectionData {
 public:
  LidarData lidar;
  CameraData camera;
};

class Projection {
 public:
  Projection(ros::NodeHandle node, ros::NodeHandle private_nh, std::string ip,
             int render_mode = 0) {
    pandora_cameras_output[0] =
        node.advertise<sensor_msgs::Image>("pandora_projection0", 10);
    pandora_cameras_output[1] =
        node.advertise<sensor_msgs::Image>("pandora_projection1", 10);
    pandora_cameras_output[2] =
        node.advertise<sensor_msgs::Image>("pandora_projection2", 10);
    pandora_cameras_output[3] =
        node.advertise<sensor_msgs::Image>("pandora_projection3", 10);
    pandora_cameras_output[4] =
        node.advertise<sensor_msgs::Image>("pandora_projection4", 10);
    ip_ = ip;
    pthread_mutex_init(&cld_lock_, NULL);
    pthread_mutex_init(&pic_lock_, NULL);
    pthread_mutex_init(&proj_lock_, NULL);

    sem_init(&proj_sem_, 0, 0);

    render_mode_ = render_mode;  // 0, intensity; 1, height; 2, distance
    min_v = 1;
    max_v = 30;
    point_size = 1;
    clr_mode = 4;
    point_type = 0;
    thinkness = 1;
    got_calibration_ = 0;

    compute_color_map(clr_mode, clr_vec);

    // init distortion as 0.0
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 5; ++j) {
        distortion[i][j] = 0.0;
      }
    }

    proj_thr_ =
        new boost::thread(boost::bind(&Projection::ProjectionTask, this));

    pandora = new Pandora(
        ip_, 2368, 10110, boost::bind(&Projection::LidarCallBack, this, _1, _2),
        NULL, 13500, 9870,
        boost::bind(&Projection::CameraCallBack, this, _1, _2, _3, _4), 1, 0,
        std::string("hesai40"));
    pandora->Start();
  }

  void LidarCallBack(boost::shared_ptr<PPointCloud>, double);
  void CameraCallBack(boost::shared_ptr<cv::Mat> matp, double timestamp,
                      int picid, bool distortion);

 private:
  void TryToProjection();
  void ProjectionTask();

  pthread_mutex_t cld_lock_;
  pthread_mutex_t pic_lock_;
  pthread_mutex_t proj_lock_;

  sem_t proj_sem_;

  std::list<LidarData> cld_list_;
  std::list<CameraData> pic_list_;
  std::list<ProjectionData> proj_list_;

  std::string ip_;

  Pandora *pandora;

  int render_mode_;
  double min_v;
  double max_v;
  int point_size;
  int thinkness;
  int point_type;
  int clr_mode;
  std::vector<cv::Scalar> clr_vec;

  double intrinsic[5][4];
  double distortion[5][5];
  double extrinsic[5][6];

  int got_calibration_;

  ros::Publisher pandora_cameras_output[5];

  boost::thread *proj_thr_;
};

double gap_map[5][2] = {
    {0.000, 0.025}, {0.000, 0.025}, {0.025, 0.050},
    {0.050, 0.075}, {0.075, 0.100},
};

void Projection::ProjectionTask() {
  while (true) {
    sem_wait(&proj_sem_);
    ProjectionData proj_data;

    pthread_mutex_lock(&proj_lock_);
    proj_data = proj_list_.front();
    proj_list_.pop_front();
    pthread_mutex_unlock(&proj_lock_);

    cv_bridge::CvImage cv_ptr =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", *proj_data.camera.matp);
    if (!project_cloud_to_image(
            proj_data.lidar.cld, intrinsic[proj_data.camera.picid],
            distortion[proj_data.camera.picid],
            extrinsic[proj_data.camera.picid], render_mode_, min_v, max_v,
            clr_vec, point_size, thinkness, point_type, cv_ptr.image)) {
      std::cout << "error projection" << std::endl;
    }
    
    sensor_msgs::ImagePtr msg = cv_ptr.toImageMsg();
    pandora_cameras_output[proj_data.camera.picid].publish(msg);
  }
}

void Projection::TryToProjection() {
  int cld_size = 0;
  pthread_mutex_lock(&cld_lock_);
  cld_size = cld_list_.size();
  pthread_mutex_unlock(&cld_lock_);

  // empty cloud
  if (cld_size == 0) {
    pthread_mutex_lock(&pic_lock_);
    while (pic_list_.size() > 5) {
      // drop when cloud is empty;
      pic_list_.pop_front();
    }
    pthread_mutex_unlock(&pic_lock_);
    return;
  }

  pthread_mutex_lock(&cld_lock_);
  pthread_mutex_lock(&pic_lock_);

  while (pic_list_.size() > 0) {
    int found = false;
    CameraData camera_data = pic_list_.front();
    double front_gap = camera_data.timestamp - cld_list_.front().timestamp;

    // picture is too late !!!
    if (front_gap < 0) {
      printf("Camera [%d]'s data is too late\n", camera_data.picid);
      pic_list_.pop_front();
      continue;
    }

    std::list<LidarData>::iterator it;
    for (it = cld_list_.begin(); it != cld_list_.end(); ++it) {
      /* code */
      double gap = camera_data.timestamp - it->timestamp;
      if (gap > gap_map[camera_data.picid][0] &&
          gap < gap_map[camera_data.picid][1]) {
        pic_list_.pop_front();

        ProjectionData proj_data;
        proj_data.camera = camera_data;
        proj_data.lidar = *it;

        pthread_mutex_lock(&proj_lock_);
        proj_list_.push_back(proj_data);
        pthread_mutex_unlock(&proj_lock_);

        sem_post(&proj_sem_);

        found = true;
        break;
      }
    }

    if (!found) break;
  }

  pthread_mutex_unlock(&pic_lock_);
  pthread_mutex_unlock(&cld_lock_);
}

void Projection::LidarCallBack(boost::shared_ptr<PPointCloud> cld,
                               double timestamp) {
  LidarData lidar_data;
  lidar_data.cld = cld;
  lidar_data.timestamp = timestamp;

  pthread_mutex_lock(&cld_lock_);

  cld_list_.push_back(lidar_data);

  // only cache 1 frames;
  if (cld_list_.size() > 1) {
    cld_list_.pop_front();
  }

  pthread_mutex_unlock(&cld_lock_);
}

void Projection::CameraCallBack(boost::shared_ptr<cv::Mat> matp,
                                double timestamp, int picid, bool distortion) {
  if (distortion && !got_calibration_) {
    // get parameters;
    CameraCalibration calibs[5];
    pandora->GetCameraCalibration(calibs);

    for (int i = 0; i < 5; ++i) {
      /* code */
      convert_camera_matrix(calibs[i].cameraK, intrinsic[i]);

      convert_quater_to_trans(calibs[i].cameraR, calibs[i].cameraT,
                              extrinsic[i]);

      convert_inv_extrinsics(extrinsic[i], extrinsic[i]);
    }
    got_calibration_ = 1;
  }
  CameraData camera_data;
  camera_data.matp = matp;
  camera_data.timestamp = timestamp;
  camera_data.picid = picid;
  camera_data.distortion = distortion;

  pthread_mutex_lock(&pic_lock_);
  pic_list_.push_back(camera_data);
  pthread_mutex_unlock(&pic_lock_);

  TryToProjection();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pandora_projection");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;

  Projection projection(node, nh, std::string("192.168.20.51"));
  ros::Rate loop_rate(5);

  ros::spin();
}
