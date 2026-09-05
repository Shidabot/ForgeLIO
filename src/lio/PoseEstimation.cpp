#include "Estimator/Estimator.h"
#include "utils/numerical_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
typedef pcl::PointXYZINormal PointType;

int WINDOWSIZE;
bool LidarIMUInited = false;
boost::shared_ptr<std::list<Estimator::LidarFrame>> lidarFrameList;
pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
Estimator* estimator;

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserOdometryPath;
ros::Publisher pubFullLaserCloud;
tf::StampedTransform laserOdometryTrans;
tf::TransformBroadcaster* tfBroadcaster;
ros::Publisher pubGps;

bool newfullCloud = false;

Eigen::Matrix4d transformAftMapped = Eigen::Matrix4d::Identity();

std::mutex _mutexLidarQueue;
std::queue<sensor_msgs::PointCloud2ConstPtr> _lidarMsgQueue;
std::mutex _mutexIMUQueue;
std::condition_variable _cvIMUQueue;
std::deque<sensor_msgs::ImuConstPtr> _imuMsgQueue;
Eigen::Matrix4d exTlb;
Eigen::Matrix3d exRlb, exRbl;
Eigen::Vector3d exPlb, exPbl;
Eigen::Vector3d GravityVector;
float filter_parameter_corner = 0.2;
float filter_parameter_surf = 0.4;
int IMU_Mode = 2;
std::string imu_topic = "/livox/imu";
// Corrected LiDAR-clock timestamp = raw IMU timestamp - imu_time_offset.
// A positive value therefore means that the IMU clock is ahead of LiDAR.
double imu_time_offset = 0.0;
int imu_wait_timeout_ms = 1000;
double max_imu_gap = 0.1;
double last_imu_arrival_time = -1.0;
sensor_msgs::NavSatFix gps;
int pushCount = 0;
double startTime = 0;

nav_msgs::Path laserOdoPath;

/** \brief publish odometry infomation
  * \param[in] newPose: pose to be published
  * \param[in] timefullCloud: time stamp
  */
void pubOdometry(const Eigen::Matrix4d& newPose, double& timefullCloud){
  nav_msgs::Odometry laserOdometry;

  Eigen::Matrix3d Rcurr = newPose.topLeftCorner(3, 3);
  Eigen::Quaterniond newQuat(Rcurr);
  Eigen::Vector3d newPosition = newPose.topRightCorner(3, 1);
  laserOdometry.header.frame_id = "/world";
  laserOdometry.child_frame_id = "/livox_frame";
  laserOdometry.header.stamp = ros::Time().fromSec(timefullCloud);
  laserOdometry.pose.pose.orientation.x = newQuat.x();
  laserOdometry.pose.pose.orientation.y = newQuat.y();
  laserOdometry.pose.pose.orientation.z = newQuat.z();
  laserOdometry.pose.pose.orientation.w = newQuat.w();
  laserOdometry.pose.pose.position.x = newPosition.x();
  laserOdometry.pose.pose.position.y = newPosition.y();
  laserOdometry.pose.pose.position.z = newPosition.z();
  pubLaserOdometry.publish(laserOdometry);

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  laserOdoPath.header.stamp = laserOdometry.header.stamp;
  laserOdoPath.poses.push_back(laserPose);
  laserOdoPath.header.frame_id = "/world";
  pubLaserOdometryPath.publish(laserOdoPath);

  laserOdometryTrans.frame_id_ = "/world";
  laserOdometryTrans.child_frame_id_ = "/livox_frame";
  laserOdometryTrans.stamp_ = ros::Time().fromSec(timefullCloud);
  laserOdometryTrans.setRotation(tf::Quaternion(newQuat.x(), newQuat.y(), newQuat.z(), newQuat.w()));
  laserOdometryTrans.setOrigin(tf::Vector3(newPosition.x(), newPosition.y(), newPosition.z()));
  tfBroadcaster->sendTransform(laserOdometryTrans);

	gps.header.stamp = ros::Time().fromSec(timefullCloud);
	gps.header.frame_id = "world";
	gps.latitude = newPosition.x();
	gps.longitude = newPosition.y();
	gps.altitude = newPosition.z();
	gps.position_covariance = {
					Rcurr(0, 0), Rcurr(1, 0), Rcurr(2, 0),
					Rcurr(0, 1), Rcurr(1, 1), Rcurr(2, 1),
					Rcurr(0, 2), Rcurr(1, 2), Rcurr(2, 2)
	};
	pubGps.publish(gps);

}

void fullCallBack(const sensor_msgs::PointCloud2ConstPtr &msg){
  // push lidar msg to queue
	std::unique_lock<std::mutex> lock(_mutexLidarQueue);
  _lidarMsgQueue.push(msg);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg){
  if (!imu_msg) {
    return;
  }

  const double raw_time = imu_msg->header.stamp.toSec();
  const double corrected_time = raw_time - imu_time_offset;
  const bool finite_measurement =
      std::isfinite(corrected_time) && corrected_time >= 0.0 &&
      std::isfinite(imu_msg->angular_velocity.x) &&
      std::isfinite(imu_msg->angular_velocity.y) &&
      std::isfinite(imu_msg->angular_velocity.z) &&
      std::isfinite(imu_msg->linear_acceleration.x) &&
      std::isfinite(imu_msg->linear_acceleration.y) &&
      std::isfinite(imu_msg->linear_acceleration.z);
  if (!finite_measurement) {
    ROS_WARN_THROTTLE(1.0, "Discarding an IMU message with a non-finite timestamp or measurement.");
    return;
  }

  sensor_msgs::ImuPtr corrected_msg(new sensor_msgs::Imu(*imu_msg));
  corrected_msg->header.stamp.fromSec(corrected_time);

  {
    std::lock_guard<std::mutex> lock(_mutexIMUQueue);
    if (last_imu_arrival_time > 0.0 &&
        corrected_time < last_imu_arrival_time - 1.0) {
      ROS_WARN("Detected an IMU timestamp reset; clearing the synchronization buffer.");
      _imuMsgQueue.clear();
      last_imu_arrival_time = corrected_time;
    } else {
      last_imu_arrival_time = std::max(last_imu_arrival_time,
                                       corrected_time);
    }
    // ROS normally delivers this topic in order.  In case multiple transport
    // threads reorder a small number of packets, insert by corrected timestamp
    // so preintegration never receives a negative dt.
    const auto insertion_point = std::lower_bound(
        _imuMsgQueue.begin(), _imuMsgQueue.end(), corrected_time,
        [](const sensor_msgs::ImuConstPtr& sample, double timestamp) {
          return sample->header.stamp.toSec() < timestamp;
        });
    const double duplicate_epsilon = 1e-9;
    if (insertion_point != _imuMsgQueue.end() &&
        std::fabs((*insertion_point)->header.stamp.toSec() - corrected_time) <=
            duplicate_epsilon) {
      ROS_WARN_THROTTLE(1.0, "Discarding a duplicate IMU timestamp.");
      return;
    }
    _imuMsgQueue.insert(insertion_point, corrected_msg);
  }
  _cvIMUQueue.notify_all();
}

namespace {

const double kImuTimestampEpsilon = 1e-9;

sensor_msgs::ImuConstPtr InterpolateImuSample(
    const sensor_msgs::ImuConstPtr& before,
    const sensor_msgs::ImuConstPtr& after,
    double timestamp) {
  if (!before || !after) {
    return sensor_msgs::ImuConstPtr();
  }

  const double time_before = before->header.stamp.toSec();
  const double time_after = after->header.stamp.toSec();
  const double span = time_after - time_before;
  if (!std::isfinite(span) || span <= kImuTimestampEpsilon ||
      span > max_imu_gap ||
      timestamp < time_before - kImuTimestampEpsilon ||
      timestamp > time_after + kImuTimestampEpsilon) {
    return sensor_msgs::ImuConstPtr();
  }

  const double alpha = std::max(0.0, std::min(1.0,
      (timestamp - time_before) / span));
  const double beta = 1.0 - alpha;
  sensor_msgs::ImuPtr sample(new sensor_msgs::Imu(*before));
  sample->header.stamp.fromSec(timestamp);
  sample->angular_velocity.x =
      beta * before->angular_velocity.x + alpha * after->angular_velocity.x;
  sample->angular_velocity.y =
      beta * before->angular_velocity.y + alpha * after->angular_velocity.y;
  sample->angular_velocity.z =
      beta * before->angular_velocity.z + alpha * after->angular_velocity.z;
  sample->linear_acceleration.x =
      beta * before->linear_acceleration.x + alpha * after->linear_acceleration.x;
  sample->linear_acceleration.y =
      beta * before->linear_acceleration.y + alpha * after->linear_acceleration.y;
  sample->linear_acceleration.z =
      beta * before->linear_acceleration.z + alpha * after->linear_acceleration.z;
  return sample;
}

sensor_msgs::ImuConstPtr SampleImuAtTimeLocked(double timestamp) {
  const auto after = std::lower_bound(
      _imuMsgQueue.begin(), _imuMsgQueue.end(), timestamp,
      [](const sensor_msgs::ImuConstPtr& sample, double requested_time) {
        return sample->header.stamp.toSec() < requested_time;
      });

  if (after != _imuMsgQueue.end() &&
      std::fabs((*after)->header.stamp.toSec() - timestamp) <=
          kImuTimestampEpsilon) {
    return *after;
  }
  if (after == _imuMsgQueue.begin() || after == _imuMsgQueue.end()) {
    return sensor_msgs::ImuConstPtr();
  }
  return InterpolateImuSample(*std::prev(after), *after, timestamp);
}

bool ImuBufferCoversIntervalLocked(double start_time, double end_time) {
  if (_imuMsgQueue.size() < 2) {
    return false;
  }
  return _imuMsgQueue.front()->header.stamp.toSec() <=
             start_time + kImuTimestampEpsilon &&
         _imuMsgQueue.back()->header.stamp.toSec() >=
             end_time - kImuTimestampEpsilon;
}

}  // namespace

/** \brief get IMU messages in a certain time interval
  * \param[in] startTime: left boundary of time interval
  * \param[in] endTime: right boundary of time interval
  * \param[in] vimuMsg: store IMU messages
  */
bool fetchImuMsgs(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg){
  std::unique_lock<std::mutex> lock(_mutexIMUQueue);
  vimuMsg.clear();
  if (!std::isfinite(startTime) || !std::isfinite(endTime) ||
      endTime <= startTime + kImuTimestampEpsilon) {
    ROS_WARN_THROTTLE(1.0, "Invalid LiDAR interval supplied to the IMU synchronizer.");
    return false;
  }

  const int wait_ms = std::max(0, imu_wait_timeout_ms);
  if (!_cvIMUQueue.wait_for(
          lock, std::chrono::milliseconds(wait_ms),
          [startTime, endTime] {
            return ImuBufferCoversIntervalLocked(startTime, endTime);
          })) {
    return false;
  }

  // Keep exactly one real sample at or before startTime.  It is required to
  // interpolate the beginning of this interval and the following interval.
  while (_imuMsgQueue.size() >= 2 &&
         _imuMsgQueue[1]->header.stamp.toSec() <=
             startTime + kImuTimestampEpsilon) {
    _imuMsgQueue.pop_front();
  }

  const sensor_msgs::ImuConstPtr start_sample =
      SampleImuAtTimeLocked(startTime);
  const sensor_msgs::ImuConstPtr end_sample = SampleImuAtTimeLocked(endTime);
  if (!start_sample || !end_sample) {
    ROS_WARN_THROTTLE(1.0, "Unable to interpolate both IMU interval boundaries.");
    return false;
  }

  // The explicit start sample has dt=0 for the current preintegrator, but it
  // makes the interval self-contained and permits midpoint integration without
  // changing the synchronization contract.
  vimuMsg.push_back(start_sample);
  for (const auto& sample : _imuMsgQueue) {
    const double timestamp = sample->header.stamp.toSec();
    if (timestamp > startTime + kImuTimestampEpsilon &&
        timestamp < endTime - kImuTimestampEpsilon) {
      vimuMsg.push_back(sample);
    }
  }
  vimuMsg.push_back(end_sample);

  for (std::size_t i = 1; i < vimuMsg.size(); ++i) {
    const double dt = vimuMsg[i]->header.stamp.toSec() -
                      vimuMsg[i - 1]->header.stamp.toSec();
    if (!std::isfinite(dt) || dt <= kImuTimestampEpsilon ||
        dt > max_imu_gap) {
      ROS_WARN_THROTTLE(1.0,
                        "Rejecting a LiDAR frame because its IMU interval contains a timestamp gap.");
      vimuMsg.clear();
      return false;
    }
  }

  // Preserve the last real sample at or before endTime so it can bracket the
  // next interval's start.  Never consume the first sample after endTime.
  while (_imuMsgQueue.size() >= 2 &&
         _imuMsgQueue[1]->header.stamp.toSec() <=
             endTime + kImuTimestampEpsilon) {
    _imuMsgQueue.pop_front();
  }
  return vimuMsg.size() >= 2;
}

/** \brief Remove Lidar Distortion
  * \param[in] cloud: lidar cloud need to be undistorted
  * \param[in] dRlc: delta rotation
  * \param[in] dtlc: delta displacement
  */
void RemoveLidarDistortion(pcl::PointCloud<PointType>::Ptr& cloud,
                           const Eigen::Matrix3d& dRlc, const Eigen::Vector3d& dtlc){
  int PointsNum = cloud->points.size();
  for (int i = 0; i < PointsNum; i++) {
    Eigen::Vector3d startP;
    float s = cloud->points[i].normal_x;
    Eigen::Quaterniond qlc = Eigen::Quaterniond(dRlc).normalized();
    Eigen::Quaterniond delta_qlc = Eigen::Quaterniond::Identity().slerp(s, qlc).normalized();
    const Eigen::Vector3d delta_Plc = s * dtlc;
    startP = delta_qlc * Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z) + delta_Plc;
    Eigen::Vector3d _po = dRlc.transpose() * (startP - dtlc);

    cloud->points[i].x = _po(0);
    cloud->points[i].y = _po(1);
    cloud->points[i].z = _po(2);
    cloud->points[i].normal_x = 1.0;
  }
}


bool TryMAPInitialization() {

  Eigen::Vector3d average_acc = -lidarFrameList->begin()->imuIntegrator.GetAverageAcc();
  double info_g = std::fabs(9.805 - average_acc.norm());
  average_acc = average_acc * 9.805 / average_acc.norm();

  // calculate the initial gravity direction
  double para_quat[4];
  para_quat[0] = 1;
  para_quat[1] = 0;
  para_quat[2] = 0;
  para_quat[3] = 0;


  ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization();
  ceres::Problem problem_quat;
  
  problem_quat.AddParameterBlock(para_quat, 4, quatParam);

  problem_quat.AddResidualBlock(Cost_Initial_G::Create(average_acc),
                                nullptr,
                                para_quat);

  ceres::Solver::Options options_quat;
  ceres::Solver::Summary summary_quat;
  ceres::Solve(options_quat, &problem_quat, &summary_quat);

  Eigen::Quaterniond q_wg(para_quat[0], para_quat[1], para_quat[2], para_quat[3]);


  //build prior factor of LIO initialization
  Eigen::Vector3d prior_r = Eigen::Vector3d::Zero();
  Eigen::Vector3d prior_ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d prior_bg = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> prior_v;
  int v_size = lidarFrameList->size();
  for(int i = 0; i < v_size; i++) {
    prior_v.push_back(Eigen::Vector3d::Zero());
  }
  Sophus::SO3d SO3_R_wg(q_wg.toRotationMatrix());
  prior_r = SO3_R_wg.log();
  
  for (int i = 1; i < v_size; i++){
    auto iter = lidarFrameList->begin();
    auto iter_next = lidarFrameList->begin();
    std::advance(iter, i-1);
    std::advance(iter_next, i);

    Eigen::Vector3d velo_imu = (iter_next->P - iter->P + iter_next->Q*exPlb - iter->Q*exPlb) / (iter_next->timeStamp - iter->timeStamp);
    prior_v[i] = velo_imu;
  }
  prior_v[0] = prior_v[1];

  double para_v[v_size][3];
  double para_r[3];
  double para_ba[3];
  double para_bg[3];

  for(int i = 0; i < 3; i++) {
    para_r[i] = 0;
    para_ba[i] = 0;
    para_bg[i] = 0;
  }

  for(int i = 0; i < v_size; i++) {
    for(int j = 0; j < 3; j++) {
      para_v[i][j] = prior_v[i][j];
    }
  }

  Eigen::Matrix<double, 3, 3> sqrt_information_r = 2000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_ba = 1000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_bg = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_v = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  problem.AddParameterBlock(para_r, 3);
  problem.AddParameterBlock(para_ba, 3);
  problem.AddParameterBlock(para_bg, 3);
  for(int i = 0; i < v_size; i++) {
    problem.AddParameterBlock(para_v[i], 3);
  }
  
  // add CostFunction
  problem.AddResidualBlock(Cost_Initialization_Prior_R::Create(prior_r, sqrt_information_r),
                           nullptr,
                           para_r);
  
  problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_ba, sqrt_information_ba),
                           nullptr,
                           para_ba);
  problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_bg, sqrt_information_bg),
                           nullptr,
                           para_bg);

  for(int i = 0; i < v_size; i++) {
    problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_v[i], sqrt_information_v),
                             nullptr,
                             para_v[i]);
  }

  for(int i = 1; i < v_size; i++) {
    auto iter = lidarFrameList->begin();
    auto iter_next = lidarFrameList->begin();
    std::advance(iter, i-1);
    std::advance(iter_next, i);

    Eigen::Vector3d pi = iter->P + iter->Q*exPlb;
    Sophus::SO3d SO3_Ri(iter->Q*exRlb);
    Eigen::Vector3d ri = SO3_Ri.log();
    Eigen::Vector3d pj = iter_next->P + iter_next->Q*exPlb;
    Sophus::SO3d SO3_Rj(iter_next->Q*exRlb);
    Eigen::Vector3d rj = SO3_Rj.log();

    const Eigen::Matrix<double, 9, 9> imu_covariance =
        iter_next->imuIntegrator.GetCovariance().block<9,9>(0,0);
    Eigen::Matrix<double, 9, 9> sqrt_information_imu;
    if (!lio_livox::numerics::ComputeRegularizedSqrtInformation<9>(
            imu_covariance, &sqrt_information_imu)) {
      ROS_WARN_THROTTLE(1.0,
                        "Invalid IMU covariance during initialization; retrying initialization.");
      return false;
    }

    problem.AddResidualBlock(Cost_Initialization_IMU::Create(iter_next->imuIntegrator,
                                                                   ri,
                                                                   rj,
                                                                   pj-pi,
                                                                   sqrt_information_imu),
                             nullptr,
                             para_r,
                             para_v[i-1],
                             para_v[i],
                             para_ba,
                             para_bg);
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 6;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  Eigen::Vector3d r_wg(para_r[0], para_r[1], para_r[2]);
  GravityVector = Sophus::SO3d::exp(r_wg) * Eigen::Vector3d(0, 0, -9.805);

  Eigen::Vector3d ba_vec(para_ba[0], para_ba[1], para_ba[2]);
  Eigen::Vector3d bg_vec(para_bg[0], para_bg[1], para_bg[2]);

  if(ba_vec.norm() > 0.5 || bg_vec.norm() > 0.5) {
    ROS_WARN("Too Large Biases! Initialization Failed!");
    return false;
  }

  for(int i = 0; i < v_size; i++) {
    auto iter = lidarFrameList->begin();
    std::advance(iter, i);
    iter->ba = ba_vec;
    iter->bg = bg_vec;
    Eigen::Vector3d bv_vec(para_v[i][0], para_v[i][1], para_v[i][2]);
    if((bv_vec - prior_v[i]).norm() > 2.0) {
      ROS_WARN("Too Large Velocity! Initialization Failed!");
      std::cout<<"delta v norm: "<<(bv_vec - prior_v[i]).norm()<<std::endl;
      return false;
    }
    iter->V = bv_vec;
  }

  for(size_t i = 0; i < v_size - 1; i++){
    auto laser_trans_i = lidarFrameList->begin();
    auto laser_trans_j = lidarFrameList->begin();
    std::advance(laser_trans_i, i);
    std::advance(laser_trans_j, i+1);
    laser_trans_j->imuIntegrator.PreIntegration(laser_trans_i->timeStamp, laser_trans_i->bg, laser_trans_i->ba);
  }


  // //if IMU success initialized
  WINDOWSIZE = Estimator::SLIDEWINDOWSIZE;
  while(lidarFrameList->size() > WINDOWSIZE){
	  lidarFrameList->pop_front();
  }
	Eigen::Vector3d Pwl = lidarFrameList->back().P;
	Eigen::Quaterniond Qwl = lidarFrameList->back().Q;
	lidarFrameList->back().P = Pwl + Qwl*exPlb;
	lidarFrameList->back().Q = Qwl * exRlb;

	// std::cout << "\n=============================\n| Initialization Successful |"<<"\n=============================\n" << std::endl;
  
  return true;
}


/** \brief Mapping main thread
  */
void process(){
  double time_last_lidar = -1;
  double time_curr_lidar = -1;
  Eigen::Matrix3d delta_Rl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_tl = Eigen::Vector3d::Zero();
	Eigen::Matrix3d delta_Rb = Eigen::Matrix3d::Identity();
	Eigen::Vector3d delta_tb = Eigen::Vector3d::Zero();
  std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
  while(ros::ok()){
    newfullCloud = false;
    laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
	  std::unique_lock<std::mutex> lock_lidar(_mutexLidarQueue);
    if(!_lidarMsgQueue.empty()){
      // get new lidar msg
      time_curr_lidar = _lidarMsgQueue.front()->header.stamp.toSec();
      pcl::fromROSMsg(*_lidarMsgQueue.front(), *laserCloudFullRes);
      _lidarMsgQueue.pop();
      newfullCloud = true;
    }
    lock_lidar.unlock();

    if(newfullCloud){

      nav_msgs::Odometry debugInfo;
      debugInfo.pose.pose.position.x = 0;
      debugInfo.pose.pose.position.y = 0;
      debugInfo.pose.pose.position.z = 0;
      if(IMU_Mode > 0 && time_last_lidar > 0){
        // get IMU msg int the Specified time interval
        vimuMsg.clear();
        if (!fetchImuMsgs(time_last_lidar, time_curr_lidar, vimuMsg)) {
          ROS_WARN_THROTTLE(1.0,
                            "Timed out waiting for an IMU interval that brackets the LiDAR frame.");
        }
      }
      // this lidar frame init
      Estimator::LidarFrame lidarFrame;
      lidarFrame.laserCloud = laserCloudFullRes;
      lidarFrame.timeStamp = time_curr_lidar;

	    boost::shared_ptr<std::list<Estimator::LidarFrame>> lidar_list;
	    if(!vimuMsg.empty()){
	    	if(!LidarIMUInited) {
	    		// if get IMU msg successfully, use gyro integration to update delta_Rl
			    lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
			    lidarFrame.imuIntegrator.GyroIntegration(time_last_lidar);
			    delta_Rb = lidarFrame.imuIntegrator.GetDeltaQ().toRotationMatrix();
			    delta_Rl = exTlb.topLeftCorner(3, 3) * delta_Rb * exTlb.topLeftCorner(3, 3).transpose();

			    // predict current lidar pose
			    lidarFrame.P = transformAftMapped.topLeftCorner(3,3) * delta_tb
			                   + transformAftMapped.topRightCorner(3,1);
			    Eigen::Matrix3d m3d = transformAftMapped.topLeftCorner(3,3) * delta_Rb;
			    lidarFrame.Q = m3d;

			    lidar_list.reset(new std::list<Estimator::LidarFrame>);
			    lidar_list->push_back(lidarFrame);
		    }else{
			    // if get IMU msg successfully, use pre-integration to update delta lidar pose
			    lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
			    lidarFrame.imuIntegrator.PreIntegration(lidarFrameList->back().timeStamp, lidarFrameList->back().bg, lidarFrameList->back().ba);

			    const Eigen::Vector3d& Pwbpre = lidarFrameList->back().P;
			    const Eigen::Quaterniond& Qwbpre = lidarFrameList->back().Q;
			    const Eigen::Vector3d& Vwbpre = lidarFrameList->back().V;

			    const Eigen::Quaterniond& dQ =  lidarFrame.imuIntegrator.GetDeltaQ();
			    const Eigen::Vector3d& dP = lidarFrame.imuIntegrator.GetDeltaP();
			    const Eigen::Vector3d& dV = lidarFrame.imuIntegrator.GetDeltaV();
			    double dt = lidarFrame.imuIntegrator.GetDeltaTime();

			    lidarFrame.Q = Qwbpre * dQ;
			    lidarFrame.P = Pwbpre + Vwbpre*dt + 0.5*GravityVector*dt*dt + Qwbpre*(dP);
			    lidarFrame.V = Vwbpre + GravityVector*dt + Qwbpre*(dV);
			    lidarFrame.bg = lidarFrameList->back().bg;
			    lidarFrame.ba = lidarFrameList->back().ba;

			    Eigen::Quaterniond Qwlpre = Qwbpre * Eigen::Quaterniond(exRbl);
			    Eigen::Vector3d Pwlpre = Qwbpre * exPbl + Pwbpre;

			    Eigen::Quaterniond Qwl = lidarFrame.Q * Eigen::Quaterniond(exRbl);
			    Eigen::Vector3d Pwl = lidarFrame.Q * exPbl + lidarFrame.P;

			    delta_Rl = Qwlpre.conjugate() * Qwl;
			    delta_tl = Qwlpre.conjugate() * (Pwl - Pwlpre);
			    delta_Rb = dQ.toRotationMatrix();
			    delta_tb = dP;

			    lidarFrameList->push_back(lidarFrame);
			    lidarFrameList->pop_front();
			    lidar_list = lidarFrameList;
	    	}
	    }else{
	    	if(LidarIMUInited){
	    	  ROS_WARN_THROTTLE(1.0, "IMU data is unavailable for the current LiDAR frame; dropping this frame and continuing.");
	    	  time_last_lidar = time_curr_lidar;
	    	  continue;
	    	}
	    	else{
			    // predict current lidar pose
			    lidarFrame.P = transformAftMapped.topLeftCorner(3,3) * delta_tb
			                   + transformAftMapped.topRightCorner(3,1);
			    Eigen::Matrix3d m3d = transformAftMapped.topLeftCorner(3,3) * delta_Rb;
			    lidarFrame.Q = m3d;

			    lidar_list.reset(new std::list<Estimator::LidarFrame>);
			    lidar_list->push_back(lidarFrame);
	    	}
	    }

	    // remove lidar distortion
	    RemoveLidarDistortion(laserCloudFullRes, delta_Rl, delta_tl);

      // optimize current lidar pose with IMU
      estimator->EstimateLidarPose(*lidar_list, exTlb, GravityVector, debugInfo);

      pcl::PointCloud<PointType>::Ptr laserCloudCornerMap(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr laserCloudSurfMap(new pcl::PointCloud<PointType>());

	    Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
	    transformTobeMapped.topLeftCorner(3,3) = lidar_list->front().Q * exRbl;
	    transformTobeMapped.topRightCorner(3,1) = lidar_list->front().Q * exPbl + lidar_list->front().P;

	    // update delta transformation
	    delta_Rb = transformAftMapped.topLeftCorner(3, 3).transpose() * lidar_list->front().Q.toRotationMatrix();
	    delta_tb = transformAftMapped.topLeftCorner(3, 3).transpose() * (lidar_list->front().P - transformAftMapped.topRightCorner(3, 1));
	    Eigen::Matrix3d Rwlpre = transformAftMapped.topLeftCorner(3, 3) * exRbl;
	    Eigen::Vector3d Pwlpre = transformAftMapped.topLeftCorner(3, 3) * exPbl + transformAftMapped.topRightCorner(3, 1);
	    delta_Rl = Rwlpre.transpose() * transformTobeMapped.topLeftCorner(3,3);
	    delta_tl = Rwlpre.transpose() * (transformTobeMapped.topRightCorner(3,1) - Pwlpre);
	    transformAftMapped.topLeftCorner(3,3) = lidar_list->front().Q.toRotationMatrix();
	    transformAftMapped.topRightCorner(3,1) = lidar_list->front().P;

	    // publish odometry rostopic
	    pubOdometry(transformTobeMapped, lidar_list->front().timeStamp);

      // publish lidar points
      int laserCloudFullResNum = lidar_list->front().laserCloud->points.size();
      pcl::PointCloud<PointType>::Ptr laserCloudAfterEstimate(new pcl::PointCloud<PointType>());
      laserCloudAfterEstimate->reserve(laserCloudFullResNum);
      for (int i = 0; i < laserCloudFullResNum; i++) {
        PointType temp_point;
        MAP_MANAGER::pointAssociateToMap(&lidar_list->front().laserCloud->points[i], &temp_point, transformTobeMapped);
        laserCloudAfterEstimate->push_back(temp_point);
      }
      sensor_msgs::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(*laserCloudAfterEstimate, laserCloudMsg);
      laserCloudMsg.header.frame_id = "/world";
      laserCloudMsg.header.stamp.fromSec(lidar_list->front().timeStamp);
      pubFullLaserCloud.publish(laserCloudMsg);

	    // if tightly coupled IMU message, start IMU initialization
	    if(IMU_Mode > 1 && !LidarIMUInited){
		    // update lidar frame pose
		    lidarFrame.P = transformTobeMapped.topRightCorner(3,1);
		    Eigen::Matrix3d m3d = transformTobeMapped.topLeftCorner(3,3);
		    lidarFrame.Q = m3d;

		    // static int pushCount = 0;
		    if(pushCount == 0){
			    lidarFrameList->push_back(lidarFrame);
			    lidarFrameList->back().imuIntegrator.Reset();
			    if(lidarFrameList->size() > WINDOWSIZE)
				    lidarFrameList->pop_front();
		    }else{
			    lidarFrameList->back().laserCloud = lidarFrame.laserCloud;
			    lidarFrameList->back().imuIntegrator.PushIMUMsg(vimuMsg);
			    lidarFrameList->back().timeStamp = lidarFrame.timeStamp;
			    lidarFrameList->back().P = lidarFrame.P;
			    lidarFrameList->back().Q = lidarFrame.Q;
		    }
		    pushCount++;
		    if (pushCount >= 3){
			    pushCount = 0;
			    if(lidarFrameList->size() > 1){
				    auto iterRight = std::prev(lidarFrameList->end());
				    auto iterLeft = std::prev(std::prev(lidarFrameList->end()));
				    iterRight->imuIntegrator.PreIntegration(iterLeft->timeStamp, iterLeft->bg, iterLeft->ba);
			    }

          if (lidarFrameList->size() == int(WINDOWSIZE / 1.5)) {
					  startTime = lidarFrameList->back().timeStamp;
			    }

			    if (!LidarIMUInited && lidarFrameList->size() == WINDOWSIZE && lidarFrameList->front().timeStamp >= startTime){
            std::cout<<"**************Start MAP Initialization!!!******************"<<std::endl;
				    if(TryMAPInitialization()){
              LidarIMUInited = true;
					    pushCount = 0;
              startTime = 0;
				    }
            std::cout<<"**************Finish MAP Initialization!!!******************"<<std::endl;
			    }

		    }
	    }
      time_last_lidar = time_curr_lidar;

    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PoseEstimation");
  ros::NodeHandle nodeHandler("~");

  ros::param::get("~filter_parameter_corner",filter_parameter_corner);
  ros::param::get("~filter_parameter_surf",filter_parameter_surf);
	ros::param::param<int>("~IMU_Mode", IMU_Mode, 2);
	ros::param::param<std::string>("~imu_topic", imu_topic, "/livox/imu");
	ros::param::param<double>("~imu_time_offset", imu_time_offset, 0.0);
	ros::param::param<int>("~imu_wait_timeout_ms", imu_wait_timeout_ms, 1000);
	ros::param::param<double>("~max_imu_gap", max_imu_gap, 0.1);
	if (!std::isfinite(imu_time_offset)) {
		ROS_WARN("Invalid imu_time_offset; using 0.0 seconds.");
		imu_time_offset = 0.0;
	}
	if (!std::isfinite(max_imu_gap) || max_imu_gap < 1e-3) {
		ROS_WARN("Invalid max_imu_gap; using 0.1 seconds.");
		max_imu_gap = 0.1;
	}
	bool save_map = true;
	std::string map_file_path = "/tmp/lio_livox_global_map.pcd";
	double save_map_leaf_size = 0.2;
	double lidar_huber_delta = 0.1;
	double lidar_outlier_threshold = 1.0;
	ros::param::param<bool>("~save_map", save_map, true);
	ros::param::param<std::string>("~map_file_path", map_file_path,
								std::string("/tmp/lio_livox_global_map.pcd"));
	ros::param::param<double>("~save_map_leaf_size", save_map_leaf_size, 0.2);
	ros::param::param<double>("~lidar_huber_delta", lidar_huber_delta, 0.1);
	ros::param::param<double>("~lidar_outlier_threshold", lidar_outlier_threshold, 1.0);
	std::vector<double> vecTlb;
	ros::param::get("~Extrinsic_Tlb",vecTlb);

  // set extrinsic matrix between lidar & IMU
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
	R << vecTlb[0], vecTlb[1], vecTlb[2],
	     vecTlb[4], vecTlb[5], vecTlb[6],
	     vecTlb[8], vecTlb[9], vecTlb[10];
	t << vecTlb[3], vecTlb[7], vecTlb[11];
  Eigen::Quaterniond qr(R);
  R = qr.normalized().toRotationMatrix();
  exTlb.topLeftCorner(3,3) = R;
  exTlb.topRightCorner(3,1) = t;
  exRlb = R;
  exRbl = R.transpose();
  exPlb = t;
  exPbl = -1.0 * exRbl * exPlb;

  ros::Subscriber subFullCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>("/livox_full_cloud", 10, fullCallBack);
  ros::Subscriber sub_imu;
  if(IMU_Mode > 0)
    sub_imu = nodeHandler.subscribe(imu_topic, 2000, imu_callback, ros::TransportHints().unreliable());
  if(IMU_Mode < 2)
    WINDOWSIZE = 1;
  else
    WINDOWSIZE = 20;

  pubFullLaserCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_full_cloud_mapped", 10);
  pubLaserOdometry = nodeHandler.advertise<nav_msgs::Odometry> ("/livox_odometry_mapped", 5);
  pubLaserOdometryPath = nodeHandler.advertise<nav_msgs::Path> ("/livox_odometry_path_mapped", 5);
	pubGps = nodeHandler.advertise<sensor_msgs::NavSatFix>("/lidar", 1000);

  tfBroadcaster = new tf::TransformBroadcaster();

  laserCloudFullRes.reset(new pcl::PointCloud<PointType>);
  estimator = new Estimator(filter_parameter_corner, filter_parameter_surf,
                            save_map, map_file_path,
                            static_cast<float>(save_map_leaf_size),
                            lidar_huber_delta, lidar_outlier_threshold);
	lidarFrameList.reset(new std::list<Estimator::LidarFrame>);

  std::thread thread_process{process};
  ros::spin();

  if(thread_process.joinable()){
    thread_process.join();
  }
  delete estimator;
  estimator = nullptr;
  delete tfBroadcaster;
  tfBroadcaster = nullptr;

  return 0;
}
