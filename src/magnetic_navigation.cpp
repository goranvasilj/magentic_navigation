/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <magnetic_navigation/dynparamConfig.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>


/*tf*/
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* for operations with matrices */
#include <Eigen/Dense>

//}

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

namespace magnetic_navigation
{

/* class MagneticNavigation //{ */

class MagneticNavigation : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;
  enum OPERATION_MODE {
  	FOLLOW_LINE = 0, LEFT_RIGHT = 1, TEST_TAKEOFF_LAND = 2
  };
  int left_right_direction;
  //transform listener
  tf::TransformListener transform_listener;

  /* ros parameters */
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void              callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
  std::atomic<bool> have_goal_ = false;
  std::atomic<bool> waypoint_reached_ = false;

  // | --------------------- timer callbacks -------------------- |

  void           timerPublishDistToWaypoint(const ros::TimerEvent& te);
  ros::Publisher pub_dist_to_waypoint_;
  ros::Timer     timer_publish_dist_to_waypoint_;
  int            _rate_timer_publish_dist_to_waypoint_;

  void           timerPublishSetReference(const ros::TimerEvent& te);
  ros::Publisher pub_reference_;
  ros::Timer     timer_publisher_reference_;
  int            _rate_timer_publisher_reference_;

  void       timerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |

  bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_waypoints_following_;

  bool               callbackStopWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_stop_waypoints_following_;

  bool               callbackFlyToFirstWaypoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_fly_to_first_waypoint_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  // | -------------------- loading waypoints ------------------- |

  std::vector<mrs_msgs::Reference> waypoints_;
  std::string                      _waypoints_frame_;
  bool                             waypoints_loaded_ = false;
  mrs_msgs::Reference              current_waypoint_;
  std::mutex                       mutex_current_waypoint_;
  int                              idx_current_waypoint_;
  int                              n_waypoints_;
  int                              _n_loops_;
  int                              c_loop_;
  std::mutex                       mutex_waypoint_idle_time_;
  Eigen::MatrixXd                  _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  typedef magnetic_navigation::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<magnetic_navigation::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                                      mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                        reconfigure_server_;
  void                                                                        callbackDynamicReconfigure(Config& config, uint32_t level);
  magnetic_navigation::dynparamConfig                                      last_drs_config_;

  // | --------------------- waypoint idling -------------------- |

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  double     _waypoint_idle_time_;
  double     _waypoint_desired_dist_;
  void       timerIdling(const ros::TimerEvent& te);


  //test
  double xpos,ypos,zpos, x_ref,y_ref,z_ref, heading, heading_ref;

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
};

//}

/* onInit() //{ */

void MagneticNavigation::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  have_goal_        = false;
  is_idling_        = false;
  waypoints_loaded_ = false;
  xpos=0;
  ypos=0;
  zpos=3;
  x_ref=2.2;
  y_ref=0;
  z_ref=0;
  heading=1.57;
  heading_ref=1.57;

  left_right_direction=0;

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "MagneticNavigation");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("land_at_the_end", _land_end_);
  param_loader.loadParam("n_loops", _n_loops_);
  param_loader.loadParam("waypoint_desired_distance", _waypoint_desired_dist_);
  param_loader.loadParam("waypoint_idle_time", _waypoint_idle_time_);
  param_loader.loadParam("waypoints_frame", _waypoints_frame_);
  param_loader.loadParam("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
  param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;
  param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;
  ROS_INFO_STREAM_ONCE("[MagneticNavigation]: " << n_waypoints_ << " waypoints loaded");
  ROS_INFO_STREAM_ONCE("[MagneticNavigation]: " << _n_loops_ << " loops requested");

  /* load offset of all waypoints as a static matrix from config file */
  param_loader.loadMatrixKnown("offset", _offset_, 1, 4);
  offsetPoints(waypoints_, _offset_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MagneticNavigation]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "MagneticNavigation";
  shopts.no_message_timeout = ros::Duration(1.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in",
                                                                                            &MagneticNavigation::callbackControlManagerDiag, this);

  // | ------------------ initialize publishers ----------------- |

  pub_dist_to_waypoint_ = nh.advertise<mrs_msgs::Float64Stamped>("dist_to_waypoint_out", 1);
  pub_reference_        = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | -------------------- initialize timers ------------------- |

  timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(_rate_timer_publish_dist_to_waypoint_), &MagneticNavigation::timerPublishDistToWaypoint, this);

  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &MagneticNavigation::timerCheckSubscribers, this);

  // you can disable autostarting of the timer by the last argument
  timer_publisher_reference_ = nh.createTimer(ros::Rate(_rate_timer_publisher_reference_), &MagneticNavigation::timerPublishSetReference, this, false, true);

  // | --------------- initialize service servers --------------- |

  srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &MagneticNavigation::callbackStartWaypointFollowing, this);
  srv_server_stop_waypoints_following_  = nh.advertiseService("stop_waypoints_following_in", &MagneticNavigation::callbackStopWaypointFollowing, this);
  srv_server_fly_to_first_waypoint_     = nh.advertiseService("fly_to_first_waypoint_in", &MagneticNavigation::callbackFlyToFirstWaypoint, this);

  // | --------------- initialize service clients --------------- |

  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out");

  // | ---------- initialize dynamic reconfigure server --------- |

  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&MagneticNavigation::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);
    last_drs_config_.waypoint_idle_time = _waypoint_idle_time_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);

  ROS_INFO_ONCE("[MagneticNavigation]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */

void MagneticNavigation::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MagneticNavigation]: Received first control manager diagnostics msg");

  // get the variable under the mutex
  mrs_msgs::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  //ROS_INFO("[MagneticNavigation]: Distance to waypoint: %.2f", dist);

  if (have_goal_ && !diagnostics->tracker_status.have_goal) {
    have_goal_ = false;

    if (dist < _waypoint_desired_dist_) {
      waypoint_reached_ = true;
      ROS_INFO("[MagneticNavigation]: Waypoint reached.");

      /* start idling at the reached waypoint */
      is_idling_ = true;

      ros::NodeHandle nh("~");
      timer_idling_ = nh.createTimer(ros::Duration(_waypoint_idle_time_), &MagneticNavigation::timerIdling, this,
                                     true);  // the last boolean argument makes the timer run only once

      ROS_INFO("[MagneticNavigation]: Idling for %.2f seconds.", _waypoint_idle_time_);
    }
  }
}

//}


double QuaternionToHeading(double qw,double qx,double qy,double qz)
{
	  double siny = +2.0 * (qw * qz + qy * qx);
	  double cosy = +1.0 - 2.0 * (qx * qx + qz * qz);
	  double heading1 = atan2(siny, cosy);      // in radians
	  return heading1;
}


// | --------------------- timer callbacks -------------------- |

/* timerPublishSetReference() //{ */

void MagneticNavigation::timerPublishSetReference([[maybe_unused]] const ros::TimerEvent& te) {


	if (!is_initialized_) {
    return;
  }

  static int first=0;
  
  if (first<20)
  {
  	first++;
  	return;
  }
  first++;
  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());
  
    double current_heading=QuaternionToHeading(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);

  
  static ros::Time start_time = ros::Time::now();
  static bool landed=false;
  OPERATION_MODE mode=OPERATION_MODE::FOLLOW_LINE;
  double x_diff, y_diff, z_diff, heading_diff;
  if (mode == OPERATION_MODE::TEST_TAKEOFF_LAND)
  {
	  ros::Duration duration=ros::Time::now()- start_time;
	  if (duration.toSec()>10 && landed == false)
	  {
		  landed=true;

		  ROS_INFO("[MagneticNavigation]: Calling land service.");
		  std_srvs::Trigger srv_land_call;
		  srv_client_land_.call(srv_land_call);
	  }
	  return;
  }



  /* return if the uav is still flying to the previous waypoints */
 /* if (have_goal_) {
    return;
  }

  /* return if the UAV is idling at a waypoint */
/*  if (is_idling_) {
    return;
  }*/
/*  xpos=xpos+0.1;
  ypos=ypos+0.1;*/
  mrs_msgs::Reference ref;


  tf::StampedTransform transform;
  ros::Time t = ros::Time(0);
  try {
	  transform_listener.lookupTransform("/magnetometer_center", "/power_line0", t, transform);

  }
  catch (tf::TransformException ex){
  	ROS_ERROR("%s",ex.what());
  	return;
  }
  double qw=transform.getRotation().getW();
  double qx=transform.getRotation().getX();
  double qy=transform.getRotation().getY();
  double qz=transform.getRotation().getZ();

  double heading1 =QuaternionToHeading(qw,qx,qy,qz);



  std::cout<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<" "<<heading1<<"    powerline heading "<< current_heading+heading1<<std::endl;


  double dx=transform.getOrigin().x();
  double dy=transform.getOrigin().y();
  double dz=transform.getOrigin().z();

  if (fabs(heading_ref-heading1) >3.14159/2)
  {
	  heading1=heading1+3.14159265;
  }
	static int count_goal_reached=0;
  double position_gain=0.02;
  double max_position_speed=0.01;
  double heading_gain=0.01;
  double max_heading_speed=0.001;
  if (first==20)
  {	
  	xpos=current_pose.position.x;
  	ypos=current_pose.position.y;
  	zpos=current_pose.position.z;
  	heading=current_heading;
  }
/*  xpos=xpos*0.5+0.5*current_pose.position.x;
  ypos=ypos*0.5+0.5*current_pose.position.y;
  zpos=zpos*0.5+0.5*current_pose.position.z;*/
  heading=heading*0.5+0.5*current_heading;

  if (mode == OPERATION_MODE::FOLLOW_LINE)
  {
	  double x_diff=(x_ref-dx)*position_gain;
	  double y_diff=0;//(y_ref-dy)*position_gain;
	  double z_diff=(z_ref-dz)*position_gain;
	  double heading_diff=(heading_ref-heading1)*heading_gain;
	  if (heading_diff>3.14159) heading_diff=heading_diff-2*3.14159;
	  
	  if (heading_diff<-3.14159) heading_diff=heading_diff+2*3.14159;

	  if (fabs(x_diff)>max_position_speed) x_diff=(x_diff)/fabs(x_diff)*max_position_speed;
	  if (fabs(y_diff)>max_position_speed) y_diff=(y_diff)/fabs(y_diff)*max_position_speed;
	  if (fabs(z_diff)>max_position_speed) z_diff=(z_diff)/fabs(z_diff)*max_position_speed;
	  if (fabs(heading_diff)>max_heading_speed) heading_diff=(heading_diff)/fabs(heading_diff)*max_heading_speed;

	  if (fabs(x_ref-dx)<0.4 /*&& fabs(y_ref-dy)<0.2*/ && fabs(z_ref-dz)<0.4 && fabs(heading_ref-heading1)<0.2)
	  {
		 count_goal_reached++;
		  y_diff=y_diff+0.5;
	  }
	  std::cout<<"heading "<< xpos<< " "<<ypos<<" "<<zpos<<" "<<heading<<"   goal reached count"<<count_goal_reached<<std::endl;
	  heading=heading-heading_diff;
	  xpos=xpos-(cos(heading)*x_diff-sin(heading)*y_diff);
	  ypos=ypos-(sin(heading)*x_diff+cos(heading)*y_diff);
	  zpos=zpos-z_diff;

	  std::cout<<"goal "<< -(cos(heading)*x_diff-sin(heading)*y_diff)<< " "<<-(sin(heading)*x_diff+cos(heading)*y_diff)<<" "<<z_diff<<" "<<heading_diff<<std::endl;
  }
  if (mode == OPERATION_MODE::LEFT_RIGHT)
  {
	  double x_ref1=1;
	  double y_ref1=-1;
	  double z_ref1=1;
	  double x_ref2=1;
	  double y_ref2=6;
	  double z_ref2=1;



	  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());


	  double xwire= current_pose.position.x+(cos(heading)*dx-sin(heading)*dy);
	  double ywire = current_pose.position.y+(sin(heading)*dx+cos(heading)*dy);
	  double zwire = current_pose.position.z+dz;
	  printf("wire pos %.2f %.2f %.2f\n",xwire,ywire,zwire);
	  double xpos1=0,ypos1=0,zpos1=0;
	  if (left_right_direction==0)
	  {
		  xpos1=x_ref1;
		  ypos1=y_ref1;
		  zpos1=z_ref1;
	  }
	  if (left_right_direction==1)
	  {
		  xpos1=x_ref2;
		  ypos1=y_ref2;
		  zpos1=z_ref2;
	  }
	  position_gain=0.02;
	  double vx=(xpos1-current_pose.position.x)*position_gain;
	  double vy=(ypos1-current_pose.position.y)*position_gain;
	  double vz=(zpos1-current_pose.position.z)*position_gain*2;

	  printf("velocities %.2f %.2f %.2f   %.2f %.2f %.2f\n",vx,vy,vz,xpos,ypos,zpos);
	  max_position_speed=0.05;
	  if (fabs(vx)>max_position_speed) vx=(vx)/fabs(vx)*max_position_speed;
	  if (fabs(vy)>max_position_speed) vy=(vy)/fabs(vy)*max_position_speed;
	  if (fabs(vz)>max_position_speed) vz=(vz)/fabs(vz)*max_position_speed;

	  double ddx=xpos+vx-xwire;
	  double ddy=ypos+vy-ywire;
	  double ddz=zpos+vz-zwire;


//	  if (sqrt((dx*dx+dy*dy+dz*dz))<1)
	  if (sqrt((ddx*ddx+ddy*ddy+ddz*ddz))<1)
	  {
		  double reaction_gain=0.2;
		  printf("Original vz %.2f\n",vz);
//		  vz=vz+(1-sqrt((dx*dx+dy*dy+dz*dz)))*reaction_gain;
		  vz=vz+(1-sqrt((ddx*ddx+ddy*ddy+ddz*ddz)))*reaction_gain;
		  if (fabs(vz)>max_position_speed) vz=(vz)/fabs(vz)*max_position_speed;
	  }




	  if (fabs(xpos1-current_pose.position.x)+fabs(ypos1-current_pose.position.y)+fabs(zpos1-current_pose.position.z)<0.2)
	  {
		  left_right_direction=1-left_right_direction;
		  printf("change left right %d\n",left_right_direction);
	  }
	  xpos=xpos+vx;
	  ypos=ypos+vy;
	  zpos=zpos+vz;

  }
  printf("%.2f %.2f %.2f\n",xpos,ypos,zpos);
  ref.heading=heading;
  ref.position.x=xpos;
  ref.position.y=ypos;
  ref.position.z=zpos;

  /* shutdown node after flying through all the waypoints (call land service before) */
  /*if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    ROS_INFO("[MagneticNavigation]: Finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      ROS_INFO("[MagneticNavigation]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {
        ROS_INFO("[MagneticNavigation]: Calling land service.");
        std_srvs::Trigger srv_land_call;
        srv_client_land_.call(srv_land_call);
      }

      ROS_INFO("[MagneticNavigation]: Shutting down.");
      ros::shutdown();
      return;

    } else {
      ROS_INFO("[MagneticNavigation]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }


  /* create new waypoint msg */
  mrs_msgs::ReferenceStamped new_waypoint;



  // set the frame id in which the reference is expressed
  new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  new_waypoint.header.stamp    = ros::Time::now();

  new_waypoint.reference = ref;//waypoints_.at(idx_current_waypoint_);

  // set the variable under the mutex
  mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

  ROS_INFO("[MagneticNavigation]: Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
           new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

  try {
    pub_reference_.publish(new_waypoint);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
  }

  if (waypoint_reached_) {
    idx_current_waypoint_++;
    waypoint_reached_ = false;
  }

//  have_goal_ = true;
}

//}

/* timerPublishDistToWaypoint() //{ */

void MagneticNavigation::timerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  ROS_INFO("[MagneticNavigation]: Distance to waypoint: %.2f", dist);

  mrs_msgs::Float64Stamped dist_msg;
  // it is important to set the frame id correctly !!
  dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  dist_msg.header.stamp    = ros::Time::now();
  dist_msg.value           = dist;

  try {
    pub_dist_to_waypoint_.publish(dist_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_dist_to_waypoint_.getTopic().c_str());
  }
}

//}

/* timerCheckSubscribers() //{ */

void MagneticNavigation::timerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[MagneticNavigation]: Not received uav odom msg since node launch.");
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[MagneticNavigation]: Not received tracker diagnostics msg since node launch.");
  }
}

//}

/* timerIdling() //{ */

void MagneticNavigation::timerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[MagneticNavigation]: Idling finished");
  is_idling_ = false;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */

bool MagneticNavigation::callbackStartWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[MagneticNavigation]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_.start();

    ROS_INFO("[MagneticNavigation]: Starting waypoint following.");

    res.success = true;
    res.message = "Starting waypoint following.";

  } else {

    ROS_WARN("[MagneticNavigation]: Cannot start waypoint following, waypoints are not set.");
    res.success = false;
    res.message = "Waypoints not set.";
  }

  return true;
}

//}

/* //{ callbackStopWaypointFollowing() */

bool MagneticNavigation::callbackStopWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[MagneticNavigation]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_.stop();

  ROS_INFO("[MagneticNavigation]: Waypoint following stopped.");

  res.success = true;
  res.message = "Waypoint following stopped.";

  return true;
}

//}

/* //{ callbackFlyToFirstWaypoint() */

bool MagneticNavigation::callbackFlyToFirstWaypoint([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[MagneticNavigation]: Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = ros::Time::now();
    new_waypoint.reference       = waypoints_.at(0);

    mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

    // set the variable under the mutex

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    have_goal_ = true;

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
    }

    std::stringstream ss;
    ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
       << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    ROS_INFO_STREAM_THROTTLE(1.0, "[MagneticNavigation]: " << ss.str());

    res.success = true;
    res.message = ss.str();

  } else {

    ROS_WARN("[MagneticNavigation]: Cannot fly to first waypoint, waypoints not loaded!");

    res.success = false;
    res.message = "Waypoints not loaded";
  }

  return true;
}

//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */

void MagneticNavigation::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[MagneticNavigation]:"
      "Reconfigure Request: "
      "Waypoint idle time: %.2f",
      config.waypoint_idle_time);

  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);

    _waypoint_idle_time_ = config.waypoint_idle_time;
  }
}

//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */

std::vector<mrs_msgs::Reference> MagneticNavigation::matrixToPoints(const Eigen::MatrixXd& matrix) {

  std::vector<mrs_msgs::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {

    mrs_msgs::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);

    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void MagneticNavigation::offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {

    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */

double MagneticNavigation::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

//}

}  // namespace magnetic_navigation

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(magnetic_navigation::MagneticNavigation, nodelet::Nodelet);
