/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatFix.h>

#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/String.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/quadratic_thrust_model.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/gps_conversions.h>
#include <tuple>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;
using mat2_t = Eigen::Matrix2Xd;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace multi_lander
{

/* defines //{ */

typedef struct
{
  vec2_t from;
  vec2_t to;
} Segment_t;

//}

/* //{ class MultiLander */

class MultiLander : public nodelet::Nodelet {

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

public:
  virtual void onInit();

  int _service_n_attempts_;

  std::string _tracker_;
  std::string _constraints_;
  std::string _controller_;
  std::string _altitude_estimator_;
  double      _min_distance_before_landing_;

  // subscribers
  std::vector<mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>>     sh_uav_diags_;
  std::vector<mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>> sh_control_diags_;

  // publishers
  ros::Publisher publisher_flight_time_;

  // service servers
  ros::ServiceServer service_server_next_;

  // service callbacks
  bool callbackNext([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // service clients
  std::vector<mrs_lib::ServiceClientHandler<std_srvs::Trigger>> sch_land_home_;
  std::vector<mrs_lib::ServiceClientHandler<mrs_msgs::String>>  sch_switch_controller_;
  std::vector<mrs_lib::ServiceClientHandler<mrs_msgs::String>>  sch_switch_tracker_;
  std::vector<mrs_lib::ServiceClientHandler<mrs_msgs::String>>  sch_set_constraints_;
  std::vector<mrs_lib::ServiceClientHandler<std_srvs::SetBool>> sch_enable_callbacks_;
  std::vector<mrs_lib::ServiceClientHandler<mrs_msgs::String>>  sch_change_alt_estimator_;

  std::vector<bool> was_flying_;
  std::mutex        mutex_was_flying_;

  std::vector<bool> sent_home_;

  // timers
  ros::Timer timer_main_;
  double     _timer_main_rate_;

  // other uav names
  std::vector<std::string> _uav_names_;

  // timer callbacks
  void timerMain(const ros::TimerEvent& event);

  // routines
  bool                                      landHome(const int& id);
  bool                                      switchTracker(const int& id);
  bool                                      switchController(const int& id);
  bool                                      setConstraints(const int& id);
  bool                                      changeAltEstimator(const int& id);
  bool                                      setCallbacks(const int& id, const bool& value);
  std::optional<std::tuple<double, double>> getHomePosition(const int& id);
  std::optional<std::tuple<double, double>> getCurrentPosition(const int& id);
  bool                                      isReady(const int& id);
  bool                                      flyingHome(const int& id);
  bool                                      isActive(const int& id);

  double distFromSegment(const vec2_t& point, const vec2_t& seg1, const vec2_t& seg2);

  int        last_sent_home_  = -1;
  bool       uav_flying_home_ = false;
  std::mutex mutex_last_sent_home_;
};

//}

/* //{ onInit() */

void MultiLander::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[MultiLander]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "MultiLander");

  param_loader.loadParam("timer_main/rate", _timer_main_rate_);

  param_loader.loadParam("controller", _controller_);
  param_loader.loadParam("tracker", _tracker_);
  param_loader.loadParam("constraints", _constraints_);
  param_loader.loadParam("altitude_estimator", _altitude_estimator_);
  param_loader.loadParam("min_distance_before_landing", _min_distance_before_landing_);

  // params
  std::string uav_manager_diag_topic;
  std::string control_manager_diag_topic;
  std::string uav_home_gps_topic;
  std::string land_home_service;
  std::string switch_controller_service;
  std::string switch_tracker_service;
  std::string set_constraints_service;
  std::string enable_callbacks_service;
  std::string change_alt_estimator_service;

  param_loader.loadParam("uav_manager_diag_topic", uav_manager_diag_topic);
  param_loader.loadParam("control_manager_diag_topic", control_manager_diag_topic);
  param_loader.loadParam("land_home_service", land_home_service);
  param_loader.loadParam("switch_controller_service", switch_controller_service);
  param_loader.loadParam("switch_tracker_service", switch_tracker_service);
  param_loader.loadParam("set_constraints_service", set_constraints_service);
  param_loader.loadParam("enable_callbacks_service", enable_callbacks_service);
  param_loader.loadParam("change_alt_estimator_service", change_alt_estimator_service);

  param_loader.loadParam("network/robot_names", _uav_names_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MultiLander]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MultiLander";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // create subscribers for uav topics
  for (const auto& name : _uav_names_) {

    std::string uav_diag_full_topic               = std::string("/") + name + std::string("/") + uav_manager_diag_topic;
    std::string control_diag_full_topic           = std::string("/") + name + std::string("/") + control_manager_diag_topic;
    std::string land_home_full_service            = std::string("/") + name + std::string("/") + land_home_service;
    std::string switch_controller_full_service    = std::string("/") + name + std::string("/") + switch_controller_service;
    std::string switch_tracker_full_service       = std::string("/") + name + std::string("/") + switch_tracker_service;
    std::string set_constraints_full_service      = std::string("/") + name + std::string("/") + set_constraints_service;
    std::string enable_callbacks_full_service     = std::string("/") + name + std::string("/") + enable_callbacks_service;
    std::string change_alt_estimator_full_service = std::string("/") + name + std::string("/") + change_alt_estimator_service;

    ROS_INFO("[MultiLander]: subscribing to %s", uav_diag_full_topic.c_str());
    sh_uav_diags_.push_back(mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>(shopts, uav_diag_full_topic));

    ROS_INFO("[MultiLander]: subscribing to %s", control_diag_full_topic.c_str());
    sh_control_diags_.push_back(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, control_diag_full_topic));

    ROS_INFO("[MultiLander]: hooking service to %s", land_home_full_service.c_str());
    sch_land_home_.push_back(mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, land_home_full_service));

    ROS_INFO("[MultiLander]: hooking service to %s", switch_controller_full_service.c_str());
    sch_switch_controller_.push_back(mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, switch_controller_full_service));

    ROS_INFO("[MultiLander]: hooking service to %s", switch_tracker_full_service.c_str());
    sch_switch_tracker_.push_back(mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, switch_tracker_full_service));

    ROS_INFO("[MultiLander]: hooking service to %s", set_constraints_full_service.c_str());
    sch_set_constraints_.push_back(mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, set_constraints_full_service));

    ROS_INFO("[MultiLander]: hooking service to %s", enable_callbacks_full_service.c_str());
    sch_enable_callbacks_.push_back(mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, enable_callbacks_full_service));

    ROS_INFO("[MultiLander]: hooking service to %s", change_alt_estimator_full_service.c_str());
    sch_change_alt_estimator_.push_back(mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, change_alt_estimator_full_service));

    was_flying_.push_back(false);
    sent_home_.push_back(false);
  }

  // | --------------------- service servers -------------------- |

  service_server_next_ = nh_.advertiseService("next_in", &MultiLander::callbackNext, this);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_timer_main_rate_), &MultiLander::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[MultiLander]: initialized");
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* distFromSegment() //{ */

double MultiLander::distFromSegment(const vec2_t& point, const vec2_t& seg1, const vec2_t& seg2) {

  vec2_t segment_vector = seg2 - seg1;
  double segment_len    = segment_vector.norm();

  vec2_t segment_vector_norm = segment_vector;
  segment_vector_norm.normalize();

  double point_coordinate = segment_vector_norm.dot(point - seg1);

  if (point_coordinate < 0) {
    return (point - seg1).norm();
  } else if (point_coordinate > segment_len) {
    return (point - seg2).norm();
  } else {

    mat2_t segment_projector = segment_vector_norm * segment_vector_norm.transpose();
    vec2_t projection        = seg1 + segment_projector * (point - seg1);

    return (point - projection).norm();
  }
}

//}

/* landHome() //{ */

bool MultiLander::landHome(const int& id) {

  std_srvs::Trigger srv;

  bool success = sch_land_home_[id].call(srv, _service_n_attempts_);

  if (!success) {
    ROS_ERROR("[MultiLander]: land home for %s failed", _uav_names_[id].c_str());
  }

  return success;
}

//}

/* switchTracker() //{ */

bool MultiLander::switchTracker(const int& id) {

  mrs_msgs::String srv;
  srv.request.value = _tracker_;

  return sch_switch_tracker_[id].call(srv, _service_n_attempts_);
}

//}

/* switchController() //{ */

bool MultiLander::switchController(const int& id) {

  mrs_msgs::String srv;
  srv.request.value = _controller_;

  return sch_switch_controller_[id].call(srv, _service_n_attempts_);
}

//}

/* setConstraints() //{ */

bool MultiLander::setConstraints(const int& id) {

  mrs_msgs::String srv;
  srv.request.value = _constraints_;

  return sch_set_constraints_[id].call(srv, _service_n_attempts_);
}

//}

/* changeAltEstimator() //{ */

bool MultiLander::changeAltEstimator(const int& id) {

  mrs_msgs::String srv;
  srv.request.value = _altitude_estimator_;

  return sch_change_alt_estimator_[id].call(srv, _service_n_attempts_);
}

//}

/* setCallbacks() //{ */

bool MultiLander::setCallbacks(const int& id, const bool& value) {

  std_srvs::SetBool srv;
  srv.request.data = value;

  return sch_enable_callbacks_[id].call(srv, _service_n_attempts_);
}

//}

/* getHomePosition() //{ */

std::optional<std::tuple<double, double>> MultiLander::getHomePosition(const int& id) {

  if (!sh_uav_diags_[id].hasMsg()) {
    return {};
  }

  if ((ros::Time::now() - sh_uav_diags_[id].lastMsgTime()).toSec() > 3.0) {
    return {};
  }

  const double lat = sh_uav_diags_[id].getMsg()->home_latitude;
  const double lon = sh_uav_diags_[id].getMsg()->home_longitude;

  double x, y;

  mrs_lib::UTM(lat, lon, &x, &y);

  return std::make_tuple(x, y);
}

//}

/* getCurrentPosition() //{ */

std::optional<std::tuple<double, double>> MultiLander::getCurrentPosition(const int& id) {

  if (!sh_uav_diags_[id].hasMsg()) {
    return {};
  }

  if ((ros::Time::now() - sh_uav_diags_[id].lastMsgTime()).toSec() > 3.0) {
    return {};
  }

  const double lat = sh_uav_diags_[id].getMsg()->cur_latitude;
  const double lon = sh_uav_diags_[id].getMsg()->cur_longitude;

  double x, y;

  mrs_lib::UTM(lat, lon, &x, &y);

  return std::make_tuple(x, y);
}

//}

/* isActive() //{ */

bool MultiLander::isActive(const int& id) {

  if (sh_uav_diags_[id].hasMsg()) {
    if ((ros::Time::now() - sh_uav_diags_[id].lastMsgTime()).toSec() < 10.0) {
      return true;
    }
  }

  return false;
}

//}

/* isReady() //{ */

bool MultiLander::isReady(const int& id) {

  if (!sh_control_diags_[id].hasMsg()) {
    return false;
  }

  if ((ros::Time::now() - sh_control_diags_[id].lastMsgTime()).toSec() > 3.0) {
    return false;
  }

  bool flying_normally     = sh_control_diags_[id].getMsg()->flying_normally;
  bool have_goal           = sh_control_diags_[id].getMsg()->tracker_status.have_goal;
  bool tracking_trajectory = sh_control_diags_[id].getMsg()->tracker_status.tracking_trajectory;
  bool sent_home           = sent_home_[id];

  return flying_normally && !have_goal && !tracking_trajectory && !sent_home;
}

//}

/* flyingHome() //{ */

bool MultiLander::flyingHome(const int& id) {

  if (!sh_control_diags_[id].hasMsg()) {
    return false;
  }

  if ((ros::Time::now() - sh_control_diags_[id].lastMsgTime()).toSec() > 3.0) {
    return false;
  }

  bool have_goal      = sh_control_diags_[id].getMsg()->tracker_status.have_goal;
  bool tracker_active = sh_control_diags_[id].getMsg()->active_tracker == _tracker_;

  return have_goal && tracker_active;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ timerMain() */

void MultiLander::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[MultiLander]: timerMain() spinning");

  {
    std::scoped_lock lock(mutex_was_flying_);

    for (size_t i = 0; i < _uav_names_.size(); i++) {

      if (!was_flying_[i] && isActive(i)) {

        was_flying_[i] = true;

        ROS_INFO("[MultiLander]: %s is active", _uav_names_[i].c_str());
      }
    }
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackNext() */

bool MultiLander::callbackNext([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  std::stringstream ss;

  auto uav_flying_home = mrs_lib::get_mutexed(mutex_last_sent_home_, uav_flying_home_);
  auto last_sent_home  = mrs_lib::get_mutexed(mutex_last_sent_home_, last_sent_home_);

  if (uav_flying_home) {

    if (!flyingHome(last_sent_home)) {

      ROS_INFO("[MultiLander]: %s is home", _uav_names_[last_sent_home].c_str());

      uav_flying_home_ = false;
    }
  }

  // | ----------------- find the active uav ids ---------------- |

  std::vector<int> active_uavs;

  /* get active uavs //{ */

  for (size_t i = 0; i < _uav_names_.size(); i++) {

    if (was_flying_[i] && isActive(i) && isReady(i)) {
      active_uavs.push_back(i);
    }
  }

  //}

  if (active_uavs.size() == 0) {
    ss << "no UAVs are flying";
    res.message = ss.str();
    res.success = false;
    ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());
    return true;
  }

  // | ---------------- build the homing segments --------------- |

  std::vector<Segment_t> homing_segments;

  /* build homing segments //{ */

  for (size_t uav = 0; uav < active_uavs.size(); uav++) {

    int uav_id = active_uavs[uav];

    double xh = 0;
    double yh = 0;
    double x  = 0;
    double y  = 0;

    auto home = getHomePosition(uav_id);

    if (home) {
      std::tie(xh, yh) = home.value();
    } else {
      ss << "could not get home position for " << _uav_names_[uav_id].c_str();
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());
      return true;
    }

    auto cur = getCurrentPosition(uav_id);

    if (cur) {
      std::tie(x, y) = cur.value();
    } else {
      ss << "could not get current position for " << _uav_names_[uav_id].c_str();
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());
      return true;
    }

    ROS_INFO("[MultiLander]: %s: home: %.2f, %.2f, current %.2f, %.2f", _uav_names_[uav_id].c_str(), xh, yh, x, y);

    Segment_t segment;
    segment.from[0] = x;
    segment.from[1] = y;
    segment.to[0]   = xh;
    segment.to[1]   = yh;

    homing_segments.push_back(segment);
  }

  //}

  // | ----- find distances betwen homing segments and uavs ----- |

  std::vector<double> uavs_distances;

  /* find distances //{ */

  for (size_t uav = 0; uav < active_uavs.size(); uav++) {

    int uav_id = active_uavs[uav];

    double min_distance = std::numeric_limits<double>::max();

    for (size_t other = 0; other < active_uavs.size(); other++) {

      int other_id = active_uavs[other];

      if (uav_id != other_id) {

        // get the distance of the other UAV to this UAV's homing segment
        const double distance = distFromSegment(homing_segments[other].from, homing_segments[uav].from, homing_segments[uav].to);

        ROS_INFO("[MultiLander]: %s segment vs %s = %.2f", _uav_names_[uav].c_str(), _uav_names_[other].c_str(), distance);

        if (distance < min_distance) {
          min_distance = distance;
        }
      }
    }

    uavs_distances.push_back(min_distance);
  }

  //}

  // | --------- find the UAV with the largest distance --------- |

  double max_distance     = std::numeric_limits<double>::lowest();
  int    candidate_uav_id = -1;

  for (size_t i = 0; i < active_uavs.size(); i++) {

    if (uavs_distances[i] > max_distance) {
      max_distance     = uavs_distances[i];
      candidate_uav_id = active_uavs[i];
    }
  }

  if (uav_flying_home) {

    double last_x, last_y;
    double cur_x, cur_y;

    {
      auto result = getCurrentPosition(last_sent_home);

      if (result) {
        std::tie(last_x, last_y) = result.value();
      } else {
        ss << "could not get current position for " << _uav_names_[last_sent_home].c_str();
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());
        return true;
      }
    }

    {
      auto result = getCurrentPosition(candidate_uav_id);

      if (result) {
        std::tie(cur_x, cur_y) = result.value();
      } else {
        ss << "could not get current position for " << _uav_names_[candidate_uav_id].c_str();
        res.message = ss.str();
        res.success = false;
        ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());
        return true;
      }
    }

    if (flyingHome(last_sent_home) && sqrt(pow(last_x - cur_x, 2) + pow(last_y - cur_y, 2)) < _min_distance_before_landing_) {

      ss << "waiting for " << _uav_names_[last_sent_home].c_str() << " to get home";
      res.message = ss.str();
      res.success = false;
      ROS_ERROR_STREAM("[MultiLander]: Failed: " << ss.str());

      return true;
    }
  }

  if (candidate_uav_id >= 0) {

    ROS_INFO("[MultiLander]: landing with %s, min distance %.2f", _uav_names_[candidate_uav_id].c_str(), max_distance);

    switchTracker(candidate_uav_id);

    ros::Duration(0.1).sleep();

    switchController(candidate_uav_id);

    ros::Duration(0.1).sleep();

    setConstraints(candidate_uav_id);

    ros::Duration(0.1).sleep();

    changeAltEstimator(candidate_uav_id);

    ros::Duration(0.1).sleep();

    landHome(candidate_uav_id);

    sent_home_[candidate_uav_id] = true;

    {
      std::scoped_lock lock(mutex_last_sent_home_);

      uav_flying_home_ = true;
      last_sent_home_  = candidate_uav_id;
    }

  } else {
    ROS_ERROR("[MultiLander]: did not find UAV to land home");
  }

  res.message = "yes";
  res.success = true;
  return true;
}

//}

}  // namespace multi_lander

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_lander::MultiLander, nodelet::Nodelet)
