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

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace multi_lander
{

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

  std::vector<bool> was_flying_;
  std::mutex        mutex_was_flying_;

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
  std::optional<std::tuple<double, double>> getHomePosition(const int& id);
  std::optional<std::tuple<double, double>> getCurrentPosition(const int& id);
  std::optional<bool>                       isReady(const int& id);
  bool                                      isActive(const int& id);
};

//}

/* //{ onInit() */

void MultiLander::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[MultiLander]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "MultiLander");

  param_loader.loadParam("timer_main/rate", _timer_main_rate_);

  // params
  std::string uav_manager_diag_topic;
  std::string control_manager_diag_topic;
  std::string uav_home_gps_topic;
  std::string land_home_service;
  std::string switch_controller_service;
  std::string switch_tracker_service;
  std::string set_constraints_service;

  param_loader.loadParam("uav_manager_diag_topic", uav_manager_diag_topic);
  param_loader.loadParam("control_manager_diag_topic", control_manager_diag_topic);
  param_loader.loadParam("land_home_service", land_home_service);
  param_loader.loadParam("switch_controller_service", switch_controller_service);
  param_loader.loadParam("switch_tracker_service", switch_tracker_service);
  param_loader.loadParam("set_constraints_service", set_constraints_service);

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

    std::string uav_diag_full_topic            = std::string("/") + name + std::string("/") + uav_manager_diag_topic;
    std::string control_diag_full_topic        = std::string("/") + name + std::string("/") + control_manager_diag_topic;
    std::string land_home_full_service         = std::string("/") + name + std::string("/") + land_home_service;
    std::string switch_controller_full_service = std::string("/") + name + std::string("/") + switch_controller_service;
    std::string switch_tracker_full_service    = std::string("/") + name + std::string("/") + switch_tracker_service;
    std::string set_constraints_full_service   = std::string("/") + name + std::string("/") + set_constraints_service;

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

    was_flying_.push_back(false);
  }

  // | --------------------- service servers -------------------- |

  service_server_next_ = nh_.advertiseService("next", &MultiLander::callbackNext, this);

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

/* landHome() //{ */

bool MultiLander::landHome(const int& id) {

  std_srvs::Trigger srv;

  return sch_land_home_[id].call(srv, _service_n_attempts_);
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

std::optional<bool> MultiLander::isReady(const int& id) {

  if (!sh_control_diags_[id].hasMsg()) {
    return {};
  }

  if ((ros::Time::now() - sh_control_diags_[id].lastMsgTime()).toSec() > 3.0) {
    return {};
  }

  bool flying_normally     = sh_control_diags_[id].getMsg()->flying_normally;
  bool have_goal           = sh_control_diags_[id].getMsg()->tracker_status.have_goal;
  bool tracking_trajectory = sh_control_diags_[id].getMsg()->tracker_status.tracking_trajectory;

  return flying_normally && !have_goal && !tracking_trajectory;
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

  for (size_t i; i < size(_uav_names_); i++) {

    if (isActive(i)) {

      std::scoped_lock lock(mutex_was_flying_);
      was_flying_[i] = true;

      ROS_INFO("[MultiLander]: %s is active", _uav_names_[i].c_str());
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | --------------------- topic callbacks -------------------- |

// | -------------------- service callbacks ------------------- |

/* //{ callbackNext() */

bool MultiLander::callbackNext([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_)
    return false;

  return true;
}

//}


}  // namespace multi_lander

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_lander::MultiLander, nodelet::Nodelet)
