#ifndef MULTI_ROSBAG_CONTROLLER
#define MULTI_ROSBAG_CONTROLLER

#include <vector>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>

class MultiRosbagController
{
public:
  MultiRosbagController();
  MultiRosbagController(std::vector<std::string> rosbag_names);
  ~MultiRosbagController();

  void openRosbag(std::vector<std::string> rosbag_names);
  bool findTopic(std::string topic_name);
  bool setTopic(std::string topic_name);
  int selectTopicPriority(std::string prior_topic, std::string pos_topic);
  void addQueries(rosbag::View& view);
  void resetTopic();

private:
  // Variables
  int num_rosbag_;
  std::vector<rosbag::Bag> rosbags_;
  std::vector<std::string> topics_;
  std::vector<std::set<std::string>> topic_list_;
  bool is_topic_listed_;
};

#endif
