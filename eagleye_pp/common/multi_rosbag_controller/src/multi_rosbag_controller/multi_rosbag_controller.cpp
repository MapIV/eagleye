#include <multi_rosbag_controller/multi_rosbag_controller.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

MultiRosbagController::MultiRosbagController()
{
}

MultiRosbagController::MultiRosbagController(std::vector<std::string> rosbag_names)
{
  openRosbag(rosbag_names);
}

void MultiRosbagController::openRosbag(std::vector<std::string> rosbag_names)
{
  num_rosbag_ = rosbag_names.size();
  rosbags_.resize(num_rosbag_);
  topic_list_.resize(num_rosbag_);

  for (int i = 0; i < num_rosbag_; i++)
  {
    rosbags_[i].open(rosbag_names[i], rosbag::bagmode::Read);
    std::cout << "Opened : " << rosbag_names[i] << std::endl;
  }

  std::cout << "ROSBAG count: " << num_rosbag_ << std::endl;

  is_topic_listed_ = false;
}

MultiRosbagController::~MultiRosbagController()
{
  for (int i = 0; i < num_rosbag_; i++)
  {
    rosbags_[i].close();
  }
}

bool MultiRosbagController::findTopic(std::string topic_name)
{
  // Collect all topic names in this rosbag
  if (!is_topic_listed_)
  {
    for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
    {
      rosbag::View search_topic(rosbags_[bag_id]);
      std::vector<const rosbag::ConnectionInfo*> connection_infos = search_topic.getConnections();

      BOOST_FOREACH (const rosbag::ConnectionInfo* info, connection_infos)
      {
        // If the current topic is not already in topic_list_[bag_id], add
        if (topic_list_[bag_id].find(info->topic) == topic_list_[bag_id].end())
        {
          topic_list_[bag_id].insert(info->topic);
        }
      }
    }
    is_topic_listed_ = true;
  }

  // Search the query topic
  for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
  {
    if (topic_list_[bag_id].find(topic_name) == topic_list_[bag_id].end())
    {
      return false;
    }
  }

  return true;
}

bool MultiRosbagController::setTopic(std::string topic_name)
{
  if (!findTopic(topic_name))
  {
    return false;
  }
  topics_.push_back(topic_name);
  return true;
}

int MultiRosbagController::selectTopicPriority(std::string prior_topic, std::string post_topic)
{
  if (setTopic(prior_topic))
  {
    return 0;
  }
  else if (setTopic(post_topic))
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

void MultiRosbagController::addQueries(rosbag::View& view)
{
  for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
  {
    view.addQuery(rosbags_[bag_id], rosbag::TopicQuery(topics_));
  }
}

void MultiRosbagController::resetTopic()
{
  topics_.clear();
}
