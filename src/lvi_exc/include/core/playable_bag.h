#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
#include <functional>
#include <queue>

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_ros/buffer.h"


class PlayableBag {
 public:
  // Handles messages early, i.e. when they are about to enter the buffer.
  // Returns a boolean indicating whether the message should enter the buffer.
  using FilteringEarlyMessageHandler =
      std::function<bool /* forward_message_to_buffer */ (
          const rosbag::MessageInstance&)>;

  PlayableBag(const std::string& bag_filename, int bag_id, ros::Time start_time,
              ros::Time end_time, ros::Duration buffer_delay,
              FilteringEarlyMessageHandler filtering_early_message_handler);

  ros::Time PeekMessageTime() const;
  rosbag::MessageInstance GetNextMessage();
  bool IsMessageAvailable() const;
  std::tuple<ros::Time, ros::Time> GetBeginEndTime() const;

  int bag_id() const;

 private:
  void AdvanceOneMessage();
  void AdvanceUntilMessageAvailable();

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::const_iterator view_iterator_;
  bool finished_;
  const int bag_id_;
  const std::string bag_filename_;
  const double duration_in_seconds_;
  int log_counter_;
  std::deque<rosbag::MessageInstance> buffered_messages_;
  const ::ros::Duration buffer_delay_;
  FilteringEarlyMessageHandler filtering_early_message_handler_;
};

class PlayableBagMultiplexer {
 public:
  void AddPlayableBag(PlayableBag playable_bag);

  // Returns the next message from the multiplexed (merge-sorted) message
  // stream, along with the bag id corresponding to the message, and whether
  // this was the last message in that bag.
  std::tuple<rosbag::MessageInstance, int /* bag_id */,
             bool /* is_last_message_in_bag */>
  GetNextMessage();

  bool IsMessageAvailable() const;
  ros::Time PeekMessageTime() const;

 private:
  struct BagMessageItem {
    ros::Time message_timestamp;
    int bag_index;
    struct TimestampIsGreater {
      bool operator()(const BagMessageItem& l, const BagMessageItem& r) {
        return l.message_timestamp > r.message_timestamp;
      }
    };
  };

  std::vector<PlayableBag> playable_bags_;
  std::priority_queue<BagMessageItem, std::vector<BagMessageItem>,
                      BagMessageItem::TimestampIsGreater>
      next_message_queue_;
};


#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
