#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class MergeArraysNode : public rclcpp::Node
{
public:
  MergeArraysNode() : Node("merge_arrays_node")
  {
    // Publisher
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/output/array", 10);

    // Subscribers
    sub1_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/input/array1", 10, std::bind(&MergeArraysNode::callBackArray1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/input/array2", 10, std::bind(&MergeArraysNode::callBackArray2, this, std::placeholders::_1));
  }

private:
  // Callback for array1, store msg->data in array1 when we receive it, then try to merge arrays
  void callBackArray1(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    array1_ = msg->data;
    tryMerge();
  }

  // Callback for array2, store msg->data in array2 when we receive it, then try to merge arrays
  void callBackArray2(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    array2_ = msg->data;
    tryMerge();
  }

  // Try to merge array1 and array2 and publish the merged array
  void tryMerge()
  {
    // If an array has not been received, we cannot merge yet
    if (array1_.empty() || array2_.empty())
    {
      return;
    }
    std_msgs::msg::Int32MultiArray merged_msg;
    merged_msg.data = mergeSorted(array1_, array2_);
    publisher_->publish(merged_msg);
  }

  // Helper to merge two sorted arrays into a resultant array that is also sorted
  std::vector<int32_t> mergeSorted(const std::vector<int32_t> &a, const std::vector<int32_t> &b)
  {
    std::vector<int32_t> result;
    size_t i = 0;
    size_t j = 0;
    
    // Basic approach to merging, compare a[i] and b[j], choose the smaller of the two, and increment the corresponding index
    while (i < a.size() && j < b.size())
    {
      if (a[i] < b[j])
      {
        result.push_back(a[i++]);
      }
      else
      {
        result.push_back(b[j++]);
      }
    }

    // Add any leftovers
    while (i < a.size())
    {
      result.push_back(a[i++]);
    }
    while (j < b.size())
    {
      result.push_back(b[j++]);
    }
    return result;
  }

  // ROS objects
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub2_;

  // Store arrays when received
  std::vector<int32_t> array1_;
  std::vector<int32_t> array2_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MergeArraysNode>());
    rclcpp::shutdown();
    return 0;

}