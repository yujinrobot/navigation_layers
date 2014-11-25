#include <range_sensor_layer/extended_range_sensor_layer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/names.h>

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::ExtendedRangeSensorLayer, costmap_2d::Layer)

namespace range_sensor_layer
{

void ExtendedRangeSensorLayer::onInitialize()
{
  RangeSensorLayer::onInitialize();

  ros::NodeHandle private_nh("~/" + name_);

  XmlRpc::XmlRpcValue layer_update_topics;

  private_nh.param("layer_update_topics", layer_update_topics, layer_update_topics);

  parseExternalSubscribers(layer_update_topics, private_nh);
}

void ExtendedRangeSensorLayer::parseExternalSubscribers(XmlRpc::XmlRpcValue layer_update_topics,
                                                        ros::NodeHandle& private_nh)
{
  if (!layer_update_topics.valid() || layer_update_topics.getType() != XmlRpc::XmlRpcValue::TypeArray
      || layer_update_topics.size() < 1)
  {
    return;
  }

  std::string full_topic_name = "";

  for (unsigned int i = 0; i < layer_update_topics.size(); i++)
  {
    full_topic_name = ros::names::parentNamespace(private_nh.getNamespace()) + "/"
        + static_cast<std::string>(layer_update_topics[i]);

    external_update_subs_.push_back(
        private_nh.subscribe(full_topic_name, 100, &ExtendedRangeSensorLayer::externalMapUpdateInterface, this));
    ROS_INFO("%s: Subscribed to topic %s", name_.c_str(), external_update_subs_.back().getTopic().c_str());
  }
}

void ExtendedRangeSensorLayer::externalMapUpdateInterface(const nav_msgs::GridCellsConstPtr& cell_updates)
{
  geometry_msgs::Point cell_update;

  ROS_DEBUG("%s is getting %d cell updates", name_.c_str(), (unsigned int )cell_updates->cells.size());

  for (int i = 0; i < cell_updates->cells.size(); ++i)
  {
    cell_update = cell_updates->cells[i];
    setCost(cell_update.x, cell_update.y, (unsigned char)cell_update.z);
  }
}

} //end namespace
