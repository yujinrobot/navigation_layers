#ifndef RANGE_SENSOR_LAYER_EXTENDED_RANGE_SENSOR_LAYER_H_
#define RANGE_SENSOR_LAYER_EXTENDED_RANGE_SENSOR_LAYER_H_

#include <range_sensor_layer/range_sensor_layer.h>
#include <nav_msgs/GridCells.h>

namespace range_sensor_layer
{

class ExtendedRangeSensorLayer : public RangeSensorLayer
{
public:
  virtual void onInitialize();

protected:
  void parseExternalSubscribers(XmlRpc::XmlRpcValue layer_update_topics, ros::NodeHandle& private_nh);
  void externalMapUpdateInterface(const nav_msgs::GridCellsConstPtr& cell_updates);

  std::vector<ros::Subscriber> external_update_subs_;
};

}//end namespace

#endif /* RANGE_SENSOR_LAYER_EXTENDED_RANGE_SENSOR_LAYER_H_ */
