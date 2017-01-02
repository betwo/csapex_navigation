
/// PROJECT
#include <csapex_ros/ros_node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <geometry_msgs/PointStamped.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class SetLookatPoint : public RosNode
{
public:
    SetLookatPoint()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        RosNode::setup(modifier);

        in_pose_ = modifier.addInput<TransformMessage>("Input");
    }

    void setupROS() override
    {
        pub_ = getRosHandler().nh()->advertise<geometry_msgs::PointStamped>("/look_at", 10, false);
    }

    void processROS() override
    {
        auto pose_m = msg::getMessage<TransformMessage>(in_pose_);
        tf::Transform pose = pose_m->value;

        geometry_msgs::PointStamped lookat;
        lookat.point.x = pose.getOrigin().x();
        lookat.point.y = pose.getOrigin().y();
        lookat.point.z = 0;

        lookat.header.frame_id = pose_m->frame_id;
        lookat.header.stamp.fromNSec(pose_m->stamp_micro_seconds * 1e3);

        if(lookat.header.frame_id.empty()) {
            node_handle_->setWarning("No frame set for lookat point, using 'map'");
            lookat.header.frame_id = "map";
        }

        pub_.publish(lookat);
    }

private:
    Input* in_pose_;

    ros::Publisher pub_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::SetLookatPoint, csapex::Node)

