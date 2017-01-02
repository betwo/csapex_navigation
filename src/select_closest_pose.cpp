
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_ros/tf_listener.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <geometry_msgs/PoseArray.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class SelectClosestPose : public Node
{
public:
    SelectClosestPose()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_pose_ = modifier.addInput<TransformMessage>("Pose");
        in_ = modifier.addInput<geometry_msgs::PoseArray>("Input");
        out_pose_ = modifier.addOutput<geometry_msgs::Pose>("Output");
        out_trafo_ = modifier.addOutput<TransformMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto poses = msg::getMessage<geometry_msgs::PoseArray>(in_);
        if(!poses->poses.empty()) {

            auto pose_m = msg::getMessage<TransformMessage>(in_pose_);
            tf::Transform pose = pose_m->value;

            if(pose_m->frame_id != poses->header.frame_id) {
                LockedTFListener l = TFListener::getLocked();
                apex_assert(l.l);
                auto listener = l.l->tfl;
                apex_assert(listener);
                tf::TransformListener& tfl = *listener;

                ros::Time time = poses->header.stamp;
                if(!tfl.waitForTransform(poses->header.frame_id, pose_m->frame_id, time, ros::Duration(0.1))) {
                    node_handle_->setError(std::string("cannot transform pose from ") + poses->header.frame_id +
                                           " to " + pose_m->frame_id + " at time " + std::to_string(time.toSec()));

                    time = ros::Time(0);
                    if(!tfl.waitForTransform(poses->header.frame_id, pose_m->frame_id, time, ros::Duration(10))) {
                       return;
                    }
                }

                tf::StampedTransform poses_T_pose;
                tfl.lookupTransform(poses->header.frame_id, pose_m->frame_id, time, poses_T_pose);
                pose = poses_T_pose * pose;
            }

            geometry_msgs::Pose closest;
            tf::Transform closest_pose;
            double minimum_distance = std::numeric_limits<double>::max();

            double rx = pose.getOrigin().x();
            double ry = pose.getOrigin().y();

            for(const geometry_msgs::Pose& pose : poses->poses) {
                double distance = std::hypot(pose.position.x - rx, pose.position.y - ry);
                if(distance < minimum_distance) {
                    minimum_distance = distance;
                    closest = pose;
                    tf::poseMsgToTF(closest, closest_pose);
                }
            }

            msg::publish(out_pose_, std::make_shared<geometry_msgs::Pose>(closest));

            auto closest_pose_msg = std::make_shared<TransformMessage>(poses->header.frame_id, "closest");
            closest_pose_msg->value = closest_pose;
            msg::publish(out_trafo_, closest_pose_msg);
        }
    }

private:
    Input* in_pose_;
    Input* in_;
    Output* out_pose_;
    Output* out_trafo_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::SelectClosestPose, csapex::Node)

