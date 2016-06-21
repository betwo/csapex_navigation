/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/model/token.h>
#include <csapex/signal/event.h>
#include <csapex_transform/transform_message.h>
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

using namespace csapex::connection_types;

namespace csapex
{

class BackUp : public Node
{
public:
    BackUp()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_pose_ = modifier.addInput<TransformMessage>("robot pose");

        out_twist_ = modifier.addOutput<geometry_msgs::Twist>("command");

        event_at_goal_ = modifier.addEvent("at goal");

        node_handle_->setProcessingEnabled(false);
    }

    void activation()
    {
        last_error_pos_ = std::numeric_limits<double>::infinity();
        has_start_pose_ = false;

        node_handle_->setProcessingEnabled(true);
    }

    void deactivation()
    {
        node_handle_->setProcessingEnabled(false);
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("distance", -5.0, 5.0, 0.0, 0.01), distance_);

        parameters.addParameter(param::ParameterFactory::declareRange("speed/min", 0.0, 1.0, 0.15, 0.01), min_speed_);
        parameters.addParameter(param::ParameterFactory::declareRange("speed/max", 0.0, 1.0, 0.3, 0.01), max_speed_);
    }

    void process()
    {
        pose_ = msg::getMessage<TransformMessage>(in_pose_);

        tf::Pose pose = pose_->value ;

        if(!has_start_pose_) {
            start_pose_ = pose;
        }
        has_start_pose_= true;

        tf::Transform delta = start_pose_.inverse() * pose;

        double distance = delta.getOrigin().length();
        if(distance_ < 0) {
            distance *= -1.0;
        }

        geometry_msgs::Twist twist;

        double error = distance_ - distance;

        bool overshoot = distance_ > 0.0 ? (distance > distance_) : (distance < distance_);

        bool pos_good = std::abs(error) < 0.01 || overshoot;


        if(pos_good) {
            event_at_goal_->trigger();
            twist.linear.x = 0.0;

        } else {
            twist.linear.x = error * 0.6;

            double speed = std::abs(twist.linear.x);

            if(speed > max_speed_) {
                twist.linear.x *= max_speed_ / speed;
            } else if(speed < min_speed_) {
                twist.linear.x *= min_speed_ / speed;
            }
        }

        last_error_pos_ = error;

        twist.linear.y = 0;
        twist.angular.z = 0;

        msg::publish(out_twist_, std::make_shared<geometry_msgs::Twist>(twist));
    }


private:
    Input* in_pose_;
    Output* out_twist_;

    Event* event_at_goal_;

    TransformMessage::ConstPtr pose_;

    bool has_start_pose_;
    tf::Pose start_pose_;

    double last_error_pos_;

    double distance_;

    double max_speed_;
    double min_speed_;
};

}

CSAPEX_REGISTER_CLASS(csapex::BackUp, csapex::Node)


