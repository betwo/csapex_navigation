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

class MoveRobot : public Node
{
public:
    MoveRobot()
        : moving_(false), has_start_pose_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_pose_ = modifier.addInput<TransformMessage>("robot pose");

        out_twist_ = modifier.addOutput<geometry_msgs::Twist>("command");

        modifier.addSlot("move", [this]() {
            moving_ = true;
            yield();
        });

        event_at_goal_ = modifier.addEvent("at goal pose");
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("distance/x", -5.0, 5.0, 0.1, 0.01), distance_x_);
        parameters.addParameter(param::ParameterFactory::declareRange("distance/y", -5.0, 5.0, 0.0, 0.01), distance_y_);

        parameters.addParameter(param::ParameterFactory::declareRange("speed/distance_factor", 0.0, 3.0, 0.6, 0.01), distance_factor_);
        parameters.addParameter(param::ParameterFactory::declareRange("speed/min", 0.0, 1.0, 0.15, 0.01), min_speed_);
        parameters.addParameter(param::ParameterFactory::declareRange("speed/max", 0.0, 1.0, 0.3, 0.01), max_speed_);
    }

    void process()
    {
        pose_ = msg::getMessage<TransformMessage>(in_pose_);

        if(!moving_) {
            return;
        }

        tf::Pose pose = pose_->value ;

        if(!has_start_pose_) {
            start_pose_ = pose;
            has_start_pose_= true;
        }

        tf::Transform delta = start_pose_.inverse() * pose;

        double distance_max = std::hypot(distance_x_, distance_y_);

        double distance = delta.getOrigin().length();

        geometry_msgs::Twist twist;

        double error_x = distance_x_ - delta.getOrigin().x();
        double error_y = distance_y_ - delta.getOrigin().y();

        bool overshoot = distance > distance_max;

        bool pos_good = (std::max(std::abs(error_x), std::abs(error_y)) < 0.01 )|| overshoot;


        if(pos_good) {
            msg::trigger(event_at_goal_);
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;

            has_start_pose_ = false;
            moving_ = false;

        } else {
            twist.linear.x = error_x * distance_factor_;
            twist.linear.y = error_y * distance_factor_;

            double speed = std::hypot(twist.linear.x, twist.linear.y);

            if(speed > max_speed_) {
                double correction = max_speed_ / speed;
                twist.linear.x *= correction;
                twist.linear.y *= correction;

            } else if(speed < min_speed_) {
                double correction = min_speed_ / speed;
                twist.linear.x *= correction;
                twist.linear.y *= correction;
            }
        }

        twist.angular.z = 0;

        msg::publish(out_twist_, std::make_shared<geometry_msgs::Twist>(twist));
    }


private:
    Input* in_pose_;
    Output* out_twist_;

    Event* event_at_goal_;

    TransformMessage::ConstPtr pose_;

    bool moving_;

    bool has_start_pose_;
    tf::Pose start_pose_;

    double distance_factor_;

    double distance_x_;
    double distance_y_;

    double max_speed_;
    double min_speed_;
};

}

CSAPEX_REGISTER_CLASS(csapex::MoveRobot, csapex::Node)


