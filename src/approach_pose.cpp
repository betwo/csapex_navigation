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
#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

using namespace csapex::connection_types;

namespace csapex
{

class ApproachPose : public Node
{
public:
    ApproachPose()
        : has_last_goal_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_pose_ = modifier.addInput<TransformMessage>("robot pose");

        out_twist_ = modifier.addOutput<geometry_msgs::Twist>("command");
        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("markers");

        event_at_goal_ = modifier.addEvent("at goal");
        event_error_ = modifier.addEvent("error");

        modifier.addSlot<TransformMessage>("odom pose", [this](const TokenPtr& token) {
            TransformMessage::ConstPtr new_goal_ = std::dynamic_pointer_cast<TransformMessage const>(token->getTokenData());
            apex_assert(new_goal_);

            if(has_last_goal_) {
                double distance_to_last = new_goal_->value.getOrigin().distance(last_goal_.getOrigin());
                if(distance_to_last > allowed_variation_) {
                    return;
                }
            }

            goal_ = new_goal_;

        });


        node_handle_->setProcessingEnabled(false);
    }

    void activation()
    {
        last_error_pos_ = std::numeric_limits<double>::infinity();
        node_handle_->setProcessingEnabled(true);
    }

    void deactivation()
    {
        node_handle_->setProcessingEnabled(false);
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareRange("distance", -2.0, 2.0, 0.0, 0.01), distance_);

        parameters.addParameter(param::factory::declareRange("min_distance_to_robot", 0.1, 1.0, 0.4, 0.01), min_distance_to_robot_);
        parameters.addParameter(param::factory::declareRange("min_distance", 0.0, 3.0, 1.5, 0.01), min_offset_distance_);

        parameters.addParameter(param::factory::declareRange("speed/min", 0.0, 1.0, 0.15, 0.01), min_speed_);
        parameters.addParameter(param::factory::declareRange("speed/max", 0.0, 1.0, 0.3, 0.01), max_speed_);

        parameters.addParameter(param::factory::declareRange("allowed_variation", 0.0, 1.0, 0.5, 0.01), allowed_variation_);

        parameters.addParameter(param::factory::declareRange("error/okay", 0.0, 1.0, 0.07, 0.01), error_okay_);
        parameters.addParameter(param::factory::declareRange("error/max", 0.0, 1.0, 0.15, 0.01), error_max_);

        parameters.addParameter(param::factory::declareRange("steer/max", 0.0, M_PI/2, M_PI/8, 0.001), max_psi_);
    }

    void process()
    {
        if(!goal_) {
            awarn << "no goal set!" << std::endl;
            return;
        }

        pose_ = msg::getMessage<TransformMessage>(in_pose_);

        tf::Pose pose_object_odom = goal_->value;
        last_goal_ = pose_object_odom;
        has_last_goal_= true;


        tf::Transform odom_to_base_link = pose_->value;
        tf::Pose pose_object_base_link = odom_to_base_link * pose_object_odom;

        // offset_moves
        double offset_distance = distance_;


        tf::Transform goal_offset(tf::createIdentityQuaternion(), tf::Vector3(distance_, 0, 0));
        tf::Pose goal_error = pose_object_base_link * goal_offset;

        tf::Pose error = goal_error;
        while(offset_distance > -min_offset_distance_) {
            tf::Transform offset(tf::createIdentityQuaternion(), tf::Vector3(offset_distance, 0, 0));
            error = pose_object_base_link * offset;
            if(error.getOrigin().length() < min_distance_to_robot_) {
                break;
            }

            offset_distance -= 0.05;
        }

        error.setRotation(tf::createQuaternionFromYaw(std::atan2(error.getOrigin().y(), error.getOrigin().x())));

        driveToPose(error, goal_error);

        if(msg::isConnected(out_marker_)) {
            visualization_msgs::Marker orientation_marker = makeMarker(0.0, 1.0, 0.0, "error", 1);

            orientation_marker.header.frame_id = "/base_link";
            orientation_marker.pose.position.x = error.getOrigin().x();
            orientation_marker.pose.position.y = error.getOrigin().y();
            tf::quaternionTFToMsg(error.getRotation(), orientation_marker.pose.orientation);
            orientation_marker.type = visualization_msgs::Marker::ARROW;
            orientation_marker.scale.x = 0.5;
            orientation_marker.scale.y = 0.075;
            orientation_marker.scale.z = 0.075;

            std::shared_ptr<visualization_msgs::MarkerArray> markers = std::make_shared<visualization_msgs::MarkerArray>();
            markers->markers.push_back(orientation_marker);

            visualization_msgs::Marker error_marker = makeMarker(1.0, 0.0, 0.0, "error", 0);
            error_marker.header.frame_id = "/base_link";
            geometry_msgs::Point start, end;
            end.x = error.getOrigin().x();
            end.y = error.getOrigin().y();
            error_marker.points.push_back(start);
            error_marker.points.push_back(end);
            error_marker.type = visualization_msgs::Marker::ARROW;
            error_marker.scale.x = 0.025;
            error_marker.scale.y = 0.075;
            error_marker.scale.z = 0;

            markers->markers.push_back(error_marker);

            msg::publish(out_marker_, markers);
        }
    }


    void driveToPose(const tf::Pose &error, const tf::Pose &goal_error)
    {
        geometry_msgs::Twist twist;
        //    double error_yaw = tf::getYaw(error.getRotation());

        double e = std::hypot(error.getOrigin().x(), error.getOrigin().y());
        double ge = std::hypot(goal_error.getOrigin().x(), goal_error.getOrigin().y());

        bool pos_good = ge < error_okay_ || (e < error_max_ && e > last_error_pos_);

        double error_yaw = std::atan2(error.getOrigin().y(), error.getOrigin().x());

        if(std::abs(error.getOrigin().y()) < 0.1) {
            error_yaw = tf::getYaw(error.getRotation());
        }

        if(std::abs(ge) < 0.05) {
            error_yaw = 0;
        }

        if(pos_good) {
            msg::trigger(event_at_goal_);

            has_last_goal_ = false;

        } else {
            double ex = error.getOrigin().x() ;
//            double ey = error.getOrigin().y();
            twist.linear.x = ex * 0.6;
            double speed = std::abs(twist.linear.x);

            double max_psi = max_psi_ * (1 + ge / 2.0);

            if(speed > max_speed_) {
                twist.linear.x *= max_speed_ / speed;
                twist.linear.y *= max_speed_ / speed;
            } else if(speed < min_speed_) {
                twist.linear.x *= min_speed_ / speed;
                twist.linear.y *= min_speed_ / speed;
            }

//            double psi = std::pow(1.75 * error_yaw, 2) * (error_yaw >= 0 ? 1.0 : -1.0);
            double psi = error_yaw;


            if(std::abs(psi) > max_psi) {
                psi = psi / std::abs(psi) * max_psi;
            }

            twist.angular.z = psi;
        }

        last_error_pos_ = ge;

        msg::publish(out_twist_, std::make_shared<geometry_msgs::Twist>(twist));
    }

    static visualization_msgs::Marker makeMarker(float r, float g, float b, const std::string& ns, int id)
    {
        visualization_msgs::Marker marker;
        marker.ns = ns;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = id;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.type = visualization_msgs::Marker::CUBE;

        return marker;
    }


private:
    Input* in_pose_;
    Output* out_twist_;
    Output* out_marker_;

    Event* event_at_goal_;
    Event* event_error_;

    TransformMessage::ConstPtr pose_;
    TransformMessage::ConstPtr goal_;

    bool has_last_goal_;
    tf::Pose last_goal_;

    double distance_;
    double last_error_pos_;

    double error_max_;
    double error_okay_;

    double min_distance_to_robot_;
    double min_offset_distance_;

    double max_speed_;
    double min_speed_;

    double max_psi_;

    double allowed_variation_;
};

}

CSAPEX_REGISTER_CLASS(csapex::ApproachPose, csapex::Node)


