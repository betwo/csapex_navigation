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

/// PROJECT
#include <csapex_navigation/pid.hpp>

using namespace csapex::connection_types;

namespace csapex
{

class ApproachPoseWithTrailer : public Node
{
public:
    ApproachPoseWithTrailer()
        : has_last_goal_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_pose_ = modifier.addInput<TransformMessage>("robot pose");
        in_pose_trailer_  = modifier.addInput<TransformMessage>("trailer pose (relative)");

        out_twist_ = modifier.addOutput<geometry_msgs::Twist>("command");
        out_marker_ = modifier.addOutput<visualization_msgs::MarkerArray>("markers");

        event_at_goal_ = modifier.addEvent("at goal");
        event_error_ = modifier.addEvent("error");

        modifier.addTypedSlot<TransformMessage>("odom pose", [this](const TokenPtr& token) {
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
        parameters.addParameter(param::ParameterFactory::declareRange("distance", -2.0, 2.0, 0.0, 0.01), distance_);

        parameters.addParameter(param::ParameterFactory::declareRange("trailer/length", 0.0, 2.0, 1.2, 0.01), trailer_length_);

        parameters.addParameter(param::ParameterFactory::declareRange("min_distance_to_robot", 0.1, 1.0, 0.8, 0.01), min_distance_to_robot_);
        parameters.addParameter(param::ParameterFactory::declareRange("min_distance", 0.0, 3.0, 2.0, 0.01), min_offset_distance_);

        parameters.addParameter(param::ParameterFactory::declareRange("speed/min", 0.0, 1.0, 0.15, 0.01), min_speed_);
        parameters.addParameter(param::ParameterFactory::declareRange("speed/max", 0.0, 1.0, 0.3, 0.01), max_speed_);

        parameters.addParameter(param::ParameterFactory::declareRange("error/okay", 0.0, 1.0, 0.06, 0.01), error_okay_);
        parameters.addParameter(param::ParameterFactory::declareRange("error/max", 0.0, 1.0, 0.16, 0.01), error_max_);

        parameters.addParameter(param::ParameterFactory::declareRange("allowed_variation", 0.0, 1.0, 0.5, 0.01), allowed_variation_);

        parameters.addParameter(param::ParameterFactory::declareRange("steer/max", 0.0, M_PI/2, M_PI/2, 0.001), max_psi_);

        parameters.addParameter(param::ParameterFactory::declareRange("steer/p", 0.0, 2.0, 0.93, 0.001),
                                [this](param::Parameter* p) {
            pid_psi_.setKP(p->as<double>());
        });
        parameters.addParameter(param::ParameterFactory::declareRange("steer/i", 0.0, 1.0, 0.2, 0.0001),
                                [this](param::Parameter* p) {
            pid_psi_.setKI(p->as<double>());
        });
        parameters.addParameter(param::ParameterFactory::declareRange("steer/d", 0.0, 1.0, 0.1, 0.0001),
                                [this](param::Parameter* p) {
            pid_psi_.setKD(p->as<double>());
        });
    }

    void process()
    {
        if(!goal_) {
            awarn << "no goal set!" << std::endl;
            return;
        }

        pose_ = msg::getMessage<TransformMessage>(in_pose_);

        auto pose_trailer_ = msg::getMessage<TransformMessage>(in_pose_trailer_);
        base_to_trailer_link_ = pose_trailer_->value;

        tf::Transform trailer_link_to_trailer_back(tf::createIdentityQuaternion(), tf::Vector3(-trailer_length_, 0, 0));
        tf::Pose base_to_trailer_back = base_to_trailer_link_ * trailer_link_to_trailer_back;

        tf::Pose odom_to_base_goal = goal_->value;
        last_goal_ = odom_to_base_goal;
        has_last_goal_= true;


        tf::Transform odom_to_base_link = pose_->value;
        tf::Transform goal_offset(tf::createIdentityQuaternion(), tf::Vector3(distance_, 0, 0));
        tf::Pose base_link_to_goal = odom_to_base_link.inverse() * odom_to_base_goal * goal_offset;

        // offset_moves
        double offset_distance = distance_;


        tf::Pose base_link_to_base_goal = base_link_to_goal;
        while(offset_distance > -min_offset_distance_) {
            tf::Transform offset(tf::createIdentityQuaternion(), tf::Vector3(offset_distance, 0, 0));
            base_link_to_base_goal = base_link_to_goal * offset;
            if(base_link_to_base_goal.getOrigin().length() < min_distance_to_robot_) {
                break;
            }

            offset_distance -= 0.05;
        }

        tf::Pose base_link_to_trailer_goal = base_link_to_base_goal * trailer_link_to_trailer_back;

//        tf::Transform trailer_error = trailer_back_pose.inverse() * trailer_goal_pose;
        tf::Transform trailer_back_to_trailer_goal = base_to_trailer_back.inverse() * base_link_to_trailer_goal;

        //base_link_to_base_goal.setRotation(tf::createQuaternionFromYaw(std::atan2(base_link_to_base_goal.getOrigin().y(), base_link_to_base_goal.getOrigin().x())));


        driveToPose(base_link_to_base_goal, trailer_back_to_trailer_goal, base_link_to_goal);

        if(msg::isConnected(out_marker_)) {
            std::shared_ptr<visualization_msgs::MarkerArray> markers = std::make_shared<visualization_msgs::MarkerArray>();

            {
                visualization_msgs::Marker line_marker = makeMarker(0.0, 0.7, 0.0, "line", 2);
                line_marker.header.frame_id = "/base_link";
                line_marker.header.stamp = ros::Time::now();
                geometry_msgs::Point start, end;
                tf::Transform goal_to_goal_front(tf::createIdentityQuaternion(), tf::Vector3(10, 0, 0));
                tf::Pose line_start = base_link_to_goal * goal_to_goal_front;
                start.x = line_start.getOrigin().x();
                start.y = line_start.getOrigin().y();
                tf::Pose line_end = base_link_to_goal * goal_to_goal_front.inverse();
                end.x = line_end.getOrigin().x();
                end.y = line_end.getOrigin().y();
                line_marker.points.push_back(start);
                line_marker.points.push_back(end);
                line_marker.type = visualization_msgs::Marker::LINE_STRIP;
                line_marker.scale.x = 0.025;
                line_marker.scale.y = 0.075;
                line_marker.scale.z = 0;

                markers->markers.push_back(line_marker);
            }

            {
                visualization_msgs::Marker orientation_marker = makeMarker(0.0, 1.0, 0.0, "error", 1);

                orientation_marker.header.frame_id = "/base_link";
                orientation_marker.header.stamp = ros::Time::now();
                orientation_marker.pose.position.x = base_link_to_base_goal.getOrigin().x();
                orientation_marker.pose.position.y = base_link_to_base_goal.getOrigin().y();
                tf::quaternionTFToMsg(base_link_to_base_goal.getRotation(), orientation_marker.pose.orientation);
                orientation_marker.type = visualization_msgs::Marker::ARROW;
                orientation_marker.scale.x = 0.5;
                orientation_marker.scale.y = 0.075;
                orientation_marker.scale.z = 0.075;

                markers->markers.push_back(orientation_marker);
            }

            {
                visualization_msgs::Marker error_marker = makeMarker(1.0, 0.0, 0.0, "error", 0);
                error_marker.header.frame_id = "/base_link";
                error_marker.header.stamp = ros::Time::now();
                geometry_msgs::Point arrow_start, arrow_end;
                arrow_end.x = base_link_to_base_goal.getOrigin().x();
                arrow_end.y = base_link_to_base_goal.getOrigin().y();
                error_marker.points.push_back(arrow_start);
                error_marker.points.push_back(arrow_end);
                error_marker.type = visualization_msgs::Marker::ARROW;
                error_marker.scale.x = 0.025;
                error_marker.scale.y = 0.075;
                error_marker.scale.z = 0;

                markers->markers.push_back(error_marker);
            }

            {
                visualization_msgs::Marker error_marker = makeMarker(1.0, 1.0, 0.0, "error_trailer", 3);
                error_marker.header.frame_id = "/base_link";
                error_marker.header.stamp = ros::Time::now();

                geometry_msgs::Point arrow_start, arrow_end;
                arrow_start.x = base_to_trailer_back.getOrigin().x();
                arrow_start.y = base_to_trailer_back.getOrigin().y();
                arrow_end.x = base_link_to_trailer_goal.getOrigin().x();
                arrow_end.y = base_link_to_trailer_goal.getOrigin().y();
                error_marker.points.push_back(arrow_start);
                error_marker.points.push_back(arrow_end);
                error_marker.scale.x = 0.025;
                error_marker.scale.y = 0.075;
                error_marker.scale.z = 0;

                error_marker.type = visualization_msgs::Marker::ARROW;

//                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(trailer_back_to_trailer_goal.getOrigin().angle(tf::Vector3(1,0,0))), error_marker.pose.orientation);
//                tf::pointTFToMsg(base_to_trailer_back.getOrigin(), error_marker.pose.position);

//                error_marker.scale.x = trailer_back_to_trailer_goal.getOrigin().length();
//                error_marker.scale.y = 0.025;
//                error_marker.scale.z = 0.025;


                markers->markers.push_back(error_marker);
            }

            msg::publish(out_marker_, markers);
        }
    }

    void driveToPose(const tf::Pose &base_link_to_base_goal, const tf::Pose& trailer_back_to_trailer_goal, const tf::Pose &base_link_to_goal)
    {
        tf::Pose error;

        tf::Transform goal_to_base_link = base_link_to_goal.inverse();
        double goal_ex = std::abs(goal_to_base_link.getOrigin().x());

        tf::Transform trailer_goal_t_trailer_back = trailer_back_to_trailer_goal.inverse();

        bool position_trailer = goal_ex > trailer_length_ && std::abs(trailer_goal_t_trailer_back.getOrigin().y()) > trailer_length_ * 0.2;

        if(position_trailer) {
            error = trailer_back_to_trailer_goal;
            double error_yaw = std::atan2(error.getOrigin().y(), error.getOrigin().x());
            ainfo << "trailer error yaw: " << error_yaw << std::endl;
        } else {
            error = base_link_to_base_goal;
        }

        geometry_msgs::Twist twist;
        //    double error_yaw = tf::getYaw(error.getRotation());

        bool pos_good = goal_ex < error_okay_ || (goal_ex < error_max_ && goal_ex > last_error_pos_);
        last_error_pos_ = goal_ex;

        double error_yaw;
        if(goal_ex  >= min_offset_distance_) {
            error_yaw = std::atan2(error.getOrigin().y(), error.getOrigin().x());
            if(!position_trailer) {
                error_yaw *= 1.5;
            } else {
                double tey = trailer_goal_t_trailer_back.getOrigin().y();
                if(std::abs(tey) > trailer_length_ / 2.0) {
                    error_yaw = 2.0 * std::atan2(trailer_back_to_trailer_goal.getOrigin().y(), trailer_back_to_trailer_goal.getOrigin().x());
                } else {
                    error_yaw = 0.5 * tey;
                }
                ainfo << "trailer error: " << error_yaw << std::endl;
            }
        } else {
            ainfo << "ey: " << goal_to_base_link.getOrigin().y() << std::endl;
            error_yaw = 4.0 * goal_to_base_link.getOrigin().y();
        }

        if(pos_good) {
            event_at_goal_->trigger();

            has_last_goal_ = false;

        } else {
            double ex = error.getOrigin().x();
            twist.linear.x = ex * 0.6;
            double speed = std::abs(twist.linear.x);

            if(speed > max_speed_) {
                twist.linear.x *= max_speed_ / speed;
                twist.linear.y *= max_speed_ / speed;
            } else if(speed < min_speed_) {
                twist.linear.x *= min_speed_ / speed;
                twist.linear.y *= min_speed_ / speed;
            }

            //            double psi = std::pow(1.75 * error_yaw, 2) * (error_yaw >= 0 ? 1.0 : -1.0);
            double psi = error_yaw;


            if(std::abs(psi) > max_psi_) {
                psi = psi / std::abs(psi) * max_psi_;
            }

            twist.angular.z = pid_psi_.regulate(psi);
        }

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
    Input* in_pose_trailer_;
    Output* out_twist_;
    Output* out_marker_;

    Event* event_at_goal_;
    Event* event_error_;

    TransformMessage::ConstPtr pose_;
    TransformMessage::ConstPtr goal_;

    tf::Transform base_to_trailer_link_;

    bool has_last_goal_;
    tf::Pose last_goal_;

    double distance_;

    double trailer_length_;

    double last_error_pos_;

    double error_max_;
    double error_okay_;

    double min_distance_to_robot_;
    double min_offset_distance_;

    double max_speed_;
    double min_speed_;

    double max_psi_;

    double allowed_variation_;

    PID<double> pid_psi_;
};

}

CSAPEX_REGISTER_CLASS(csapex::ApproachPoseWithTrailer, csapex::Node)


