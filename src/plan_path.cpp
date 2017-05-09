/// COMPONENT
#include <csapex_ros/actionlib_node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <path_msgs/PlanPathAction.h>

/// SYSTEM
#include <tf/tf.h>

using namespace csapex::connection_types;

namespace csapex
{

class PlanPath : public ChanneledActionlibNode<path_msgs::PlanPathAction>
{
public:
    PlanPath()
    {
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        ActionlibNode<path_msgs::PlanPathAction>::setupParameters(parameters);

        parameters.addParameter(param::ParameterFactory::declareText("algorithm", "generic"), algorithm_);
        parameters.addParameter(param::ParameterFactory::declareText("planner_topic", "plan_path"), channel_);

        parameters.addParameter<decltype(options_.grow_obstacles),bool>(param::ParameterFactory::declareBool
                                                                        ("grow_obstacles", true), options_.grow_obstacles);
        parameters.addConditionalParameter<decltype(options_.obstacle_growth_radius),double>(param::ParameterFactory::declareRange(
                                                                                 "grow_obstacles/radius", 0.0, 10.0, 1.0, 0.01),
                                                                                 [this]() -> bool {
                                                                                     return options_.grow_obstacles;
                                                                                 }, options_.obstacle_growth_radius);

        parameters.addParameter<decltype(options_.max_search_duration),double>(param::ParameterFactory::declareRange(
                                                                                       "max_search_duration", 0.0, 30.0, 10.0, 0.01),
                                                                                   options_.max_search_duration);

        auto cond_generic = [this](){
            return algorithm_ == "generic";
        };

        parameters.addConditionalParameter<decltype(options_.reversed),bool>(
                    param::ParameterFactory::declareValue("planner/reversed", true),
                    cond_generic, options_.reversed);
        parameters.addConditionalParameter<decltype(options_.goal_dist_threshold),double>(
                    param::ParameterFactory::declareValue("planner/goal_dist_threshold", 0.15),
                    cond_generic, options_.goal_dist_threshold);
        parameters.addConditionalParameter<decltype(options_.goal_angle_threshold_degree),double>(
                    param::ParameterFactory::declareValue("planner/goal_angle_threshold", 25.0),
                    cond_generic, options_.goal_angle_threshold_degree);
        parameters.addConditionalParameter<decltype(options_.allow_forward),bool>(
                    param::ParameterFactory::declareValue("planner/allow_forward", true),
                    cond_generic, options_.allow_forward);
        parameters.addConditionalParameter<decltype(options_.allow_backward),bool>(
                    param::ParameterFactory::declareValue("planner/allow_backward", false),
                    cond_generic, options_.allow_backward);
        parameters.addConditionalParameter<decltype(options_.ackermann_la),double>(
                    param::ParameterFactory::declareValue("planner/ackermann_la", 1.2),
                    cond_generic, options_.ackermann_la);
        parameters.addConditionalParameter<decltype(options_.ackermann_steer_steps),int>(
                    param::ParameterFactory::declareValue("planner/ackermann_steer_steps", 2),
                    cond_generic, options_.ackermann_steer_steps);
        parameters.addConditionalParameter<decltype(options_.ackermann_max_steer_angle_degree),double>(
                    param::ParameterFactory::declareValue("planner/ackermann_max_steer_angle", 40.0),
                    cond_generic, options_.ackermann_max_steer_angle_degree);
        parameters.addConditionalParameter<decltype(options_.ackermann_steer_delta_degree),double>(
                    param::ParameterFactory::declareValue("planner/ackermann_steer_delta", 20.0),
                    cond_generic, options_.ackermann_steer_delta_degree);
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        ActionlibNode<path_msgs::PlanPathAction>::setup(modifier);

        in_goal_ = modifier.addInput<TransformMessage>("Goal");
        in_pose_ = modifier.addOptionalInput<TransformMessage>("Start");
        out_path_ = modifier.addOutput<path_msgs::PathSequence>("Path");
    }

    void setupROS()
    {
        ActionlibNode<path_msgs::PlanPathAction>::setupROS();

        debug_pub = getRosHandler().nh()->advertise<path_msgs::PlanPathGoal>("/goal_debug2", 1, true);
    }

    void processResultCallback(const actionlib::SimpleClientGoalState& state, const boost::shared_ptr<typename path_msgs::PlanPathResult const>& result) override
    {
        if(!result->path.paths.empty()) {
            msg::publish(out_path_, std::make_shared<path_msgs::PathSequence>(result->path));
        } else {
            msg::trigger(event_error_);
        }
    }

    std::string getChannel() const override
    {
        return channel_;
    }

    void getGoal(path_msgs::PlanPathGoal& goal) override
    {
        if(msg::hasMessage(in_pose_)) {
            start_ = msg::getMessage<TransformMessage>(in_pose_);
        } else {
            start_.reset();
        }

        goal_ = msg::getMessage<TransformMessage>(in_goal_);
        apex_assert(goal_);

        goal.goal.type = path_msgs::Goal::GOAL_TYPE_POSE;

        tf::poseTFToMsg(goal_->value, goal.goal.pose.pose);

        goal.goal.pose.header.frame_id = goal_->frame_id;
        goal.goal.pose.header.stamp.fromNSec(goal_->stamp_micro_seconds * 1e3);
        goal.goal.planning_algorithm.data = algorithm_;
        goal.goal.planning_channel.data = channel_;
        goal.options = options_;

        if(start_) {
            goal.use_start = true;
            tf::poseTFToMsg(start_->value, goal.start.pose);

        } else {
            goal.use_start = false;
        }

        debug_pub.publish(goal);
    }

private:
    Input* in_goal_;
    Input* in_pose_;

    Output* out_path_;

    path_msgs::PlannerOptions options_;

    TransformMessage::ConstPtr start_;
    TransformMessage::ConstPtr goal_;

    std::string algorithm_;
    std::string channel_;

    ros::Publisher debug_pub;
};

}

CSAPEX_REGISTER_CLASS(csapex::PlanPath, csapex::Node)


