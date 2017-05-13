/// COMPONENT
#include <csapex_ros/actionlib_node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <path_msgs/NavigateToGoalAction.h>

/// SYSTEM
#include <tf/tf.h>

using namespace csapex::connection_types;

namespace csapex
{

class GoTo : public ActionlibNode<path_msgs::NavigateToGoalAction>
{
public:
    GoTo()
        : cmd_sent_(false)
    {
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        std::map<std::string, int> init_modes = {
            {"STOP", (int) path_msgs::FollowerOptions::INIT_MODE_STOP},
            {"CONTINUE", (int) path_msgs::FollowerOptions::INIT_MODE_CONTINUE}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("init_mode", init_modes, 0), init_mode_);

        std::map<std::string, int> failure_modes = {
            {"ABORT", (int) path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT},
            {"REPLAN", (int) path_msgs::NavigateToGoalGoal::FAILURE_MODE_REPLAN}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("failure_mode", failure_modes, 0), failure_mode_);

        parameters.addParameter(param::ParameterFactory::declareText("planner/algorithm", "gneric"), planning_algorithm_);
        parameters.addParameter(param::ParameterFactory::declareText("planner/topic", "plan_path"), planning_channel_);


        parameters.addParameter(param::ParameterFactory::declareText("follower/topic", "follow_path"), following_channel_);
        parameters.addParameter(param::ParameterFactory::declareText("follower/robot_controller", ""), controller_);
        parameters.addParameter(param::ParameterFactory::declareText("follower/local_planner", ""), local_planner_);
        parameters.addParameter(param::ParameterFactory::declareText("follower/collision_avoider", ""), collision_avoider_);
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        RosNode::setup(modifier);

        event_at_goal_ = modifier.addEvent("at goal");
        event_error_ = modifier.addEvent("error");

        modifier.addTypedSlot<TransformMessage>("goto", [this](const TokenPtr& token) {
            if(cmd_sent_ || token->getActivityModifier() != ActivityModifier::ACTIVATE) {
                return;
            }

            getRosHandler().waitForConnection();

            goal_ = std::dynamic_pointer_cast<TransformMessage const>(token->getTokenData());
            apex_assert(goal_);

            if(!active_client_->isServerConnected()) {
                awarn << "waiting for navigation server" << std::endl;
                active_client_->waitForServer();
            }

            path_msgs::NavigateToGoalGoal goal_msg;

            tf::poseTFToMsg(goal_->value, goal_msg.goal.pose.pose);
            goal_msg.goal.pose.header.frame_id = goal_->frame_id;
            goal_msg.goal.pose.header.stamp.fromNSec(goal_->stamp_micro_seconds * 1e3);
            goal_msg.goal.planning_algorithm.data = planning_algorithm_;
            goal_msg.goal.planning_channel.data = planning_channel_;

            goal_msg.follower_options.init_mode = init_mode_;
            goal_msg.follower_options.velocity = 0.5;
            goal_msg.follower_options.robot_controller.data = controller_;
            goal_msg.follower_options.local_planner.data = local_planner_;
            goal_msg.follower_options.collision_avoider.data = collision_avoider_;

            goal_msg.failure_mode = failure_mode_;

            ainfo << "sending goal " << goal_msg << std::endl;
            sendGoal(goal_msg);

            cmd_sent_ = true;
        });
    }

    void setupROS()
    {
        setupClient("navigate_to_goal", true);
    }

    void activation()
    {
    }

    void deactivation()
    {
        cmd_sent_ = false;
    }

    virtual void processResultCallback(const actionlib::SimpleClientGoalState&, const path_msgs::NavigateToGoalResultConstPtr& result)
    {
        goal_.reset();

        if(result->status == path_msgs::NavigateToGoalResult::STATUS_SUCCESS) {
            msg::trigger(event_at_goal_);
        } else {
            msg::trigger(event_error_);
        }
    }


private:
    Event* event_at_goal_;

    TransformMessage::ConstPtr goal_;

    std::string planning_algorithm_;
    std::string planning_channel_;

    std::string following_channel_;
    std::string controller_;
    std::string local_planner_;
    std::string collision_avoider_;

    int init_mode_;
    int failure_mode_;

    bool cmd_sent_;
};

}

CSAPEX_REGISTER_CLASS(csapex::GoTo, csapex::Node)


