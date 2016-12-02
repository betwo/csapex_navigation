/// COMPONENT
#include <csapex_ros/ros_node.h>
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

/// PROJECT
#include <path_msgs/NavigateToGoalAction.h>

/// SYSTEM
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>

using namespace csapex::connection_types;

namespace csapex
{

class GoTo : public RosNode
{
public:
    GoTo()
        : cmd_sent_(false)
    {
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        std::map<std::string, int> init_modes = {
            {"STOP", (int) path_msgs::NavigateToGoalGoal::INIT_MODE_STOP},
            {"CONTINUE", (int) path_msgs::NavigateToGoalGoal::INIT_MODE_CONTINUE}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("init_mode", init_modes, 0), init_mode_);

        std::map<std::string, int> failure_modes = {
            {"ABORT", (int) path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT},
            {"REPLAN", (int) path_msgs::NavigateToGoalGoal::FAILURE_MODE_REPLAN}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("failure_mode", failure_modes, 0), failure_mode_);

        parameters.addParameter(param::ParameterFactory::declareText("algorithm", "patsy_forward"), planning_algorithm_);
        parameters.addParameter(param::ParameterFactory::declareText("channel", "plan_path"), planning_channel_);

        parameters.addParameter(param::ParameterFactory::declareText("following_algorithm", ""), following_algorithm_);
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

            if(!client_->isServerConnected()) {
                awarn << "waiting for navigation server" << std::endl;
                client_->waitForServer();
            }

            path_msgs::NavigateToGoalGoal goal_msg;

            tf::poseTFToMsg(goal_->value, goal_msg.goal.pose.pose);
            goal_msg.goal.pose.header.frame_id = goal_->frame_id;
            goal_msg.goal.pose.header.stamp.fromNSec(goal_->stamp_micro_seconds * 1e3);
            goal_msg.goal.planning_algorithm.data = planning_algorithm_;
            goal_msg.goal.following_algorithm.data = following_algorithm_;
            goal_msg.goal.planning_channel.data = planning_channel_;

            goal_msg.init_mode = init_mode_;
            goal_msg.failure_mode = failure_mode_;

            goal_msg.velocity = 0.5;

            ainfo << "sending goal " << goal_msg << std::endl;
            client_->sendGoal(goal_msg,
                             boost::bind(&GoTo::goalCallback, this, _1, _2),
                             boost::bind(&GoTo::activeCallback, this),
                             boost::bind(&GoTo::feedbackCallback, this, _1));

            cmd_sent_ = true;
        });
    }

    void setupROS()
    {
        client_ = std::make_shared<actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction>>("navigate_to_goal", true);
    }

    void activation()
    {
    }

    void deactivation()
    {
        cmd_sent_ = false;
    }

    void goalCallback(const actionlib::SimpleClientGoalState&, const path_msgs::NavigateToGoalResultConstPtr& result)
    {
        goal_.reset();

        if(!result) {
            aerr << "Received an empty result message. Did the action server crash?" << std::endl;
            event_error_->trigger();
            return;
        }

        if(result->status == path_msgs::NavigateToGoalResult::STATUS_SUCCESS) {
            event_at_goal_->trigger();
        } else {
            event_error_->trigger();
        }
    }

    void feedbackCallback(const path_msgs::NavigateToGoalFeedbackConstPtr&)
    {

    }

    void activeCallback()
    {

    }

    void processROS()
    {
    }


private:
    Event* event_at_goal_;
    Event* event_error_;

    TransformMessage::ConstPtr goal_;

    std::string planning_algorithm_;
    std::string planning_channel_;
    std::string following_algorithm_;

    int init_mode_;
    int failure_mode_;

    std::shared_ptr<actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction>> client_;

    bool cmd_sent_;
};

}

CSAPEX_REGISTER_CLASS(csapex::GoTo, csapex::Node)


