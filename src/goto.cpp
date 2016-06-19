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
        parameters.addParameter(param::ParameterFactory::declareText("algorithm", "patsy_forward"), algorithm_);
        parameters.addParameter(param::ParameterFactory::declareText("channel", "plan_path"), channel_);
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        RosNode::setup(modifier);

        event_at_goal_ = modifier.addEvent("at goal");
        event_error_ = modifier.addEvent("error");

        modifier.addTypedSlot<TransformMessage>("goto", [this](const TokenPtr& token) {
            if(cmd_sent_ || !token->isActive()) {
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
            goal_msg.goal.algorithm.data = algorithm_;
            goal_msg.goal.channel.data = channel_;

            goal_msg.failure_mode = path_msgs::NavigateToGoalGoal::FAILURE_MODE_REPLAN;
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

    std::string algorithm_;
    std::string channel_;

    std::shared_ptr<actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction>> client_;

    bool cmd_sent_;
};

}

CSAPEX_REGISTER_CLASS(csapex::GoTo, csapex::Node)


