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
#include <path_msgs/FollowPathAction.h>

/// SYSTEM
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>

using namespace csapex::connection_types;

namespace csapex
{

class FollowPath : public RosNode
{
public:
    FollowPath()
    {
    }

    bool isAsynchronous() const
    {
        return true;
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareTrigger("abort"), [this](param::Parameter*){
            tearDown();
        });

        parameters.addParameter(param::ParameterFactory::declareTrigger("detach"), [this](param::Parameter*){
            detach();
        });

        std::map<std::string, int> init_modes = {
            {"STOP", (int) path_msgs::FollowPathGoal::INIT_MODE_STOP},
            {"CONTINUE", (int) path_msgs::FollowPathGoal::INIT_MODE_CONTINUE}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("init_mode", init_modes, 0), init_mode_);

        parameters.addParameter(param::ParameterFactory::declareRange("target_velocity", 0.0, 5.0, 1.0, 0.01), velocity_);

        parameters.addParameter(param::ParameterFactory::declareText("follower_topic", "follow_path"), channel_);
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        RosNode::setup(modifier);

        in_path_ = modifier.addInput<path_msgs::PathSequence>("Path");

        event_done_ = modifier.addEvent("done");
        event_error_ = modifier.addEvent("error");
    }

    void setupROS()
    {
        getRosHandler().registerShutdownCallback([this](){
            tearDown();
        });
    }

    void tearDown()
    {
        if(client) {
            aerr << "Aborting path following" << std::endl;
            client->cancelAllGoals();

            client.reset();
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        }
    }
    void detach()
    {
        if(client) {
            aerr << "Detaching path following" << std::endl;
            client->stopTrackingGoal();

            client.reset();
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        }
    }

    void goalCallback(const actionlib::SimpleClientGoalState&, const path_msgs::FollowPathResultConstPtr& result)
    {
        if(!result) {
            aerr << "Received an empty result message. Did the action server crash?" << std::endl;
            event_error_->trigger();

        } else {
            if(result->status != path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS) {
                event_error_->trigger();
            } else {
                event_done_->trigger();
            }
        }

        if(client) {
            client.reset();
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        }
    }

    void feedbackCallback(const path_msgs::FollowPathFeedbackConstPtr&)
    {

    }

    void activeCallback()
    {

    }

    void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters,  Continuation continuation) override
    {
        continuation_ = continuation;

        RosNode::process();
    }

    void processROS()
    {
        auto pos = clients_.find(channel_);
        if(pos == clients_.end()) {
            clients_[channel_] = std::make_shared<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>(channel_, true);
        }
        client = clients_.at(channel_);

        std::shared_ptr<path_msgs::PathSequence const> path = msg::getMessage<path_msgs::PathSequence>(in_path_);
        apex_assert(path);

        if(!client->isServerConnected()) {
            awarn << "waiting for path following server " << channel_ << std::endl;
            if(!client->waitForServer(ros::Duration(1.0))) {
                throw std::runtime_error("unknown path planning channel");
            }
        }

        path_msgs::FollowPathGoal goal_msg;

        goal_msg.init_mode = init_mode_;
        goal_msg.velocity = velocity_;
        goal_msg.path = *path;

//        ainfo << "sending goal " << goal_msg << std::endl;
        client->sendGoal(goal_msg,
                         boost::bind(&FollowPath::goalCallback, this, _1, _2),
                         boost::bind(&FollowPath::activeCallback, this),
                         boost::bind(&FollowPath::feedbackCallback, this, _1));
    }


private:
    Input* in_path_;

    Event* event_done_;
    Event* event_error_;

    std::string channel_;

    int init_mode_;
    double velocity_;

    std::shared_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>> client;
    std::map<std::string, std::shared_ptr<actionlib::SimpleActionClient<path_msgs::FollowPathAction>>> clients_;

    Continuation continuation_;
};

}

CSAPEX_REGISTER_CLASS(csapex::FollowPath, csapex::Node)


