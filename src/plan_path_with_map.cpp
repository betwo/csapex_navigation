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
#include <path_msgs/PlanPathAction.h>

/// SYSTEM
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>

using namespace csapex::connection_types;

namespace csapex
{

class PlanPathWithMap : public RosNode
{
public:
    PlanPathWithMap()
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

        parameters.addParameter(param::ParameterFactory::declareText("algorithm", "patsy_forward"), algorithm_);
        parameters.addParameter(param::ParameterFactory::declareText("planner_topic", "plan_path"), channel_);

        parameters.addParameter(param::ParameterFactory::declareRange("min_distance", 0.0, 10.0, 2.0, 0.01), min_dist_);

        parameters.addParameter(param::ParameterFactory::declareBool("grow_obstacles", true), grow_obstacles_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("grow_obstacles/radius", 0.0, 10.0, 1.0, 0.01), grow_obstacles_, obstacle_growth_);

        parameters.addParameter(param::ParameterFactory::declareRange("max_search_duration", 0.0, 30.0, 10.0, 0.01), max_search_duration_);

        parameters.addParameter(param::ParameterFactory::declareOutputText("feedback"));
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        RosNode::setup(modifier);

        in_goal_ = modifier.addInput<nav_msgs::OccupancyGrid>("Goal");
        in_pose_ = modifier.addOptionalInput<TransformMessage>("Start");
        out_path_ = modifier.addOutput<path_msgs::PathSequence>("Path");

        event_error_ = modifier.addEvent("error");
    }

    void setupROS()
    {
        connection_ = getRosHandler().shutdown.connect([this](){
            tearDown();
        });
    }

    void tearDown()
    {
        if(client) {
            aerr << "Aborting path planning" << std::endl;
            client->cancelGoal();

            client.reset();
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});

            setParameter("feedback", std::string("aborted"));
        }
    }

    void goalCallback(const actionlib::SimpleClientGoalState&, const path_msgs::PlanPathResultConstPtr& result)
    {
        goal_.reset();

        if(!result) {
            aerr << "Received an empty result message. Did the action server crash?" << std::endl;
            event_error_->trigger();

        } else {
            if(!result->path.paths.empty()) {
                msg::publish(out_path_, std::make_shared<path_msgs::PathSequence>(result->path));
            } else {
                event_error_->trigger();
            }
        }

        client.reset();
        continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
    }

    void feedbackCallback(const path_msgs::PlanPathFeedbackConstPtr& fb)
    {
        std::string msg;
        switch(fb->status) {
        case path_msgs::PlanPathFeedback::STATUS_DONE:
            msg = "DONE";
            break;
        case path_msgs::PlanPathFeedback::STATUS_PLANNING:
            msg = "PLANNING";
            break;
        case path_msgs::PlanPathFeedback::STATUS_POST_PROCESSING:
            msg = "POST PROCESSING";
            break;
        case path_msgs::PlanPathFeedback::STATUS_PRE_PROCESSING:
            msg = "PRE PROCESSING";
            break;
        case path_msgs::PlanPathFeedback::STATUS_PLANNING_FAILED:
            msg = "PLANNING FAILED";
            break;

        default:
            msg = "???";
        }

        setParameter("feedback", std::string("STATUS: ") + msg);
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
        aerr << "start path planning" << std::endl;

        auto pos = clients_.find(channel_);
        if(pos == clients_.end()) {
            clients_[channel_] = std::make_shared<actionlib::SimpleActionClient<path_msgs::PlanPathAction>>(channel_, true);
        }
        client = clients_.at(channel_);

        if(msg::hasMessage(in_pose_)) {
            start_ = msg::getMessage<TransformMessage>(in_pose_);
        } else {
            start_.reset();
        }

        goal_ = msg::getMessage<nav_msgs::OccupancyGrid>(in_goal_);
        apex_assert(goal_);

        if(!client->isServerConnected()) {
            awarn << "waiting for path planner server " << channel_ << std::endl;
            if(!client->waitForServer(ros::Duration(1.0))) {
                throw std::runtime_error("unknown path planning channel");
            }
        }

        path_msgs::PlanPathGoal goal_msg;
        goal_msg.goal.type = path_msgs::Goal::GOAL_TYPE_MAP;

        goal_msg.goal.map = *goal_;

        goal_msg.goal.min_dist = min_dist_;
        goal_msg.goal.pose.header = goal_->header;
        goal_msg.goal.planning_algorithm.data = algorithm_;
        goal_msg.goal.planning_channel.data = channel_;

        goal_msg.goal.grow_obstacles = grow_obstacles_;
        if(grow_obstacles_) {
            goal_msg.goal.obstacle_growth_radius = obstacle_growth_;
        }

        goal_msg.goal.max_search_duration = max_search_duration_;

        if(start_) {
            goal_msg.use_start = true;
            tf::poseTFToMsg(start_->value, goal_msg.start.pose);

        } else {
            goal_msg.use_start = false;
        }

        client->sendGoal(goal_msg,
                         boost::bind(&PlanPathWithMap::goalCallback, this, _1, _2),
                         boost::bind(&PlanPathWithMap::activeCallback, this),
                         boost::bind(&PlanPathWithMap::feedbackCallback, this, _1));
    }


private:
    Input* in_goal_;
    Input* in_pose_;

    Output* out_path_;

    Event* event_error_;

    TransformMessage::ConstPtr start_;
    std::shared_ptr<nav_msgs::OccupancyGrid const> goal_;

    std::string algorithm_;
    std::string channel_;

    double min_dist_;

    bool grow_obstacles_;
    double obstacle_growth_;

    double max_search_duration_;

    std::shared_ptr<actionlib::SimpleActionClient<path_msgs::PlanPathAction>> client;
    std::map<std::string, std::shared_ptr<actionlib::SimpleActionClient<path_msgs::PlanPathAction>>> clients_;

    Continuation continuation_;

    slim_signal::ScopedConnection connection_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PlanPathWithMap, csapex::Node)


