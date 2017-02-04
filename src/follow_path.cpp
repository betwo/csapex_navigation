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
#include <path_msgs/FollowPathAction.h>

/// SYSTEM
#include <tf/tf.h>

using namespace csapex::connection_types;

namespace csapex
{

class FollowPath : public ChanneledActionlibNode<path_msgs::FollowPathAction>
{
public:
    FollowPath()
    {
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        ChanneledActionlibNode<path_msgs::FollowPathAction>::setupParameters(parameters);

        std::map<std::string, int> init_modes = {
            {"STOP", (int) path_msgs::FollowPathGoal::INIT_MODE_STOP},
            {"CONTINUE", (int) path_msgs::FollowPathGoal::INIT_MODE_CONTINUE}
        };
        parameters.addParameter(param::ParameterFactory::declareParameterSet("init_mode", init_modes, 0), init_mode_);

        parameters.addParameter(param::ParameterFactory::declareRange("target_velocity", 0.0, 5.0, 1.0, 0.01), velocity_);

        parameters.addParameter(param::ParameterFactory::declareText("follower_topic", "follow_path"), channel_);

        parameters.addParameter(param::ParameterFactory::declareText("robot_controller", ""), controller_);
        parameters.addParameter(param::ParameterFactory::declareText("local_planner", ""), local_planner_);
        parameters.addParameter(param::ParameterFactory::declareText("obstacle_avoider", ""), obstacle_avoider_);
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        ChanneledActionlibNode<path_msgs::FollowPathAction>::setup(modifier);

        in_path_ = modifier.addInput<path_msgs::PathSequence>("Path");

        event_obstacle_ = modifier.addEvent("obstacle");
        event_no_local_path_ = modifier.addEvent("no local path");
    }


    void processResultCallback(const actionlib::SimpleClientGoalState&, const path_msgs::FollowPathResultConstPtr& result) override
    {
        if(result->status != path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS) {
            msg::trigger(event_error_);
        } else {
            msg::trigger(event_done_);
        }
    }

    void feedbackCallback(const path_msgs::FollowPathFeedbackConstPtr& feedback) override
    {
        switch(feedback->status) {
        case path_msgs::FollowPathFeedback::MOTION_STATUS_MOVING:
            break;
        case path_msgs::FollowPathFeedback::MOTION_STATUS_OBSTACLE:
            break;
        case path_msgs::FollowPathFeedback::MOTION_STATUS_NO_LOCAL_PATH:
            break;
        }

    }

    virtual std::string getChannel() const
    {
        return channel_;
    }

    void getGoal(path_msgs::FollowPathGoal& goal)
    {
        std::shared_ptr<path_msgs::PathSequence const> path = msg::getMessage<path_msgs::PathSequence>(in_path_);
        apex_assert(path);

        goal.init_mode = init_mode_;
        goal.velocity = velocity_;
        goal.path = *path;
        goal.robot_controller.data = controller_;
        goal.local_planner.data = local_planner_;
        goal.obstacle_avoider.data = obstacle_avoider_;
    }


private:
    Input* in_path_;

    Event* event_obstacle_;
    Event* event_no_local_path_;

    std::string channel_;
    std::string controller_;
    std::string local_planner_;
    std::string obstacle_avoider_;

    int init_mode_;
    double velocity_;
};

}

CSAPEX_REGISTER_CLASS(csapex::FollowPath, csapex::Node)


