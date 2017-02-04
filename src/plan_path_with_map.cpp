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

class PlanPathWithMap : public ChanneledActionlibNode<path_msgs::PlanPathAction>
{
public:
    PlanPathWithMap()
    {
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
        ActionlibNode<path_msgs::PlanPathAction>::setupParameters(parameters);

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
        ActionlibNode<path_msgs::PlanPathAction>::setupROS();

        in_goal_ = modifier.addInput<nav_msgs::OccupancyGrid>("Goal");
        in_pose_ = modifier.addOptionalInput<TransformMessage>("Start");
        out_path_ = modifier.addOutput<path_msgs::PathSequence>("Path");
    }

    void abortAction() override
    {
        ActionlibNode<path_msgs::PlanPathAction>::abortAction();
        setParameter("feedback", std::string("aborted"));
    }


    void processResultCallback(const actionlib::SimpleClientGoalState& state, const boost::shared_ptr<typename path_msgs::PlanPathResult const>& result) override
    {
        if(!result->path.paths.empty()) {
            msg::publish(out_path_, std::make_shared<path_msgs::PathSequence>(result->path));
        } else {
            msg::trigger(event_error_);
        }
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

        goal_ = msg::getMessage<nav_msgs::OccupancyGrid>(in_goal_);
        apex_assert(goal_);


        goal.goal.type = path_msgs::Goal::GOAL_TYPE_MAP;

        goal.goal.map = *goal_;

        goal.goal.min_dist = min_dist_;
        goal.goal.pose.header = goal_->header;
        goal.goal.planning_algorithm.data = algorithm_;
        goal.goal.planning_channel.data = channel_;

        goal.goal.grow_obstacles = grow_obstacles_;
        if(grow_obstacles_) {
            goal.goal.obstacle_growth_radius = obstacle_growth_;
        }

        goal.goal.max_search_duration = max_search_duration_;

        if(start_) {
            goal.use_start = true;
            tf::poseTFToMsg(start_->value, goal.start.pose);

        } else {
            goal.use_start = false;
        }
    }


private:
    Input* in_goal_;
    Input* in_pose_;

    Output* out_path_;

    TransformMessage::ConstPtr start_;
    std::shared_ptr<nav_msgs::OccupancyGrid const> goal_;

    std::string algorithm_;
    std::string channel_;

    double min_dist_;

    bool grow_obstacles_;
    double obstacle_growth_;

    double max_search_duration_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PlanPathWithMap, csapex::Node)


