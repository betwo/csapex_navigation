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
#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>

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

        parameters.addParameter(param::factory::declareText("algorithm", "generic"), algorithm_);
        parameters.addParameter(param::factory::declareText("planner_topic", "plan_path"), channel_);

        parameters.addParameter(param::factory::declareRange("min_distance", 0.0, 10.0, 2.0, 0.01), min_dist_);

        parameters.addParameter<decltype(options_.grow_obstacles),bool>(param::factory::declareBool
                                                                        ("grow_obstacles", true), options_.grow_obstacles);
        parameters.addConditionalParameter<decltype(options_.obstacle_growth_radius),double>(param::factory::declareRange(
                                                                                 "grow_obstacles/radius", 0.0, 10.0, 1.0, 0.01),
                                                                                 [this]() -> bool {
                                                                                     return options_.grow_obstacles;
                                                                                 }, options_.obstacle_growth_radius);

        parameters.addParameter<decltype(options_.max_search_duration),double>(param::factory::declareRange(
                                                                                       "max_search_duration", 0.0, 30.0, 10.0, 0.01),
                                                                                   options_.max_search_duration);


        parameters.addParameter(param::factory::declareRange("planner/map_search_min_value", 0, 255, 100, 1),
                                map_search_min_value_);
        parameters.addParameter(param::factory::declareRange("planner/map_search_min_candidates", 1, 256, 64, 1),
                                map_search_min_candidates_);

        auto cond_generic = [this](){
            return algorithm_ == "generic";
        };



        parameters.addConditionalParameter<decltype(options_.reversed),bool>(
                    param::factory::declareValue("planner/reversed", true),
                    cond_generic, options_.reversed);
        parameters.addConditionalParameter<decltype(options_.goal_dist_threshold),double>(
                    param::factory::declareValue("planner/goal_dist_threshold", 0.15),
                    cond_generic, options_.goal_dist_threshold);
        parameters.addConditionalParameter<decltype(options_.goal_angle_threshold_degree),double>(
                    param::factory::declareValue("planner/goal_angle_threshold", 25.0),
                    cond_generic, options_.goal_angle_threshold_degree);
        parameters.addConditionalParameter<decltype(options_.allow_forward),bool>(
                    param::factory::declareValue("planner/allow_forward", true),
                    cond_generic, options_.allow_forward);
        parameters.addConditionalParameter<decltype(options_.allow_backward),bool>(
                    param::factory::declareValue("planner/allow_backward", false),
                    cond_generic, options_.allow_backward);
        parameters.addConditionalParameter<decltype(options_.ackermann_la),double>(
                    param::factory::declareValue("planner/ackermann_la", 1.2),
                    cond_generic, options_.ackermann_la);
        parameters.addConditionalParameter<decltype(options_.ackermann_steer_steps),int>(
                    param::factory::declareValue("planner/ackermann_steer_steps", 2),
                    cond_generic, options_.ackermann_steer_steps);
        parameters.addConditionalParameter<decltype(options_.ackermann_max_steer_angle_degree),double>(
                    param::factory::declareValue("planner/ackermann_max_steer_angle", 40.0),
                    cond_generic, options_.ackermann_max_steer_angle_degree);
        parameters.addConditionalParameter<decltype(options_.ackermann_steer_delta_degree),double>(
                    param::factory::declareValue("planner/ackermann_steer_delta", 20.0),
                    cond_generic, options_.ackermann_steer_delta_degree);

        parameters.addParameter(param::factory::declareOutputText("feedback"));
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        ActionlibNode<path_msgs::PlanPathAction>::setup(modifier);

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

    void feedbackCallback(const path_msgs::PlanPathFeedbackConstPtr& fb) override
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
        goal.goal.map_search_min_value = map_search_min_value_;
        goal.goal.map_search_min_candidates = map_search_min_candidates_;

        goal.options = options_;

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

    path_msgs::PlannerOptions options_;

    TransformMessage::ConstPtr start_;
    std::shared_ptr<nav_msgs::OccupancyGrid const> goal_;

    std::string algorithm_;
    std::string channel_;

    double min_dist_;
    int map_search_min_value_;
    int map_search_min_candidates_;
};

}

CSAPEX_REGISTER_CLASS(csapex::PlanPathWithMap, csapex::Node)


