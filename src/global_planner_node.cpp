#include <ros/ros.h>
#include <mae_global_planner/GlobalPlanner.h>
#include <mae_global_planner/SubtourPlanner.h>
#include <mae_global_planner/Structs.h>
#include <mae_global_planner/GlobalPlanService.h>
#include <mae_global_planner/SubtourPlanService.h>
#include <mae_global_planner/utils.h>
class GlobalPlannerNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    GlobalPlanner global_planner_;
    SubtourPlanner subtour_planner_;

    ros::ServiceServer subtour_service_;
    ros::ServiceServer global_plan_service_;

public:
    GlobalPlannerNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
    {
        global_planner_ = GlobalPlanner(nh_private_.param("resource_sharing_bias", 0.1));

        subtour_planner_ = SubtourPlanner(nh_private_.param("population_size", 50),
                                          nh_private_.param("tournament_size", 10),
                                          nh_private_.param("mutation_rate", 0.6),
                                          nh_private_.param("p2opt_rate", 0.3),
                                          nh_private_.param("social_disaster_rate", 0.1));

        subtour_service_ = nh_.advertiseService("make_subtour", &GlobalPlannerNode::subtourCallback, this);
        global_plan_service_ = nh_.advertiseService("make_global_plan", &GlobalPlannerNode::globalPlanCallback, this);
    }

private:
    bool subtourCallback(mae_global_planner::SubtourPlanService::Request &req, mae_global_planner::SubtourPlanService::Response &res)
    {
        ROS_INFO("Subtour service called");
        try
        {
            subtour_planner_.initialize(toPoints(req.targets), req.subtour_length, toPoint(req.starting_position));
            subtour_planner_.evolve(req.timeout_ms, true);
            res.subtour = toPointArray(subtour_planner_.getBestPath());
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Subtour service failed: %s", e.what());
            return false;
        }
    }

    bool globalPlanCallback(mae_global_planner::GlobalPlanService::Request &req, mae_global_planner::GlobalPlanService::Response &res)
    {
        ROS_INFO("Global plan service called");
        try
        {
            global_planner_.initialize(toPaths(req.paths));
            global_planner_.evolve(req.timeout_ms);
            res.global_plan = toGeometryPlan(global_planner_.getBestPlan());
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Global plan service failed: %s", e.what());
            return false;
        }
        return true;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    GlobalPlannerNode global_planner_node(nh, private_nh);
    ros::spin();
}