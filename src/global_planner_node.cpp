#include <ros/ros.h>
#include <mae_global_planner/GlobalPlanner.h>
#include <mae_global_planner/Structs.h>
#include <mae_global_planner/PlanService.h>
#include <mae_global_planner/GlobalPlanService.h>
#include <mae_global_planner/utils.h>
class GlobalPlannerNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceServer plan_service_;
    ros::ServiceServer global_plan_service_;

public:
    GlobalPlannerNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
    {

        plan_service_ = nh_.advertiseService("global_planner_node/make_single_plan", &GlobalPlannerNode::planCallback, this);
        global_plan_service_ = nh_.advertiseService("global_planner_node/make_global_plan", &GlobalPlannerNode::globalPlanCallback, this);
    }

private:
    bool planCallback(mae_global_planner::PlanService::Request &req, mae_global_planner::PlanService::Response &res)
    {
        GlobalPlanner global_planner_ = GlobalPlanner(nh_private_.param("population_size", 50),
                                                      nh_private_.param("tournament_size", 10),
                                                      nh_private_.param("mutation_rate", 0.6),
                                                      nh_private_.param("p2opt_rate", 0.3),
                                                      nh_private_.param("social_disaster_rate", 0.1));

        ROS_INFO("Make Plan service called");
        try
        {
            global_planner_.initialize(toPoints(req.targets), req.targets.points.size(), toPoint(req.starting_position));
            global_planner_.evolve(req.timeout_ms, true);
            res.plan = toPointArray(global_planner_.getBestPath());
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Make Plan service failed: %s", e.what());
            return false;
        }
    }
    bool globalPlanCallback(mae_global_planner::GlobalPlanService::Request &req, mae_global_planner::GlobalPlanService::Response &res)
    {
        GlobalPlanner global_planner_ = GlobalPlanner(nh_private_.param("population_size", 50),
                                                      nh_private_.param("tournament_size", 10),
                                                      nh_private_.param("mutation_rate", 0.6),
                                                      nh_private_.param("p2opt_rate", 0.3),
                                                      nh_private_.param("social_disaster_rate", 0.1));

        ROS_INFO("Make Global Plan service called");
        try
        {
            std::vector<Point> targets = toPoints(req.targets);
            std::vector<Point> starting_positions = toPoints(req.starting_positions);

            std::vector<Point> cluster_centroids = starting_positions;

            std::vector<std::vector<Point>> cluster_points = GlobalPlanner::kmeans(targets, cluster_centroids.size(), cluster_centroids, 1000, 0.01);
            for (int i = 0; i < cluster_points.size(); i++)
            {
                if (!cluster_points[i].empty())
                {
                    global_planner_.initialize(cluster_points[i], cluster_points[i].size(), starting_positions[i]);
                    global_planner_.evolve(req.timeout_ms / cluster_centroids.size(), false);
                    res.global_plan.push_back(toPointArray(global_planner_.getBestPath()));
                }
                else
                {
                    mae_global_planner::PointArray empty_plan;
                    empty_plan.points.push_back(toGeometryPoint(starting_positions[i]));
                    res.global_plan.push_back(empty_plan);
                }
            }

            return true;
        }

        catch (const std::exception &e)
        {
            ROS_ERROR("Make Global Plan service failed: %s", e.what());
            return false;
        }
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