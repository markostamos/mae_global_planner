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

    GlobalPlanner global_planner_;

    ros::ServiceServer plan_service_;
    ros::ServiceServer global_plan_service_;

public:
    GlobalPlannerNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
    {

        global_planner_ = GlobalPlanner(nh_private_.param("population_size", 50),
                                        nh_private_.param("tournament_size", 10),
                                        nh_private_.param("mutation_rate", 0.6),
                                        nh_private_.param("p2opt_rate", 0.3),
                                        nh_private_.param("social_disaster_rate", 0.1));

        plan_service_ = nh_.advertiseService("global_planner_node/make_single_plan", &GlobalPlannerNode::planCallback, this);
        global_plan_service_ = nh_.advertiseService("global_planner_node/make_global_plan", &GlobalPlannerNode::globalPlanCallback, this);
    }

private:
    bool planCallback(mae_global_planner::PlanService::Request &req, mae_global_planner::PlanService::Response &res)
    {
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
        ROS_INFO("Make Global Plan service called");
        try
        {
            std::vector<Point> targets = toPoints(req.targets);
            std::vector<Point> starting_positions = toPoints(req.starting_positions);
            int timeout_ms = req.timeout_ms;

            if (targets.size() <= starting_positions.size())
            {
                for (const auto &starting_position : starting_positions)
                {
                    auto target = *std::min_element(targets.begin(), targets.end(), [&starting_position](const Point &a, const Point &b)
                                                    { return starting_position.dist3(a) < starting_position.dist3(b); });
                    res.global_plan.push_back(toPointArray(std::vector<Point>({starting_position, target})));
                }
            }
            else
            {
                std::vector<Point> cluster_centroids = starting_positions;
                std::vector<std::vector<Point>> cluster_points = global_planner_.kmeans(targets, cluster_centroids.size(), cluster_centroids, 1000, 0.01);
                for (int i = 0; i < cluster_centroids.size(); i++)
                {
                    if (cluster_points[i].empty())
                    {
                        res.global_plan.push_back(mae_global_planner::PointArray());
                    }
                    else
                    {
                        global_planner_.initialize(cluster_points[i], cluster_points[i].size(), starting_positions[i]);
                        global_planner_.evolve(timeout_ms / cluster_centroids.size(), false);
                        res.global_plan.push_back(toPointArray(global_planner_.getBestPath()));
                    }
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