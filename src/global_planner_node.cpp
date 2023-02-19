#include <ros/ros.h>
#include <mae_global_planner/GlobalPlanner.h>
#include <mae_global_planner/Structs.h>
#include <mae_global_planner/PlanService.h>
#include <mae_global_planner/GlobalPlanService.h>
#include <mae_global_planner/utils.h>
#include <thread>
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
        ROS_INFO("Make Global Plan service called");
        try
        {
            std::vector<Point> targets = toPoints(req.targets);
            std::vector<Point> starting_positions = toPoints(req.starting_positions);
            std::vector<Point> cluster_centroids = starting_positions;
            std::vector<std::vector<Point>> cluster_points = GlobalPlanner::kmeans(targets, cluster_centroids.size(), cluster_centroids, 1000, 0.01);
            res.global_plan.resize(cluster_centroids.size());

            std::vector<std::thread> exec_threads;

            for (int i = 0; i < cluster_points.size(); i++)
            {
                exec_threads.push_back(std::thread(&GlobalPlannerNode::calculateAgentPlan,
                                                   this,
                                                   cluster_points[i],
                                                   starting_positions[i],
                                                   req.timeout_ms,
                                                   i,
                                                   std::ref(res)));
            }

            // join threads

            for (auto &t : exec_threads)
                t.join();

            return true;
        }

        catch (const std::exception &e)
        {
            ROS_ERROR("Make Global Plan service failed: %s", e.what());
            return false;
        }
    }

    void calculateAgentPlan(const std::vector<Point> &targets, const Point &starting_position, const int &timeout_ms, const int index, mae_global_planner::GlobalPlanService::Response &res)
    {
        GlobalPlanner global_planner_ = GlobalPlanner(nh_private_.param("population_size", 50),
                                                      nh_private_.param("tournament_size", 10),
                                                      nh_private_.param("mutation_rate", 0.6),
                                                      nh_private_.param("p2opt_rate", 0.3),
                                                      nh_private_.param("social_disaster_rate", 0.1));
        if (!targets.empty())
        {
            global_planner_.initialize(targets, targets.size(), starting_position);
            global_planner_.evolve(timeout_ms, true);
            res.global_plan[index] = toPointArray(global_planner_.getBestPath());
        }
        else
        {
            mae_global_planner::PointArray empty_plan;
            empty_plan.points.push_back(toGeometryPoint(starting_position));
            res.global_plan[index] = empty_plan;
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