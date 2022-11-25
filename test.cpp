// TARGET POINTS CREATION
#define MIN -100
#define MAX 100
#define CENTERS 20
#define POINTS_PER_CENTER 10
#define STDDEV 10
#define NUM_AGENTS 4
#define PLANNER_TIMEOUT 300

// PLANNER PARAMS
#define POPULATION_SIZE 50
#define TOURNAMENT_SIZE 10
#define MUTATION_RATE 0.6
#define P2OPT_RATE 0.3
#define SOCIAL_DISASTER_RATE 0.1

#include <iostream>
#include <fstream>
#include "mae_global_planner/GlobalPlanner.h"
#include "mae_global_planner/utils.h"

void writeToCSV(std::vector<Point> &points, std::string &&filename)
{
    std::ofstream file;
    file.open(filename);
    for (const auto &point : points)
    {
        file << point.x << "," << point.y << "," << point.z << std::endl;
    }
    file.close();
}

void writeMultiToCSV(std::vector<std::vector<Point>> &clusters, std::string &&filename)
{
    std::ofstream file;
    file.open(filename);
    for (const auto &cluster : clusters)
    {
        for (const auto &point : cluster)
        {
            file << point.x << "," << point.y << "," << point.z << std::endl;
        }
        file << std::endl;
    }
    file.close();
}

int main(int argc, char *argv[])
{

    // Generate random targets
    std::vector<Point> targets;
    for (int i = 0; i < CENTERS; i++)
    {
        createRandomPointsGaussian(&targets, POINTS_PER_CENTER, getRandomPoint(MIN, MAX), STDDEV, false);
    }

    // Generate random agent positions
    std::vector<Point> agents;
    for (int i = 0; i < NUM_AGENTS; i++)
    {
        agents.emplace_back(getRandomPoint(MIN, MAX));
    }

    // Perform K-Means Clustering on targets
    std::vector<Point> centroids = agents;
    std::vector<std::vector<Point>> clusters = GlobalPlanner::kmeans(targets, NUM_AGENTS, centroids, 1000, 0);

    // initialize Global planner
    GlobalPlanner global_planner_ = GlobalPlanner(POPULATION_SIZE,
                                                  TOURNAMENT_SIZE,
                                                  MUTATION_RATE,
                                                  P2OPT_RATE,
                                                  SOCIAL_DISASTER_RATE);

    // Optimize paths for each agent

    std::vector<std::vector<Point>> global_plan;
    std::vector<std::vector<Point>> initial_plan;
    for (int i = 0; i < clusters.size(); i++)
    {
        if (!clusters[i].empty())
        {
            global_planner_.initialize(clusters[i], clusters[i].size(), agents[i]);
            initial_plan.push_back(getRandomElement(global_planner_.getPopulation()).getPoints());
            global_planner_.evolve(PLANNER_TIMEOUT / centroids.size(), false);
            global_plan.push_back(global_planner_.getBestPath().getPoints());
        }
        else
        {
            global_plan.push_back(std::vector<Point>());
        }
    }

    writeToCSV(agents, "agents.csv");
    writeToCSV(targets, "targets.csv");
    writeMultiToCSV(clusters, "clusters.csv");
    writeMultiToCSV(global_plan, "plan.csv");
    writeMultiToCSV(initial_plan, "initial_plan.csv");

    return 0;
}