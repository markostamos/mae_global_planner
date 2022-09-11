#include <mae_global_planner/GlobalPlanner.h>
#include <mae_global_planner/utils.h>
#include <mae_global_planner/Structs.h>
#include <vector>
#include <chrono>
#include <iostream>

GlobalPlanner::GlobalPlanner(float &&resource_sharing_bias)
{
    resource_sharing_bias_ = resource_sharing_bias;
}
void GlobalPlanner::initialize(const std::vector<Path> &plan_paths)
{
    current_plan_ = GlobalPlan(plan_paths, resource_sharing_bias_);
    best_plan_ = current_plan_;
}
void GlobalPlanner::evolutionStep()
{
    crossover();
    extendedOptHeuristic();
    removeIntersections();
    simpleOptSwap();

    current_plan_.getCost() < best_plan_.getCost() ? best_plan_ = current_plan_ : best_plan_ = best_plan_;
}
const GlobalPlan &GlobalPlanner::getPlan() const
{
    return current_plan_;
}
const GlobalPlan &GlobalPlanner::getBestPlan() const
{
    return best_plan_;
}

void GlobalPlanner::crossover()
{
    /*
        Gets the best and worst subtours from the current global plan.
        Generates two random cutting points one for each subtour.
        Cuts the subtours in the cutting points.
        Joins the subtours in the new order.
    */

    std::vector<Path> paths = current_plan_.getPaths();
    std::vector<Path> new_paths;
    while (paths.size() > 1)
    {
        Path best = *std::min_element(paths.begin(), paths.end());
        Path worst = *std::max_element(paths.begin(), paths.end());

        paths.erase(std::remove(paths.begin(), paths.end(), best), paths.end());
        paths.erase(std::remove(paths.begin(), paths.end(), worst), paths.end());

        int cut_point_1 = getRandomInt(1, best.getSize() - 1);
        int cut_point_2 = getRandomInt(1, worst.getSize() - 1);

        Path child1 = best.getSubPath(0, cut_point_1) + worst.getSubPath(cut_point_2, worst.getSize());
        Path child2 = worst.getSubPath(0, cut_point_2) + best.getSubPath(cut_point_1, best.getSize());

        new_paths.push_back(child1);
        new_paths.push_back(child2);
    }
    if (paths.size() == 1)
        new_paths.push_back(paths[0]);

    current_plan_.updatePlan(new_paths);
}

void GlobalPlanner::simpleOptSwap()
{
    auto do2Opt = [](std::vector<Point> &points, int i, int j)
    { std::reverse(points.begin() + i + 1, points.begin() + j + 1); };

    std::vector<Path> new_paths;
    for (const auto &path : current_plan_.getPaths())
    {

        std::vector<Point> points = path.getPoints();
        int current_length = path.getLength();
        int n = points.size();
        bool improvement = true;

        while (improvement)
        {
            improvement = false;

            for (int i = 1; i < n - 2; i++)
            {
                for (int j = i + 1; j < n - 1; j++)
                {
                    int length_delta = -points[i].dist2(points[i + 1]) - points[j].dist2(points[j + 1]) + points[i].dist2(points[j]) + points[i + 1].dist2(points[j + 1]);

                    if (length_delta < 0)
                    {
                        do2Opt(points, i, j);
                        current_length += length_delta;
                        improvement = true;
                    }
                }
            }
        }
        new_paths.emplace_back(Path(points));
    }
    current_plan_.updatePlan(new_paths);
}

void GlobalPlanner::extendedOptHeuristic()
{
    std::vector<Path> paths = current_plan_.getPaths();

    for (int i = 0; i < paths.size() - 1; i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            twoPathOptSwap(&paths[i], &paths[j]);
        }
    }

    current_plan_.updatePlan(paths);
}

void GlobalPlanner::twoPathOptSwap(Path *path1, Path *path2)
{
    int best_length = path1->getLength() + path2->getLength();

    bool improvement = true;
    while (improvement)
    {
        improvement = false;
        for (int i = 1; i < path1->getSize(); i++)
        {
            for (int j = 1; j < path2->getSize(); j++)
            {
                Point tmp = path1->getPoint(i);
                path1->setPoint(i, path2->getPoint(j));
                path2->setPoint(j, tmp);

                if (path1->getLength() + path2->getLength() < best_length)
                {
                    best_length = path1->getLength() + path2->getLength();
                    improvement = true;
                }
                else
                {
                    tmp = path1->getPoint(i);
                    path1->setPoint(i, path2->getPoint(j));
                    path2->setPoint(j, tmp);
                }
            }
        }
    }
}

void GlobalPlanner::removeIntersectionsFromPath(Path *path1, Path *path2)
{
    /*
       Checks for intersections between two paths.
       If there is an intersection, the two paths swap the points that are intersecting.

    */
    auto linesIntersect = [](Point a, Point b, Point c, Point d)
    {
        float det, gamma, lambda;
        det = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (det == 0)
        {
            return false;
        }
        else
        {
            lambda = ((d.y - c.y) * (d.x - a.x) + (c.x - d.x) * (d.y - a.y)) / det;
            gamma = ((a.y - b.y) * (d.x - a.x) + (b.x - a.x) * (d.y - a.y)) / det;
            return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
        }
    };

    bool improvement = true;
    while (improvement)
    {
        improvement = false;
        for (int i = 0; i < path1->getSize() - 1; i++)
        {
            for (int j = 0; j < path2->getSize() - 1; j++)
            {
                if (linesIntersect(path1->getPoint(i), path1->getPoint(i + 1), path2->getPoint(j), path2->getPoint(j + 1)))
                {

                    Point tmp = path1->getPoint(i + 1);
                    path1->setPoint(i + 1, path2->getPoint(j + 1));
                    path2->setPoint(j + 1, tmp);
                    improvement = true;
                }
            }
        }
    }
}
void GlobalPlanner::removeIntersections()
{
    /*
        Applies the removeIntersections algorithm to each pair of paths(subtours).
    */

    std::vector<Path> paths = current_plan_.getPaths();
    for (int i = 0; i < paths.size() - 1; i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            removeIntersectionsFromPath(&paths[i], &paths[j]);
        }
    }

    current_plan_.updatePlan(paths);
}

void GlobalPlanner::evolve(int timeout_ms, bool info)
{

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    int generation = 0;
    float cost = best_plan_.getCost();

    while (duration < timeout_ms)
    {
        evolutionStep();

        if (current_plan_.getCost() < cost && info)
        {
            std::cout << "Generation: " << generation << " Cost: " << current_plan_.getCost() << std::endl;
            cost = current_plan_.getCost();
        }

        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        generation++;
    }

    std::cout << "Finished in " << generation << " generations with min cost " << getBestPlan().getCost() << std::endl;
}