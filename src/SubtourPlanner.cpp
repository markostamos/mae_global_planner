#include <mae_global_planner/SubtourPlanner.h>
#include <mae_global_planner/utils.h>
#include <mae_global_planner/Structs.h>
#include <random>
#include <chrono>
#include <stdexcept>
#include <vector>
SubtourPlanner::SubtourPlanner(int &&population_size,
                               int &&tournament_size,
                               float &&mutation_rate,
                               float &&p2opt_rate,
                               float &&social_disaster_rate)
{
    if (tournament_size > population_size)
        throw std::invalid_argument("Tournament size must be smaller than population size");

    population_size_ = population_size;
    tournament_size_ = tournament_size;

    mutation_rate_ = mutation_rate;
    p2opt_rate_ = p2opt_rate;
    social_disaster_rate_ = social_disaster_rate;
}

void SubtourPlanner::initialize(const std::vector<Point> &targets, int subtour_len, const Point &starting_position)
{
    if (targets.empty())
        throw std::invalid_argument("Targets vector is empty");
    if (subtour_len > targets.size())
        throw std::invalid_argument("Subtour length must be smaller than the number of targets");

    subtour_len_ = subtour_len;
    targets_ = targets;
    starting_position_ = starting_position;
    population_.clear();

    // Random shuffling to avoid 2 copies of the same point in a path.
    std::vector<int> selection_pool(targets_.size());
    std::iota(selection_pool.begin(), selection_pool.end(), 0);
    std::mt19937 rng(std::random_device{}());

    for (int i = 0; i < population_size_; i++)
    {
        std::shuffle(begin(selection_pool), end(selection_pool), rng);

        std::vector<Point> new_path = {starting_position};
        for (int j = 0; j < subtour_len; j++)
        {
            new_path.push_back(targets_[selection_pool[j]]);
        }
        population_.emplace_back(new_path);
    }
}

const std::vector<Path> &SubtourPlanner::getPopulation() const
{
    return population_;
}

float SubtourPlanner::getAveragePathLen(std::vector<Path> paths) const
{
    if (paths.empty())
        paths = population_;

    float sum = std::accumulate(paths.begin(), paths.end(), 0.0, [](float sum, const Path &path)
                                { return sum + path.getLength(); });

    return sum / paths.size();
}

void SubtourPlanner::evolutionStep()
{
    std::vector<Path> new_population;
    std::vector<Path> mating_pool(population_size_ / 2);

    selection(&mating_pool, &new_population, tournament_size_);
    geneticOperators(&mating_pool, &new_population);
    removeWorstPaths(&new_population);
    socialDisaster(&new_population, social_disaster_rate_);
    elitisism(&new_population);
    population_ = new_population;
}

void SubtourPlanner::selection(std::vector<Path> *mating_pool, std::vector<Path> *new_population, int tournament_size)
{
    *new_population = removeDuplicates(population_);
    std::vector<Path> tournament_pool(tournament_size);
    for (int i = 0; i < population_size_ / 2; i++)
    {
        std::for_each(tournament_pool.begin(), tournament_pool.end(), [&](Path &path)
                      { path = (*new_population)[getRandomInt(0, new_population->size() - 1)]; });

        (*mating_pool)[i] = getBestPath(tournament_pool);
    }
}

void SubtourPlanner::geneticOperators(std::vector<Path> *mating_pool, std::vector<Path> *new_population)
{
    Path child;
    while (new_population->size() < (int)(population_size_ * 1.5))
    {
        child = getRandomElement(*mating_pool);
        mutate(&child, mutation_rate_);
        optSwapHeuristic(&child, p2opt_rate_);
        new_population->push_back(child);
    }
}

void SubtourPlanner::mutate(Path *path, float mutation_rate)
{

    if (getRandomFloat(0, 1) < mutation_rate)
    {
        std::vector<Point> new_points = path->getPoints();
        if (subtour_len_ < targets_.size())
        {
            Point rand_point = getRandomElement(targets_);

            while (includes(new_points, rand_point))
                rand_point = getRandomElement(targets_);

            int mutation_index = getRandomInt(1, subtour_len_ - 1);

            new_points[mutation_index] = rand_point;
        }
        else
        {
            int mutation_index1 = getRandomInt(1, new_points.size() - 1);
            int mutation_index2 = getRandomInt(1, new_points.size() - 1);
            std::swap(new_points[mutation_index1], new_points[mutation_index2]);
        }
        path->updatePath(new_points);
    }
}

void SubtourPlanner::removeWorstPaths(std::vector<Path> *new_population)
{
    std::sort(new_population->begin(), new_population->end());

    new_population->erase(new_population->begin() + population_size_, new_population->end());
}

Path SubtourPlanner::getBestPath(std::vector<Path> paths) const
{
    if (paths.empty())
        paths = population_;

    return *std::min_element(paths.begin(), paths.end());
}

void SubtourPlanner::optSwapHeuristic(Path *path, float p2opt_rate)
{

    if (getRandomFloat(0, 1) < p2opt_rate)
    {
        auto do2Opt = [](std::vector<Point> &points, int i, int j)
        { std::reverse(points.begin() + i + 1, points.begin() + j + 1); };

        std::vector<Point> points = path->getPoints();
        int current_length = path->getLength();
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
        path->updatePath(points);
    }
}

void SubtourPlanner::evolve(int timeout_ms, bool info)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    int generation = 0;
    float best_length = std::numeric_limits<float>::max();
    while (duration < timeout_ms && subtour_len_ > 1)
    {
        evolutionStep();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        generation++;
        if (getBestPath().getLength() < best_length && info)
        {
            best_length = getBestPath().getLength();

            std::cout << "Generation: " << generation << " Best: " << best_length << std::endl;
        }
    }
    std::cout << "Finished in " << generation << " generations with length " << getBestPath().getLength() << std::endl;
}

void SubtourPlanner::socialDisaster(std::vector<Path> *new_population, float social_disaster_rate)
{
    std::vector<Point> new_points;
    Point new_target;

    for (int i = 0; i < (int)(population_size_ * social_disaster_rate); i++)
    {
        new_points = {starting_position_};
        // add n random points to new_points
        while (new_points.size() < subtour_len_ + 1)
        {
            new_target = getRandomElement(targets_);
            if (!includes(new_points, new_target))
                new_points.push_back(new_target);
        }
        getRandomElementRef(*new_population).updatePath(new_points);
    }
}

void SubtourPlanner::elitisism(std::vector<Path> *new_population)
{
    getRandomElementRef(*new_population) = getBestPath();
}