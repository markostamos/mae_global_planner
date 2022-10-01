#ifndef SUBTOUR_GA_H
#define SUBTOUR_GA_H
#include <vector>
#include <mae_global_planner/Structs.h>
class GlobalPlanner
{
    /**
     * @brief Class that implements a genetic algorithm for solving the Subtour traveling salesman problem.
     *        It is used to generate a set of paths that are then used by the Global Planner to generate
     *        an optimal team strategy.
     *        It uses the following operators:
     *         - Tournament selection
     *         - Mutation (Introduces new points in the path)
     *         - 2-opt (Locally optimizes the path)
     *         - Social disaster (Randomly discards a percentage of the population)
     *         - Elitism (The best path(Subtour) is always retained in the population)
     */
public:
    float mutation_rate_;
    float p2opt_rate_;
    float social_disaster_rate_;
    int tournament_size_;

private:
    std::vector<Path> population_;
    std::vector<Point> targets_;
    Point starting_position_;
    int population_size_;
    int subtour_len_;

public:
    GlobalPlanner(){};
    /**
     * @brief Construct a new plannerobject
     *
     * @param population_size Size of the population to be evolved.
     * @param tournament_size Size of the tournament used for selection. Bigger tournament size means less diversity in the population selection.
     * @param mutation_rate Probability of a mutation occuring in a path (0-1).
     * @param p2opt_rate Probability of a 2-opt local optimization occuring in a path(0-1).
     * @param social_disaster_rate Percentage of the population that is randomly discarded in each generation(0-1).
     */
    GlobalPlanner(int &&population_size,
                  int &&tournament_size,
                  float &&mutation_rate,
                  float &&p2opt_rate,
                  float &&social_disaster_rate);

    /**
     * @brief Initializes the genetic algorithm.
     *
     * @param targets Total number of targets available to be used in the subtour.
     * @param subtour_length Number of targets in each subtour.
     * @param starting_position The first point in the subtour that remains fixed throughout the evolution.
     */
    void initialize(const std::vector<Point> &targets, int subtour_length, const Point &starting_position);

    /**
     * @brief Evolves the population for a single generation.
     *
     */
    void evolutionStep();

    /**
     * @brief Evolvues the population until time limit is reached.
     *
     * @param timeout_ms Time limit in milliseconds.
     * @param info If true, prints cost information when a new best path is found.
     */
    void evolve(int timeout_ms, bool info = false);

    /**
     * @brief Returns the current population.
     *
     * @return Path
     */
    const std::vector<Path> &getPopulation() const;

    float getAveragePathLen(std::vector<Path> paths = {}) const;

    Path getBestPath(std::vector<Path> paths = {}) const;

    /**
     * @brief Kmeans clustering to assign targets to each agent.
     *
     * @param points Targets to be clustered.
     * @param k Number of clusters.
     * @param centroids Initial centroids(agent's starting positions).
     * @param max_iterations Maximum number of iterations before algorithm ends.
     * @param tolerance Distance tolerance in meters for convergence.
     * @return std::vector<std::vector<Point>>
     */
    static std::vector<std::vector<Point>> kmeans(const std::vector<Point> &points, int k, std::vector<Point> &centroids, int max_iterations, float tolerance);

private:
    /**
     * @brief Local optimization of a path using the 2-opt heuristic.
     * @param Path to be optimized.
     * @param p2opt_rate Probability of a 2-opt local optimization occuring in the path(0-1).
     */
    void optSwapHeuristic(Path *path, float p2opt_rate);

    /**
     * @brief Applies genetic operators to the mating pool to generate a new population.
     * @param mating_pool
     * @param new_population
     */
    void geneticOperators(std::vector<Path> *mating_pool, std::vector<Path> *new_population);

    /**
     * @brief Removes the worst paths from the population.
     * @param new_population
     */
    void removeWorstPaths(std::vector<Path> *new_population);

    /**
     * @brief Mutates a path by introducing new points in it, thus exploring new solutions.
     * @param path
     * @param mutation_rate Probability of a mutation occuring in a path (0-1).
     */
    void mutate(Path *path, float mutation_rate);

    /**
     * @brief Performs a tournament selection to select the best paths from the population.
     * @param mating_pool
     * @param new_population
     * @param tournament_size Size of the tournament used for selection. Bigger tournament size means less diversity in the population selection.
     */
    void selection(std::vector<Path> *mating_pool, std::vector<Path> *new_population, int tournament_size);

    /**
     * @brief Performs a social disaster, randomly discarding a percentage of the population.
     * @param new_population
     * @param tournament_size Size of the tournament used for selection. Bigger tournament size means less diversity in the population selection.
     * @param social_disaster_rate Percentage of the population that is randomly discarded in each generation(0-1).
     */
    void socialDisaster(std::vector<Path> *new_population, float social_disaster_rate);

    /**
     * @brief Adds the best path to the new population.
     * @param new_population
     */
    void elitisism(std::vector<Path> *new_population);
};

#endif // SUBTOUR_GA_H