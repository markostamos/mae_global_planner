#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <mae_global_planner/Structs.h>

class GlobalPlanner
{
    /**
     * @brief Class responsible for generating a global optimal plan given a set of local optimal paths by
     *        minimizing the Cost function = max_subtour_len + resource_sharing_bias * subtour_variance.
     *        A plan consists of a set of subtours, to a total of NUMBER_OF_TARGETS subtours and gets
     *        evolved using a genetic algorithm with the following operators:
     *          -Crossover: Double cutting point crossover.
     *          -2-opt: 2-opt heuristic applied to each subtour.
     *          -extended 2-opt: 2-opt heuristic between two subtours
     *
     * @param resource_sharing_bias How much the global plan cost should be shared between the individual plans
     *
     */

public:
    float resource_sharing_bias_;

private:
    GlobalPlan current_plan_;
    GlobalPlan best_plan_;

public:
    GlobalPlanner(float &&resource_sharing_bias);

    /**
     * @brief Initializes the global planner with a set of pre computed optimal subpaths.
     *
     * @param plan The set of pre computed optimal subpaths.
     */
    void initialize(const std::vector<Path> &plan);

    /**
     * @brief Calculates the next generation of the global plan.
     */
    void evolutionStep();

    /**
     * @brief Evolves the current global plan until the timeout is reached.
     *
     * @param timeout_ms timeout in milliseconds.
     * @param info Print info about the new best plan when it is found.
     */
    void evolve(int timeout_ms, bool info = false);

    /**
     * @brief Returns the current global plan.
     *
     * @returns The current global plan.
     */
    const GlobalPlan &getPlan() const;

    /**
     * @brief Returns the best global plan found so far.
     *
     * @returns The best global plan found so far.
     */
    const GlobalPlan &getBestPlan() const;

private:
    /**
     * @brief Performs 2-cut Point crossover between all the subtours in the current plan
     *        iteratively mating the best and worst subtours until the new plan is complete.
     */
    void crossover();

    /**
     * @brief Removes intersections between the subtours in the current plan.
     * @param Path1 to be altered.
     * @param Path2 to be altered.
     */
    void removeIntersectionsFromPath(Path *path1, Path *path2);

    /**
     * @brief Removes intersections in the whole Global Plan.
     */
    void removeIntersections();

    /**
     * @brief Performs 2-opt heuristic to locally optimize each subtour.
     */
    void simpleOptSwap();

    /**
     * @brief Performs extended 2-opt heuristic to each pair of subtours in the current global plan.
     */
    void extendedOptHeuristic();

    /**
     * @brief Performs 2-opt heuristic to locally optimize two subtours.
     *
     * @param path1 to be altered.
     * @param path2 to be altered.
     */
    void twoPathOptSwap(Path *path1, Path *path2);
};

#endif // GLOBAL_PLANNER_H