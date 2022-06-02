/*
# state_space.h

An abstract state space template, for use in RRT searches. Some essential RRT concepts for
exploring the state space must be defined in inherited classes for the system of interest.

Keenan Albee, 2021
MIT Space Systems Lab

Adapted from a Georgia Tech RoboJackets template.
*/

#ifndef STATE_SPACE_H_
#define STATE_SPACE_H_

#include <iostream>

#include <rattle_rrt/types.h>
#include <rattle_rrt/obstacle.h>


namespace rrt {

/*
 * A state space represents the set of possible states for a planning problem.
 * This includes valid state transitions, finding random states, distance function, etc.
 * This class is abstract and must be subclassed in order to provide actual functionality.
 */
template <typename T>
class StateSpace {
 public:
    StateSpace() {}
    virtual ~StateSpace() {}  // destructor

    /*
     * Generate a random state within the bounds of the state space.
     * Return: A random state.
     */
    virtual T find_x_rand(unsigned int* seed) const = 0;

    /*
     * Return: a new state, along with the info for arriving there.
     */
    virtual std::tuple<T, Vec3, Vec3, int> find_x_new_data(const T& source, const T& target, double TIMESTEP) = 0;

    /*
     * Return: a new state, along with the info for arriving there, adaptive.
     */
    virtual std::tuple<T, Vec3, Vec3, int> find_x_new_data(
        const T& source, const T& target, double TIMESTEP, double MAX_TIMESTEP) = 0;

    /*
     * Finds a state in the direction of @target from @source.state().
     * This new state will potentially be added to the tree.  No need to do
     * any validation on the state before returning, the tree will handle
     * that.
     */
    virtual T find_x_new(const T& source, const T& target, double stepSize) = 0;

    /*
     * An overloaded version designed for use in adaptive stepsize control.
     */
    virtual T find_x_new(const T& source, const T& target,
                                double minStepSize, double maxStepSize) = 0;

    /*
     * Calculate the distance between two states. Ideally, this is the cost-to-go
     */
    virtual double get_dist(const T& from, const T& to) const = 0;

    /*
     * Check if a state is within bounds and obstacle-free
     */
    virtual bool is_state_valid(const T& state, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) = 0;

    /*
     * Check motion validity from one state to another (check along path).
     * Returns a boolean indicating whether or not a direct motion from one state to another is valid.
     */
    virtual bool is_segment_valid(const T& start, const T& goal, const std::vector<Obstacle>& obstacles = std::vector<Obstacle>()) = 0;

    /*
     * Return the dimension of this state space
     */
    virtual int get_d() const = 0;
};

}  // namespace rrt
#endif  // STATE_SPACE_H_
