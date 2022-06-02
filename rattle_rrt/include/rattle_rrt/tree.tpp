/*
# tree.tpp

The primary template for RRT tree operations. <typename T>
is a std::shared_pointer<stateSpace> class.

Credit to Georgia Tech RoboJackets RRT 2D demonstration from
which the 2D template was originally adapted.

Distributed under Apache License v2.0

Keenan Albee, 2020
MIT Space Sytems Lab
*/

#ifndef TREE_H_
#define TREE_H_
// #define ENABLE_LOGGING

#include <rattle_rrt/state_space.tpp>
#include <rattle_rrt/types.h>
#include <rattle_rrt/state_space_2d.h>
#include <rattle_rrt/state_space_4d.h>
#include <rattle_rrt/state_space_6d_ground.h>
#include <rattle_rrt/state_space_6d_iss.h>
#include <rattle_rrt/state_space_13d.h>

#include <flann/flann.hpp>

#include <stdlib.h>
#include <utility>
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <vector>
#include <list>
#include <deque>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <unistd.h>  // debugging

#include <ros/package.h>

namespace rrt {

/* **************************************************************************
 * Node
 * ************************************************************************** */
template <typename T>
// typedef rrt::Vec6 T;
class Node {
  /* Base class for an RRT tree node, of state type <T>. A Node is a wrapper around a state.
  */
  private:
    std::list<Node<T>*> _children;  // List of child Nodes
    Vec3 _F;  // Force action to arrive here
    Vec3 _T;  // Torque action to arrive here

  public:
    Node<T>* parent_ = nullptr;  // The parent of the Node
    T state_;         // The state of the Node, MUST BE EIGEN
    std::vector<double> state_vec_;  // A vector of the Node's state, needed for Flann

    int is_trapped_;  // did the node get trapped? (used to skip)

    explicit Node(const T& state): 
      state_(state), state_vec_(state.size()) {
      for (int i = 0; i < state.size(); ++i)
          state_vec_[i] = state(i);
      _F << 0, 0, 0;
      _T << 0, 0, 0;
      is_trapped_ = 0;
    }

    Node(const T& state, Node<T>* parent):
      parent_(parent), state_(state), state_vec_(state.size()) {
      parent_->_children.push_back(this);
      for (int i = 0; i < state.size(); ++i)
          state_vec_[i] = state(i);
      _F << 0, 0, 0;
      _T << 0, 0, 0;
      is_trapped_ = 0;
    }

    Node(const T& state, Node<T>* parent, Vec3 prev_F, Vec3 prev_T):
      _F(prev_F), _T(prev_T), parent_(parent), state_(state), state_vec_(state.size()) {
      parent_->_children.push_back(this);
      for (int i = 0; i < state.size(); ++i)
          state_vec_[i] = state(i);
      is_trapped_ = 0;
    }

    /* ************************************************************************** */
    int depth() const {
      /* Gets the number of ancestors, 0 if no parent.
       */
      int n = 0;
      for (Node<T>* ancestor = parent_; ancestor != nullptr;
           ancestor = ancestor->parent_) {
          n++;
      }
      return n;
    }
};


/* **************************************************************************
 * Tree (RRT)
 * ************************************************************************** */
template <typename T>
// typedef rrt::Vec6 T;
class Tree {
  /*
  This provides a base class for RRT trees. Core functions specific to
  a state space are defined in a custom state_space*.h file, inherited from state_space.h.

  <T> is the type that represents an Eigen state within the stateSpace that
  the tree is searching.
  - rrt::Vec2
  - rrt::Vec6
  - rrt::Vec13

  USAGE:
  1) Create a new Tree with the appropriate StateSpace RRT::Tree<T> tree(stateSpacePtr);

  2) Set the start and goal states, along with optional params

    tree->start_state_ = T s;
    tree->goal_state_ = T g;
    tree->set_params( ... );

  3) Run the RRT algorithm!
    Option (1) Call the build_RRT() method - it will grow the tree
      until it finds a solution or runs out of iterations.

    Option (2) Call initialize_tree(), then extend() repeatedly. Basdically, mimic calls of build_RRT().

  4) 
  - Use getPath() to get the series of states that make up the solution
  - Use getEdges() to get a std::vector of RRT edges(two Nodes)
  */
 private:
    std::shared_ptr<StateSpace<T>> _stateSpacePtr{};  // the stateSpacePtr object to reference for searches
    std::deque<Node<T>> _nodes{};                                        // deque of Nodes
    flann::Index<flann::L2_Simple<double>> _kdtree;                      // kdtree for NN
    std::unordered_map<T, Node<T>*, std::function<size_t(T)>> _nodemap;  // hashmap of state -> node
    std::ofstream _outfile;

 public:
    T goal_state_;  // goalState
    T start_state_; // startState

    int dimensions_;  // dimension of state vector
    int max_iterations_;    // max iterations before quitting
    double goal_bias_;      // [0,1] to bias searching toward goal
    double goal_max_dist_;   // maximum distance to declare success (includes weighting)
    double timestep_;      // time period of evaluation
    double max_timestep_;  // max time period of evaluation
    bool is_ASC_enabled_;  // enable adaptive step control
    bool remember_actions_;  // should actions from the last node be saved?
    std::vector<Obstacle> obstacles_;  // obstacles to check for
    bool params_set_;   // are the parameters set?

    unsigned int seed_;  // random seed
    Node<T>* best_node_;  // the best node (in terms of lowest cost)

    // Constructors
    Tree(const Tree&) = delete;            // turns off copy constructor
    Tree& operator=(const Tree&) = delete;  // turns off assignment operator

    Tree(std::shared_ptr<StateSpace<T>> stateSpacePtr) :  
      _stateSpacePtr(stateSpacePtr),
      _kdtree(flann::KDTreeSingleIndexParams()),  // _kdtree(flann::LinearIndexParams()),  // performs a brute force search
      _nodemap(1000, state_hash),  // number of buckets, hash function
      dimensions_(stateSpacePtr->get_d()),
      params_set_(false),
      best_node_(nullptr) {  // size of the state vector

      // initialize
      // srand(time(NULL));  // seed rand_r
      seed_ = time(NULL);

      #ifdef ENABLE_LOGGING
      _outfile.open("rrt_output.csv");
      #endif
   }  // end Tree()

    /* ************************************************************************** */
    /* Create root node from provided start state.
     */
    void initialize_tree(const T start_state, const T goal_state, std::vector<Obstacle>& obstacles) {
      /* Set up essential data structures after start and goal state set.
      */
      if (!_stateSpacePtr->is_state_valid(start_state, obstacles) ){
        throw std::invalid_argument("The start state is invalid!");
      }

      if (!_stateSpacePtr->is_state_valid(goal_state, obstacles) ){
        throw std::invalid_argument("The goal state is invalid!");
      }

      reset(true);  // reset the root
      _nodes.emplace_back(start_state);  // Node(state)
      _nodemap.insert(std::pair<T, Node<T>*>(start_state, &_nodes.back()));
      // build the kdtree, adding in the first node
      // data ptr, n_points, dim
      _kdtree.buildIndex(flann::Matrix<double>(
                          const_cast<double*>( reinterpret_cast<const double*>(&(rootNode()->state_)) ),
                          1, dimensions_));                
    }

    /* ************************************************************************** */
    /* Set parameters before running.
     */
    void set_params(int maxIterations, bool ASCEnabled, double timestep, double max_timestep,
                    double goal_bias, double goal_max_dist, bool remember_actions, std::vector<Obstacle> obstacles_in = std::vector<Obstacle>()) {
        /* Defaults to NO obstacles if not provided.
        */
        max_iterations_ = maxIterations;
        is_ASC_enabled_ = ASCEnabled;
        timestep_ = timestep;
        max_timestep_ = max_timestep;

        if (goal_bias < 0 || goal_bias > 1) {
          throw std::invalid_argument("The goal bias must be a number between 0.0 and 1.0");
        }
        goal_bias_ = goal_bias;

        goal_max_dist_ = goal_max_dist;
        remember_actions_ = remember_actions;

        if (obstacles_in.size() > 0){
          obstacles_ = obstacles_in;
        }

        params_set_ = true;
    }

    /* ************************************************************************** */
    bool build_RRT() {
      /* Executes the RRT algorithm with the given start state.
      */
      // Have the parameters been set?
      if (params_set_ == false) {
        return false;
      }

      initialize_tree(start_state_, goal_state_, obstacles_);  // initialize the tree

      //  Grow the tree until goal reached or run out of iterations
      Node<T>* newNode;
      double r;
      for (int i = 0; i < max_iterations_; i++) {
        r = rand_r(&seed_) / (static_cast<double>(RAND_MAX));  // r normazlied to [0,1]
        if (r < goal_bias_) {
            newNode = extend(goal_state_);  // goal biased
        } else {
            newNode = extend(_stateSpacePtr->find_x_rand(&seed_));  // generate a random point in the state space
        }

        // got a newNode
        if (newNode){
          update_best_node(newNode);  // update the best_node

          // success if within goal tolerance
          if (_stateSpacePtr->get_dist(newNode->state_, goal_state_) < goal_max_dist_) {

            #ifdef ENABLE_LOGGING
            _outfile.close();
            write_path("rrt_output.csv");
            #endif
            
            std::cout << "Node attempts: " << i << std::endl;
            return true;
          }
        }
      }
      std::cout << "Node attempts: " << max_iterations_ << std::endl;

      #ifdef ENABLE_LOGGING
      _outfile.close();
      #endif

      return false;
    }


    void update_best_node(Node<T>* new_node) { 
      /* Update the lowest cost node.
      */
     // if not set, take new_node
     if (!best_node_) {
       best_node_ = new_node;
     }
     else {
      double new_dist = _stateSpacePtr->get_dist(new_node->state_, goal_state_);
      double best_dist = _stateSpacePtr->get_dist(best_node_->state_, goal_state_);
      if (new_dist < best_dist) {
        best_node_ = new_node;
      }
     }
    }

    /* ************************************************************************** */
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
      /* Grow the tree in the direction of target using find_x_new(), check for x_new/segment validity,
      * and store the Node. The closest tree point is used for source if nullptr
      */
      // If not given a source point, try to find nearest neighbor of target
      if (!source) {
          source = nearest(target, nullptr);
          if (!source || source->is_trapped_ == 1) {
              return nullptr;
          }
      }

      // Get a state that's closest in the direction of target from source
      int extend_success = 1;
      T x_new;
      Vec3 prev_F, prev_T;

      if (is_ASC_enabled_ && remember_actions_) {
          std::tie(x_new, prev_F, prev_T, extend_success) =
               _stateSpacePtr->find_x_new_data(source->state_, target, timestep_, max_timestep_);
      }
      else if (is_ASC_enabled_ && !remember_actions_) {
          x_new = _stateSpacePtr->find_x_new(source->state_, target, timestep_, max_timestep_);
      }
      else if (!is_ASC_enabled_ && remember_actions_) {
          std::tie(x_new, prev_F, prev_T, extend_success) =
               _stateSpacePtr->find_x_new_data(source->state_, target, timestep_);
      }
      else {
          x_new = _stateSpacePtr->find_x_new(source->state_, target, timestep_);
      }

      // What if no x_new can be produced from x_near?
      if (extend_success == 0) {
          source->is_trapped_ = 1;  // mark node as trapped
          return nullptr;
      }

      //  Make sure there's actually a valid path from x_near to x_new.  If not, abort
      if (!_stateSpacePtr->is_segment_valid(source->state_, x_new, obstacles_) ) {
          return nullptr;
      }

      #ifdef ENABLE_LOGGING
      write_x_new(x_new, source->state_);
      #endif

      // Add a node to _nodes and the _kdtree for x_new (Node is instantiated here!)
      if (remember_actions_) {
          _nodes.emplace_back(x_new, source, prev_F, prev_T);  // Node(state, parent, prev_F, prev_T)
      } else {
          _nodes.emplace_back(x_new, source);  // Node(state, parent)
      }

      _kdtree.addPoints(flann::Matrix<double>(_nodes.back().state_vec_.data(), 1, dimensions_));
      _nodemap.insert(std::pair<T, Node<T>*>(x_new, &_nodes.back()));
      return &_nodes.back();  // pointer to last added Node
    }

    /* ************************************************************************** */
    Node<T>* nearest(const T& state, double* distanceOut = nullptr) {
      /* Find the Node in the tree closest to state, using a FLANN search (O(log(N))
      Notably, if only a single node has been added then that node will be the output (start_state).
       */
      flann::Matrix<double> query;

      // Conversion to std::vector from Eigen::vector
      std::vector<double> state_vec;
      for (int i = 0; i < state.size(); ++i) {
        state_vec.push_back(state(i));
      }

      query = flann::Matrix<double>(state_vec.data(), 1, dimensions_);

      std::vector<int> i(query.rows);
      flann::Matrix<int> indices(i.data(), query.rows, 1);

      std::vector<double> d(query.rows);
      flann::Matrix<double> dists(d.data(), query.rows, 1);

      // state for NN, indices to return, dists to return, number of neighbors, search parameters
      _kdtree.knnSearch(query, indices, dists, 1, flann::SearchParams());

      T point;
      point = (T)_kdtree.getPoint(indices[0][0]);  // <T> state, first NN of tree 1
      if (distanceOut)  // return distanceOut if it is requested
          *distanceOut = _stateSpacePtr->get_dist(state, point);

      return _nodemap[point];
    }

    /* ************************************************************************** */
    void reset(bool eraseRoot = false) {
      /* Removes Nodes from _nodes and _nodemap so build_RRT() can run again.
       */
        _kdtree = flann::Index<flann::L2_Simple<double>>(
          flann::KDTreeSingleIndexParams());
        if (eraseRoot) {
          _nodes.clear();
          _nodemap.clear();
        }
        else if (_nodes.size() > 1) {
          T root = rootNode()->state_;
          _nodemap.clear();
          _nodes.clear();
          _nodes.emplace_back(root);  // Node(state)
          _nodemap.insert(std::pair<T, Node<T>*>(root, &_nodes.back()));
          _kdtree.buildIndex(flann::Matrix<double>(
                  const_cast<double*>(reinterpret_cast<const double*>(&(rootNode()->state_)))
                  , 1, dimensions_));
      }
    }

    /* ************************************************************************** */
    std::vector<Node<T>> get_path_nodes(const Node<T>* target = nullptr,
                                        bool reverse = false) const {
      /* Returns a vector of nodes. Called by outside functions.
       */
      std::vector<Node<T>> path;

      // Recursively add nodes to vector
      add_path_nodes( [&path](const Node<T> NodeI) {path.push_back(NodeI);},
                      target,
                      reverse);
      return path;
    }

    /* ************************************************************************** */
    std::vector<std::tuple<T, T>> get_rrt_edges( ) const {
      /* Returns a vector of tuples of edge pairs. Called by outside functions.
         TODO: can make this more efficient with pass by ref.

         @return:
         vec of std::tuple<T, T>, of <child, parent>
       */
      std::vector<std::tuple<T, T>> edges;

      // Recursively add nodes to vector
      for (size_t i = 0; i < _nodes.size(); i++) {
        if (_nodes[i].parent_ != nullptr) {
        std::tuple<T, T> edge = std::tie(_nodes[i].parent_->state_, _nodes[i].state_);
        // std::tuple<T, T> edge = std::tie(_nodes[i].state_, _nodes[i].state_);
        edges.push_back(edge);
        }
      }
      return edges;
    }

    /* ************************************************************************** */
    void add_path_nodes(std::function<void(const Node<T> NodeI)> callback,
                                           const Node<T>* target = nullptr,
                                           bool reverse = false) const {
      /* Returns a vector of nodes, starting with the last node added.
      */
      const Node<T>* node = (target != nullptr) ? target : best_node_;  // grab the lastNode

      if (reverse) {
        while (node) {
            callback(*node);
            node = node->parent_;
        }
      }
      else {
        // Collect states in leaf->root order...
        std::vector<const Node<T>*> nodes;
        while (node) {
            nodes.push_back(node);
            node = node->parent_;
        };
        // ...then reverese the order of nodes
        for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++) {
            callback(**itr);
        }
      }
    }

    /* ************************************************************************** */
    const Node<T>* rootNode() const {
      /* The root Node, or nullptr if none exists
       */
      if (_nodes.empty()) return nullptr;
      return &_nodes.front();
    }


    /* ************************************************************************** */
    const Node<T>* lastNode() const {
      /* The most recent Node added to the tree
      */
      if (_nodes.empty()) return nullptr;
      return &_nodes.back();
    }


    /* ************************************************************************** */
    const std::deque<Node<T>>& allNodes() const { 
      /* Provide all Nodes in a deque
      */
      return _nodes;
    }

    /* ************************************************************************** */
    void write_x_new(T x_new) {
      /* Write out a line of type <T>
       */
      for (int i = 0; i < x_new.size(); ++i) {
          _outfile << x_new(i) << ",";
      }
      _outfile << "\n";
    }

    /* ************************************************************************** */
    void write_x_new(T x_new, T x_nearest) {
      /* Write out a line of two type <T>s
       */
      for (int i = 0; i < x_new.size(); ++i) {
          _outfile << x_new(i) << ",";
      }
      _outfile << "\n";
      for (int i = 0; i < x_nearest.size(); ++i) {
          _outfile << x_nearest(i) << ",";
      }
      _outfile << "\n";
    }

    /* ************************************************************************** */
    void write_path(std::string RRT_OUTPUT_FILE) {
      /* Write out the solution path to file
      */
      std::ofstream path_file;
      std::string DATA_PATH = ros::package::getPath("data")+"/output/" + RRT_OUTPUT_FILE;
      path_file.open(DATA_PATH);

      std::vector<Node<T>> path = get_path_nodes();
      // printf("Solution path length:\n");
      // printf("%lu\n", path.size());

      // write the RRT waypoints
      for (auto each : path) {
          for (int i = 0; i < each.state_.size(); ++i) {
              path_file << each.state_(i) << ",";
          }
          path_file << "\n";
      }

      // write the goal state
      for (int i = 0; i < goal_state_.size(); ++i) {
          path_file << goal_state_(i) << ",";
      }
      path_file << "\n";
      path_file.close();
    }

    /* ************************************************************************** */
    static size_t state_hash(T state) {
      /* Templated hash function, so that we can look up nodes using states
      */
      // Combine hashes into a single seed (key) for this state
      size_t seed = 0;
      for (int i = 0; i < state.size(); i++) {
          boost::hash_combine(seed, state(i));
      }
      return seed;
    }
};
}  // namespace rrt

#endif  // TREE_H_
