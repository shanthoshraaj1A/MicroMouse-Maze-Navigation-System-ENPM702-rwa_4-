/**
 * @file algorithm.hpp
 * @brief Abstract base class for micromouse path-planning algorithms.
 *
* @details
 * Common interface for DFS, BFS, A*, etc. Holds shared state (maze, start,
 * goal, visited set) and defines the required `solve()` method.
 *
 * Designed for repeated replanning: the same instance is reused after new
 * walls are discovered. The maze is held by non-const reference to allow
 * external updates.
 *
 *@authors 
 * Team Members:
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#pragma once
#include <memory>
#include <string>
#include <stack>
#include <set>
#include <array>
#include "node.hpp"
#include "maze.hpp"

namespace micro_mouse {
  
  
/**
 * @class Algorithm
 * @brief Base class for all path-planning algorithms.
 *
 * @details Provides shared data and interface. Concrete algorithms override
 *          `solve()` to return a path from the current position to the goal.
 */
  class Algorithm : public std::enable_shared_from_this<Algorithm> {

    protected:
      //==================================================================//
      //                    Protected members (derived-class API)
      //==================================================================//
      /** @brief Algorithm kind ("DFS", "BFS", etc.) used for logging and debug. */
      std::string        type_;

      /** @brief Set of visited (x,y) Node positions used during search. */
      std::set<Node>  visited_;

      /** @brief Current start node (robot's present position). */
      Node              start_;

      /** @brief Goal coordinates as {x, y}. */
      std::array<int, 2> goal_;

      /** @brief Non-owning mutable reference to the Maze used for planning. */
      Maze&              maze_; 

    public:
      /**
       * @brief Constructs the algorithm.
       * @param type  Name of the algorithm ("DFS", "BFS", ...)
       * @param maze  Mutable reference to the shared maze
       * @param start Current robot node
       * @param goal  Goal cell {x, y}
       */
      Algorithm(const std::string        type, 
                const std::set<Node>& visited,
                const Node              start,
                const std::array<int, 2> goal,
                Maze&                    maze
              ): type_{type}, 
              visited_{visited}, 
              start_{start}, 
              goal_{goal},
              maze_{maze} {
      }

      /** @brief Virtual destructor – enables polymorphic deletion. */
      virtual ~Algorithm() = default;
      //==================================================================//
      //                            Core API
      //==================================================================//
      /**
       * @brief Compute a planned path from start to goal.
       * @return Vector of `Node` objects representing the path.
       */
      virtual std::vector<Node> solve() = 0;

      //==================================================================//
      //                            Accessors
      //==================================================================//
      /** @brief Returns the name of the concrete algorithm. */
      [[nodiscard]] std::string        get_type()    const noexcept { return type_; }

      /** @brief Returns a copy of the current visited set (for debugging/logging). */
      [[nodiscard]] std::set<Node>     get_visited() const noexcept { return visited_; }

      /** @brief Returns the current start node. */
      [[nodiscard]] Node               get_start()   const noexcept { return start_; }
      /** @brief Returns the current goal coordinates `{x, y}`. */
      [[nodiscard]] std::array<int, 2> get_goal()    const noexcept { return goal_; }

      /**
       * @brief Returns a mutable reference to the maze.
       *
       * @note Provided for the robot controller to update discovered walls.
       *       Planning algorithms must never modify the maze themselves.
       */      
      [[nodiscard]] Maze&              get_maze()    const noexcept { return maze_; }
      
      
      //==================================================================//
      //                            Mutators
      //==================================================================//
      /** @brief Updates the algorithm name. */
      void set_type(    const std::string        type) {type_    = type;}

      /** @brief Replaces the entire visited set – useful for warm-starting. */
      void set_visited( const std::set<Node>  visited) {visited_ = visited;}

      /** @brief Updates the start node (called after each successful move). */
      void set_start(   const Node              start) {start_   = start;}

      /** @brief Changes the goal – useful for multi-goal runs or return trips. */
      void set_goal(    const std::array<int, 2> goal) {goal_    = goal;}
      

  
};


}

