/**
 * @file bfs.hpp
 * @brief Breadth-First Search (BFS) planning algorithm interface.
 *
 * @details
 * BFS expands neighbors with a FIFO queue in NESW order (North, East, South, West) leveraging
 * the robot's current internal representation of the Maze discovered. It returns the first valid
 * path found to the goal.
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
#include <queue>
#include <set>
#include <array>
#include "algorithm.hpp"

namespace micro_mouse {

  /**
   * @class BFS
   * @brief Breadth-First Search planning algorithm.
   *
   * @details Implements standard graph BFS using a FIFO queue. The algorithm
   * expands nodes in NESW order (North, East, South, West) using the robot's
   * internal Maze representation and returns the first valid path found to the
   * configured goal.
   */
  class BFS final : public Algorithm {
    
    private:
      /** @brief Frontier FIFO queue used by BFS to store nodes. 
       * BFS owns its queue (Composition) */
      std::queue<Node> queue_;
      /**
       * @brief Check whether a prospective node is a valid move.
       * @param next_node Node representing a candidate location to move to.
       * @return True if the location is in bounds and not blocked by walls,
       *         false otherwise.
       */
      bool is_valid_move(Node next_node) const;


    public:
      /**
       * @brief Construct a BFS planner.
       * @param maze  Reference to the mutable Maze used for planning (BFS does
       *              not take ownership).
       * @param start Starting Node for the search (robot's current position).
       * @param goal  Goal coordinates as {x, y}.
       */
      BFS(Maze& maze, Node start, std::array<int, 2> goal);

      /**
       * @brief Access the algorithm's internal FIFO queue (for testing/debug).
       * @return Read-only reference to the underlying search queue.
       */
      [[nodiscard]] const std::queue<Node>& get_queue() const noexcept {
        return queue_;
      }
      
      /**
       * @brief Replace the internal FIFO queue. Useful for replaying or
       *        unit-testing specific queue states.
       * @param queue New queue to assign (copy semantics).
       */
      void set_queue( const std::queue<Node>&  queue) {
        queue_ = queue;
      }

      /**
       * @brief Compute a path from the current start to the goal using BFS.
       * @return Sequence of Node objects representing the planned path.
       *
       * @note Overrides Algorithm::solve(). BFS guarantees the shortest path in
       *       terms of number of moves on an unweighted grid.
       */
      virtual std::vector<Node> solve() override;

      /**
       * @brief Reconstructs the path from a discovered goal node back to start.
       * @param curr The node (usually a goal node) from which to begin
       *             backtracking towards the start.
       * @return Ordered vector of Node objects from start -> goal.
       */
      std::vector<Node> backtrack(const Node& curr) const;

    };

}// End micro_mouse namespace