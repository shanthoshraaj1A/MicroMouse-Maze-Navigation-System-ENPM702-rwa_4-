/**
 * @file dfs.hpp
 * @brief Declares the DFS (Depth-First Search) planning algorithm.
 *
 * @details
 * DFS expands neighbors in NESW order (North, East, South, West) and uses
 * only the Maze’s discovered internal map. It returns the first valid
 * path found to the goal.
 *
 * DFS is used as the primary planning algorithm for real robot execution
 * and interacts with Robot through the polymorphic Algorithm base class.
 * 
 * @authors 
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
#include "algorithm.hpp"


namespace micro_mouse {
  /**
   * @class DFS
   * @brief Concrete Algorithm implementing depth-first search (DFS).
   *
   * @details
   * DFS explores neighbor nodes using a LIFO stack (NESW ordering: North,
   * East, South, West). It uses only the Maze's discovered internal map and
   * returns the first valid path found to the goal. This class overrides
   * the abstract API provided by `Algorithm` and owns its private stack
   * (composition).
   */
  class DFS final : public Algorithm {
    private:
      /** @brief LIFO stack used to drive depth-first exploration frontier.
       * DFS owns its stack (Composition) */
      std::stack<Node> stack_;
      /**
       * @brief Check if a candidate node is inside the maze and not a wall.
       *
       * @param next_node Candidate `Node` representing the robot's next position.
       * @return True if the node lies inside the maze bounds and is free to visit.
       */
      bool is_valid_move(const Node& next_node) const;

    public:
      /**
       * @brief Construct a DFS planner.
       * @param maze Mutable reference to the maze used for planning.
       * @param start The start node for the search.
       * @param goal The goal coordinates as an `{x, y}` array.
       */
      DFS(Maze& maze, Node start, std::array<int, 2> goal);


      /**
       * @brief Obtains a const reference to the DFS internal stack.
       *
       * @return const reference to the internal LIFO `std::stack<Node>` used
       *         by the algorithm.
       */
      [[nodiscard]] const std::stack<Node>& get_stack() const noexcept {
        return stack_;
      }
      
      /**
       * @brief Replace the internal stack (useful for testing / warm-starting).
       * @param stack A copy of a stack to become the internal state.
       */
      void set_stack(const std::stack<Node>& stack) {
        stack_ = stack;
      }

      /**
       * @brief Compute a path from the current start to the goal using DFS.
       *
       * @details
       * Performs depth-first exploration using the internal stack and the
       * maze's discovered map. Returns the first valid path encountered to
       * the goal cell.
       *
       * @return Vector of `Node` objects representing the path (start -> goal).
       */
      virtual std::vector<Node> solve() override;
      /**
       * @brief Generate a path by following predecessors from `curr` back to `start`.
       *
       * @param curr Node referencing the goal (or intermediate) node to begin backtracking.
       * @return Vector of `Node` objects representing the reconstructed path.
       */
      std::vector<Node> backtrack(const Node& curr) const;



    };//class DFS

}// End micro_mouse namespace