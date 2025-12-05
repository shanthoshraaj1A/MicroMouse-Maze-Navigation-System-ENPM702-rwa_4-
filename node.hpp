/**
 * @file node.hpp
 * @brief Defines the Node data structure used by BFS and DFS planning algorithms.
 *
 * @details
 * A Node corresponds to a single maze cell and stores:
 * - `x_`, `y_` : integer grid coordinates
 * - d_         : direction of motion used to reach this node
 * - parent_x   : x coordinate of parent node
 * - parent_y   : y coordinate of parent node
 *
 * Direction values:
 *   'n' = north  
 *   'e' = east  
 *   's' = south  
 *   'w' = west  
 *
 * Nodes implement a strict ordering (`operator<`) so they can be
 * stored in ordered STL containers such as `std::set` and `std::map`.
 * @authors 
 * Team Members:
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#pragma once
#include <cmath>
#include <list>

  /**
   * @class Node
   * @brief Lightweight value type representing a maze cell (state) used by
   *        planning algorithms.
   *
   * @details
   * Node stores the grid coordinates, the direction used to reach this cell
   * and the parent coordinates (used by backtracking to reconstruct paths).
   * It implements strict ordering so instances can be stored in ordered
   * containers (e.g., std::set, std::map) used by the algorithms.
   */
  class Node {

    private:
      /** @brief X coordinate (column) of the cell. */
      int x_{0};

      /** @brief Y coordinate (row) of the cell. */
      int y_{0};

      /** @brief Direction used to enter this cell ('n','e','s','w'). */
      char d_{'n'};

      /** @brief Parent cell X coordinate used for backtracking. */
      int parent_x_{};

      /** @brief Parent cell Y coordinate used for backtracking. */
      int parent_y_{};
    public:
      /** @brief Default-construct an empty node (0,0,'n'). */
      Node() = default;

      /**
       * @brief Create a Node with full state and parent coordinates.
       * @param x X coordinate (column) of the cell
       * @param y Y coordinate (row) of the cell
       * @param d Direction used to enter this cell ('n','e','s','w')
       * @param parent_x X coordinate of the predecessor cell
       * @param parent_y Y coordinate of the predecessor cell
       */
      Node(int x,
           int y,
           char d,
           int parent_x,
           int parent_y)
          : x_(x),
        y_(y),
        d_(d),
        parent_x_(parent_x),
        parent_y_(parent_y) {
      }

      /** @brief X coordinate (column) of this node. */
      [[nodiscard]] int        get_x() const noexcept { return x_; }

      /** @brief Y coordinate (row) of this node. */
      [[nodiscard]] int        get_y() const noexcept { return y_; }

      /** @brief Direction used to reach this node ('n','e','s','w'). */
      [[nodiscard]] char       get_d() const noexcept { return d_; }

      /** @brief Parent node X coordinate (used by backtracking). */
      [[nodiscard]] int get_parent_x() const noexcept { return parent_x_; }

      /** @brief Parent node Y coordinate (used by backtracking). */
      [[nodiscard]] int get_parent_y() const noexcept { return parent_y_; }


      /** @brief Set the X coordinate for this node. */
      void set_x(       int        x) { x_ = x; }

      /** @brief Set the Y coordinate for this node. */
      void set_y(       int        y) { y_ = y; }

      /** @brief Set the direction used to reach this node. */
      void set_d(       char       d) { d_ = d; }

      /** @brief Set the parent X coordinate. */
      void set_parent_x(int parent_x) { parent_x_ = parent_x; }

      /** @brief Set the parent Y coordinate. */
      void set_parent_y(int parent_y) { parent_y_ = parent_y; }

      /**
       * @brief Strict ordering operator used for storing nodes in ordered
       *        containers (std::set, std::map).
       *
       * @details Override of '<' operator for the visited_.find 
       * to traverse the tree ignoring parent x/y 
       * The operator compares nodes by (y, x) — parent coordinates
       * are intentionally ignored so identity is solely the cell position.
       * This is convenient for search data structures that care only about
       * visited positions and not how they were reached.
       *
       * @param other Node to compare against.
       * @return true if this node is ordered before `other`.
       */
      bool operator<(const Node& other) const {
        return std::tie(y_, x_) < std::tie(other.y_, other.x_);
      }

  };  // class Node

