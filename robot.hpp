/**
 * @file robot.hpp
 * @brief Declares the Robot class that interacts with the Maze simulator.
 *
 * @details
 * The Robot class is responsible for:
 * - Maintaining the robot's internal pose (x, y, direction)
 * - Executing motion commands through the Maze API
 * - Reading live sensors (front, left, right)
 * - Updating the internal map stored in the Maze class
 * - Running a planning algorithm (DFS under Option A)
 * - Following a path and triggering replanning when the *front sensor*
 *   detects a previously unknown wall
 *
 * This class is the only component permitted to use live sensor data.
 * Planning algorithms (DFS/BFS) use ONLY the discovered map.
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#pragma once
#include <memory>
#include "algorithm.hpp"
#include "maze_api.hpp"
#include "node.hpp"
#include <vector>
#include <map>


namespace micro_mouse {

  /**
   * @class Robot
   * @brief High-level controller coordinating sensing, planning, and execution.
   *
   * @details
   * Robot owns a Maze instance and an Algorithm pointer. It performs:
   * - `start_exploration()` to compute an initial path
   * - `follow_path_with_replan()` to execute and replan when blocked
 */
  /**
   * @class Robot
   * @brief High-level robot controller responsible for sense-plan-act cycle.
   *
   * @details Robot owns the `Maze` instance (composition) and holds a
   * pointer to the selected planning `Algorithm` (aggregation). It reads
   * live sensors, updates the discovered map, executes motion commands via
   * the `Maze` API, and coordinates planning + replanning during exploration.
   */
  class Robot {

    private:
      // COMPOSITION: Robot has-a Maze (Lifecycles are bound)
      /** @brief The robot's owned Maze instance (discovered map + control API). */
      Maze maze_;

      // AGGREGATION: Robot uses-an Algorithm (Lifecycles are independent)
      /** @brief Shared pointer to the currently assigned planning Algorithm. */
      std::shared_ptr<Algorithm> algo_;
      
      // Robot State
      /** @brief Robot X coordinate (column). */
      int   x_;

      /** @brief Robot Y coordinate (row). */
      int   y_;

      /** @brief Robot facing direction as a character ('n','e','s','w'). */
      char  d_;     // 'n', 's', 'e', 'w'

      /** @brief Numeric angle corresponding to `d_` (0,90,180,270). */
      int   d_ang_; // 0, 90, 180, 270

      /** @brief Conversion map from numeric angle -> direction char. */
      std::map<int, char> ang_to_d_;

      /** @brief Conversion map from direction char -> numeric angle. */
      std::map<char, int> d_to_ang_;
      
      
      /**
       * @brief Ordered list of nodes the robot has visited (execution trace).
       *
       * @details The path vector is used by the robot while following a
       * computed plan (populated by planning algorithms) and for logging
       * / visualization purposes.
       */
      std::vector<Node> path_;

    public:
      /**
       * @brief Default-construct a Robot.
       *
       * @details Produces a robot at (0,0) facing north on a default 16x16
       * maze. The algorithm pointer is initially null and the path contains
       * a single dummy Node representing the start.
       */
      Robot() 
          : maze_(Maze(16, 16)), 
            x_(0), 
            y_(0), 
            d_('n'),
            d_ang_(0), 
            algo_(nullptr),
            path_{Node(0, 0, 'n', -1, -1)},
            ang_to_d_{{0, 'n'}, {90, 'e'}, {180, 's'}, {270, 'w'} },
            d_to_ang_{{'n', 0}, {'e', 90}, {'s', 180}, {'w', 270} }
      {}

      /**
       * @brief Construct a Robot with custom initialisation.
       * @param x Initial x position (column)
       * @param y Initial y position (row)
       * @param d Initial facing direction ('n','e','s','w')
       * @param d_ang Numeric angle corresponding to `d` (0/90/180/270)
       * @param width Maze width (columns)
       * @param height Maze height (rows)
       * @param algo Shared pointer to the planning algorithm to use.
       * @param path Initial path vector to follow / replay.
       */
      Robot(int x, int y, char d, int d_ang, int width, int height, 
        std::shared_ptr<Algorithm> algo,
        std::vector<Node> path) 
          : maze_(Maze(width, height)), 
            x_(x), 
            y_(y), 
            d_(d),
            d_ang_(d_ang),
            algo_(algo),
            path_(path),
            ang_to_d_{{0, 'n'}, {90, 'e'}, {180, 's'}, {270, 'w'} },
            d_to_ang_{{'n', 0}, {'e', 90}, {'s', 180}, {'w', 270} }
      {}
      
      /**
       * @brief Initiate exploration using the assigned planning algorithm.
       *
       * @return Vector of Nodes representing the planned path from the
       *         current robot position to the configured goal. Returns an
       *         empty vector when no algorithm is set.
       */
      std::vector<Node> start_exploration() {
          if (algo_) {
              return algo_->solve();
          }
          return {};
      }

      // Getters
      /** @brief Current X coordinate (column) of the robot. */
      [[nodiscard]]  int get_x() const noexcept { return x_; }

      /** @brief Current Y coordinate (row) of the robot. */
      [[nodiscard]]  int get_y() const noexcept { return y_; }

      /** @brief Current facing direction ('n','e','s','w'). */
      [[nodiscard]] char get_d() const noexcept { return d_; }
      
      /**
       * @brief Access the robot's Maze instance.
       * @return Mutable reference to the robot's internal Maze.
       */
      Maze& get_maze() { return maze_; }


      // Setters
      /**
       * @brief Update robot's X coordinate (column).
       * @param x New X coordinate value.
       */
      void set_x(int x)  { x_ = x;}

      /**
       * @brief Update robot's Y coordinate (row).
       * @param y New Y coordinate value.
       */
      void set_y(int y)  { y_ = y;}

      /**
       * @brief Update robot's facing direction.
       * @param d New facing direction ('n','e','s','w').
       */
      void set_d(char d) { d_ = d;}

      /**
       * @brief Assign or replace the planning algorithm used by the robot.
       * @param algorithm Shared pointer to a concrete Algorithm instance.
       */
      void set_algorithm(std::shared_ptr<Algorithm> algorithm) {
        algo_ = algorithm;
      }
      
      /**
       * @brief Turn the robot left 90 degrees (counter-clockwise).
       * @details Updates robot's internal pose and issues the command to the
       *          simulator via the Maze API.
       */
      void turn_left();
     
      /**
       * @brief Turn the robot right 90 degrees (clockwise).
       * @details Updates robot's internal pose and issues the command to the
       *          simulator via the Maze API.
       */
      void turn_right();
      
      /**
       * @brief Move the robot forward by a number of cells.
       * @param distance Number of cells to move forward (default 1).
       * @details The method updates the internal pose and forwards the
       *          command to the simulator.
       */
      void move_forward(int distance = 1);
      
      /**
       * @brief Rotate the robot (via repeated right turns) until it faces
       *        the requested direction.
       * @param target_d Desired direction ('n','e','s','w').
       */
      void face_direction(char target_d);
      
      /**
       * @brief Follow a path and re-plan when blocked.
       *
       * @details Executes a given path node-by-node. If a previously-unknown
       * wall blocks progress the method updates the internal map and triggers
       * a replanning call to the assigned algorithm. The updated path is
       * returned via the `path` parameter.
       *
       * @param path Reference to a vector of Nodes describing the path to
       *             execute. May be modified in-place when replanning occurs.
       * @return true if the robot successfully followed the path (or
       *         successfully replanned and continued). Returns false when no
       *         feasible path to the goal exists.
       */
      bool path_follow_replan(std::vector<Node>& path);
      
      /**
       * @brief Normalize a raw angle into the [0, 360) range.
       * @param angle_raw Angle in degrees (may be negative or >360).
       * @return Normalized integer angle inside [0, 360).
       */
      int normalize_angle(int angle_raw) const;
      
      /**
       * @brief Update the internal discovered map and the simulator with
       *        detected walls from the robot's current pose.
       *
       * @param index Index used to determine which of the robot's local
       *              sensors we are mapping (implementation detail).
       * @param dir_enum Mapping table converting numeric angles to direction
       *                 characters (e.g., 0 -> 'n').
       * @param wall_dirs Array of three chars describing walls seen at the
       *                  robot's current position (front, left, right).
       * @param next_wall_dirs Array of chars describing walls in the next
       *                       neighbouring cell in the same order.
       * @param map Reference to the discovered Map to update (height x width x 4).
       */
      /**
       * @brief Update the internal discovered map and the simulator with
       *        detected walls from the robot's current pose.
       *
       * @param index Index of the local sensor being processed (0=left,1=front,2=right)
       * @param wall_dirs Array of three chars describing walls seen at the
       *                  robot's current position (front, left, right).
       * @param next_wall_dirs Array of chars describing walls in the next
       *                       neighbouring cell in the same order.
       * @param map Reference to the discovered Map to update (height x width x 4).
       */
      void set_walls(const int index,
            const std::array<char, 3>& wall_dirs,
            const std::array<char, 3>& next_wall_dirs,
            Map& map);
      /**
       * @brief Get coordinates of the cell reachable by moving one step in
       *        the given direction from the robot's current position.
       * @param dir Direction character ('n','e','s','w').
       * @return Pair {x, y} coordinates of the neighbour cell.
       */
      std::pair<int, int> get_neighbor_coords(const char dir) const;
      /**
       * @brief Determine which compass direction the robot must face to move
       *        from the current cell by the delta (dx, dy).
       * @param dx Delta X (next_x - current_x)
       * @param dy Delta Y (next_y - current_y)
       * @return Direction character ('n','e','s','w').
       */
      char get_target_direction(const int &dx, const int &dy) const;
      /**
       * @brief Print the algorithm's computed path for debugging.
       * @param path Path vector of nodes from start to goal to display.
       */
      void show_algo_path(std::vector<Node>& path);
      
  };

}