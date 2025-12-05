/**
 * @file maze.hpp
 * @brief High-level Maze wrapper that stores the discovered map and safely
 *        interfaces with the simulator via MazeControlAPI.
 *
 * @details
 * The Maze class is the *only* component allowed to communicate directly with
 * the simulator. It maintains an internal discovered map of walls and exposes
 * safe wrappers for:
 * - Wall sensors (front / left / right)
 * - Robot motion commands (move_forward / turn_left / turn_right)
 * - Visualization tools (colors, text, wall markings)
 *
 * Planning algorithms (DFS/BFS) must use only `has_known_wall()` and may not
 * query live simulator sensors directly. Only the Robot updates the internal
 * map using live sensor data.
 * 
 * @authors
 * Team Members:
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */
#pragma once
#include <vector>
#include <stdexcept>
#include <fstream>
#include "maze_api.hpp"
#include <array>
#include <map>
#include <cstdint>  
#include <algorithm> 
#include <iostream>

namespace micro_mouse {

/**
 * @brief type alias for internal map data structure
 */
using Map = std::vector<std::vector<std::array<bool, 4>>>;

/**
 * @class Maze
 * @brief Wrapper around MazeControlAPI maintaining the discovered maze map.
 *
 * @details
 * This class owns the internal map (`map_`) and all safe read/write operations.
 * All other components must interact with the simulator **only through Maze**.
 */
/**
 * @class Maze
 * @brief Wrapper around the simulator API which stores the discovered map and
 *        provides safe read/write helpers used by Robot and planning modules.
 *
 * @details The Maze class is intentionally the single, safe interface to the
 * simulator. It stores an internal discovered map of wall booleans for every
 * cell and exposes helpers for sensor reads, motion commands, and visualization
 * helpers. External code must not call the simulator API directly — they should
 * use this class so the Robot component can coordinate discovery and updates.
 */
class Maze {
    private:
        /** @brief Maze width (number of columns / x dimension). */
        int width_;

        /** @brief Maze height (number of rows / y dimension). */
        int height_;

        /** @brief Internal discovered map: height x width x 4 walls (N,E,S,W).
         *  map_[row][col][i] is true when wall i exists for that cell.
         */
        Map map_;

    public:

        /**
         * @brief Construct a Maze from a fully-initialized discovered map.
         * @param initial_map 3D vector of shape [height][width][4] with a
         *                     boolean for each wall (N, E, S, W) in every cell.
         *
         * @throws std::invalid_argument if the provided map is empty or has zero
         *         width/height.
         */
        Maze(const Map& initial_map) {
            if (initial_map.empty() || initial_map[0].empty()) {
                throw std::invalid_argument("Initial map cannot be empty or have zero width.");
            }
            // Initialize dimensions and map
            height_ = static_cast<int>(initial_map.size());
            width_  = static_cast<int>(initial_map[0].size());
            map_    = initial_map; 
        }
        /**
         * @brief Construct an empty maze of the provided size (no known walls).
         * @param width  Maze width (number of columns / x dimension).
         * @param height Maze height (number of rows / y dimension).
         */
        Maze(const int width, const int height) {
            height_ = height;
            width_ = width;
            const std::array<bool, 4> initial_walls{{false, false, false, false}};
            const std::vector<std::array<bool, 4>> row(width, initial_walls);
            map_ = Map(height, row);
        }
                     
        /**
         * @brief Return a read-only reference to the internal discovered map.
         * @return const Map& The internal [height][width][4] booleans.
         */
        [[nodiscard]] const Map&   get_map() const noexcept { return map_; }

        /**
         * @brief Return a mutable reference to the internal map — use with care.
         * @note Intended for internal tests or controlled updates (Robot only).
         */
        [[nodiscard]] Map& get_map() noexcept { return map_; }

        /** @brief Maze width (number of columns / x dimension). */
        [[nodiscard]] const int  get_width() const noexcept { return width_; }

        /** @brief Maze height (number of rows / y dimension). */
        [[nodiscard]] const int get_height() const noexcept { return height_; }
        
        // Setters
        /** @brief Set the maze width. */
        void set_width( const int  width) {width_  = width;}

        /** @brief Set the maze height. */
        void set_height(const int height) {height_ = height;}

        /**
         * @brief Set a wall in the simulator and the internal map.
         * @param x X coordinate of the cell.
         * @param y Y coordinate of the cell.
         * @param direction One of 'N'/'E'/'S'/'W' indicating which wall to set.
         */
        void set_wall(int x, int y, char direction) {
            micro_mouse::MazeControlAPI::set_wall(x, y, direction);
        }

        /**
         * @brief Query whether the simulator (or discovered map) reports a left
         *        wall from the robot's current pose.
         * @return true if a wall is present on the robot's left side.
         */
        bool has_wall_left() const {
            return micro_mouse::MazeControlAPI::has_wall_left();
        }
        /** @brief True when the robot's right side reports a wall. */
        bool has_wall_right() const {
            return micro_mouse::MazeControlAPI::has_wall_right();
        }
        /** @brief True when the robot's front side reports a wall. */
        bool has_wall_front() const {
            return micro_mouse::MazeControlAPI::has_wall_front();
        }

        /**
         * @brief Move the robot forward in the simulator.
         * @param distance Number of cells to move forward (default: 1).
         */
        void move_forward(int distance = 1){
            return micro_mouse::MazeControlAPI::move_forward(distance);
        }

        /** @brief Turn the robot 90 degrees right in the simulator. */
        void turn_right(){
            return micro_mouse::MazeControlAPI::turn_right();
        }

        /** @brief Turn the robot 90 degrees left in the simulator. */
        void turn_left(){
            return micro_mouse::MazeControlAPI::turn_left();
        }

        /**
         * @brief Clear a previously set wall in the simulator for a cell.
         * @param x X coordinate, @param y Y coordinate, @param direction wall char.
         */
        void clear_wall(int x, int y, char direction){
            return micro_mouse::MazeControlAPI::clear_wall(x , y, direction);
        }

        /**
         * @brief Mark a cell with a color in the simulator for visualization.
         * @param color char code for color (implementation defined in simulator).
         */
        void set_color(int x, int y, char color){
            return micro_mouse::MazeControlAPI::set_color(x , y, color);
        }

        /** @brief Clear any color marking on the specified cell. */
        void clear_color(int x, int y){
            return micro_mouse::MazeControlAPI::clear_color(x , y);
        }

        /** @brief Clear color markings for the entire maze in the simulator. */
        void clear_all_color(){
            return micro_mouse::MazeControlAPI::clear_all_color();
        }

        /**
         * @brief Place text in a cell for visualization (simulator-only).
         * @param text The text to draw at the requested cell.
         */
        void set_text(int x, int y, const std::string &text){
            return micro_mouse::MazeControlAPI::set_text(x, y, text);
        }

        /** @brief Clear any text placed on a specific cell. */
        void clear_text(int x, int y){
            return micro_mouse::MazeControlAPI::clear_text(x, y);
        }

        /** @brief Clear all text annotations in the simulator. */
        void clear_all_text(){
            return micro_mouse::MazeControlAPI::clear_all_text();
        }

        /** @brief Query whether the simulator has been reset since last check. */
        bool was_reset() const{
            return micro_mouse::MazeControlAPI::was_reset();
        }
        /** @brief Acknowledge and clear the simulator reset flag. */
        void ack_reset(){
            return micro_mouse::MazeControlAPI::ack_reset();
        }

        /**
         * @brief Log textual information via the simulator GUI / console.
         * @param text Message to appear in the simulator log.
         */
        void log(std::string_view text){
            return micro_mouse::MazeControlAPI::log(text);
        }

};// class Maze
  
}