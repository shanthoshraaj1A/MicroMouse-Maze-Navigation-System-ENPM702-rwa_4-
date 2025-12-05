/**
 * @file robot.cpp
 * @brief Implements Robot movement, sensing, and DFS or BFS Path following and replanning logic.
 *
 * @details
 * The Robot class is responsible for:
 *  - Tracking internal pose: (x, y, direction)
 *  - Querying front/left/right live sensors
 *  - Updating the discovered map inside Maze
 *  - Following path output from search algorithm
 *  - Triggering replanning when any new discovered wall would block any portion of the path being executed.
 *
 * All movement and sensing is performed through the Maze API wrapper.
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */
#include <iostream>
#include <string>
#include "robot.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <map>


using MMS = micro_mouse::MazeControlAPI;

namespace micro_mouse{

    // Use a signed integer for arithmetic safety and cross-platform clarity.
    constexpr int full_circle = 360;
    
    // Single, pre-initialized map so callers can use .at() safely without
    // recreating a map on each function invocation.
    static const std::map<char, int> dir_enum{{'n', 0}, {'e', 1}, {'s', 2}, {'w', 3}};


    int Robot::normalize_angle(int angle_raw) const{
        //Using modular arithmetic that works for any
        // integer (positive or negative). The expression ensures the result
        // lies in [0, full_circle).
        return ((angle_raw % full_circle) + full_circle) % full_circle;
    }


    void Robot::turn_left(){
      maze_.turn_left();
      d_ang_ -= 90;
      d_ang_ = normalize_angle(d_ang_);
      d_     = ang_to_d_[d_ang_];
    }


    void Robot::turn_right(){
      maze_.turn_right();
      d_ang_ += 90;
      d_ang_ = normalize_angle(d_ang_);
      d_     = ang_to_d_[d_ang_];
    }


    void Robot::move_forward(int distance){

        if(maze_.has_wall_front()) return;

        maze_.move_forward(distance);
    }
        

    void Robot::face_direction(char target_d){

        if (d_ == target_d) return;

        while (d_ != target_d){
            turn_right();
        }
    }


    std::pair<int, int> Robot::get_neighbor_coords(const char dir) const{
        // Get neighboring cell coordinates based on which direction we are checking
        if (dir == 'n') return {x_, y_ + 1};
        if (dir == 's') return {x_, y_ - 1};
        if (dir == 'e') return {x_ + 1, y_};
        if (dir == 'w') return {x_ - 1, y_};
        return {x_, y_};
    };


    void Robot::set_walls(const int index,
                            const std::array<char, 3>& wall_dirs,
                            const std::array<char, 3>& next_wall_dirs,
                            Map& map){
                                
        // map_wall_index is current cell
        const int map_wall_index      = dir_enum.at(wall_dirs[index]);

        // next_map_wall_index is neighboring cell such that we update the current cell 
        // and its neighboring cell wall at the same time (i.e. north wall in first cell and south wall in neighboring cell)
        const int next_map_wall_index = dir_enum.at(next_wall_dirs[index]);

        // Set wall within Maze Simulator and our internal representation of the maze for current cell
        maze_.set_wall(x_, y_, wall_dirs[index]);
        map[y_][x_][map_wall_index] = true;

        // Get neighboring cell based on which wall we are checking with index (left, front, or right)
        auto [nx, ny] = get_neighbor_coords(wall_dirs[index]);

        // Set wall within Maze Simulator and our internal representation of the maze for next cell
        // Check if neighboring cell is within bounds of maze/map
        if (nx >= 0 && nx < maze_.get_width() &&
            ny >= 0 && ny < maze_.get_height() ) {

            map[ny][nx][next_map_wall_index] = true;
            maze_.set_wall(nx, ny, next_wall_dirs[index]);
        }
    }
    
    char Robot::get_target_direction(const int &dx, const int &dy) const{
        char target_d = d_;
        if (     dx ==  1) target_d = 'e';
        else if (dx == -1) target_d = 'w';
        else if (dy ==  1) target_d = 'n';
        else if (dy == -1) target_d = 's';

        return target_d;
    }

    void Robot::show_algo_path(std::vector<Node>& path){
        maze_.clear_all_color();
        for (size_t j{0}; j < path.size(); j++ ){
            const Node curr = path[j];
            maze_.set_color(curr.get_x(), curr.get_y(), 'c');
        }
    }


    bool Robot::path_follow_replan(std::vector<Node>& path) {

        if (path.empty()) return false;

        // Use the pre-initialized dir_enum map (use .at() for safety).
        std::array<char, 3> wall_dirs;
        std::array<char, 3> next_wall_dirs;
        char                target_d{d_};
        bool                map_updated {false};
        // index not needed now (we pass constants when calling set_walls)

        Map& map  = maze_.get_map();
        
        while (path.size() > 1) {

            // Start at index 1 (skip start node that robot is already located in)
            show_algo_path(path);
            
            // 1.  Get Next_Node Information
            const Node next_node = path[1];
            int dx         = next_node.get_x() - x_;
            int dy         = next_node.get_y() - y_;

            // 2. Rotate robot to face toward next node in path
            target_d = get_target_direction(dx, dy);
            face_direction(target_d);
            
            // 3. Sense Walls
            bool wall_left   = maze_.has_wall_left();
            bool wall_front  = maze_.has_wall_front();
            bool wall_right  = maze_.has_wall_right();
            
            // Define direction of walls in order: left, front, right:
            // walls_dirs: direction of walls in robot's current cell
            // next_wall_dirs: Direction of walls in neighboring cell
            switch (d_) {
                case 'n':
                    wall_dirs      = {'w', 'n', 'e'};
                    next_wall_dirs = {'e', 's', 'w'};
                    break;
                case 'w':
                    wall_dirs      = {'s', 'w', 'n'};
                    next_wall_dirs = {'n', 'e', 's'};
                    break;
                case 'e':
                    wall_dirs      = {'n', 'e', 's'};
                    next_wall_dirs = {'s', 'w', 'n'};
                    break;
                case 's':
                    wall_dirs      = {'e', 's', 'w'};
                    next_wall_dirs = {'w', 'n', 'e'};
                    break;
                default:
                    // keep default behaviour when direction is unknown
                    break;
            }

            // 4. Update Maze Simulator API and our internal representation of the maze
            if (wall_left){
                set_walls(0, wall_dirs, next_wall_dirs, map);
            }
            if (wall_front) {
                set_walls(1, wall_dirs, next_wall_dirs, map);
            }
            if (wall_right) {
                set_walls(2, wall_dirs, next_wall_dirs, map);
            }

            // if any walls detected check if new walls block path
            if(wall_front || wall_left || wall_right) {
                map_updated = true;
            }
            // if we set any new walls, check if the walls would block any of our remaining path
            if (map_updated) {
                // Get index of direction we are moving toward next cell 
                int move_dir      = dir_enum.at(target_d);
                bool path_blocked = false;
                
                // Check from current cell to end of path whether any new walls block our path
                for (size_t k = 0; k < path.size()-1; ++k) {
                    const Node& curr = path[k];
                    const Node& next = path[k+1];
                    dx        = next.get_x() - curr.get_x();
                    dy        = next.get_y() - curr.get_y();
                    target_d  = get_target_direction(dx, dy);
                    move_dir  = dir_enum.at(target_d);
                    
                    // if new wall blocking path, break out of for loop and replan
                    if (map[curr.get_y()][curr.get_x()][move_dir]) {
                        path_blocked = true;
                        break;
                    }
                }
                if (path_blocked){
                    std::cerr << "Path blocked! Replanning...\n";
                        
                    // Reset start to current position and direction, then rerun DFS with updated maze state
                    algo_ -> set_start(Node(x_, y_, d_, -1, -1));
                    std::vector<Node> new_path = algo_ -> solve();
                    
                    if (new_path.empty()) return false;

                    path = new_path;
                    
                    continue;
                }
            }
            
            // Move to next cell we have guaranteed is not blocked
            if (!wall_front){
                move_forward(1);
                // Update internal state
                x_ = next_node.get_x();
                y_ = next_node.get_y();
                path.erase(path.begin());
            } 
        }
        std::cerr << "Goal Reached!\n";
        return true;
    }

} // end namespace micro_mouse