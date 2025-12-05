/**
 * @file dfs.cpp
 * @brief Implements the DFS (Depth-First Search) planning algorithm.
 *
 * @details
 * DFS (Depth-First Search):
 *  - Expands neighbors in NESW priority order
 *  - Uses only on discovered Walls in Maze
 *  - Returns path from current robot position to goal
 *  - Used both for initial planning and replanning during maze exploration
 *  - Does not control the robot; 
 *
 * Neighbor exploration priority:
 *      N → E → S → W
 * Explores nodes in LIFO order so nodes are pushed in reverse order:
 *      W → S →  E → N
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#include <iostream>
#include <string>
#include "maze_api.hpp"
#include "dfs.hpp"
#include <array>
#include <vector>
#include <algorithm>

namespace micro_mouse{

  DFS::DFS(Maze& maze, Node start, std::array<int, 2> goal) 
        : Algorithm(
            "DFS",                  
            std::set<Node>{},      // Pass empty set for visited
            start,                 // Start at given start node
            goal,                  // Randomnly selected goal node from (7,7), (7,8), (8,7), 8,8)
            maze                   // Pass maze reference
          )
  {
      std::cerr << "DFS Initialized. Goal: " << goal_[0] << "," << goal_[1] << '\n';
  }

  bool DFS::is_valid_move(const Node& next_node) const{
        
    const int width  = maze_.get_width();
    const int height = maze_.get_height();
    const Map& map   = maze_.get_map(); 
    const int next_x       = next_node.get_x();
    const int next_y       = next_node.get_y();
    const char d           = next_node.get_d();

    // Check if move is within bounds of maze
    if ((next_x  <  0)     || 
        (next_x >=  width) ||
        (next_y  <  0)     || 
        (next_y >=  height )) {
          return false;
    }
    
    // Check if wall would block next move
    if       ((d == 'n') && ( map[next_y-1][next_x][0])) { return false; } 
    else if  ((d == 'e') && ( map[next_y][next_x-1][1])) { return false; } 
    else if  ((d == 's') && ( map[next_y+1][next_x][2])) { return false; } 
    else if  ((d == 'w') && ( map[next_y][next_x+1][3])) { return false; }

    return true;
  }


  std::vector<Node> DFS::backtrack(const Node& curr) const{
      std::vector<Node> path_to_goal;
      Node backtrack = curr;

      while (backtrack.get_parent_x() != -1) {
        path_to_goal.push_back(backtrack);
        
        // Overloaded operator< to traverse set tree only using x and y coordinates
        const Node parent_node(backtrack.get_parent_x(), 
                         backtrack.get_parent_y(),
                         backtrack.get_d(), 
                         -1, 
                         -1);
        auto find_parent = visited_.find(parent_node);

        if (find_parent != visited_.end()) {
          backtrack = *find_parent;
        } else {
          std::cerr << "ERROR: Path not continuous from goal to start during DFS Backtrack from Goal to Start" << '\n';
          break;
        }
      
      }
      path_to_goal.push_back(backtrack); // Add Start Node to end

      // Reverse path so it is in order start -> goal
      std::reverse(path_to_goal.begin(), path_to_goal.end());

      return path_to_goal;

  }


  std::vector<Node> DFS::solve(){

    // Empty stack from any prior calls by assigning to an empty stack 
    stack_ = std::stack<Node>{};
    // Clear visited set so we can re-explore nodes if necessary
    visited_.clear();

    bool   moved{false};
    stack_.push(start_);


    while (!stack_.empty()) {
      Node  curr{ stack_.top() };
      const int curr_x{ curr.get_x() };
      const int curr_y{ curr.get_y() };

      // Check if we are at goal, if so, backtrack from goal to start and return path
      if ((curr_x == goal_[0]) && (curr_y == goal_[1])){
          return backtrack(curr);
      }

      // Check if node has already been visited, if true, pop it
      if (visited_.count(curr)) {
        stack_.pop();
        continue;
      }

      // Mark Node as visited, reset moved to false at start of every loop
      visited_.insert(curr);
      moved = false;
      
      // Potential Moves
      const Node north_node(curr_x,     curr_y + 1, 'n', curr_x, curr_y);
      const Node east_node( curr_x + 1, curr_y,     'e', curr_x, curr_y);
      const Node south_node(curr_x,     curr_y - 1, 's', curr_x, curr_y);
      const Node west_node( curr_x - 1, curr_y,     'w', curr_x, curr_y);

      // Check if each potential move is valid and not already visited:
      // Check in reverse priority order due to stack
      //   i.e. north is priority order and added to stack last (first to pop)
      for (const Node& next_node : {west_node, south_node, east_node, north_node}){
        if ( is_valid_move(next_node) && visited_.count(next_node) == 0 ) {
              stack_.push(next_node);
              moved = true;
           }
      }

      // If we haven't moved pop current node (backtrack)
      if (!moved) {
        stack_.pop();
        std::cerr << "Backtracking" << '\n';
      }
    }

  std::cerr << " Error: No Path Found \n";
  return {};

  }


} // end namespace micro_mouse