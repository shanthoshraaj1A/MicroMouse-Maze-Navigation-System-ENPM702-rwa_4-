/**
 * @file bfs.cpp
 * @brief Implements the BFS (Breadth-First Search) planning algorithm.
 *
 * @details
 * BFS (Breadth-First Search):
 * - Uses only the Maze’s *discovered* wall map (never live sensors)
 * - Generates shortest paths on an unweighted grid if the map is correct
 * - Explores neighbors in NESW priority order (North → East → South → West)
 * - Explores nodes in FIFO order 
 * - Does not control the robot; used only for path generation
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
#include "bfs.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <queue>

namespace micro_mouse{

  BFS::BFS(Maze& maze, Node start, std::array<int, 2> goal) 
        : Algorithm(
            "BFS",                  
            std::set<Node>{},      // Pass empty set for visited
            start,                 // Start at given start node
            goal,                  // Randomnly selected goal node from (7,7), (7,8), (8,7), 8,8)
            maze                   // Pass maze reference
          )
  {
      std::cerr << "BFS Initialized. Goal: " << goal_[0] << "," << goal_[1] << '\n';
  }

  bool BFS::is_valid_move(Node next_node) const{
        
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


  std::vector<Node> BFS::backtrack(const Node& curr) const{
      std::vector<Node> path_to_goal;
      Node backtrack = curr;

      while (backtrack.get_parent_x() != -1) {
        path_to_goal.push_back(backtrack);
        
        // Overloaded operator< to traverse set tree only using x and y coordinates
        Node parent_node(backtrack.get_parent_x(), 
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
  

  std::vector<Node> BFS::solve(){

    // Empty queue from any prior calls assigning an empty one
    queue_ = std::queue<Node>{};
    // Clear visited set so we can re-explore nodes if necessary
    visited_.clear();

    bool   moved{false};
    queue_.push(start_);
    visited_.insert(start_);


    while (!queue_.empty()) {
      const Node&  curr{ queue_.front() };
      queue_.pop();
      const int& curr_x{ curr.get_x() };
      const int& curr_y{ curr.get_y() };

      // Check if we are at goal, if so, backtrack from goal to start and return path
      if ((curr_x == goal_[0]) && (curr_y == goal_[1])){
          return backtrack(curr);
      }

      // Mark Node as visited, reset moved to false at start of every loop
      
      moved = false;
      
      // Potential Moves
      const Node north_node(curr_x,     curr_y + 1, 'n', curr_x, curr_y);
      const Node east_node( curr_x + 1, curr_y,     'e', curr_x, curr_y);
      const Node south_node(curr_x,     curr_y - 1, 's', curr_x, curr_y);
      const Node west_node( curr_x - 1, curr_y,     'w', curr_x, curr_y);

      // Check if each potential move is valid and not already visited:
      // Check in reverse priority order due to queue
      //   i.e. north is priority order and added to queue last (first to pop)
      for (const Node &next_node : {west_node, south_node, east_node, north_node}){
        if ( is_valid_move(next_node) && visited_.count(next_node) == 0 ) {
              visited_.insert(next_node);
              queue_.push(next_node);
              moved = true;
           }
      }

    }

  std::cerr << " Error: No Path Found \n";
  return {};

  }


} // end namespace micro_mouse