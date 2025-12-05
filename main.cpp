/**
 * @file main.cpp
 * @brief Entry point for the Micromouse project.
 *
 * @details
 * Program Flow:
 *  1. Initialize Robot and Maze
 *  2. Randomly select one of the 4 center goal cells (7,7), (7,8), (8,7), (8,8)
 *  3. Mark Start/Goal nodes on maze
 *  4. Run DFS or BFS to generate an initial path based on algorithm_type ("DFS" for DFS, "BFS" for BFS)
 *  5. Robot follows path marked on map and performs replanning when path is blocked by discovered wall
 *  6. Run BFS & DFS on the discovered map for visualization (bonus requirement)
 *
 * The robot always begins at coordinate (0,0) facing north
 *
 * @authors
 *  - Shanthosh Raaj Mohanram Mageswari
 *  - Chris Collins
 *  - Jonathan Crespo
 *  - Lucas Janniche
 */

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <random>
#include <array>
#include "robot.hpp"
#include "dfs.hpp"
#include <ctime>
#include "bfs.hpp"

void log(const std::string& text) {
  std::cerr << text << std::endl;
}

using namespace micro_mouse;


int main() {
  //set algorithm to use, options include DFS or BFS
  const std::string algorithm_type{"DFS"}; 
  std::shared_ptr<Algorithm> algo {};

  const Node start{0, 0, 'n', -1, -1};
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  // Create vector of 4 possible goal nodes:
  std::vector<std::array<int, 2>> goals;
  goals.emplace_back(std::array<int,2>{7,8});
  goals.emplace_back(std::array<int,2>{7,7});
  goals.emplace_back(std::array<int,2>{8,7});
  goals.emplace_back(std::array<int,2>{8,8});
  const int random_index = std::rand() % 4;

  // Select goal randomnly out of the 4 options:
  const std::array<int, 2>goal{goals[random_index]};
 

  auto robot = std::make_shared<Robot>();

  robot -> get_maze().set_color(start.get_x(), start.get_y(), 'G');
  robot -> get_maze().set_text(start.get_x(), start.get_y(), "S");
  robot -> get_maze().set_color(goal[0], goal[1], 'y');
  robot -> get_maze().set_text(goal[0], goal[1], "G");

  std::shared_ptr<Algorithm> bfs = std::make_shared<BFS>(robot -> get_maze(), start, goal);
  std::shared_ptr<Algorithm> dfs = std::make_shared<DFS>(robot -> get_maze(), start, goal);

  
  
  if (algorithm_type == "BFS"){
    robot -> set_algorithm(bfs);
    std::cerr << "Executing BFS\n";
  } else {
    robot -> set_algorithm(dfs);
    std::cerr << "Executing DFS\n";
  }

  std::vector<Node> path = robot -> start_exploration();
  std::cerr << algorithm_type << " initial path length: " << path.size() << "\n";
  
  if (!path.empty()) {
        std::cerr << "Path found! Executing...\n";
        // Move Robot along path output from our search algorithm, sensing and updating the 
        // maze and our internal representation of the maze's wall state as it progresses
        // If new walls are detected check if our path would be blocked and return search algorithm on
        // the updated state of the maze
        robot->path_follow_replan(path);
  } else {
        std::cerr << "No path found.\n";
  }

  DFS dfs_at_end(robot -> get_maze(), start, goal);
  BFS bfs_at_end(robot -> get_maze(), start, goal);

  std::vector<Node> dfs_path_at_end{dfs_at_end.solve()};
  std::vector<Node> bfs_path_at_end{bfs_at_end.solve()};

  std::cerr << " DFS path length (on discovered map): "
              << dfs_path_at_end.size() << "\n";
  std::cerr << " BFS path length (on discovered map): "
              << bfs_path_at_end.size() << "\n";

  robot -> get_maze().clear_all_color();
  // Color BFS path for display (does not affect robot)
  for (const auto &n : bfs_path_at_end) {
      robot -> get_maze().set_color(n.get_x(), n.get_y(), 'y');
  }
  for (const auto &n : dfs_path_at_end) {
      robot -> get_maze().set_color(n.get_x(), n.get_y(), 'b');
  }


  return 0;

}