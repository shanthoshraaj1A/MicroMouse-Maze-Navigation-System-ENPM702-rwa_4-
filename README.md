# Team Members ENPM702 Group 5
- Shanthosh Raaj Mohanram Mageswari
- Chris Collins
- Jonathan Crespo
- Lucas Janniche

# Overview
This project implements autonomous maze navigation for the Micromouse robot using:
- Depth-First Search (DFS) or Breadth-First Search (BFS) for planning and replanning, search type is set in main (algorithm_type)  
- Incremental maze discovery using front, left, and right wall sensors
- Online replanning when a previously unknown wall blocks any part of the robot's intended path
- Internal occupancy map updated only from sensor data
  - Internal map is maintained std::vector<std::vector<std::array<bool, 4>>>
- A bonus comparison of DFS vs. BFS implementation for visualization on discovered map

The project uses the following maze simulator for testing https://github.com/mackorone/mms. Credits to their authors. 

The robot begins at (0,0) facing North and must navigate to one of four center goal cells randomly selected.

<img width="1918" height="1022" alt="image" src="https://github.com/user-attachments/assets/5d48c95b-82bc-4c6e-aaed-5935a7b5f8a5" />

# How the Program Works
1. **Initial Planning**
   - Search Algorithm runs on the initially empty map.  
   - A path to the goal is generated using NESW priority.

2. **Execution**
   - For each step, the robot:
     - Turns toward its next move
     - Reads wall sensors (front, left, right)
     - Updates its internal map using only sensor data
     - If any newly detected walls would block the robot along its current path, than 
       - the search algorithm replans from the robot’s current position using updated map
       - the replanned path to gal is shown on the maze in cyan

3. **Goal Reached**
   - The robot continuously replans until it reaches the goal.

4. **Bonus: BFS vs. DFS Comparison**
   - After reaching the goal, both BFS and DFS are from start to goal *only on the discovered map*.
   - The BFS path is visualized in yellow.
   - The DFS path is visualized in  blue.
   - These paths do not influence the robot motion
<img width="1917" height="1018" alt="image" src="https://github.com/user-attachments/assets/7cb48945-11bf-44f8-a8d1-bf023699ce96" />

# Build Instructions
From the project root:
On your first build
```sh 
cmake -S . -B build
```
After use the following command:
```sh 
cmake --build build
```
As another option, you can use the Visual Studio Code C/C++ and CMake extensions as the example in the following video.

https://github.com/user-attachments/assets/d80d27cb-3b71-483e-ad32-9526f1db6a96

This produces:

build/rwa4_cpp

<img width="587" height="473" alt="image" src="https://github.com/user-attachments/assets/f26ca0f9-e0ea-4cdd-a4b0-77d8f2c14bb5" />

# Run Instructions
1. Open the Micromouse Simulator (`mms`).

https://github.com/user-attachments/assets/1ffdc626-a2c4-4097-9e7d-988af0eaab13

2. Load any classical maze. For more info check this link https://github.com/micromouseonline/mazefiles
   
https://github.com/user-attachments/assets/40d0b32e-3857-4ac8-bc31-cf6d23ffc590

3. Set mouse program to:
    - Name:       rwa4_cpp
    - Directory:  ENPM702_RWA_4/build --> *The full path to the build folder*
    - Run:       ./rwa4.cpp
<img width="1051" height="263" alt="image" src="https://github.com/user-attachments/assets/72075c93-5687-4761-a4f8-42dd19cd616f" />

4. Click **Run**.

https://github.com/user-attachments/assets/a9cac033-f48b-4354-a4d5-f34110e787d5

The robot will:
- Compute an initial path based on the search algorithm set in main.cpp (algorithm_type)  
- Move step-by-step  
- Replan when blocked  
- Reach the goal  
- Display BFS shortest path (bonus) and DFS path from start to goal on discovered map.

To change between algorithms look for the following line inside the *main.cpp* file and define it between 'BFS' or 'DFS' or even your future algorithm using the implementation. 

<img width="1832" height="393" alt="image" src="https://github.com/user-attachments/assets/c9351a34-d4a7-45fc-81d8-dd98e5c5efde" />

https://github.com/user-attachments/assets/8e2e1821-df99-492a-b49f-61368c6e93c5

# Included Algorithms

## DFS (Default State)
- Used for both initial planning and replanning.
- Uses NESW priority.
- Works only from discovered walls.
- Complies with Option A of the assignment.

## BFS (Bonus)
- Finds shortest path on the discovered map.
- Visualized in cyan.
- Does not affect robot execution.
