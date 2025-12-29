# 🐹 Hamster Robot: Autonomous Maze Solver
An autonomous robotics project that utilizes the A* Search algorithm and infrared proximity sensors to navigate, map, and solve complex 5x7 grid mazes. 

# 🌟 Key Features
* **Optimistic A\* Exploration:** The robot begins with an empty map and uses A* to calculate the shortest path to the goal based on "known" facts. As it discovers walls, it dynamically replans its route in real-time.
* **Haptic-Style Alignment:** To overcome mechanical drift and sensor noise, I implemented a "Back-and-Forth" alignment routine. The robot physically aligns itself against walls to reset its coordinate precision.
* **Three-Phase Execution:** Exploration: Navigates from Start to Goal while mapping walls.Mapping: Calculates the absolute shortest return path based on the completed map.Speed Run: Executes a "Final Run" at high speed with minimized alignment stops.
* **Real-Time Maze Visualization:** The terminal outputs a live ASCII map of the robot's "mental model" of the maze at every step.

# 🛠️ Tech Stack
* Language: Python 3
* Algorithm: A* (Heuristic: Manhattan Distance)
* Hardware: HamsterS Robot (Roboid API)
* Mathematics: NumPy (for sensor data processing and floor-averaging)

# 🧠 The Logic: How it Works
The robot treats the maze as a coordinate system (r, c).

**1. Pathfinding (The Brain)** 
Using ```heapq``` for priority queuing, the A* algorithm calculates the $f\_score$ for each neighboring cell:

### f(n) = g(n) + h(n)

Where:

* g(n) is the cost from the start to the current cell.
* h(n)$ is the Manhattan Distance heuristic to the goal.

**2. Wall Detection (The Senses)**
The robot uses ```left_proximity()``` and ```right_proximity()``` sensors. To ensure accuracy, I implemented a Dual-Check System: the robot takes sensor readings, moves slightly, takes a second reading, and averages them (```np.floor```) to filter out mechanical jitters.

# 📊 Performance
* **Class Rank:** ranked as a top-tier performer in a class of 40, recognized for developing one of the most efficient maze-solving paths in the final project.
* **Maze Size:** 5x7 Grid (35 possible nodes).
* **Success Rate:** 100% completion in test runs with zero "lost" states due to robust alignment logic.

# 🚀 How to Run
1. Connect your HamsterS robot via Bluetooth.
2. Ensure the roboid library is installed.
3. Update the start_pos and end_pos in Main.py to match your physical maze.
4. Run the script:
    ```
    python Main.py
# Project Reflections
"The biggest challenge was mechanical drift—the robot would slowly tilt after 3-4 turns. I solved this by implementing a 'Wall-Align' function that treats the physical maze walls as a correction tool rather than just an obstacle. This allowed my robot to stay perfectly centered while others often crashed."













