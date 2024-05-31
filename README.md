# Path Planning
## Introduction 
- **Purpose and Importance:** Path Planning enables autonomous robots to navigate environments, crucial for applications like delivery drones, robotic arms and autonomous vehicules.
## General Path Planning 

- **Definition:** Path planning is the process of finding a viable path from a start point to an endpoint while avoiding obstacles.

- **Challenges:** Dynamic environments, obstacle avoidance, efficiency, computational constraints.

- **Common Algorithms:**
    - **A Algorithm:** Combines cost to reach a node and estimated cost to the goal.
    - **Dijkstra's Algorithm:** Uniform cost search, suitable for grid-based maps.
    - **RRT (rapidly-exploring Random Trees):** Efficient in high-dimensional spaces.

## Mathematical Equations
- **Path Cost Calculation:**  
    - *g(n) = cost from start to n*  
    - *h(n) = heuristic estimate from n to goal*
    - *f(n) = g(n) + h(n)*
