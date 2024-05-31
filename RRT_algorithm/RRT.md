# RRT Algorithm 

## Table of Contents
- [Introduction](#introduction)
  - [Mathematical Formulation](#Mathematical-Formulation)
  - [Pseudo-code for RRT](#Pseudo-codeforRRT)
  - [Implementation in Python](#ImplementationinPython)
  - [Figures and Visualizations](#FiguresandVisualizations)
- [Usage](#usage)
  - [Code Explanation](#Code-Explanation)
  - [Running Locally](#running-locally)


## Introduction

- **Concept:** RRT is probabilistic algorithm that builds a tree by randomly sampling point in the space.

- **Advanteges:** Efficient in high dimensional and complex spaces, suitable for dynamic environments.

- **Applications:** Used in robotics for motion planning, especilly in large, cluttered environments.

### Mathematical Formulation

- **Tree Expansion**

   - **Sampling:** Randomly sample a point $x_{\text{rand}}$ in the space.
     
   - **Nearest Neighbor:** Find the nearest node $x_{\text{near}}$ towards $x_{\text{rand}}$:
               <div style="text-align: center;">
                $x_{\text{new}} = x_{\text{near}} + \epsilon \frac{x_{\text{rand}} - x_{\text{near}}}{\|x_{\text{rand}} - x_{\text{near}}\|}$
               </div>
               
   - **Update Tree:** Add $x_{\text{new}}$ to the tree
   
### Pseudo-code for RRT
```Python
def rrt(start, goal, max_iters, space):
    tree = [start]
    for _ in range(max_iters):
        x_rand = sample_envir(space)
        x_near = nearest(tree, x_rand)
        x_new = expand(x_near, x_rand)
        if is_free(x_near, x_new, space):
            tree.append(x_new)
            if reached_goal(x_new, goal):
                return path_to_goal(tree, start, goal)
    return None
```

### Implementation in Python 
- **Key Functions:**
  
     - **sample_envir:** Method to randomly sample a point within the map dimensions.
       
     - **nearest:** Method to find the nearest node to a given node in the graph.
       
     - **isFree:** Method to check if the last added node is in collision with any obstacles.
       
     - **expand:** Method to expand the RRT graph by adding a new node.
       
     - **path_to_goal:** Method to generate a path from the start to the goal using the parent relationships.
### Figures and Visualizations 
- **Pathfinding in Action:** 
 
| ![image](https://github.com/GhassenHafsiaINSAT/Path_Planning/assets/110825502/72078da2-c1f9-4e87-b79d-9c6cf8e18dcc)| ![image](https://github.com/GhassenHafsiaINSAT/Path_Planning/assets/110825502/a1f0b83d-d8aa-466e-a91a-36a0f2ce3d3a) | ![image](https://github.com/GhassenHafsiaINSAT/Path_Planning/assets/110825502/a0595ba6-e680-4110-8258-73daa7b3938f) |
|---------------------------------------|---------------------------------------|---------------------------------------|

## Usage 
### Code Explanation

- **RRTbasePY.py:** This file contains the implementation of the RRT algorithm. It defines two classes: *RRTMap* and *RRTGraph*.  

   - *RRTMap Class*: Manages the visualization of the map and obstacles. It includes methods to draw the map, obstacles, and paths.  

   - *RRTGraph Class*: Manages the RRT graph structure and algorithm. It includes methods for adding nodes, connecting edges, sampling the environment, and generating paths.  

- **main.py:** This file uses the RRT algorithm from *RRTbasePY.py* to plan a path in a 2D environment using Pygame for visualization.

   - It initializes the RRTMap and RRTGraph classes.  
   - Obstacles and map dimensions are defined.  
   - The RRT algorithm is executed with visualization using Pygame.  

### Running Locally
- If you want to execute this project locally, you have to:
  
  1. **Clone the repository:**
     
     ```sh
     git clone https://github.com/GhassenHafsiaINSAT/Path_Planning.git
     ```
  3. **Navigate to the project directory:**
     
     ```sh
     cd Path_Planning/RRT_algorithm
     ```
     
  4. **Install dependencies:** Make sure you have Python and pygame installed on your local machine.
       
     ```bash
     pip install pygame
     ```
     
  5. **Start the application:**
     
     ```sh
     python main.py
     ```

       
       
