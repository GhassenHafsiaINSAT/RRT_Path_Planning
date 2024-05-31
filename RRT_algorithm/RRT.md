# RRT Algorithm 

- **Concept:** RRT is probabilistic algorithm that builds a tree by randomly sampling point in the space.

- **Advanteges:** Efficient in high dimensional and complex spaces, suitable for dynamic environments.

- **Applications:** Used in robotics for motion planning, especilly in large, cluttered environments.

## Mathematical Formulation

1. **Tree Expansion**

   - **Sampling:** Randomly sample a point $x_{\text{rand}}$ in the space.
     
   - **Nearest Neighbor:** Find the nearest node $x_{\text{near}}$ towards $x_{\text{rand}}$:
               <div style="text-align: center;">
                $x_{\text{new}} = x_{\text{near}} + \epsilon \frac{x_{\text{rand}} - x_{\text{near}}}{\|x_{\text{rand}} - x_{\text{near}}\|}$
               </div>
               
   - **Update Tree:** Add $x_{\text{new}}$ to the tree
   
## Pseudo-code for RRT
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

## Implementation in Python 
- **Key Functions:**
  
     - **sample_envir:** Method to randomly sample a point within the map dimensions.
       
     - **nearest:** Method to find the nearest node to a given node in the graph.
       
     - **isFree:** Method to check if the last added node is in collision with any obstacles.
       
     - **expand:** Method to expand the RRT graph by adding a new node.
       
     - **path_to_goal:** Method to generate a path from the start to the goal using the parent relationships.
       
       
