# RRT Path Planning

This module implements the Rapidly Exploring Random Trees (RRT) algorithm for path planning in a 2D environment.

## Table of Contents
- [Introduction](#introduction)
- [Files](#files)
- [Usage](#usage)
  - [Prerequisites](#prerequisites)
  - [Running Locally](#running-locally)
  - [Docker](#docker)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This code provides a simple implementation of the RRT algorithm for path planning. It consists of two files:
- `RRTbasePY.py`: Contains the RRTGraph and RRTMap classes implementing the RRT algorithm.
- `main.py`: A sample script demonstrating the usage of the RRT algorithm with a graphical representation.

## Files

1. `RRTbasePY.py`: Contains the RRTGraph and RRTMap classes implementing the RRT algorithm.

2. `main.py`: Sample script demonstrating the usage of the RRT algorithm with a graphical representation.


## Code Explanation

### RRTbasePY.py

This file (`/path/to/RRTbasePY.py`) contains the implementation of the RRT algorithm. It defines two classes: `RRTMap` and `RRTGraph`.

- **RRTMap Class**: Manages the visualization of the map and obstacles. It includes methods to draw the map, obstacles, and paths.

- **RRTGraph Class**: Manages the RRT graph structure and algorithm. It includes methods for adding nodes, connecting edges, sampling the environment, and generating paths.

### main.py

This file (`/path/to/main.py`) uses the RRT algorithm from `RRTbasePY.py` to plan a path in a 2D environment using Pygame for visualization.

- It initializes the RRTMap and RRTGraph classes.
- Obstacles and map dimensions are defined.
- The RRT algorithm is executed with visualization using Pygame.


### Prerequisites

Make sure you have Python and pygame installed on your local machine.

```bash
pip install pygame

git clone https://github.com/your-username/rrt-path-planning.git
cd rrt-path-planning

python main.py
