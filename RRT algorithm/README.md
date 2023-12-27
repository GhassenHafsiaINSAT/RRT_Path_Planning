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

## Usage

### Prerequisites

Make sure you have Python and pygame installed on your local machine.

```bash
pip install pygame

git clone https://github.com/your-username/rrt-path-planning.git
cd rrt-path-planning

python main.py
