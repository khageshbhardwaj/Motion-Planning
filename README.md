# ECE276B PR2 Spring 2024
# Motion Planning Algorithms: A* and RRT

## Overview

This project compares the performance of two popular path planning algorithms: the search-based A* algorithm and the sampling-based Rapidly-Exploring Random Tree (RRT) algorithm. The comparison is made in various 3-D environments, focusing on path quality, computational efficiency, and adaptability.

## Directory Structure
├── main_astar.py
├── AStar.py
├── main_RRT.py
├── rrt_algorithm/
│ ├── rrt.py
│ ├── rrt_base.py
│ ├── tree.py
│ ├── geometry.py
│ └── (additional files required for RRT)
├── requirements.txt
└── README.md


## A* Algorithm

### Files:
- `main_astar.py`: Main file to run the A* algorithm.
- `AStar.py`: Contains the A* class along with the custom collision checking class.

### Running the A* Algorithm:
To run the A* algorithm on all possible environments, execute:
- `python main_astar.py`

## RRT Algorithm

### Files:
- `main_RRT.py`: Main file to run the RRT algorithm.

## Running the RRT Algorithm:
- `python main_RRT.py`

### Requirements
- `pip install -r requirements.txt`

