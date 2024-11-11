# Path Planning Algorithms


## Overview

Welcome to the **Path Planning Algorithms** project! This project aims to provide a comprehensive overview and implementation of various path planning algorithms used in robotics. Through interactive videos and detailed explanations, users can explore fundamental techniques such as Bug Algorithms, Probabilistic Roadmaps (PRM), Rapidly-exploring Random Trees (RRT), Artificial Potential Based Planner (APBP), and Simultaneous Localization and Mapping (SLAM).


## Algorithms Covered

### Bug Algorithms

#### Bug1

The **Bug1** algorithm involves a robot navigating towards a goal while only detecting collisions, without using any other sensors besides a collision sensor. In this algorithm, the robot moves towards the target, and when it collides with an obstacle, it remembers the collision point. Then, it moves around the obstacle until it returns to the initial collision point. During this process, the robot also remembers the point closest to the goal. Once the robot returns to this closest point, it continues moving directly towards the target.


#### Bug2

**Bug2** is very similar to Bug1, but with one key difference: the robot knows the "line" that connects the start position to the goal. The robot moves towards the goal, and when it collides with an obstacle, it follows the same behavior as Bug1 by moving around the obstacle. However, as soon as the robot finds the line connecting the start to the goal, it immediately follows it towards the goal.


#### Tangent Bug

The **Tangent Bug** algorithm uses a radar sensor in addition to a collision sensor. This algorithm allows the robot to detect obstacles in its path using the radar sensor, helping it calculate the shortest path around the obstacle while still progressing towards the goal. The radar sensor continuously measures the distance and direction to obstacles, enabling the robot to decide when to follow the obstacle boundary and when to move directly towards the goal.


### Artificial Potential Based Planner (APBP)

The **Artificial Potential Based Planner (APBP)** algorithm is based on gradient descent and ascent. We define the location of obstacles and apply "forces" towards the goal and away from obstacles. The main idea is that the goal creates an attractive field, while obstacles create repulsive fields. It is important to note that these forces are initially defined in the workspace, but need to be mapped to the configuration space for planning. However, a critical issue with this algorithm is that the robot can get stuck in local minima, requiring a new planner to be designed to overcome this limitation.


### Sampling Based Methods

#### Probabilistic Roadmap Method (PRM)

The **Probabilistic Roadmap Method (PRM)** involves sampling the configuration space, performing collision checks with obstacles, building a roadmap, finding neighbors, and connecting edges. PRM works by creating a network of feasible paths through random sampling and connecting collision-free points. It is particularly suitable for high-dimensional, complex environments where a roadmap can be precomputed. However, because it is an "offline" planning method that requires a complete roadmap to be built before finding a solution, it often has a higher computational cost compared to RRT, which can operate incrementally.



#### Rapidly-exploring Random Trees (RRT)

**Rapidly-exploring Random Trees (RRT)** is a sampling-based algorithm for path planning. It works by sampling random points in the configuration space, finding the nearest node in the existing tree, and expanding towards the sampled point. This process is repeated until the goal is reached. It is efficient in complex environments but can result in suboptimal paths.



