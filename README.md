> This Project has a OOP version -> https://github.com/zlaazlaa/-DWA-
# DWA (Dynamic Window Approach) Path Planning Algorithm

This is a C++ implementation of the DWA (Dynamic Window Approach) path planning algorithm. The DWA algorithm is a popular method used in robotics and autonomous vehicles for real-time motion planning. It takes into account the robot's dynamic constraints and environment obstacles to generate a safe and efficient trajectory towards a given destination.

## Introduction

The DWA algorithm is designed to calculate the optimal velocity and angular velocity of a robot given its current state and the goal position. It considers the robot's maximum velocity, maximum angular velocity, maximum velocity acceleration, and maximum angular acceleration as its dynamic constraints.

## Usage

To use the DWA algorithm, you need to provide a map of the environment where the robot operates. The map should be represented as a 2D array, where each element represents an obstacle or free space.

**Input:**

- `Map`: A 2D array representing the environment map, where obstacles are denoted as '1' and free spaces are denoted as '0'.
- `Car_Coordinate`: The current coordinate of the robot in the map.
- `Now_Angle`: The current angle of the robot.
- `Now_Velocity`: The current velocity of the robot.
- `Now_Angular_Velocity`: The current angular velocity of the robot.
- `Car_Destination`: The destination coordinate of the robot in the map.
- `Model`: A `Bot_Model` struct representing the dynamic constraints of the robot.

**Output:**

- `Target`: A `Pair` struct containing the optimal target velocity and target angular velocity for the robot to follow.

## Code Structure

The code is divided into several parts:

1. **Namespace and Constants**: Definitions of namespaces and constants used in the algorithm.
2. **Structures**: Definitions of various structures used in the algorithm, such as `Bot_Model`, `Pair`, `Coordinate`, and `Node`.
3. **Helper Functions**: Implementation of helper functions like `MIN`, `MAX`, `Calc_Dist`, `Legal_Coordinate`, `Get_Trajectory`, and `Get_Dist_To_Obstacle`.
4. **A\* Path Planning**: Implementation of the A* path planning algorithm to find a safe and optimal path.
5. **Main Function**: The main function that uses the DWA algorithm to calculate the optimal trajectory for the robot.

## Example

Here is an example of how to use the DWA algorithm:

```
cppCopy code#include <iostream>
// Add other necessary include statements here

// ... (Copy the main.cpp content here)

int main() {
    // Example map (represented as a 2D array)
    char MM[Max_Range][Max_Range] = {
        // Fill in your map data here
    };

    // Example coordinates and parameters
    Get_DWA_Answer::Coordinate Car_Coordinate = {0, 35};
    Get_DWA_Answer::Coordinate Car_Destination = {119, 35};
    double Angle = 50;
    double Velocity = 0;
    double Angular_Velocity = 0;
    Get_DWA_Answer::Bot_Model Model = {1.0, 60, 0.5, 60};

    // Call the DWA function to get the optimal target velocity and angular velocity
    Get_DWA_Answer::Pair Target = Get_DWA_Answer::DWA(MM, Car_Coordinate, Angle, Velocity, Angular_Velocity,
                                                     Car_Destination, Model);

    // Output the results
    std::cout << "Target Velocity: " << Target.Target_Velocity << std::endl;
    std::cout << "Target Angular Velocity: " << Target.Target_Angular_Velocity << std::endl;

    return 0;
}
```

Replace the map data and example coordinates with your actual data to use the DWA algorithm for your specific application.

## Note

The code provided is a simple implementation of the DWA algorithm and may need further optimization and refinement for specific use cases. Additionally, this document only serves as a brief overview of the code; detailed explanations of the algorithm are not included. If you require further understanding of the DWA algorithm, additional resources and research are recommended.

For more complex scenarios and real-world applications, consider integrating this code with your robotics or autonomous vehicle system and adapting it to your specific hardware and sensor inputs.
