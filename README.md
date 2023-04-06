# FlightPathPro
The FlightPathPro is a tool that is developed to generate high dynamic paths and trajectories from waypoints using B-splines while considering a key factor in flight mechanics, the G factor. The tool adds and deletes waypoints when necessary to ensure that the trajectory is flightable. Using the generated trajectory and the unmanned aerial vehicle's positions, the tool finds a matching trajectory and calculates the control error that is needed to enable the UAS to follow the trajectory.

+ B-splines are a type of mathematical function that are often used in computer graphics and animation to create smooth and continuous curves. By using B-splines to generate flight paths, the Flight Path Generator is able to create high dynamic paths and trajectories that are suitable for UAVs.

+ One important factor in flight mechanics is the G-factor, which is a measure of the acceleration that an aircraft experiences. By taking the G-factor into account when generating flight paths, the Flight Path Generator is able to create trajectories that are safe and effective for UAVs.

+ Another important feature of the Flight Path Generator is its ability to add and delete waypoints as needed to ensure that the trajectory is flightable. This helps to ensure that the UAV can follow the trajectory safely and accurately.

+ Once the trajectory has been generated, the Flight Path Generator uses the UAV's position to find a matching trajectory and calculates the control error that is needed to keep the UAV on the trajectory. This helps to ensure that the UAV can follow the trajectory as closely as possible, even in the presence of external factors such as wind or turbulence.

## Installation
1. **Clone the repository to your local machine using the following command:**

   `git clone https://github.com/abbas-sabra/Flight-Path-Generator.git`
2. **Install Matlab on your machine.**
3. **Open the project file in Matlab.**
4. **Build and run the `trajectory_example.m` in Matlab.**

## Files

  
# Credits
This Tool was created by Abbas Sabra as a personal project "Generation of highly dynamic flight paths and trajectories from waypoints for dynamic soaring close to the ground".

# License
This Tool is licensed under the MIT License. Feel free to modify and distribute the code as you see fit.
