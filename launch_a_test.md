launch a navigation test
========================

User gets to pick from these options;

1. #### select a robot
	- `pioneer`, `fetch`

2. #### select a map
    - `nerve1_empty`, `nerve1_obstacles`, `nerve2_empty`, `nerve2_obstacles`, `nerve3_empty`, `nerve3_obstacles`,	`some_other_world...`
	    - [nerve1,2,3 worlds in gazebo](https://github.com/uml-robotics/uml_3d_race/tree/master/worlds)
	    - add more
    
    - If a world with obstacles is selected, user also selects:
        - A belief condition (ie belief about the presence of obstacles, used for planning):
            - `fN`, `fP`, `tN`, `tP`

    - Select obstacle type/size/dimensionality:
	    - `2d_blocking`, `3d_blocking`
	    - `short`, `tall`, `wide`, `thin`, `??`
        - `stairs`, `pallet`, `forklift`

    - select obstacle positioning
	    - for these we need coordinates for "important" obstacle locations (like before a 90 degree turn, or at the end of a corridor) for all maps
        - obstacle placements: the concept of positions within routes such that they cause path deviations, route deviations (blocked sufficiently, cannot go through, so the robot takes a different route)
    - select obstacle grouping: 
	    - `sparse`, `dense`, `patchy`, `??`
	    - define obstacles using models that already exist, a certain relative distance from eachother; then you can predefine groups of obstacles and place them where you want; make a directory of different obstacle scenarios

3. select test:
    - `go_to_goal`

4. select number of repetitions:
    - `10`, `30`, `n`


### Sample test execution:

```bash
launch_nav_test_sim -rP -mNerv1Obs -bT -t3d -p -n30
```


### Expectations:

- Program should run the test the user selects.

- registering success: detection for when robot finishes successfully
- registering failure: need ways to register the 2 ways test could fail: collision, or didn't reach goal;
    - registering collisions in gazebo (and where in map it happened?)



  

- Gazebo position publisher for robot
	- Program should record the robot's global position at all times during the current test. 
	- (Some gazebo logs?)
	- We should output these gazebo position logs as a csv file

- Record simulation
	- [recording simulation for future reproductions](http://gazebosim.org/tutorials?cat=tools_utilities&tut=logging_playback)

- setup go_to_goal program w/ pete's package
- for each repetition, we're clearing the costmap
    - need to add service to clear costmap!




