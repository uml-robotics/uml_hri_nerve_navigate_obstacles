### Navigation test methods

- Mobile robot details
- Sensor details
- Maps/layouts
- Obstacles
  * tall=static obstacles primarily taller than robot’s laser
  * short=static obstacles primarily shorter than robot’s laser
  * negative=static obstacle w/ depth below ground level (grate, stairs, hole in ground, etc)
  * dynamic=moving obstacles (eg another robot, or vehicle, or person); fixed or random trajectories
- Knowledge conditions

## Pioneer3dx

| 2d Nerve 1| 2d Nerve2 | 2d Nerve3 | 3d Nerve1 | 3d Nerve2 | 3d Nerve3 |
| ---| --- | --- | ---| --- | --- |
| 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 |
| 2,0,2,0 | [2,0,2,0](https://github.com/uml-robotics/uml_hri_nerve_navigate_obstacles/blob/master/tests/info/pioneer_2d_nerve2_2020.md) | 2,0,2,0 | 2,0,2,0 | 2,0,2,0 | 2,0,2,0 |
| 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 |
| 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 |
| 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 |
| 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 |

## Fetch Mobile Manipulator

| 2d Nerve 1| 2d Nerve2 | 2d Nerve3 | 3d Nerve1 | 3d Nerve2 | 3d Nerve3 |
| ---| --- | --- | ---| --- | --- |
| 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 | 1,1,0,0 |
| 2,0,2,0 | 2,0,2,0 | 2,0,2,0 | 2,0,2,0 | 2,0,2,0 | 2,0,2,0 |
| 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 | 2,2,0,1 |
| 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 | 0,0,1,1 |
| 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 | 5,5,0,1 |
| 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 | 5,5,4,2 |


Nerve1:
![nist1](https://github.com/uml-robotics/uml_3d_race/blob/master/resources/screenshots/nist1.png)

Nerve2:
![nist2](https://github.com/uml-robotics/uml_3d_race/blob/master/resources/screenshots/nist2.png)

Nerve3:
![nist3](https://github.com/uml-robotics/uml_3d_race/blob/master/resources/screenshots/nist3.png)