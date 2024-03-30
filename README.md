## Introduction
This repository includes my implementations of search-based and sampling-based planning algorithms. These were done as part of coursework in the graduate course: 16782 - Planning and Decision-Making in Robotics at CMU Robotics (Fall 2023). 

Jump to section: 
- [Search-based Planners (A* and variants)](https://github.com/shivamtrip/robot-planning/edit/main/README.md#search-based-planners-a-and-variants)
- [Sampling-based Planners (RRT and variants)](https://github.com/shivamtrip/robot-planning/edit/main/README.md#sampling-based-planners-rrt-and-variants)

<br />

## Search-based Planners (A* and variants) 
**Goal:** Implement A* and its variants to enable a robot (blue) to plan a path to catch a moving target (red) <br />
**Details:** Code implemenation can be found in '_search-based_planners_' folder above.
Map | Map
:-------------------------:|:-------------------------:
<img src="https://github.com/shivamtrip/robot-planning/assets/66013750/8dd9466d-fd63-4249-9767-03796f1f1731" width="400"> &nbsp; | &nbsp; <img src = "https://github.com/shivamtrip/robot-planning/assets/66013750/fae9374c-94f9-46fa-bfca-131a0175735a" width="400"> <br />
<img src = "https://github.com/shivamtrip/robot-planning/assets/66013750/361b6b0d-95f2-461a-8ae0-a0a2463d1941" width="400"> &nbsp;| &nbsp; <img src="https://github.com/shivamtrip/robot-planning/assets/66013750/3b6bd24a-970b-48a2-9b55-609336eb7ec2" width="400" > <br />

<br />
<br />

## Sampling-based Planners (RRT and variants)
**Goal:** Implement RRT, RRT-Connect, RRT* and PRM to enable a 5-DOF robot to move end-effector from start position (bottom of map) to goal position (top-left corner) <br />
**Details:** Code implemenation can be found in '_sampling-based_planners_' folder above.
Planner | Number of Steps in Plan | Video
:-------------------------:|:-------------------------:|:-------------------------:
RRT | 11 steps | <img src="https://github.com/shivamtrip/robot-planning/assets/66013750/268e979e-ade1-47b5-8251-2da6f70fc1fd" width="400"> &nbsp;
RRT-Connect | 18 steps | <img src="https://github.com/shivamtrip/robot-planning/assets/66013750/ee1e78a0-7cb9-42c8-8178-c5625ffb320f" width="400"> &nbsp;
RRT-Star | 5 steps | <img src="https://github.com/shivamtrip/robot-planning/assets/66013750/a57cb4a3-4e7f-491b-8a5e-3d2df7a96348" width="400"> &nbsp;
PRM + A* | 4 steps | <img src="https://github.com/shivamtrip/robot-planning/assets/66013750/ba1ea4c7-d4e1-4ccf-b72d-e8de915c33ad" width="400"> &nbsp;
