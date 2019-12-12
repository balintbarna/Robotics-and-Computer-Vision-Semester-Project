# RoVi_project
This repo contains all parts of the rovi semester project
including:
- Workcell with robot, camera and object to move
- 2D -> 3D pose estimation by RGB template matching
- 3D pose estimation by point cloud from depth sensor
- Optimizing reachability
- Point-to-Point interpolate 6 points
- Interpolate with parabolic blend
- RRT Connect
- Integration into one complete solution using some of these parts

Workcell folder contains workcell we used for all other tasks
V_2D_Pose folder contains self-contained codebase for template matching. See Readme in folder.
V_3D_Pose folder contains self-contained codebase for depth sensor pose estimation. See readme in folder.
main folder contains plugin which contains the code for many of the other parts. See Readme in folder.

Sample codes from RoVi course were used as inspiration everywhere in the project.