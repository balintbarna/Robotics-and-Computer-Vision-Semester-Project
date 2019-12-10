# RoVi Porject main program

This plugin for RobWorkStudio is the glue that holds together the project parts.

To build it it is recommended to use the VM from RoVi course, rebuild robwork with the latest state from git, and set environment variables for it.

Paths may be set up assuming that this project is in /home/studen/Workspace/RoVi_project/

to build it
```
cd build
cmake ..
make -j2
```

Binary will be build in libs

Add .so to Robwork as plugin

Use buttons to try out functionality, like test reachability, rrt connect, vision, playback.

Parts of source code from the RoVi course were used in the project.