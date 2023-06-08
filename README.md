# navigation_stack

## Demo
Here are some preliminary results for motion planning based on state lattice in Frenet frame and parametric curve in the form of cubic splines:

![spline_with_lattice](https://github.com/Tensor-Robotics/navigation_stack/assets/53962958/ba3d15e8-a010-451f-8978-22912e787a23)



## Goal
The goal of this project is to build a navigation stack from ground up consisting of perception, prediction, localization, planning and controls modules. 
Each of these modules consists of two individual packages -> domain and interface. The domain package holds all the core logic whereas the interface packageis responsible for communication between modules.

This project is being implemented in ROS2.

At its current stage, the stack can generate cubic splines for an autonomous vehicle as well as localize itself using kalman filter.

The directory tree inside the src/ directory of the navigation stack looks like the following:

```
.
├── clear.sh
├── common
│   ├── domain
│   │   ├── CMakeLists.txt
│   │   ├── core_datastructures
│   │   │   ├── Point.cpp
│   │   │   ├── Point.hpp
│   │   │   ├── Pose.cpp
│   │   │   ├── Pose.hpp
│   │   │   ├── Posture.cpp
│   │   │   └── Posture.hpp
│   │   └── package.xml
│   └── interface
│       ├── CMakeLists.txt
│       ├── converters
│       │   ├── converters.cpp
│       │   └── converters.hpp
│       ├── interface_msgs
│       │   └── msg
│       │       ├── Posture.msg
│       │       └── Spline.msg
│       └── package.xml
└── modules
    ├── localization
    │   ├── domain
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   └── interface
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       └── src
    │           └── localization_publisher.cpp
    └── trajectory_generation
        ├── domain
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── spline_generation
        │       ├── cubic_spline_generator.cpp
        │       ├── cubic_spline_generator.hpp
        │       ├── i_spline_generator.hpp
        │       └── test.cpp
        └── interface
            ├── CMakeLists.txt
            ├── dev_tools
            │   └── subscriber_member_function.py
            ├── package.xml
            └── src
                ├── main.cpp
                ├── trajectory_publisher.cpp
                └── trajectory_publisher.hpp
```
