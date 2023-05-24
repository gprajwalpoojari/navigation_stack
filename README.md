# navigation_stack


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
