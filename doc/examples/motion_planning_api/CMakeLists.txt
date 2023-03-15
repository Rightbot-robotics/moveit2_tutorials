add_executable(motion_planning_api_tutorial
    src/motion_planning_api_tutorial.cpp)
target_include_directories(motion_planning_api_tutorial
  PUBLIC include)
ament_target_dependencies(motion_planning_api_tutorial
  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS motion_planning_api_tutorial
        DESTINATION lib/${PROJECT_NAME}
        )

add_executable(motion_planning_api_test
        src/motion_planning_api_test.cpp)
target_include_directories(motion_planning_api_test
        PUBLIC include)
ament_target_dependencies(motion_planning_api_test
        ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS motion_planning_api_test
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
