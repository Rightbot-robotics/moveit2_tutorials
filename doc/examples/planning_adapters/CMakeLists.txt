add_executable(planning_adapters_tutorial
    src/planning_adapters_tutorial.cpp)
target_include_directories(planning_adapters_tutorial
  PUBLIC include)
ament_target_dependencies(planning_adapters_tutorial
  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS planning_adapters_tutorial
        DESTINATION lib/${PROJECT_NAME}
        )

add_executable(planning_adapters_test
        src/planning_adapters_test.cpp)
target_include_directories(planning_adapters_test
        PUBLIC include)
ament_target_dependencies(planning_adapters_test
        ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS planning_adapters_test
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
