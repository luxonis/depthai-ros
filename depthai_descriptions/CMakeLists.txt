cmake_minimum_required(VERSION 3.8)
project(depthai_descriptions VERSION 3.3.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(FILES scripts/resolve_description.py scripts/model_gallery.py DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/model_gallery.py scripts/imu_coordinate_check.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/resolve_description.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(descriptions_test test TIMEOUT 120)
endif()


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
