cmake_minimum_required(VERSION 3.8)
project(cpp_httplib_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include(ExternalProject)
ExternalProject_Add(cpp-httplib
  GIT_REPOSITORY  https://github.com/yhirose/cpp-httplib.git
  GIT_TAG         v0.16.3
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/cpp-httplib_install
)
install(
  DIRECTORY   ${CMAKE_CURRENT_BINARY_DIR}/cpp-httplib_install/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
