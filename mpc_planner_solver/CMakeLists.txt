cmake_minimum_required(VERSION 3.8)
project(mpc_planner_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(DEPENDENCIES
  mpc_planner_util
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include(solver.cmake)

add_library(${PROJECT_NAME} SHARED
  ${solver_SOURCES}
  src/state.cpp
  src/mpc_planner_parameters.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
foreach(dir IN LISTS solver_INCLUDE_DIRS)
  get_filename_component(solver_include_dir "${dir}" ABSOLUTE)
  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${solver_include_dir}>"
  )
endforeach()
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
foreach(library IN LISTS solver_LIBRARIES)
  if(library MATCHES "^${PROJECT_SOURCE_DIR}/")
    get_filename_component(solver_library_name "${library}" NAME)
    target_link_libraries(${PROJECT_NAME}
      "$<BUILD_INTERFACE:${library}>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/${solver_library_name}>"
    )
    install(FILES "${library}" DESTINATION lib)
  else()
    target_link_libraries(${PROJECT_NAME} "${library}")
  endif()
endforeach()

if("src/acados_solver_interface.cpp" IN_LIST solver_SOURCES)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ACADOS_SOLVER)
  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/acados>"
    "${acados_include_path}"
    "${acados_include_path}/blasfeo/include"
    "${acados_include_path}/hpipm/include"
  )
  install(DIRECTORY acados/Solver
    DESTINATION include/${PROJECT_NAME}/acados
    FILES_MATCHING PATTERN "*.h"
  )
else()
  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  install(DIRECTORY Solver/include/
    DESTINATION include/${PROJECT_NAME}/
  )
endif()

option(BUILD_SOLVER_TEST "Build test_solver.cpp" OFF)

if(BUILD_TESTING AND BUILD_SOLVER_TEST)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_solver.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

install(
  TARGETS 
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
