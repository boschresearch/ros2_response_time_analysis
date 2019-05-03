cmake_minimum_required(VERSION 3.5)

function(generate_interfaces)
  file(GLOB_RECURSE ifs
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "msg/*.msg"
    "srv/*.srv")

  if(NOT ifs)
    return()
  endif()

  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME} ${ifs})
  get_default_rmw_implementation(rmw)
  find_package("${rmw}" REQUIRED)
  get_rmw_typesupport(impls "${rmw}" LANGUAGE "cpp")
  foreach(target ${${PROJECT_NAME}_EXECUTABLES}${${PROJECT_NAME}_LIBRARIES})
    foreach(impl ${impls})
      rosidl_target_interfaces(${target} ${PROJECT_NAME} ${impl})
    endforeach()
  endforeach()
endfunction()


project(model_validation)
add_compile_options(-g -Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(arbitrary_cb_time src/arbitrary_cb_time.cpp)
generate_interfaces()

install(DIRECTORY bin/ DESTINATION bin
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
ament_auto_package(INSTALL_TO_PATH)
