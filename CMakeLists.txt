cmake_minimum_required(VERSION 2.8.3)
project(ed_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  geolib2
  tue_config
  tue_filesystem
  tue_serialization
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ed_core 
    CATKIN_DEPENDS tue_config geolib2 class_loader
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wsign-compare -Wreorder")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                          ED CORE
# ------------------------------------------------------------------------------------------------

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.h)

# Declare a cpp library
add_library(ed_core
    src/entity.cpp
    src/update_request.cpp
    src/world_model.cpp
    src/transform_cache.cpp

    # World model querying
    src/world_model/transform_crawler.cpp

    # Model loading
    src/models/model_loader.cpp
    src/models/shape_loader.cpp
    src/models/xml_shape_parser.cpp

    # Logging
    src/logging.cpp

    # Serialization
    src/serialization/serialization.cpp

    src/error_context.cpp
    include/ed/error_context.h

    ${HEADER_FILES}
)
target_link_libraries(ed_core ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(ed
    src/ed.cpp
    src/server.cpp
    src/plugin_container.cpp
)
target_link_libraries(ed ed_core)

# ------------------------------------------------------------------------------------------------
#                                           EXAMPLE PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(ed_hello_world_plugin examples/plugins/hello_world/hello_world_plugin.cpp)
add_library(ed_custom_properties_plugin examples/plugins/custom_properties/custom_properties_plugin.cpp)
