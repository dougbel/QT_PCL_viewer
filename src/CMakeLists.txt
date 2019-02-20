cmake_minimum_required(VERSION 2.8.11)

project(pc_viewer)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)

find_package(VTK REQUIRED)
#find_package(PCL 1.7.1 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


include_directories(${PCL_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES  viewerwindow.cpp cloudqtdata.cpp clouddata.cpp)

add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp ${SOURCES})

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

qt5_use_modules(${PROJECT_NAME} Widgets)
