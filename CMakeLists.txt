cmake_minimum_required(VERSION 3.30)

project(GenerateCloudApp VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

if(WIN32)
    set(OpenCV_DIR "D:/Software/OpenCV/OpenCV-VC17-Build-OpenCV-4.10.0-x64")
    set(GLFW3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/glfw/GLFW")
    set(GLFW3_LIBRARY "${CMAKE_SOURCE_DIR}/thirdparty/glfw/lib-vc2022/glfw3.lib")
    file(TO_CMAKE_PATH "D:/Software/MVTec/HALCON-24.11-Progress-Steady/examples" HALCONEXAMPLES)
else()
    file(TO_CMAKE_PATH "$ENV{HALCONROOT}/examples" HALCONEXAMPLES)
    find_package(glfw3 REQUIRED)
endif()

find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${HALCONEXAMPLES}/cmake")
include(UseHALCON)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdparty
)

file(GLOB ORIGIN
    ${CMAKE_SOURCE_DIR}/*.h
    ${CMAKE_SOURCE_DIR}/*.cpp
)

file(GLOB_RECURSE HERDERS
    ${CMAKE_SOURCE_DIR}/include/*.h
    ${CMAKE_SOURCE_DIR}/include/*.hpp
    ${CMAKE_SOURCE_DIR}/thirdparty/*.h
    
)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/thirdparty/*.cpp
)

set(PROJECT_SOURCES
    ${ORIGIN}
    ${HERDERS}
    ${SOURCES}
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/core
    ${CMAKE_SOURCE_DIR}/include/gui
    ${CMAKE_SOURCE_DIR}/include/modules
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
    ${CMAKE_SOURCE_DIR}/thirdparty/glfw/GLFW
)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${GLFW3_LIBRARY})
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
target_link_libraries(${PROJECT_NAME} PRIVATE HALCON::CppInt)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})