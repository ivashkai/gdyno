cmake_minimum_required(VERSION 3.16)
project(gdyno VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gdyno STATIC
    src/GdynoLoader.cpp
)

target_include_directories(gdyno PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

option(GDYNO_BUILD_TOOLS "Build glb2gdyno converter tool" ON)

if(GDYNO_BUILD_TOOLS)
    add_executable(glb2gdyno
        tools/glb2gdyno.cpp
    )
    target_include_directories(glb2gdyno PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    )
    target_link_libraries(glb2gdyno PRIVATE gdyno)
endif()
