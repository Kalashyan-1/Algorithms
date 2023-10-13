cmake_minimum_required(VERSION 3.0)
project(Algorithms)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)


set(SOURCES 
    main.cpp
    src/ConvexHull/convexHull.cpp
    src/Graph/wgraphList.cpp
    src/PatternMatching/PatternMatch.cpp
    src/Tests/test.cpp
)

add_executable(Algorithms ${SOURCES})

target_link_libraries(Algorithms PRIVATE ${OpenCV_LIBS})
target_link_libraries(Algorithms PRIVATE nlohmann_json::nlohmann_json)
