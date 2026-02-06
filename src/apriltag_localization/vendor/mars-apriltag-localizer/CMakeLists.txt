cmake_minimum_required(VERSION 3.15)
project(marslocalizer
        VERSION 0.0.1
        DESCRIPTION "A library for camera-based localization with AprilTags."
)

option(BUILD_EXAMPLES "Build example executables" ON)
option(INSTALL_PKG_CONFIG "Install a .pc file for this library" ON)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag 3 REQUIRED)
find_package(nlohmann_json 3 REQUIRED)

include_directories(
        include/marslocalizer
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-O0)
    add_link_options(-O0)
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-O2)
    add_link_options(-O2)
endif ()

add_library(${PROJECT_NAME}
        src/marslocalizer/camera.cpp
        src/marslocalizer/detection.cpp
        src/marslocalizer/detector.cpp
        src/marslocalizer/family.cpp
        src/marslocalizer/field.cpp
        src/marslocalizer/localizer.cpp
        src/marslocalizer/overlays.cpp
        src/marslocalizer/pnp.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER include/marslocalizer/wrapper.hpp
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
)
target_link_libraries(${PROJECT_NAME}
        PUBLIC apriltag
        PUBLIC Eigen3::Eigen
        PUBLIC ${OpenCV_LIBS}
        PUBLIC nlohmann_json::nlohmann_json
)
install(
        TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
if (INSTALL_PKG_CONFIG)
    configure_file(marslocalizer.pc.in marslocalizer.pc @ONLY)
    install(
            FILES ${CMAKE_BINARY_DIR}/marslocalizer.pc
            DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
    )
endif ()

if (BUILD_EXAMPLES)
    add_executable(2d_detect
            examples/2d_detect.cpp
    )
    add_executable(2d_detect_simple
            examples/2d_detect_simple.cpp
    )
    add_executable(localize
            examples/localize.cpp
    )
    add_executable(localize_data
            examples/localize_data.cpp
    )
    add_dependencies(2d_detect ${PROJECT_NAME})
    add_dependencies(2d_detect_simple ${PROJECT_NAME})
    add_dependencies(localize ${PROJECT_NAME})
    add_dependencies(localize_data ${PROJECT_NAME})

    target_link_libraries(2d_detect
            ${PROJECT_NAME}
    )
    target_link_libraries(2d_detect_simple
            ${PROJECT_NAME}
    )
    target_link_libraries(localize
            ${PROJECT_NAME}
    )
    target_link_libraries(localize_data
            ${PROJECT_NAME}
    )
    install(
            TARGETS 2d_detect 2d_detect_simple localize localize_data
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif ()