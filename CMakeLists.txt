cmake_minimum_required(VERSION 3.5...3.10)
project(kbot VERSION 1.0.0)

add_subdirectory(extern)

# Create targets and set properties
add_executable(${PROJECT_NAME}
    "src/main.cpp"
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
)

target_compile_features(${PROJECT_NAME}
    PRIVATE
        cxx_auto_type
        cxx_generic_lambdas
        cxx_lambdas
        cxx_lambda_init_captures
        cxx_range_for
)
