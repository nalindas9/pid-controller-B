set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    test.cpp
    ../app/PIDController.cpp
)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

include_directories(
    ${CMAKE_SOURCE_DIR}/app
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include
                                           ../vendor/googletest/googlemock/include)
target_link_libraries(cpp-test PUBLIC gtest)
