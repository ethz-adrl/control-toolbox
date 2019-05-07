add_executable(ex_DampedOscillatorCustomController src/DampedOscillatorCustomController.cpp)
target_link_libraries(ex_DampedOscillatorCustomController ct_core)

add_executable(ex_DampedOscillator src/DampedOscillator.cpp)
target_link_libraries(ex_DampedOscillator ct_core)

add_executable(ex_MasspointIntegration src/MasspointIntegration.cpp)
target_link_libraries(ex_MasspointIntegration ct_core)

add_executable(plotTest src/plot/plotTest.cpp)
target_link_libraries(plotTest ct_core)

if(QWT_ENABLED)
    add_executable(plotQwtTest src/plot/plotQwtTest.cpp)
    target_link_libraries(plotQwtTest ct_core)
endif()

