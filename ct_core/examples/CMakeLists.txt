add_executable(ex_DampedOscillatorCustomController src/DampedOscillatorCustomController.cpp)
target_link_libraries(ex_DampedOscillatorCustomController ct_core)
list(APPEND core_ex_TARGETS ex_DampedOscillatorCustomController)

add_executable(ex_DampedOscillator src/DampedOscillator.cpp)
target_link_libraries(ex_DampedOscillator ct_core)
list(APPEND core_ex_TARGETS ex_DampedOscillator)

add_executable(ex_MasspointIntegration src/MasspointIntegration.cpp)
target_link_libraries(ex_MasspointIntegration ct_core)
list(APPEND core_ex_TARGETS ex_MasspointIntegration)

add_executable(plotTest src/plot/plotTest.cpp)
target_link_libraries(plotTest ct_core)
list(APPEND core_ex_TARGETS plotTest)

if(QWT_ENABLED)
    add_executable(plotQwtTest src/plot/plotQwtTest.cpp)
    target_link_libraries(plotQwtTest ct_core)
    list(APPEND core_ex_TARGETS plotQwtTest)
endif()

## install examples
include(GNUInstallDirs)
install(
    TARGETS ${core_ex_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_core
    )