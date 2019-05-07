set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h)

add_executable(ex_ConstraintOutput ConstraintExampleOutput.cpp)
target_link_libraries(ex_ConstraintOutput ct_optcon)

add_executable(ex_LQR LQR.cpp)
target_link_libraries(ex_LQR ct_optcon)

add_executable(ex_NLOC NLOC.cpp)
target_link_libraries(ex_NLOC ct_optcon)

add_executable(ex_NLOC_MPC NLOC_MPC.cpp)
target_link_libraries(ex_NLOC_MPC ct_optcon)

add_executable(ex_KalmanFiltering KalmanFiltering.cpp)
target_link_libraries(ex_KalmanFiltering ct_optcon)

add_executable(ex_KalmanDisturbanceFiltering KalmanDisturbanceFiltering.cpp)
target_link_libraries(ex_KalmanDisturbanceFiltering ct_optcon )

add_executable(ex_Nlp_2D nlp/Nlp2D.cpp)
target_link_libraries(ex_Nlp_2D ct_optcon)

add_executable(ex_Nlp_3D nlp/Nlp3D.cpp)
target_link_libraries(ex_Nlp_3D ct_optcon)

add_executable(ex_DMS DMS.cpp)
target_link_libraries(ex_DMS ct_optcon)

if(HPIPM)
    add_executable(ex_NLOC_boxConstrained NLOC_boxConstrained.cpp)
    target_link_libraries(ex_NLOC_boxConstrained ct_optcon)
    
    add_executable(ex_NLOC_generalConstrained NLOC_generalConstrained.cpp)
    target_link_libraries(ex_NLOC_generalConstrained ct_optcon)
    
    add_executable(switched_continuous_optcon_example switched_systems_optcon/switched_continuous_optcon.cpp)
    target_link_libraries(switched_continuous_optcon_example ct_optcon)
endif(HPIPM)