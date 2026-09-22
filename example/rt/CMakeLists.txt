#-------------------------------------------------------------------------------
# RT mode example executables setup

# Here puts required example source files
set(RT_EXAMPLES
    joint_position_control
    joint_impedance_control
    cartesian_impedance_control
    move_commands
    joint_s_line
    cartesian_s_line
    rt_industrial
    servoj_demo
    cr20_cartesian_impedance_test
    )

#-------------------------------------------------------------------------------
# CR20 drill plunge (cr20_drill/). Two targets rather than one, because the two
# pieces that decide where the bit goes - the frame math and the feed law - are
# pure functions of numbers and belong where a laptop can prove them wrong.

# The robot program. Lives in a subdirectory, so it cannot use the RT_EXAMPLES
# loop above, which assumes ${example}.cpp sits alongside this file.
add_executable(cr20_drill_test cr20_drill/drill_test.cpp)
target_include_directories(cr20_drill_test PUBLIC
    ${SHARED_INCLUDEDIR} ${CMAKE_CURRENT_SOURCE_DIR}/cr20_drill)
target_link_libraries(cr20_drill_test Rokae::Rokae eigen Threads::Threads)

# Off-robot verification for drill_frames.hpp and drill_cycle.hpp. Linked
# against `eigen` ALONE - no Rokae::Rokae, no Threads, no controller. That link
# line is the enforcement of "these two headers are provable on a laptop": if
# either ever acquires an SDK dependency, this target stops building, which is
# the signal that the split has been broken. Run it before any drilling
# session; see docs/drill-test-quickstart.md.
add_executable(cr20_drill_checks cr20_drill/drill_checks.cpp)
target_include_directories(cr20_drill_checks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cr20_drill)
target_link_libraries(cr20_drill_checks eigen)

foreach(example ${RT_EXAMPLES})
    add_executable(${example} ${example}.cpp ${EXAMPLE_HEADERS})
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR} ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        torque_control
        follow_joint_position
        follow_cart_position
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${EXAMPLE_HEADERS})
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${RT_EXAMPLES} ${EXAMPLES_USE_XMATEMODEL} cr20_drill_test cr20_drill_checks
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    OPTIONAL
    )
