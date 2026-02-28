################################################################################
# Build libsimulator
################################################################################
add_library(simulator STATIC
    src/AABB.cpp
    src/AABB.hpp
    src/AgentRemovalSystem.hpp
    src/Clonable.hpp
    src/CollisionFreeSpeedModel.cpp
    src/CollisionFreeSpeedModel.hpp
    src/CollisionFreeSpeedModelBuilder.cpp
    src/CollisionFreeSpeedModelBuilder.hpp
    src/CollisionFreeSpeedModelData.hpp
    src/CollisionFreeSpeedModelUpdate.hpp
     src/CollisionFreeSpeedModelV2.cpp
    src/CollisionFreeSpeedModelV2.hpp
    src/CollisionFreeSpeedModelV2Builder.cpp
    src/CollisionFreeSpeedModelV2Builder.hpp
    src/CollisionFreeSpeedModelV2Data.hpp
    src/CollisionFreeSpeedModelV2Update.hpp
    src/AnticipationVelocityModel.cpp
    src/AnticipationVelocityModel.hpp
    src/AnticipationVelocityModelBuilder.cpp
    src/AnticipationVelocityModelBuilder.hpp
    src/AnticipationVelocityModelData.hpp
    src/AnticipationVelocityModelUpdate.hpp  
    src/CollisionGeometry.cpp
    src/CollisionGeometry.hpp
    src/Ellipse.cpp
    src/Ellipse.hpp
    src/Enum.hpp
    src/GeneralizedCentrifugalForceModel.cpp
    src/GeneralizedCentrifugalForceModel.hpp
    src/GeneralizedCentrifugalForceModelBuilder.cpp
    src/GeneralizedCentrifugalForceModelBuilder.hpp
    src/GeneralizedCentrifugalForceModelData.hpp
    src/GeneralizedCentrifugalForceModelUpdate.hpp
    src/GenericAgent.hpp
    src/GeometricFunctions.hpp
    src/GeometryBuilder.cpp
    src/GeometryBuilder.hpp
    src/GeometrySwitchError.hpp
    src/Graph.hpp
    src/Journey.cpp
    src/Journey.hpp
    src/LineSegment.cpp
    src/LineSegment.hpp
    src/Logger.cpp
    src/Logger.hpp
    src/Macros.hpp
    src/Mathematics.cpp
    src/Mathematics.hpp
    src/Mesh.cpp
    src/Mesh.hpp
    src/NeighborhoodSearch.hpp
    src/OperationalDecisionSystem.hpp
    src/OperationalModel.hpp
    src/OperationalModelUpdate.hpp
    src/Point.cpp
    src/Point.hpp
    src/Polygon.cpp
    src/Polygon.hpp
    src/RoutingEngine.cpp
    src/RoutingEngine.hpp
    src/Simulation.cpp
    src/Simulation.hpp
    src/SimulationClock.cpp
    src/SimulationClock.hpp
    src/SimulationError.hpp
    src/SocialForceModel.cpp
    src/SocialForceModel.hpp
    src/SocialForceModelBuilder.cpp
    src/SocialForceModelBuilder.hpp
    src/SocialForceModelData.hpp
    src/SocialForceModelUpdate.hpp
    src/Stage.cpp
    src/Stage.hpp
    src/StageDescription.hpp
    src/StageManager.cpp
    src/StageManager.hpp
    src/StageSystem.cpp
    src/StageSystem.hpp
    src/StrategicalDesicionSystem.hpp
    src/TacticalDecisionSystem.hpp
    src/TemplateHelper.hpp
    src/Tracing.cpp
    src/Tracing.hpp
    src/UniqueID.hpp
    src/Util.hpp
)
target_compile_options(simulator PRIVATE
    ${COMMON_COMPILE_OPTIONS}
)
target_compile_definitions(simulator PUBLIC
    JPSCORE_VERSION="${PROJECT_VERSION}"
)
target_link_libraries(simulator PUBLIC
    common
    fmt::fmt
    CGAL::CGAL
    build_info
    glm::glm
)
target_link_options(simulator PUBLIC
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>,$<BOOL:${BUILD_WITH_ASAN}>>:-fsanitize=address>
)
target_include_directories(simulator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_property(TARGET simulator PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
set_property(TARGET simulator PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

################################################################################
# jupedsim native XML CLI
################################################################################
if(BUILD_XML_CLI)
    set(LIBDEFLATE_ROOT "" CACHE PATH
        "Optional root directory of libdeflate (source, build, or install prefix)")
    set(_libdeflate_hints "")
    if(LIBDEFLATE_ROOT)
        list(APPEND _libdeflate_hints
            "${LIBDEFLATE_ROOT}"
            "${LIBDEFLATE_ROOT}/build"
            "${LIBDEFLATE_ROOT}/lib"
        )
    endif()

    if(_libdeflate_hints)
        find_package(libdeflate CONFIG QUIET PATHS ${_libdeflate_hints} NO_DEFAULT_PATH)
    else()
        find_package(libdeflate CONFIG QUIET)
    endif()

    set(LIBDEFLATE_LINK_TARGET "")
    if(TARGET libdeflate::libdeflate_static)
        set(LIBDEFLATE_LINK_TARGET libdeflate::libdeflate_static)
    elseif(TARGET libdeflate::libdeflate_shared)
        set(LIBDEFLATE_LINK_TARGET libdeflate::libdeflate_shared)
    endif()

    if(NOT LIBDEFLATE_LINK_TARGET)
        if(_libdeflate_hints)
            find_path(LIBDEFLATE_INCLUDE_DIR
                NAMES libdeflate.h
                PATHS ${_libdeflate_hints}
                PATH_SUFFIXES include .
                NO_DEFAULT_PATH
            )
            find_library(LIBDEFLATE_LIBRARY
                NAMES deflate libdeflate
                PATHS ${_libdeflate_hints}
                PATH_SUFFIXES lib lib64 build .
                NO_DEFAULT_PATH
            )
        else()
            find_path(LIBDEFLATE_INCLUDE_DIR
                NAMES libdeflate.h
            )
            find_library(LIBDEFLATE_LIBRARY
                NAMES deflate libdeflate
            )
        endif()

        if(NOT LIBDEFLATE_INCLUDE_DIR OR NOT LIBDEFLATE_LIBRARY)
            message(FATAL_ERROR
                "BUILD_XML_CLI=ON requires libdeflate. "
                "Set LIBDEFLATE_ROOT to your libdeflate folder (source/build/install), "
                "or install system dev package (e.g. libdeflate-dev)."
            )
        endif()
    endif()

    add_executable(jupedsim-cli
        cli/jupedsim_cli.cpp
    )

    if(LIBDEFLATE_INCLUDE_DIR)
        target_include_directories(jupedsim-cli PRIVATE
            ${LIBDEFLATE_INCLUDE_DIR}
        )
    endif()

    target_link_libraries(jupedsim-cli PRIVATE
        simulator
        Boost::boost
        fmt::fmt
        $<$<BOOL:${LIBDEFLATE_LINK_TARGET}>:${LIBDEFLATE_LINK_TARGET}>
        $<$<BOOL:${LIBDEFLATE_LIBRARY}>:${LIBDEFLATE_LIBRARY}>
    )

    target_compile_options(jupedsim-cli PRIVATE
        ${COMMON_COMPILE_OPTIONS}
    )

    set_property(TARGET jupedsim-cli PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
    set_property(TARGET jupedsim-cli PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
endif()

################################################################################
# libsimulator unit tests
################################################################################
if (BUILD_TESTS)
    add_executable(libsimulator-tests
        test/TestAABB.cpp
        test/TestBasicPrimitiveTests.cpp
        test/TestCollisionGeometry.cpp
        test/TestGraph.cpp
        test/TestJourney.cpp
        test/TestLineSegment.cpp
        test/TestMesh.cpp
        test/TestNeighborhoodSearch.cpp
        test/TestPoint.cpp
        test/TestSimulationClock.cpp
        test/TestStage.cpp
        test/TestUniqueID.cpp
    )

    target_link_libraries(libsimulator-tests PRIVATE
        GTest::gtest
        GTest::gmock
        GTest::gtest_main
        simulator
    )

    target_compile_options(libsimulator-tests PRIVATE
        ${COMMON_COMPILE_OPTIONS}
    )

    set_property(TARGET libsimulator-tests PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
    set_property(TARGET libsimulator-tests PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
endif()

################################################################################
# libjupedsim micro benchmarks
################################################################################
if (BUILD_BENCHMARKS)
    add_executable(libsimulator-benchmarks
        benchmark/BenchmarkMain.cpp
        benchmark/benchmarkLineSegment.hpp
        benchmark/benchmarkCollisionGeometry.hpp
        benchmark/buildGeometries.hpp
    )

    target_link_libraries(libsimulator-benchmarks PRIVATE
        benchmark::benchmark
        simulator
    )

    target_compile_options(libsimulator-benchmarks PRIVATE
        ${COMMON_COMPILE_OPTIONS}
    )

    set_property(TARGET libsimulator-benchmarks PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
    set_property(TARGET libsimulator-benchmarks PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
endif ()
