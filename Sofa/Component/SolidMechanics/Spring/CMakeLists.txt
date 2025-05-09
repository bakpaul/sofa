cmake_minimum_required(VERSION 3.22)
project(Sofa.Component.SolidMechanics.Spring LANGUAGES CXX)

set(SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR "src/sofa/component/solidmechanics/spring")

set(HEADER_FILES
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/config.h.in
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/init.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/AngularSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/AngularSpringForceField.inl
    # ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/BoxStiffSpringForceField.h
    # ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/BoxStiffSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FastTriangularBendingSprings.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FastTriangularBendingSprings.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FrameSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FrameSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/GearSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/GearSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpring.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpring.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/LinearSpring.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/MeshSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/MeshSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialRestShapeSpringsForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialRestShapeSpringsForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialSpringsForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialSpringsForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadBendingSprings.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadBendingSprings.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadularBendingSprings.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadularBendingSprings.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RegularGridSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RegularGridSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RepulsiveSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RepulsiveSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RestShapeSpringsForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RestShapeSpringsForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/SpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/SpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/StiffSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/StiffSpringForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangleBendingSprings.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangleBendingSprings.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBendingSprings.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBendingSprings.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBiquadraticSpringsForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBiquadraticSpringsForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularQuadraticSpringsForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularQuadraticSpringsForceField.inl
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/VectorSpringForceField.h
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/VectorSpringForceField.inl
)

set(SOURCE_FILES
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/init.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/AngularSpringForceField.cpp
    # ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/BoxStiffSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FastTriangularBendingSprings.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/FrameSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/GearSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpring.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/JointSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/LinearSpring.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/MeshSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialRestShapeSpringsForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/PolynomialSpringsForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadBendingSprings.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/QuadularBendingSprings.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RegularGridSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RepulsiveSpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/RestShapeSpringsForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/SpringForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangleBendingSprings.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBendingSprings.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularBiquadraticSpringsForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/TriangularQuadraticSpringsForceField.cpp
    ${SOFACOMPONENTSOLIDMECHANICSSPRING_SOURCE_DIR}/VectorSpringForceField.cpp
)

sofa_find_package(Sofa.Simulation.Core REQUIRED)
sofa_find_package(Sofa.Component.Topology.Container.Grid REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Simulation.Core)
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Component.Topology.Container.Grid)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
)

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
cmake_dependent_option(SOFA_COMPONENT_SOLIDMECHANICS_SPRING_BUILD_TESTS "Compile the automatic tests" ON "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
if(SOFA_COMPONENT_SOLIDMECHANICS_SPRING_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
