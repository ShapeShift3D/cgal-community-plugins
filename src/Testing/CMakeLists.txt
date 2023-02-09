#[==[
"stkCGALModuleTests" is the name of the executable.
"tests" is the name of the variable used to store the list of tests.
"TestUtils.css" is the name of one of the cpp files containing tests
Options:

  - `NO_DATA`: The test does not need to know the test input data directory. If
    it does, it is passed on the command line via the `-D` flag.
  - `NO_VALID`: The test does not have a valid baseline image. If it does, the
    baseline is assumed to be in `../Data/Baseline/<NAME>.png` relative to the
    current source directory. If alternate baseline images are required,
    `<NAME>` may be suffixed by `_1`, `_2`, etc. The valid image is passed via
    the `-V` flag.
  - `NO_OUTPUT`: The test does not need to write out any data to the
    filesystem. If it does, a directory which may be written to is passed via
    the `-T` flag.

#]==]

vtk_module_test_data(
  ${CMAKE_CURRENT_SOURCE_DIR}/Data/self-intersectionsData.vtp
)

vtk_add_test_cxx(stkCGALModuleTests tests
  TestCGALInit.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGAL2DConstrainedDelaunayTriangulationMesher.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALSurfaceMeshTopology.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALEfficientRANSAC.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALPoissonReconstructionMesher.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALRegionGrowing.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALFillHoles.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALPolygonOffset.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGAL3DConvexHull.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALRegionFairingOperator.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGAL3DPolyhedralMesher.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALUVParametrization.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALBoolean2DMesher.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALBoolean3DMesher.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALIsotropicRemeshing.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALPolygonOrientOperator.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALPolygonSetOrientedSideClassifier.cxx,NO_DATA,NO_VALID,NO_OUTPUT
  TestCGALSelfIntersectionMeasurer.cxx,NO_VALID,NO_OUTPUT
  TestCGALAlphaWrapping.cxx,NO_VALID,NO_OUTPUT
)

vtk_test_cxx_executable(stkCGALModuleTests tests)
