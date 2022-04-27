#[==[
"stkCGALSurfaceMeshTopologyTest" is the name of the executable.
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

vtk_add_test_cxx(stkCGALModuleTests tests
  NO_DATA # For all test files
  NO_VALID # For all test files
  NO_OUTPUT # For all test files

  stkTestCGALSurfaceMeshTopology.cxx
  # Alternative syntax to specify the options per-file:
  # TestUtils.cxx,NO_DATA,NO_VALID,NO_OUTPUT

  # Add more test files here:
  )

vtk_test_cxx_executable(stkCGALModuleTests tests)
