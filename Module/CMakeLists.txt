# Find dependencies
vtk_module_find_package(PACKAGE CGAL VERSION 5.1.1 CONFIG_MODE PRIVATE)

if(NOT TARGET Eigen3)
  vtk_module_find_package(PACKAGE Eigen3 VERSION 3.3 CONFIG_MODE PRIVATE)
endif()

# Define content
set(classes
  stkCGAL2DConstrainedDelaunayTriangulationMesher
  stkCGAL3DConvexHull
  stkCGAL3DPolyhedralMesher
  stkCGALAlphaWrapping
  stkCGALARAPUVParametrization
  stkCGALBoolean2DMesher
  stkCGALBoolean3DMesher
  stkCGALDuplicateNonManifoldVertices
  stkCGALEfficientRANSAC
  stkCGALFillHoles
  stkCGALIsotropicRemeshingFilter
  stkCGALLSCMUVParametrization
  stkCGALPoissonReconstructionMesher
  stkCGALPolygonMeshSmoothingOperator
  stkCGALPolygonOffset
  stkCGALPolygonOrientOperator
  stkCGALPolygonSetOrientedSideClassifier
  stkCGALPolygonSetToPolyLineSet
  stkCGALPolygonUtilities
  stkCGALPolyLineSetToPolygonSet
  stkCGALRegionFairingOperator
  stkCGALRegionGrowing
  stkCGALSelfIntersectionMeasurer
  stkCGALSurfaceMeshTopology
  stkCGALUtilities
  stkCGALUVParametrizationUtilities
)

set(private_headers
  stkPointCloudScalarSizingField.h
)

vtk_module_add_module(STK::CGALCopyleft
  CLASSES ${classes}
  PRIVATE_HEADERS ${private_headers}
)

if(Eigen3_FOUND)
  vtk_module_link(STK::CGALCopyleft PUBLIC Eigen3::Eigen)
endif(Eigen3_FOUND)

if(CGAL_FOUND)
  include(${CGAL_USE_FILE})

  # Activate concurrency ? (turned ON by default)
  option(CGAL_ACTIVATE_CONCURRENCY
    "Enable concurrency"
    ON)

  vtk_module_definitions(STK::CGALCopyleft PRIVATE NOMINMAX)

  # Add debug flag for the ARAP algorithm
  option(CGAL_DEBUG_SMP_ARAP "Enables debugging mode for CGAL's ARAP" OFF)

  if(CGAL_DEBUG_SMP_ARAP)
    vtk_module_definitions(STK::CGALCopyleft PRIVATE CGAL_KEEP_SEQUENTIAL CGAL_SMP_ARAP_DEBUG CGAL_PARAMETERIZATION_ARAP_VERBOSE)
  endif(CGAL_DEBUG_SMP_ARAP)

  # Link with CGAL
  vtk_module_link(STK::CGALCopyleft PUBLIC CGAL::CGAL)

  if(CGAL_ACTIVATE_CONCURRENCY)
    if (NOT TARGET TBB::tbb)
      find_package(TBB REQUIRED)
    endif ()

    if(TBB_FOUND)
      vtk_module_definitions(STK::CGALCopyleft PRIVATE CGAL_LINKED_WITH_TBB)
      vtk_module_link(STK::CGALCopyleft PRIVATE TBB::tbbmalloc TBB::tbb)
    endif(TBB_FOUND)
  endif(CGAL_ACTIVATE_CONCURRENCY)
endif(CGAL_FOUND)
