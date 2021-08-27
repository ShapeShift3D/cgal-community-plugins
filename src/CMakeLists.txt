#find_package(CGAL REQUIRED)

# Find dependencies
if (NOT TARGET CGAL)
  vtk_module_find_package(PACKAGE CGAL VERSION 5.1.1 CONFIG_MODE PRIVATE)
endif ()

if (NOT TARGET Eigen3)
vtk_module_find_package(PACKAGE Eigen3 VERSION 3.3 CONFIG_MODE PRIVATE)
endif ()

# Define content
set(classes
  stkCGAL3DPolyhedralMesher
  stkCGALARAPUVParametrization
  stkCGALLSCMUVParametrization
  stkCGALUVParametrizationUtilities
  stkCGALBoolean2DMesher
  stkCGALBoolean3DMesher
  stkCGALEfficientRANSAC
  stkCGALIsotropicRemeshingFilter
  stkCGALPolygonMeshSmoothingOperator
  stkCGALPolygonOrientOperator
  stkCGALPolygonSetOrientedSideClassifier
  stkCGALPolygonSetToPolyLineSet
  stkCGALPolygonUtilities
  stkCGALPolyLineSetToPolygonSet
  stkCGALRegionGrowing
  stkCGALSelfIntersectionMeasurer
  stkCGALUtilities
)

set(private_headers
  stkPointCloudScalarSizingField.h
)

# Namespace adaptive to how the module is built (as a pure module or as plugins)
SET(NAMESPACE "")

if(ENABLE_NONFREE_MODULES)
  SET(NAMESPACE "ParaView")
endif(ENABLE_NONFREE_MODULES)
  
if(ENABLE_NONFREE_PLUGINS)
  SET(NAMESPACE "CGAL")
endif(ENABLE_NONFREE_PLUGINS)
  

vtk_module_add_module(${NAMESPACE}::stkCGAL
  CLASSES ${classes}
  PRIVATE_HEADERS ${private_headers}
)

set(xml_files
  Resources/cgal_filters.xml
  Resources/cgal_proxies.xml
)

paraview_add_server_manager_xmls(
  XMLS  ${xml_files}
)

if (Eigen3_FOUND)
  vtk_module_link(${NAMESPACE}::stkCGAL PUBLIC Eigen3::Eigen)
endif(Eigen3_FOUND)

if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  
  # Activate concurrency ? (turned ON by default)
  option(CGAL_ACTIVATE_CONCURRENCY
        "Enable concurrency"
        ON)

  vtk_module_definitions(${NAMESPACE}::stkCGAL PRIVATE NOMINMAX)

  # Add debug flag for the ARAP algorithm
  option(CGAL_DEBUG_SMP_ARAP "Enables debugging mode for CGAL's ARAP" OFF)
  if(CGAL_DEBUG_SMP_ARAP)
    vtk_module_definitions(${NAMESPACE}::stkCGAL PRIVATE CGAL_SMP_ARAP_DEBUG CGAL_PARAMETERIZATION_ARAP_VERBOSE)
  endif(CGAL_DEBUG_SMP_ARAP)
  
  # Link with CGAL
  vtk_module_link(${NAMESPACE}::stkCGAL PUBLIC CGAL::CGAL)
  
  if(CGAL_ACTIVATE_CONCURRENCY)
    find_package(TBB REQUIRED)
    if(TBB_FOUND)
      vtk_module_definitions(${NAMESPACE}::stkCGAL PRIVATE CGAL_LINKED_WITH_TBB)
      vtk_module_link(${NAMESPACE}::stkCGAL PRIVATE TBB::tbbmalloc TBB::tbb)
    endif(TBB_FOUND)
  endif(CGAL_ACTIVATE_CONCURRENCY)
endif(CGAL_FOUND)



