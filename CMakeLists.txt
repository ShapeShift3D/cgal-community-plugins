if (NOT TARGET CGAL)
  vtk_module_find_package(PACKAGE CGAL VERSION 5.1.1 CONFIG_MODE PRIVATE)
endif ()

set(classes
  stkCGAL3DPolyhedralMesher
  stkCGALAbstractUVParametrization
  stkCGALARAPUVParametrization
  stkCGALLSCMUVParametrization
  stkCGALBoolean
  stkCGALBoolean2DMesher
  stkCGALBoolean3DMesher
  stkCGALIsotropicRemeshingFilter
  stkCGALEfficientRANSAC
  stkCGALPolygonOrientOperator
  stkCGALPolygonSetOrientedSideClassifier
  stkCGALPolygonSetToPolyLineSet
  stkCGALPolygonUtilities
  stkCGALPolyLineSetToPolygonSet
  stkCGALRegionGrowing
  stkCGALSelfIntersectionMeasurer
  stkCGALUtilities
)

set(headers
  stkPointCloudScalarSizingField.h
)

vtk_module_add_module(ParaView::stkCGAL
  CLASSES ${classes}
  PRIVATE_HEADERS ${headers}
)

set(xml_files
  Resources/cgal_filters.xml
  Resources/cgal_sources.xml
)

paraview_add_server_manager_xmls(
    XMLS  ${xml_files})

#message("CGAL_FOUND? : " ${CGAL_FOUND})
if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  
  # Activate concurrency ? (turned ON by default)
  option(CGAL_ACTIVATE_CONCURRENCY
         "Enable concurrency"
         ON)

  vtk_module_definitions(ParaView::stkCGAL PRIVATE NOMINMAX)

  # Add debug flag for the ARAP algorithm
  option(CGAL_DEBUG_SMP_ARAP "Enables debugging mode for CGAL's ARAP" OFF)
  if(CGAL_DEBUG_SMP_ARAP)
    vtk_module_definitions(ParaView::stkCGAL PRIVATE CGAL_SMP_ARAP_DEBUG CGAL_PARAMETERIZATION_ARAP_VERBOSE)
  endif(CGAL_DEBUG_SMP_ARAP)
  
  # Link with CGAL
  vtk_module_link(ParaView::stkCGAL PUBLIC CGAL::CGAL)
   
  if(CGAL_ACTIVATE_CONCURRENCY)
    find_package(TBB REQUIRED)
    if(TBB_FOUND)
      vtk_module_definitions(ParaView::stkCGAL PRIVATE CGAL_LINKED_WITH_TBB)
      vtk_module_link(ParaView::stkCGAL PRIVATE TBB::tbbmalloc TBB::tbb)
    endif(TBB_FOUND)
  endif(CGAL_ACTIVATE_CONCURRENCY)
endif(CGAL_FOUND)