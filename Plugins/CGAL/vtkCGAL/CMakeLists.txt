#find_package(CGAL REQUIRED)

if (NOT TARGET CGAL)
  vtk_module_find_package(PACKAGE CGAL PRIVATE)
endif ()

if (NOT TARGET Eigen3)
vtk_module_find_package(PACKAGE Eigen3 VERSION 3.3 CONFIG_MODE PRIVATE)
endif ()

# ParaView 6.0 or later
set(classes
  vtkCGAL3DPolyhedralMesher
  vtkCGALAbstractUVParametrization
  vtkCGALARAPUVParametrization
  vtkCGALLSCMUVParametrization
  vtkCGALBoolean
  vtkCGALBoolean2DMesher
  vtkCGALBoolean3DMesher
  vtkCGALPolygonOrientOperator
  vtkCGALPolygonRecursiveSerialAppender
  vtkCGALPolygonSetInteriorCellExtract
  vtkCGALPolyLineSetToPolygonSet
  vtkCGALSelfIntersectionMeasurer
  vtkCGALUtilities)

set(headers
  vtkPointCloudScalarSizingField.h)

vtk_module_add_module(CGAL::vtkCGAL
  CLASSES ${classes}
  PRIVATE_HEADERS ${headers})

paraview_add_server_manager_xmls(
  XMLS  vtkCGALPlugins.xml)

if (Eigen3_FOUND)
  vtk_module_link(CGAL::vtkCGAL PUBLIC Eigen3::Eigen)
endif(Eigen3_FOUND)

#message("CGAL_FOUND? : " ${CGAL_FOUND})
if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  
  # Activate concurrency ? (turned ON by default)
  option(CGAL_ACTIVATE_CONCURRENCY
         "Enable concurrency"
         ON)

  vtk_module_definitions(CGAL::vtkCGAL PRIVATE NOMINMAX)
  
  # Link with CGAL
  vtk_module_link(CGAL::vtkCGAL PUBLIC CGAL::CGAL)
   
  if(CGAL_ACTIVATE_CONCURRENCY)
    find_package(TBB REQUIRED)
    if(TBB_FOUND)
      vtk_module_definitions(CGAL::vtkCGAL PRIVATE CGAL_LINKED_WITH_TBB)
      vtk_module_link(CGAL::vtkCGAL PRIVATE TBB::tbbmalloc TBB::tbb)
    endif(TBB_FOUND)
  endif(CGAL_ACTIVATE_CONCURRENCY)
endif(CGAL_FOUND)
