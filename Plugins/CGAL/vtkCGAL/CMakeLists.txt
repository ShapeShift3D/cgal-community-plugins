#find_package(CGAL REQUIRED)

vtk_module_find_package(PACKAGE CGAL PRIVATE)

if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  
  # Activate concurrency ? (turned ON by default)
  option(CGAL_ACTIVATE_CONCURRENCY
         "Enable concurrency"
         ON)
   
  if(CGAL_ACTIVATE_CONCURRENCY)
    find_package(TBB REQUIRED)
    if(TBB_FOUND)
      add_definitions( -DNOMINMAX -DCGAL_LINKED_WITH_TBB )
    endif(TBB_FOUND)
  endif(CGAL_ACTIVATE_CONCURRENCY)
endif(CGAL_FOUND)

vtk_module_find_package(PACKAGE Eigen3 VERSION 3.3 CONFIG_MODE PRIVATE)

# ParaView 6.0 or later
set(classes
  vtkCGAL3DPolyhedralMesher
  vtkCGALAbstractUVParametrization
  vtkCGALARAPUVParametrization
  vtkCGALLSCMUVParametrization
  vtkCGALBoolean
  vtkCGALBoolean3DMesher
  vtkCGALSelfIntersectionMeasurer
  vtkCGALUtilities)


set(headers
  vtkPointCloudScalarSizingField.h)


vtk_module_add_module(CGAL::vtkCGAL
  CLASSES ${classes}
  PRIVATE_HEADERS ${headers})


#vtk_module_definitions(CGAL::vtkCGAL PRIVATE CGAL_LINKED_WITH_TBB)


paraview_add_server_manager_xmls(
  XMLS  vtkCGALPlugins.xml)


# Link with CGAL
vtk_module_link(CGAL::vtkCGAL PRIVATE CGAL::CGAL)

vtk_module_link(CGAL::vtkCGAL PUBLIC Eigen3::Eigen)


if(TBB_FOUND)
  message("TBB_FOUND: " ${TBB_FOUND})
  vtk_module_link(CGAL::vtkCGAL PRIVATE TBB::tbbmalloc TBB::tbb)
endif(TBB_FOUND)