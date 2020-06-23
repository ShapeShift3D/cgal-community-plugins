find_package(CGAL REQUIRED)
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

#Paraview's macro to add the plugin. It takes care of all the vtk
#and paraview parts of the process, like link and integration
#in the UI
if(ParaView_FOUND AND PARAVIEW_USE_FILE)
  ## ParaView 5.6
  add_paraview_plugin(CGALPlugins "1.0"
    SERVER_MANAGER_XML vtkCGALPlugins.xml
    SERVER_MANAGER_SOURCES vtkCGAL3DPolyhedralMesher.cxx
                           vtkCGAL2DBoolean.cxx
						   vtkCGALBoolean.cxx
                           vtkCGALSelfIntersectionMeasurer.cxx
                           vtkCGALUtilities.cxx
						   )
  
  target_link_libraries(CGALPlugins PRIVATE CGAL::CGAL)

  if(TBB_FOUND)
    message("TBB_FOUND: " ${TBB_FOUND})
    target_link_libraries(CGALPlugins PRIVATE TBB::tbbmalloc TBB::tbb)
  endif(TBB_FOUND)
  
else()
  ## ParaView 6.0 or later
  set(classes
    vtkCGAL3DPolyhedralMesher
    vtkCGAL2DBoolean
    vtkCGALBoolean
	vtkCGALSelfIntersectionMeasurer
    vtkCGALUtilities
	)
    
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

  if(TBB_FOUND)
    message("TBB_FOUND: " ${TBB_FOUND})
    vtk_module_link(CGAL::vtkCGAL PRIVATE TBB::tbbmalloc TBB::tbb)
  endif(TBB_FOUND)
  
endif()