find_package(CGAL REQUIRED)
if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
endif(CGAL_FOUND)

#Paraview's macro to add the plugin. It takes care of all the vtk
#and paraview parts of the process, like link and integration
#in the UI
if(ParaView_FOUND AND PARAVIEW_USE_FILE)
  ## ParaView 5.6
  add_paraview_plugin(CGALPlugins "1.0"
    SERVER_MANAGER_XML vtkCGALPlugins.xml
    SERVER_MANAGER_SOURCES vtkCGAL3DPolyhedralMesher.cxx
                           vtkCGALBoolean.cxx)
  
  target_link_libraries(CGALPlugins PRIVATE CGAL::CGAL)

else()
  ## ParaView 6.0 or later
  set(classes
    vtkCGAL3DPolyhedralMesher
    vtkCGALBoolean)

  vtk_module_add_module(CGAL::vtkCGAL
    CLASSES ${classes})

  paraview_add_server_manager_xmls(
    XMLS  vtkCGALPlugins.xml)

  # Link with CGAL
  vtk_module_link(CGAL::vtkCGAL PRIVATE CGAL::CGAL)

endif()