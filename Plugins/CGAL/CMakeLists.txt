set(BUILD_SHARED_LIBS TRUE)

#Paraview's macro to add the plugin. It takes care of all the vtk
#and paraview parts of the process, like link and integration
#in the UI
if(ParaView_FOUND AND PARAVIEW_USE_FILE)
  ## ParaView 5.6
    add_subdirectory(stkCGAL)
else()
  ## ParaView 5.7.x  
  paraview_add_plugin(CGALPlugins
    VERSION "1.0"
    MODULE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/stkCGAL/vtk.module"
    MODULES CGAL::stkCGAL
    XML_DOCUMENTATION OFF 
  
)
endif()