if (NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
endif()

#Find CGAL
find_package(CGAL)
if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
endif(CGAL_FOUND)

if(ParaView_FOUND)
  #If the plugin is used internally we don't need to include.
  if (NOT ParaView_BINARY_DIR)
      include(${PARAVIEW_USE_FILE})
  endif(NOT ParaView_BINARY_DIR)
else(ParaView_FOUND)
  if(VTK_INCLUDE_DIRS)
    INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})
  endif(VTK_INCLUDE_DIRS)
endif(ParaView_FOUND)

#Find CGAL
find_package(CGAL)
if(CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
endif(CGAL_FOUND)

if(ParaView_FOUND OR VTK_INCLUDE_DIRS)
  message(NOTICE "inside")
# create a paraview plugin containing server manager xml and the server
# manager classes to build
# this plugin can be loaded on the server side

  #Paraview's macro to add the plugin. It takes care of all the vtk 
  #and paraview parts of the process, like link and integration
  #in the UI
  ADD_PARAVIEW_PLUGIN(CGAL3DPolyhedralMesher "1.0"
    SERVER_MANAGER_XML vtkCGAL3DPolyhedralMesher.xml
    SERVER_MANAGER_SOURCES vtkCGAL3DPolyhedralMesher.cxx)
endif(ParaView_FOUND OR VTK_INCLUDE_DIRS)

if(CGAL_FOUND)
    target_link_libraries(CGAL3DPolyhedralMesher PRIVATE
      CGAL::CGAL 
      ${Boost_LIBRARIES})
endif (CGAL_FOUND)