cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(CGAL-Community-Plugins)

if (NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
endif()

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# In the case where we build plugins externally, these need to be set like this.
SET(ENABLE_NONFREE_MODULES OFF)
SET(ENABLE_NONFREE_PLUGINS ON)

if(PARAVIEW_USE_FILE)
  # ParaView 5.6
  # ------------
  #If the plugin is used internally we don't need to include.
  if (NOT ParaView_BINARY_DIR)
      include(${PARAVIEW_USE_FILE})
  endif(NOT ParaView_BINARY_DIR)
  add_subdirectory(Plugins/CGAL)
else()
  # ParaView 6 or later
  # --------------------
  #  `paraview_plugin_build` uses `add_subdirectory` so it has to be in a sub-directory
  paraview_plugin_scan(
    PLUGIN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/Plugins/CGAL/paraview.plugin"
    ENABLE_BY_DEFAULT ON
    HIDE_PLUGINS_FROM_CACHE ON
    PROVIDES_PLUGINS  plugins
    REQUIRES_MODULES  required_modules
    )
  
  foreach (module IN LISTS required_modules)
    if (NOT TARGET "${module}")
      message("Missing required module: ${module}")
      return ()
    endif ()
  endforeach ()

  #Message("CMAKE_INSTALL_BINDIR" "${CMAKE_INSTALL_BINDIR}")
  #Message("CMAKE_INSTALL_LIBDIR" "${CMAKE_INSTALL_LIBDIR}")
  #Message("PARAVIEW_PLUGIN_SUBDIR" "${PARAVIEW_PLUGIN_SUBDIR}")
  
  paraview_plugin_build(
    RUNTIME_DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY_DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY_SUBDIRECTORY "${PARAVIEW_PLUGIN_SUBDIR}"
    PLUGINS ${plugins})
endif()
