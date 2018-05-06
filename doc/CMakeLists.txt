# add a target to generate API documentation with Doxygen
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

# We assume that find_package(Doxygen) is already in place from the calling script
if(NOT DOXYGEN_FOUND)
  message(FATAL_ERROR "Doxygen is needed to build the documentation.")
endif()

if(BUILD_DOCUMENTATION)

  # Build a list of the directories that doxygen will operate on
  set(doc_subdirs
    conversions
    mrpt_graphslam_2d
    )
  set(DOXYGEN_INPUT_PATHS "${PROJECT_ROOT}")
  foreach(dir ${doc_subdirs})
    set(full_path ${PROJECT_ROOT}/${dir})
    set(DOXYGEN_INPUT_PATHS "${DOXYGEN_INPUT_PATHS} ${full_path}")

    # Add all the children of the current directory
    SUBDIRLIST(CHILDREN_DIRS ${full_path})
    foreach(CHILD_DIR ${CHILDREN_DIRS})
      set(DOXYGEN_INPUT_PATHS "${DOXYGEN_INPUT_PATHS} ${full_path}/${CHILD_DIR}")
    endforeach()
  endforeach()

  set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  configure_file(${doxyfile_in} ${doxyfile} @ONLY)

  add_custom_target(doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif()
