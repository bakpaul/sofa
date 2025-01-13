message("CPACK_GENERATOR= ${CPACK_GENERATOR}")

if(CPACK_GENERATOR STREQUAL IFW)
    execute_process(COMMAND bash "${CMAKE_CURRENT_LIST_DIR}/generate-stubfiles.sh" "${CMAKE_CURRENT_LIST_DIR}/../../" "${CPACK_TEMPORARY_INSTALL_DIRECTORY}/packages/Runtime/data/" ${CMAKE_SYSTEM_NAME})
elseif(CPACK_GENERATOR STREQUAL NSIS)
    execute_process(COMMAND bash "${CMAKE_CURRENT_LIST_DIR}/generate-stubfiles.sh" "${CMAKE_CURRENT_LIST_DIR}/../../" "${CPACK_TEMPORARY_INSTALL_DIRECTORY}" "${CMAKE_SYSTEM_NAME}:NSIS")
else()
    execute_process(COMMAND bash "${CMAKE_CURRENT_LIST_DIR}/generate-stubfiles.sh" "${CMAKE_CURRENT_LIST_DIR}/../../" "${CPACK_TEMPORARY_INSTALL_DIRECTORY}" ${CMAKE_SYSTEM_NAME})
endif()

#applications
#libraries/lib
#libraries/plugins