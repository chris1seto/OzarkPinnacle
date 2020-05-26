add_subdirectory(system)
set(LWIP_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/src/include")
set(LWIP_DIR "${CMAKE_CURRENT_LIST_DIR}/")
include(src/Filelists.cmake)
target_link_libraries(lwipcore LwIPSystem)
target_link_libraries(lwipcore Platform)
target_link_libraries(lwipcore FreeRTOS)

