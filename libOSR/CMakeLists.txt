cmake_minimum_required (VERSION 3.1)
project(OSR)

option(OSR_USE_DAVIDVIVE "[Under Construction] Use the DAVID/Vive scan loader")
option(OSR_ACCURATE_TIMING "Display all timings in ms")
option(OSR_DEBUG_VISUALIZATION "Generate debug visualizations")
option(OSR_REDUCED_TIMINGS "Measure time only for high-priority nse::util::TimedBlocks")
option(OSR_GEOMETRIC_LAPLACIAN "Use the more time-consuming but higher-quality geometric formulation of the Laplacian for displacement map calculation")
option(OSR_USE_MEMORY_MAPPED_FILE "Use the boost memory-mapped file for XYZ input. This requires the boost iostreams library." OFF)

cmake_policy(SET CMP0074 OLD)

set(LIBS)
set(OSR_ADDITIONAL_INCLUDE_DIR)
set(OSR_EXTRA_DEFS)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
	"${CMAKE_CURRENT_SOURCE_DIR}/../CMake/Modules/"
	"${CMAKE_CURRENT_SOURCE_DIR}/../ext/nsessentials/CMake/Modules/")
include(SetupBuildEnvironment)

if(CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-fPIC)
endif()

# Tries to find a package via find_package. If the package cannot be found,
# falls back to the local copy in the /ext directory. pathVariable refers to
# the variable that stores the package's representative path and is used to
# evaluate if the package has been found.
function(FindOrFallBack packageName pathVariable extPath)
  find_package(${packageName})
  if(NOT ${pathVariable})
    message(STATUS "Cannot find global ${packageName} library. Falling back to the local copy under /ext")
    set(${pathVariable} ${CMAKE_CURRENT_SOURCE_DIR}/../ext/${extPath} CACHE PATH " " FORCE)
    if(NOT IS_DIRECTORY ${${pathVariable}})
      set(${pathVariable} ${pathVariable}-NOTFOUND CACHE PATH " " FORCE)
      message(FATAL_ERROR "${packageName} cannot be found. Either provide "
        "the path via ${pathVariable} or check out the corresponding submodule.")
    endif()
  endif()
endfunction()

function(PublicIncludeDir directory)
  set(OSR_ADDITIONAL_INCLUDE_DIR ${OSR_ADDITIONAL_INCLUDE_DIR} ${directory} PARENT_SCOPE)
  include_directories(${directory})
endfunction()

# ------- Public dependencies (cascade to parent) -------

PublicIncludeDir(${CMAKE_CURRENT_LIST_DIR}/include)

# Add TBB dependency
FindOrFallBack(TBB TBB_ROOT tbb)
set(TBB_BUILD_STATIC OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_SHARED ON CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS OFF CACHE BOOL " " FORCE)
add_subdirectory(${TBB_ROOT} ${CMAKE_BINARY_DIR}/ext_build/tbb)
set_property(TARGET tbb tbb_def_files PROPERTY FOLDER "dependencies")
PublicIncludeDir(${TBB_ROOT}/include)
set(LIBS ${LIBS} tbb)

# Add Eigen dependency
FindOrFallBack(Eigen3 EIGEN3_INCLUDE_DIR eigen)
PublicIncludeDir(${EIGEN3_INCLUDE_DIR})
#Do not use Eigen parallelization as we will parallelize manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")

# Add NSEssentials dependency
FindOrFallBack(NSEssentials NSE_ROOT nsessentials)
SET(NSE_WITH_TBB ON CACHE BOOL " " FORCE)
SET(NSE_WITH_EIGEN ON CACHE BOOL " " FORCE)
SET(NSE_BUILD_SHARED ON CACHE BOOL " " FORCE)
add_subdirectory(${NSE_ROOT} ${CMAKE_BINARY_DIR}/ext_build/nse)
set_property(TARGET nsessentials PROPERTY FOLDER "dependencies")
set(LIBS ${LIBS} nsessentials)
PublicIncludeDir(${NSE_ROOT}/include)
set(OSR_EXTRA_DEFS ${OSR_EXTRA_DEFS} ${NSE_EXTRA_DEFS})
add_definitions(${NSE_EXTRA_DEFS})
list (FIND NSE_EXTRA_DEFS /DNSE_BUILD_SHARED _index)
if (${_index} GREATER -1)
	set(LIBS ${LIBS} nsessentials)
endif()

# Add Boost dependency
set(BOOST_COMPONENTS)
if(OSR_USE_MEMORY_MAPPED_FILE)
	set(BOOST_COMPONENTS ${BOOST_COMPONENTS} iostreams)
	add_definitions(/DOSR_USE_MEMORY_MAPPED_FILE)
endif()

SET(Boost_INCLUDE_DIR "../ext/boost_reduced/")
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(MSVC AND (MSVC_VERSION LESS 1600))
	set(OSR_EXTRA_DEFS ${OSR_EXTRA_DEFS} -DBOOST_STDINT)
	add_definitions(-DBOOST_STDINT)
endif()
PublicIncludeDir(${Boost_INCLUDE_DIRS})
if(NOT "${BOOST_COMPONENTS}" STREQUAL "")
	message("We have some boost components")
	link_directories(${Boost_LIBRARY_DIRS})
endif()

# ------- Private dependencies -------

# Add OpenMP dependency
find_package(OpenMP)
if(OPENMP_FOUND)
  add_definitions(-DOPENMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Add DSet dependency
FindOrFallBack(DSet DSet_ROOT dset)
include_directories(${DSet_ROOT})

# Add RPLY dependency
FindOrFallBack(RPLY RPLY_ROOT rply)
SET(THIRD_PARTY ${THIRD_PARTY} ${RPLY_ROOT}/rply.c)
include_directories(${RPLY_ROOT})

# Add nanoflann dependency
FindOrFallBack(Nanoflann NANOFLANN_ROOT nanoflann)
include_directories(${NANOFLANN_ROOT})

SET(THIRD_PARTY ${THIRD_PARTY} include/3rd/ICP.h)

# ------------------------------------

if(OSR_ACCURATE_TIMING)
	add_definitions (/DACCURATE_TIMING)
endif(OSR_ACCURATE_TIMING)

if(OSR_DEBUG_VISUALIZATION)
	add_definitions(/DDEBUG_VISUALIZATION)
endif(OSR_DEBUG_VISUALIZATION)

if(OSR_REDUCED_TIMINGS)
	add_definitions(/DREDUCED_TIMINGS)
endif(OSR_REDUCED_TIMINGS)

if(OSR_GEOMETRIC_LAPLACIAN)
	add_definitions (/DGEOMETRIC_LAPLACIAN)
endif(OSR_GEOMETRIC_LAPLACIAN)

source_group(third_party FILES ${THIRD_PARTY})

set(LIBS ${LIBS} ${OSR_ADDITIONAL_LIBS})

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
	SET(OSR_ADDITIONAL_INCLUDE_DIR ${OSR_ADDITIONAL_INCLUDE_DIR} PARENT_SCOPE)
	SET(OSR_ADDITIONAL_LIBS ${OSR_ADDITIONAL_LIBS} PARENT_SCOPE)
	SET(OSR_EXTRA_DEFS ${OSR_EXTRA_DEFS} PARENT_SCOPE)
endif()

add_library(OSR SHARED	
	src/Api.cpp
	include/osr/Attributes.h
	include/osr/AttributeConsistency.h
	src/common.cpp include/osr/common.h
	src/Data.cpp include/osr/Data.h
	include/osr/MeshSettings.h
	src/field.cpp include/osr/field.h
	include/osr/Indexing.h
	include/osr/HierarchyOptions.h
	include/osr/HierarchyDecl.h
	include/osr/HierarchyDef.h
	include/osr/Hierarchy.h
	include/osr/PreparedVertexSet.h
	src/HierarchyOptimizationHelper.cpp include/osr/HierarchyOptimizationHelper.h	
	#src/HierarchyUnstructured.cpp include/HierarchyUnstructured.h
	#src/HierarchyRegularSubdiv.cpp include/HierarchyRegularSubdiv.h
	#src/HierarchyOctree.cpp include/HierarchyOctree.h
	src/HierarchyMortonMultiPass.cpp include/osr/HierarchyMortonMultiPass.h
	include/osr/HierarchyCapabilities.h
	include/osr/INeighborQueryable.h
	src/meshio.cpp include/osr/meshio.h
	src/Optimizer.cpp include/osr/Optimizer.h
	src/Scan.cpp include/osr/Scan.h
	src/ExtractedMesh.cpp include/osr/ExtractedMesh.h
	src/MeshVisitor.cpp include/osr/MeshVisitor.h
	include/osr/ExtractionUtils.h
	src/ExtractionAttributeMapping.cpp include/osr/ExtractionAttributeMapping.h			
	src/Colors.cpp include/osr/Colors.h	
	${THIRD_PARTY})

target_link_libraries(OSR ${LIBS})