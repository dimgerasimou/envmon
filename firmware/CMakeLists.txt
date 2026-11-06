cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(envmon)

# Editor support: write .clangd for this machine's toolchain (git-ignored).
# Ask GCC for its header search path with the same libc as the build.
set(GCC_QUERY_FLAGS "")
if(CONFIG_LIBC_PICOLIBC)
	list(APPEND GCC_QUERY_FLAGS -specs=picolibc.specs)
endif()
execute_process(COMMAND ${CMAKE_C_COMPILER} ${GCC_QUERY_FLAGS} -E -Wp,-v -x c /dev/null
		OUTPUT_QUIET ERROR_VARIABLE GCC_SEARCH)
string(REGEX MATCH "search starts here:\n(.*)\nEnd of search list" _ "${GCC_SEARCH}")
string(REPLACE "\n" ";" GCC_INCLUDES "${CMAKE_MATCH_1}")
set(CLANGD_INCLUDES "")
foreach(dir IN LISTS GCC_INCLUDES)
	string(STRIP "${dir}" dir)
	if(IS_DIRECTORY "${dir}")
		file(REAL_PATH "${dir}" dir)
		string(APPEND CLANGD_INCLUDES "    - -isystem${dir}\n")
	endif()
endforeach()
file(CONFIGURE OUTPUT "${CMAKE_SOURCE_DIR}/.clangd" @ONLY CONTENT [[
CompileFlags:
  CompilationDatabase: @CMAKE_BINARY_DIR@
  Remove: [-f*]
  Add:
    - --target=riscv32-esp-elf
    - -nostdinc
@CLANGD_INCLUDES@Diagnostics:
  UnusedIncludes: None
  Suppress: [atomic_op_needs_trivial_copy]
]])
