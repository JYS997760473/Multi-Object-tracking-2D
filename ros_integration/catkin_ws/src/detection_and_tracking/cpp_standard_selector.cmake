if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
  set(CMAKE_CXX_STANDARD 14)

  if(DEFINED PCL_VERSION_MINOR)
    if(${PCL_VERSION_MINOR} LESS 10)
      set(CMAKE_CXX_STANDARD 11)
    endif()
  endif()
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(CLANG_TIDY "Lint with Clang-Tidy" OFF)

if(CLANG_TIDY)
  find_program(CLANG_TIDY_PATH NAMES clang-tidy clang-tidy-8 clang-tidy-15)

  if(NOT CLANG_TIDY_PATH)
    message(WARNING "clang-tidy not found!")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_PATH}")
    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_PATH})
  endif()
endif()
