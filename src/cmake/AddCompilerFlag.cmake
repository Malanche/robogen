include (CheckCCompilerFlag)
include (CheckCXXCompilerFlag)
macro(AddCompilerFlag _flag)
   string(REGEX REPLACE "[/:= ]" "_" _flag_esc "${_flag}")
   check_c_compiler_flag("${_flag}" check_c_compiler_flag_${_flag_esc})
   check_cxx_compiler_flag("${_flag}" check_cxx_compiler_flag_${_flag_esc})
   if(check_c_compiler_flag_${_flag_esc})
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${_flag}")
   endif(check_c_compiler_flag_${_flag_esc})
   if(check_cxx_compiler_flag_${_flag_esc})
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${_flag}")
   endif(check_cxx_compiler_flag_${_flag_esc})
   if(${ARGC} EQUAL 2)
      set(${ARGV1} "${check_cxx_compiler_flag_${_flag_esc}}")
   endif(${ARGC} EQUAL 2)
endmacro(AddCompilerFlag)
