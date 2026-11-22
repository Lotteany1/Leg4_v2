###########################################################################
## Makefile generated for component 'tinympc14_lpv_kernel'. 
## 
## Makefile     : tinympc14_lpv_kernel_rtw.mk
## Generated on : Sun Nov 22 05:15:03 2026
## Final product: .\tinympc14_lpv_kernel.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = tinympc14_lpv_kernel
MAKEFILE                  = tinympc14_lpv_kernel_rtw.mk
MATLAB_ROOT               = E:\DevelopLotteany1\Matlab\app
MATLAB_BIN                = E:\DevelopLotteany1\Matlab\app\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)\win64
START_DIR                 = E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Matlab\mixmpc_pre\build\lpv_codegen\tp850abddb_f362_42a2_a0de_4ea3aaded47b
TGT_FCN_LIB               = ISO_C
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
COMPILER_COMMAND_FILE     = tinympc14_lpv_kernel_rtw_comp.rsp
CMD_FILE                  = tinympc14_lpv_kernel_rtw.rsp
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
NODEBUG                   = 1
MODELLIB                  = tinympc14_lpv_kernel.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Microsoft Visual C++ 2022 v17.0 | nmake (64-bit Windows)
# Supported Version(s):    17.0
# ToolchainInfo Version:   2024a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# NODEBUG
# cvarsdll
# cvarsmt
# conlibsmt
# ldebug
# conflags
# cflags

#-----------
# MACROS
#-----------

MW_EXTERNLIB_DIR    = $(MATLAB_ROOT)\extern\lib\win64\microsoft
MW_LIB_DIR          = $(MATLAB_ROOT)\lib\win64
CPU                 = AMD64
APPVER              = 5.02
CVARSFLAG           = $(cvarsmt)
CFLAGS_ADDITIONAL   = -D_CRT_SECURE_NO_WARNINGS
CPPFLAGS_ADDITIONAL = -EHs -D_CRT_SECURE_NO_WARNINGS /wd4251 /Zc:__cplusplus
LIBS_TOOLCHAIN      = $(conlibs)

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Microsoft Visual C Compiler
CC = cl

# Linker: Microsoft Visual C Linker
LD = link

# C++ Compiler: Microsoft Visual C++ Compiler
CPP = cl

# C++ Linker: Microsoft Visual C++ Linker
CPP_LD = link

# Archiver: Microsoft Visual C/C++ Archiver
AR = lib

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)\mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: NMAKE Utility
MAKE = nmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -Zi
C_OUTPUT_FLAG       = -Fo
LDDEBUG             = /DEBUG
OUTPUT_FLAG         = -out:
CPPDEBUG            = -Zi
CPP_OUTPUT_FLAG     = -Fo
CPPLDDEBUG          = /DEBUG
OUTPUT_FLAG         = -out:
ARDEBUG             =
STATICLIB_OUTPUT_FLAG = -out:
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @ren
RUN                 = @cmd /C

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = /nologo
CFLAGS               = $(cflags) $(CVARSFLAG) $(CFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPPFLAGS             = /TP $(cflags) $(CVARSFLAG) $(CPPFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPP_LDFLAGS          = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
CPP_SHAREDLIB_LDFLAGS  = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                         -dll -def:$(DEF_FILE)
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                       -dll -def:$(DEF_FILE)



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .\tinympc14_lpv_kernel.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=tinympc14_lpv_kernel

DEFINES = $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)\tinympc14_lpv_kernel_data.c $(START_DIR)\rt_nonfinite.c $(START_DIR)\rtGetNaN.c $(START_DIR)\rtGetInf.c $(START_DIR)\tinympc14_lpv_kernel_initialize.c $(START_DIR)\tinympc14_lpv_kernel_terminate.c $(START_DIR)\tinympc14_lpv_kernel.c $(START_DIR)\tinympc14_scheduled_zoh.c $(START_DIR)\tinympc14_scheduled_ab.c $(START_DIR)\tinympc14_fused_op.c $(START_DIR)\mldivide.c $(START_DIR)\tinympc14_fused_dyn.c $(START_DIR)\tinympc14_terminal_cubic_eval.c $(START_DIR)\tinympc14_zoh_embedded.c $(START_DIR)\tinympc14_bal_block.c $(START_DIR)\xgetrf.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = tinympc14_lpv_kernel_data.obj rt_nonfinite.obj rtGetNaN.obj rtGetInf.obj tinympc14_lpv_kernel_initialize.obj tinympc14_lpv_kernel_terminate.obj tinympc14_lpv_kernel.obj tinympc14_scheduled_zoh.obj tinympc14_scheduled_ab.obj tinympc14_fused_op.obj mldivide.obj tinympc14_fused_dyn.obj tinympc14_terminal_cubic_eval.obj tinympc14_zoh_embedded.obj tinympc14_bal_block.obj xgetrf.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = /source-charset:utf-8
CFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CFLAGS = $(CFLAGS) $(CFLAGS_) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = /source-charset:utf-8
CPPFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS = $(CPPFLAGS) $(CPPFLAGS_) $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


!include $(MATLAB_ROOT)\rtw\c\tools\vcdefs.mak


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute set_environment_variables


all : build
	@cmd /C "@echo ### Successfully generated all binary outputs."


build : set_environment_variables prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


set_environment_variables : 
	@set INCLUDE=$(INCLUDES);$(INCLUDE)
	@set LIB=$(LIB)


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@cmd /C "@echo ### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS) -out:$(PRODUCT) @$(CMD_FILE)
	@cmd /C "@echo ### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cc.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


.cxx.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cc.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cxx.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cc.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cxx.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Matlab\mixmpc_pre\deploy}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Matlab\mixmpc_pre\deploy}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Matlab\mixmpc_pre\deploy}.cc.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Matlab\mixmpc_pre\deploy}.cxx.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


tinympc14_lpv_kernel_data.obj : "$(START_DIR)\tinympc14_lpv_kernel_data.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_lpv_kernel_data.c"


rt_nonfinite.obj : "$(START_DIR)\rt_nonfinite.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\rt_nonfinite.c"


rtGetNaN.obj : "$(START_DIR)\rtGetNaN.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\rtGetNaN.c"


rtGetInf.obj : "$(START_DIR)\rtGetInf.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\rtGetInf.c"


tinympc14_lpv_kernel_initialize.obj : "$(START_DIR)\tinympc14_lpv_kernel_initialize.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_lpv_kernel_initialize.c"


tinympc14_lpv_kernel_terminate.obj : "$(START_DIR)\tinympc14_lpv_kernel_terminate.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_lpv_kernel_terminate.c"


tinympc14_lpv_kernel.obj : "$(START_DIR)\tinympc14_lpv_kernel.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_lpv_kernel.c"


tinympc14_scheduled_zoh.obj : "$(START_DIR)\tinympc14_scheduled_zoh.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_scheduled_zoh.c"


tinympc14_scheduled_ab.obj : "$(START_DIR)\tinympc14_scheduled_ab.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_scheduled_ab.c"


tinympc14_fused_op.obj : "$(START_DIR)\tinympc14_fused_op.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_fused_op.c"


mldivide.obj : "$(START_DIR)\mldivide.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\mldivide.c"


tinympc14_fused_dyn.obj : "$(START_DIR)\tinympc14_fused_dyn.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_fused_dyn.c"


tinympc14_terminal_cubic_eval.obj : "$(START_DIR)\tinympc14_terminal_cubic_eval.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_terminal_cubic_eval.c"


tinympc14_zoh_embedded.obj : "$(START_DIR)\tinympc14_zoh_embedded.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_zoh_embedded.c"


tinympc14_bal_block.obj : "$(START_DIR)\tinympc14_bal_block.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\tinympc14_bal_block.c"


xgetrf.obj : "$(START_DIR)\xgetrf.c"
	$(CC) $(CFLAGS) -Fo"$@" "$(START_DIR)\xgetrf.c"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@cmd /C "@echo ### PRODUCT = $(PRODUCT)"
	@cmd /C "@echo ### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@cmd /C "@echo ### BUILD_TYPE = $(BUILD_TYPE)"
	@cmd /C "@echo ### INCLUDES = $(INCLUDES)"
	@cmd /C "@echo ### DEFINES = $(DEFINES)"
	@cmd /C "@echo ### ALL_SRCS = $(ALL_SRCS)"
	@cmd /C "@echo ### ALL_OBJS = $(ALL_OBJS)"
	@cmd /C "@echo ### LIBS = $(LIBS)"
	@cmd /C "@echo ### MODELREF_LIBS = $(MODELREF_LIBS)"
	@cmd /C "@echo ### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@cmd /C "@echo ### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@cmd /C "@echo ### CFLAGS = $(CFLAGS)"
	@cmd /C "@echo ### LDFLAGS = $(LDFLAGS)"
	@cmd /C "@echo ### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### CPPFLAGS = $(CPPFLAGS)"
	@cmd /C "@echo ### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@cmd /C "@echo ### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### ARFLAGS = $(ARFLAGS)"
	@cmd /C "@echo ### MEX_CFLAGS = $(MEX_CFLAGS)"
	@cmd /C "@echo ### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@cmd /C "@echo ### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@cmd /C "@echo ### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@cmd /C "@echo ### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@cmd /C "@echo ### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@cmd /C "@echo ### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	@if exist $(PRODUCT) $(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


