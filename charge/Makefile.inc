mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS +=\

SRCLANG := c++

APPL_LIBS += -lm

APPL_DIRS +=

INCLUDES += \
    -I$(mkfile_path)../../../drivers/include/libcpp/spike
