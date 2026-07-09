mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS +=\
    debug_log.o \
    Tracer.o \
    DriveBase.o \
    Robot.o \
    CourseConfig.o \
    Calibrator.o \
    ColorJudge.o \
    Pid.o

SRCLANG := c++

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)app

INCLUDES += \
    -I$(mkfile_path)../../../drivers/include/libcpp/spike \
    -I$(mkfile_path)app
