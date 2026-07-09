mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS +=\
    debug_log.o \
    Tracer.o \
    DriveBase.o \
    Robot.o \
    CourseConfig.o \
    Calibrator.o \
    ColorJudge.o \
    Pid.o \
    GameRunner.o \
    SumoTask.o \
    DeliveryTask.o \
    RallyTask.o

SRCLANG := c++

APPL_LIBS += -lm

APPL_DIRS +=\
    $(mkfile_path)app \
    $(mkfile_path)app/tasks

INCLUDES += \
    -I$(mkfile_path)../../../drivers/include/libcpp/spike \
    -I$(mkfile_path)app \
    -I$(mkfile_path)app/tasks
