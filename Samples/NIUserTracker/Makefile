include ../Build/Common/CommonDefs.mak

BIN_DIR = ../Bin

INC_DIRS = ../../Include =/usr/include/ni 
INC_DIRS += .

SRC_FILES = ./*.cpp
SRC_FILES += ./utility/math/matrix/Rotation3D.cpp
SRC_FILES += ./utility/math/matrix/Transform3D.cpp
SRC_FILES += ./utility/math/geometry/UnitQuaternion.cpp
#add utilities
$(info $$SRC_FILES is [${SRC_FILES}])

EXE_NAME = Sample-NiUserTracker

ifeq "$(GLUT_SUPPORTED)" "1"
	ifeq ("$(OSTYPE)","Darwin")
		LDFLAGS += -framework OpenGL -framework GLUT
	else
		USED_LIBS += glut GL
	endif
else
	ifeq "$(GLES_SUPPORTED)" "1"
		DEFINES += USE_GLES
		USED_LIBS += GLES_CM IMGegl srv_um
	else
		DUMMY:=$(error No GLUT or GLES!)
	endif
endif

USED_LIBS += OpenNI
USED_LIBS += armadillo

LIB_DIRS += ../../Lib
include ../Build/Common/CommonCppMakefile

