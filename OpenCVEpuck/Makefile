###
### Standard Makefile for Webots controllers
### 
### Platform: Windows, Mac OS X, Linux
### Language: C, C++ and Java
### 
### Authors: Olivier Michel - www.cyberbotics.com
### Revised: Yvan Bourquin - September 30th, 2009.
###
### Uncomment the variables to customize the Makefile

### -----C Sources-----
###
### if your controller uses several C sources files:
# C_SOURCES = my_controller.c my_clever_algo.c my_gui.c

### -----C++ Sources-----
###
### if your controller uses several C++ sources files:
# CPP_SOURCES = my_controller.cpp my_clever_algo.cpp my_gui.cpp
###     or
# CC_SOURCES = my_controller.cc my_clever_algo.cc my_gui.cc

### -----C/C++ Options-----
###
### if special CFLAGS are necessary, for example to set optimization level or
### to find include files:
CFLAGS=-O3 -g -IC:\OpenCV2.1\include\opencv
###
### if your controller needs additional libraries:

LIBRARIES=-LC:\OpenCV2.1\lib -lcv210 -lcvaux210 -lcxcore210 -lcxts210 -lhighgui210 -lml210 -lopencv_ffmpeg210
#LIBRARIES=-LC:\OpenCV2.1\lib -lcv210d -lcvaux210d -lcxcore210d -lhighgui210d -lml210d -lopencv_ffmpeg210d

###
### if you want to use the C API in your C++ program add:
# USE_C_API=1

### -----Java Options-----
###
### if your Java controller needs additional libraries, you should define
### the CLASSPATH environment variable as explained in the Webots User Guide
# CLASSPATH=relative/mylib.jar

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
