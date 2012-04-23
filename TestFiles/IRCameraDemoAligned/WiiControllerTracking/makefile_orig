########################################################################
# Makefile for Wii Controller Tracking Package. See README for build
# instructions.
# Copyright (c) 2008-2009 Oliver Kreylos
#
# This file is part of the Wii Controller Tracking Package.
#
# The Wii Controller Tracking Package is free software; you can
# redistribute it and/or modify it under the terms of the GNU General
# Public License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
# The Wii Controller Tracking Package is distributed in the hope that it
# will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
# the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the Wii Controller Tracking Package; if not, write to the
# Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

#
# List of C++ source files
#

MISC_SOURCES = Misc/ThrowStdErr.cpp \
               Misc/CallbackList.cpp

THREADS_SOURCES = 

MATH_SOURCES = Math/Constants.cpp \
               Math/Random.cpp

GEOMETRY_SOURCES = Geometry/ComponentArray.cpp \
                   Geometry/Vector.cpp \
                   Geometry/Point.cpp \
                   Geometry/HVector.cpp \
                   Geometry/Box.cpp \
                   Geometry/Matrix.cpp \
                   Geometry/Rotation.cpp \
                   Geometry/TranslationTransformation.cpp \
                   Geometry/RotationTransformation.cpp \
                   Geometry/OrthonormalTransformation.cpp \
                   Geometry/Random.cpp

LIBRARY_SOURCES = $(MISC_SOURCES) \
                  $(THREADS_SOURCES) \
                  $(MATH_SOURCES) \
                  $(GEOMETRY_SOURCES) \

WIITEST_SOURCES = $(LIBRARY_SOURCES) \
                  Wiimote.cpp \
                  WiiTest.cpp

LEVENBERGMARQUARDTTEST_SOURCES = $(LIBRARY_SOURCES) \
                                 LevenbergMarquardtTest.cpp

SIMPLEWIICAMERATEST_SOURCES = $(LIBRARY_SOURCES) \
                              Wiimote.cpp \
                              SimpleWiiCameraTest.cpp

# List of all target executables:
TARGETS = WiiTest \
          LevenbergMarquardtTest \
          SimpleWiiCameraTest

# Set up compiler flags:
ifdef DEBUG
  OPTFLAGS = -g2 -O0
else
  OPTFLAGS = -g0 -O3
endif
CPPFLAGS = 

# Set up target directories for compilation products:
DEPDIR = ./d
ifdef DEBUG
  OBJDIR = ./o/debug
  EXEDIR = ./exe/debug
else
  OBJDIR = ./o
  EXEDIR = ./exe
endif

.PHONY: all
all: $(TARGETS:%=$(EXEDIR)/%)

# Build rules for target executables:
$(EXEDIR)/WiiTest: $(WIITEST_SOURCES:%.cpp=$(OBJDIR)/%.o)
$(EXEDIR)/LevenbergMarquardtTest: $(LEVENBERGMARQUARDTTEST_SOURCES:%.cpp=$(OBJDIR)/%.o)
$(EXEDIR)/SimpleWiiCameraTest: $(SIMPLEWIICAMERATEST_SOURCES:%.cpp=$(OBJDIR)/%.o)

#
# Set up libraries:
#

INCLUDEDIREXT = include
ifeq ($(shell uname -m),x86_64)
  LIBDIREXT = lib64
else
  LIBDIREXT = lib
endif

# The pthreads package
PTHREADS_BASEDIR    = /usr
PTHREADS_INCLUDEDIR = $(PTHREADS_BASEDIR)/$(INCLUDEDIREXT)
PTHREADS_LIBDIR     = $(PTHREADS_BASEDIR)/$(LIBDIREXT)
PTHREADS_LIBS       = pthread

# The user-level Bluetooth library
BLUETOOTH_BASEDIR    = /usr
BLUETOOTH_INCLUDEDIR = $(BLUETOOTH_BASEDIR)/$(INCLUDEDIREXT)
BLUETOOTH_LIBDIR     = $(BLUETOOTH_BASEDIR)/$(LIBDIREXT)
BLUETOOTH_LIBS       = bluetooth

# Local libraries
LOCAL_BASEDIR    = .
LOCAL_INCLUDEDIR = .
LOCAL_LIBDIR     = 
LOCAL_LIBS       = 

PACKAGES = PTHREADS BLUETOOTH LOCAL

# Library flags:
INCLUDEDIRS = $(patsubst %,-I%,$(foreach PACKAGE,$(PACKAGES),$($(PACKAGE)_INCLUDEDIR)))
LIBDIRS = $(patsubst %,-L%,$(foreach PACKAGE,$(PACKAGES),$($(PACKAGE)_LIBDIR)))
LIBS = $(patsubst %,-l%,$(foreach PACKAGE,$(PACKAGES),$($(PACKAGE)_LIBS)))

#
# Set up compiler:
#

# Use default g++ compiler
CPPCOMP = g++

# Check for the compiler type as it influences ABI and standard strictness
ifeq ($(shell expr `$(CPPCOMP) -dumpversion` ">=" "3.0.0"),1)
  # gcc versions 3.0.0 and up create dependency file with the same
  # root name and in the same directory as the created object file
  DEPFILETEMPLATE = '$(patsubst %.o,%.d,$@)'
else
  # gcc version 2.x.y creates dependency file with the same root name as
  # the source file in the current directory:
  DEPFILETEMPLATE = '$(*F).d'
endif
ifeq ($(shell expr `$(CPPCOMP) -dumpversion` ">=" "4.1.0"),1)
  # gcc version 4.1.0 and up require friend injection flag
  CPPFLAGS += -ffriend-injection
endif

# make rule to compile C++ source code:
CCPPFLAGS = $(OPTFLAGS) $(INCLUDEDIRS) $(CPPFLAGS)
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(DEPDIR)/$(*D)
	@mkdir -p $(OBJDIR)/$(*D)
ifdef SHOWCOMMAND
	$(CPPCOMP) -MD -c -o $@ $(CCPPFLAGS) $<
else
	@echo Compiling $<...
	@$(CPPCOMP) -MD -c -o $@ $(CCPPFLAGS) $<
endif
	@sed -e 's!\([^:]*:\)!$$(OBJDIR)/$*.o:!g' -e 's!/usr/[^ ]*!!g' -e '/^[ ]*\\$$/ d' < $(DEPFILETEMPLATE) > $(DEPDIR)/$*.d
	@rm -f $(DEPFILETEMPLATE)

# make rule to link executables:
$(EXEDIR)/%:
	@mkdir -p $(EXEDIR)/$(*D)
ifdef SHOWCOMMAND
	$(CPPCOMP) -o $@ $^ $(LIBDIRS) $(LIBS)
else
	@echo Linking $@...
	@$(CPPCOMP) -o $@ $^ $(LIBDIRS) $(LIBS)
endif

#
# Make targets:
#

.PHONY: clean
clean:
	-rm -rf $(DEPDIR)
	-rm -rf $(OBJDIR)
	-rm -rf $(EXEDIR)

# Use empty default target to treat missing prerequisites as outdated
.DEFAULT: ;

# include all automatically generated dependency files:
DEPFILES = $(shell find $(DEPDIR) -follow -name "*.d")
include $(DEPFILES)
