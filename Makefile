CFLAGS =  -I./build -Wall -I/Eigen -g -fPIC -std=c++11 -I./libs/gl3w `pkg-config --cflags glfw3`
LDFLAGS = -lpthread -ldl -lmc_rbdyn -lRBDyn -lTasks -lGL `pkg-config --static --libs glfw3`

OBJS = \
build/utils.o  \
build/MPCSolver.o  \
build/Configuration.o \
build/Joint.o \
build/Robot.o \
build/Hrp4.o \
build/Kinematics.o \
build/Hrp4Kinematics.o \
build/Task.o \
build/TaskSetPoint.o \
build/TaskTrajectory.o \
build/TaskPath.o \
build/TaskNavigation.o \
build/TaskTrajectoryVisual.o \
build/TaskPathVisual.o \
build/TaskCoM.o \
build/TaskSwFoot.o \
build/MovementPrimitive.o \
build/MotionGenerator.o \
build/State.o \
build/Node.o \
build/RRTtree.o \
build/Planner.o \
build/v_repExtPluginSkeleton.o \
build/v_repLib.o \
build/BLASReplacement.o \
build/Bounds.o \
build/Constraints.o \
build/Flipper.o \
build/Indexlist.o \
build/LAPACKReplacement.o \
build/Matrices.o\
build/MessageHandling.o \
build/Options.o \
build/OQPinterface.o \
build/QProblem.o \
build/QProblemB.o \
build/SolutionAnalysis.o \
build/SparseSolver.o \
build/SQProblem.o \
build/SQProblemSchur.o \
build/SubjectTo.o \
build/Utils.o \
build/imgui_impl_glfw_gl3.o \
build/imgui.o \
build/imgui_demo.o \
build/imgui_draw.o \
libs/gl3w/GL/gl3w.o

#QPOASES_OBJS = \
#build/BLASReplacement.cpp.o \
#build/Bounds.cpp.o \
#build/Constraints.cpp.o \
#build/Flipper.cpp.o \
#build/Indexlist.cpp.o \
#build/LAPACKReplacement.cpp.o \
#build/Matrices.cpp.o\
#build/MessageHandling.cpp.o \
#build/Options.cpp.o \
#build/OQPinterface.cpp.o \
#build/QProblem.cpp.o \
#build/QProblemB.cpp.o \
#build/SolutionAnalysis.cpp.o \
#build/SparseSolver.cpp.o \
#build/SQProblem.cpp.o \
#build/SQProblemSchur.cpp.o \
#build/SubjectTo.cpp.o \
#build/Utils.cpp.o

OS = $(shell uname -s)
ECHO=@

ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	OPTION = -shared
	EXT = so
else
	CFLAGS += -D__APPLE__
	OPTION = -dynamiclib -current_version 1.0
	EXT = dylib
endif

TARGET = ./bin/libv_repExtPluginSkeleton.$(EXT)

VREP_HOME = /home/lorenzo/AMRproject/V-REP_PRO_EDU_V3_4_0_Linux

default: v_repExtPluginSkeletonLib
	cp ./bin/libv_repExtPluginSkeleton.${EXT} ${VREP_HOME}

v_repExtPluginSkeletonLib: $(OBJS)
		@echo "Linking $(OBJS)  to $(TARGET)"
		$(ECHO)$(CXX) $(CFLAGS) $(OBJS) $(OPTION) -o $(TARGET) $(LDFLAGS) $(LIBS)

%.o: %.c
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

./build/%.o: ./%.cpp
		@echo "Compiling $< to $@"
		$(ECHO)$(CXX) $(CFLAGS) -c $< -o $@

clean:
		@echo "Cleaning $(OBJS) $(TARGET)"
		$(ECHO)rm -rf $(OBJS) $(TARGET)
