#compiler and compiler flags
CXX = g++
CXXFLAGS = -Wall -std=c++11
DEBUG_FLAGS = -g

#directories
INCLUDE_DIR = include/anello_ros_driver
BUILD_DIR = makefile_compilation/build
BIN_DIR = makefile_compilation/bin

SUBDIRS = comm messaging

SRC_DIR = src
# srcs = $(wildcard $(SRC_DIR)/**/*.cpp)
srcs = $(wildcard $(SRC_DIR)/**/*.cpp) $(wildcard $(SRC_DIR)/*.cpp)

INCLUDES = -I$(INCLUDE_DIR)
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(srcs))

EXECUTABLE = $(BIN_DIR)/anello_ros_driver.out

# default
all: $(EXECUTABLE)

#build executable
$(EXECUTABLE): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

# build object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(info mkdir -p $(BUILD_DIR))
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

#debug target
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: all

# clean
clean:
	@rm -rf $(BUILD_DIR) $(BIN_DIR)
