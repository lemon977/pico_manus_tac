# manus_ndjson_bridge Makefile
#
# 依赖文件由 scripts/setup_manus_bridge.sh 从 MANUS SDK 的 SDKMinimalClient_Linux
# 拷入本目录：ManusSDK/（include+lib）、ClientPlatformSpecific.{cpp,hpp}、
# ClientPlatformSpecificTypes.hpp、ClientLogging.hpp。
#
# 编译：make          -> ./manus_ndjson_bridge.out
#       make clean

CXX      := g++
CXXFLAGS := -O2 -g --std=c++17
LDFLAGS  := -L./ManusSDK/lib -lManusSDK_Integrated -Wl,-rpath,'$$ORIGIN/ManusSDK/lib'
INCLUDE  := -I./ManusSDK/include
OBJ_DIR  := ./objects
TARGET   := manus_ndjson_bridge.out

# 只编译桥自身；不用 ncurses/键盘层, 因此不编 ClientPlatformSpecific.cpp。
SRC      := manus_ndjson_bridge.cpp
OBJECTS  := $(SRC:%.cpp=$(OBJ_DIR)/%.o)
DEPENDENCIES := $(OBJECTS:.o=.d)

all: $(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^ $(LDFLAGS)

-include $(DEPENDENCIES)

.PHONY: all clean
clean:
	-@rm -f $(TARGET)
	-@rm -rvf $(OBJ_DIR)
