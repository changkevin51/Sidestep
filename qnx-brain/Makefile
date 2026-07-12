TARGET := v2v_brain
PARSER_TARGET := parse_gemma_output
ENABLE_GEMINI ?= 0
GEMINI_DIR ?= ../../../Gemma 3
SOURCES := main_p2p.cpp car_collision.cpp udp_network.cpp gemma_output_parser.cpp
OBJECTS := $(SOURCES:.cpp=.o)

ifeq ($(ENABLE_GEMINI),1)
STANDARD_FLAG ?= -std=c++20
GEMINI_CPPFLAGS ?= -I/opt/homebrew/opt/curl/include -I/opt/homebrew/include
GEMINI_LDLIBS ?= -L/opt/homebrew/opt/curl/lib -lcurl
CPPFLAGS += -DSIDESTEP_ENABLE_GEMINI -I"$(GEMINI_DIR)/include" $(GEMINI_CPPFLAGS)
LDLIBS += $(GEMINI_LDLIBS)
OBJECTS += gemini_client_external.o
endif

DEPS := $(OBJECTS:.o=.d)
PARSER_OBJECTS := parse_gemma_output.o gemma_output_parser.o
ALL_DEPS := $(sort $(DEPS) $(PARSER_OBJECTS:.o=.d))

CXX ?= c++
CPPFLAGS ?=
CXXFLAGS ?= -O2
LDFLAGS ?=
LDLIBS += -lm

# A configured QNX SDP exports QNX_TARGET. Select the Raspberry Pi 5's AArch64
# target explicitly: the q++ default is an x86_64 QNX target.
ifneq ($(strip $(QNX_TARGET)),)
ifeq ($(origin CXX),default)
CXX := q++
endif
QNX_VARIANT ?= gcc_ntoaarch64le
STANDARD_FLAG ?= -std=gnu++11
TARGET_FLAG := -V$(QNX_VARIANT)
PLATFORM_FLAGS := $(TARGET_FLAG)
CPPFLAGS += -D_QNX_SOURCE
LDLIBS += -lsocket
else
STANDARD_FLAG ?= -std=c++11
TARGET_FLAG :=
PLATFORM_FLAGS := -pthread -MMD -MP
LDLIBS += -pthread
endif

CXXFLAGS += $(STANDARD_FLAG) -Wall -Wextra -Wpedantic $(PLATFORM_FLAGS)

.PHONY: all clean debug parser

all: $(TARGET)

parser: $(PARSER_TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(TARGET_FLAG) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

$(PARSER_TARGET): $(PARSER_OBJECTS)
	$(CXX) $(TARGET_FLAG) $(LDFLAGS) -o $@ $(PARSER_OBJECTS) $(LDLIBS)

gemini_client_external.o:
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c "$(GEMINI_DIR)/src/gemma_client.cpp" -o $@

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

debug: clean
	$(MAKE) CXX="$(CXX)" CPPFLAGS="$(CPPFLAGS)" \
		CXXFLAGS="-O0 -g3 $(STANDARD_FLAG) -Wall -Wextra -Wpedantic $(PLATFORM_FLAGS)" \
		LDFLAGS="$(LDFLAGS)" LDLIBS="$(LDLIBS)" all

clean:
	rm -f $(TARGET) $(PARSER_TARGET) $(OBJECTS) $(PARSER_OBJECTS) $(ALL_DEPS) \
		gemini_client_external.o gemini_client_external.d

-include $(ALL_DEPS)
