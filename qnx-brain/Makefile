TARGET := v2v_brain
SOURCES := main_p2p.cpp car_collision.cpp udp_network.cpp driver_alert.cpp
OBJECTS := $(SOURCES:.cpp=.o)
DEPS := $(OBJECTS:.o=.d)

CXX ?= c++
CPPFLAGS ?=
CXXFLAGS ?= -O2
LDFLAGS ?=
LDLIBS += -lm -lcurl

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

.PHONY: all clean debug

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(TARGET_FLAG) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

debug: clean
	$(MAKE) CXX="$(CXX)" CPPFLAGS="$(CPPFLAGS)" \
		CXXFLAGS="-O0 -g3 $(STANDARD_FLAG) -Wall -Wextra -Wpedantic $(PLATFORM_FLAGS)" \
		LDFLAGS="$(LDFLAGS)" LDLIBS="$(LDLIBS)" all

clean:
	rm -f $(TARGET) $(OBJECTS) $(DEPS)

-include $(DEPS)
