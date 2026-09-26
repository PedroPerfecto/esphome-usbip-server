CXX ?= c++
COMP := ../components/usbip_server
CXXFLAGS := -std=c++17 -Wall -Wextra -I$(COMP) -I.
SRCS := $(COMP)/usbip_wire.cpp $(COMP)/usbip_device.cpp $(COMP)/usbip_stream.cpp $(COMP)/usbip_control.cpp $(COMP)/usbip_urb.cpp $(COMP)/usbip_endpoints.cpp $(COMP)/usbip_chunk.cpp $(COMP)/usbip_filter.cpp $(wildcard *.cpp)

test: build/test_runner
	./build/test_runner

build/test_runner: $(SRCS) $(wildcard $(COMP)/*.h)
	mkdir -p build
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@

clean:
	rm -rf build

.PHONY: test clean
