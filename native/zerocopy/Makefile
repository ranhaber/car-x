# cat_follow zerocopy native library (ROCK 4D / RK3576 only).
#
# Produces libcat_follow_zerocopy.so and optionally installs to /opt/car-x/lib/.
#
# Dependencies (board):
#   librga-dev, rockchip RKNN headers/libs (rknn_api.h, librknnrt.so)

LIBNAME := libcat_follow_zerocopy.so
SRC := cf_zc_session.cpp
CXXFLAGS := -O2 -fPIC -Wall -Wextra -std=c++17
LDFLAGS := -shared
LIBS := -lrga -lrknnrt

PREFIX ?= /opt/car-x
LIBDIR := $(PREFIX)/lib

VALIDATOR := ../../scripts/validate_dmabuf_rga_rknn_fd

.PHONY: all install clean validator

all: $(LIBNAME)

validator: $(LIBNAME) $(VALIDATOR)

$(VALIDATOR): $(VALIDATOR).cpp cf_zc.h
	$(CXX) -O2 -std=c++17 -o $@ $(VALIDATOR).cpp -L. -lcat_follow_zerocopy -Wl,-rpath,'$$ORIGIN/../native/zerocopy'

$(LIBNAME): $(SRC) cf_zc.h cf_zc_internal.hpp cf_zc_yolo_post.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(SRC) $(LIBS)

install: $(LIBNAME)
	install -d $(LIBDIR)
	install -m 755 $(LIBNAME) $(LIBDIR)/

clean:
	rm -f $(LIBNAME)
