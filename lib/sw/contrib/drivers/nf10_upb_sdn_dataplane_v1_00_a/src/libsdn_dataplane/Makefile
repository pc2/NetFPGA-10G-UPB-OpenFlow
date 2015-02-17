#
# This file is part of the NetFPGA 10G UPB OpenFlow Switch project
#
# Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
#
# Project Group "On-the-Fly Networking for Big Data"
# SFB 901 "On-The-Fly Computing"
#
# University of Paderborn
# Computer Engineering Group
# Pohlweg 47 - 49
# 33098 Paderborn
# Germany
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

SRCS = sdn_dataplane.cpp axi_bus_connector.cpp errors.cpp platform_flash_xl_emc.cpp flow.cpp hw_flow.cpp tcam.cpp sdn_dp_cwrapper.cpp port.cpp logging.cpp config_port_associations.cpp cam.cpp poll_thread.cpp flow_statistics.cpp
LIBHEADERS = sdn_dataplane.hpp axi_bus_connector.hpp errors.hpp errors.h errors.inc platform_flash_xl_emc.hpp config_platform_flash.hpp flow.hpp hw_flow.hpp bit_lib.hpp tcam.hpp sdn_dp_cwrapper.hpp sdn_dp_cwrapper.h port.hpp logging.hpp hw_flow_table_base.hpp config_port_associations.hpp config_fpga_addresses.hpp cam.hpp poll_thread.hpp flow_statistics.hpp
LIBS = boost_date_time boost_log boost_program_options boost_thread boost_system pthread

OBJS = $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SRCS)))

LIBVER = 1.00
LIBNAME = libsdn_dataplane_$(LIBVER)
LIBPATH = ../../lib
LIBINCLUDEPATH = ../../include
CXXFLAGS = -std=c++11 -DBOOST_ALL_DYN_LINK -I.
DOCPATH = ../../doc/libsdn_dataplane
DOCEXECUTABLES = doxygen dot
LIB = $(LIBPATH)/$(LIBNAME).la
BUILDPATH = build

ifeq ($(DEBUG),1)
	CXXFLAGS += -g -O0
else
	CXXFLAGS += -O3 -DNDEBUG
endif


.PHONY: all doxygen clean

all: $(LIB) doxygen

-include $(patsubst %.cpp,%.d,$(addprefix $(BUILDPATH)/,$(SRCS)))

$(LIB): $(addprefix $(BUILDPATH)/, $(OBJS))
	@mkdir -p $(LIBPATH)
	@libtool --mode=link $(CXX) $(addprefix -l, $(LIBS)) -g -O -o $@ $(patsubst %.o,%.lo,$^)
	@mkdir -p $(LIBINCLUDEPATH)
	cp $(LIBHEADERS) $(LIBINCLUDEPATH)

$(BUILDPATH)/%.o: %.cpp
	@mkdir -p $(BUILDPATH)
	@libtool --mode=compile $(CXX) -o $(patsubst %.o,%.lo,$@) $< -MD -MP -fPIC -c $(CXXFLAGS)
	
FOUNDDOCEXECUTABLES += $(foreach file,$(DOCEXECUTABLES),$(if $(shell which $(file)),$(file),$(warning "Could not find $(file) in path. Documentation will not be built.)))
	
doxygen: $(DOCPATH)/html/index.html
	
$(DOCPATH)/html/index.html: $(SRCS)
ifeq ($(FOUNDDOCEXECUTABLES),$(DOCEXECUTABLES))
	@mkdir -p $(DOCPATH)
	doxygen Doxyfile
endif

clean:
	rm -rf $(LIBPATH) $(BUILDPATH) $(LIBINCLUDEPATH) $(DOCPATH)


