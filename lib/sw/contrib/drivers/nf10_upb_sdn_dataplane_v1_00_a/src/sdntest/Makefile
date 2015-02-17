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

SRCS = sdntest.cpp
LIBPATHS = ../../lib
INCLUDEPATHS = ../../include
LIBS = sdn_dataplane_1.00
EXEPATH = ../../bin
EXE = $(EXEPATH)/sdntest
OBJS = $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SRCS)))
LIBMONITORPATHS = $(patsubst %,$(LIBPATHS)/lib%.la, $(OWNLIBS))

CXXFLAGS = -std=c++11 -DBOOST_ALL_DYN_LINK
DBGCXXFLAGS = -g -O0
RELCXXFLAGS = -O3 -DNDEBUG

ifeq ($(DEBUG),1)
	CXXFLAGS += $(DBGCXXFLAGS)
else
	CXXFLAGS += $(RELCXXFLAGS)
endif

.PHONY: all clean

all: $(EXE)

$(EXE): $(OBJS) $(LIBMONITORPATHS)
	@mkdir -p $(EXEPATH)
	@libtool --mode=link $(CXX) $(addprefix -L, $(LIBPATHS)) $(addprefix -l, $(LIBS)) -g -O -o $@ $^

-include $(patsubst %.cpp,%.d,$(SRCS))

$(OBJS): $(SRCS)
	$(CXX) $(CXXFLAGS) -MD -MP $(addprefix -I, $(INCLUDEPATHS)) -c -o $@ $<

clean:
	rm -f $(EXE) $(OBJS)

