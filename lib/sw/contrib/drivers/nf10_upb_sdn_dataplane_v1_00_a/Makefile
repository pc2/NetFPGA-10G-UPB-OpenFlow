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

SUBDIRS = src/libsdn_dataplane src/sdntest

.PHONY: all debug release clean $(SUBDIRS)

all debug release clean: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@ $(MAKECMDGOALS)

src/sdntest: src/libsdn_dataplane

