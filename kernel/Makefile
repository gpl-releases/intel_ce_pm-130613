#-----------------------------------------------------------------------------
# This file is provided under a dual BSD/GPLv2 license.  When using or
# redistributing this file, you may do so under either license.
#
# GPL LICENSE SUMMARY
#
# Copyright(c) 2009-2013 Intel Corporation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of version 2 of the GNU General Public License as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
# The full GNU General Public License is included in this distribution
# in the file called LICENSE.GPL.
#
# Contact Information:
#      Intel Corporation
#      2200 Mission College Blvd.
#      Santa Clara, CA  97052
#
# BSD LICENSE
#
# Copyright(c) 2009-2013 Intel Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   - Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   - Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#   - Neither the name of Intel Corporation nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#-----------------------------------------------------------------------------

ifndef COMP_ROOT
export COMP_ROOT=$(abspath ..)
endif
include $(COMP_ROOT)/Makefile.kernel

MODULE  = intel_ce_pm
TARGET  = ${MODULE}.ko
OBJS    = \
          ce2600.o          \
          ce4100.o          \
          ce4200.o          \
          ce5300.o          \
          ids.o             \
          mod.o             \
          procfs.o          \
          punit.o           \
          us_notify.o

obj-m             := $(MODULE).o
$(MODULE)-objs    := $(OBJS) version.o
$(MODULE)-y       :=

.PHONY: all
all: $(TARGET)

$(TARGET): FORCE
	@echo '--- build $@'
	make -C $(KDIR) M=$(CURDIR) modules 
	mv -f Module.symvers $(MODULE).symvers

.PHONY: debug
debug:
	make all EXTRA_CFLAGS="$(EXTRA_CFLAGS) -DDEBUG -Wno-error"

.PHONY: clean
clean:
	@make -C $(KDIR) M=$(CURDIR) clean
	rm -f ${MODULE}.symvers modules.order version.c

.PHONY: install
install: install_dev install_target

.PHONY: install_dev
install_dev:
	mkdir -p $(BUILD_DEST)/kernel/module.symvers
	cp -f ${MODULE}.symvers $(BUILD_DEST)/kernel/module.symvers/$(MODULE)

.PHONY: install_target
DEST = $(FSROOT)/lib/modules
install_target:
	mkdir -p $(DEST)
	cp -pd $(TARGET) $(DEST)

.PHONY: FORCE
FORCE:

$(src)/version.c: $(OBJS)
	@echo "char *gVersion = \"#@# $(TARGET) $(COMP_VER)\";" > $(src)/version.c

$(obj)/version.o: $(src)/version.c
