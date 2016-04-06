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
#---------------------------------------------------------------------------

COMP_ROOT=$(CURDIR)
include Makefile.include

SRC_DIRS = include kernel user util
DOC_DIRS = include

#------------------------------------------------------------------------
# functions
#------------------------------------------------------------------------

# Loop through the list of directories and run their make files with 
# the given target.  Parameters:
#		1: Makefile target to be invoked in each directory
#		2: Space-separated list of directories

make_loop = \
	@(for x in $(2); do \
		if [ -d $$x ] ; then \
			echo ; \
			echo '*********************************************************'; \
			echo make $(1) for $$x; \
			echo '*********************************************************'; \
			$(MAKE) -C $$x $(1) || exit 1; \
		fi \
	done; )

#----------------------------------------------------------------------
# Makefile targets
#----------------------------------------------------------------------

.PHONY: all
all: bld install

.PHONY: debug
debug: dbg install

.PHONY: clean
clean:
	$(call make_loop, clean, $(SRC_DIRS) $(DOC_DIRS))

.PHONY: doc
doc:
	$(call make_loop, doc, $(DOC_DIRS))

.PHONY: install
install: install_dev install_target

.PHONY: test
test:

.PHONY: bld
bld: _tell test
	$(call make_loop, all, $(SRC_DIRS))

.PHONY: dbg
dbg: _tell test doc
	$(call make_loop, debug, $(SRC_DIRS))

.PHONY: install_dev
install_dev: 
	$(call make_loop, install_dev, $(SRC_DIRS) $(DOC_DIRS))

.PHONY: install_target
install_target:
	$(call make_loop, install_target, $(SRC_DIRS))
	mkdir -p $(FSROOT)/etc/init.d
	cp init_intel_ce_pm $(FSROOT)/etc/init.d/intel_ce_pm
	cp runtime_pm $(FSROOT)/etc/init.d/

#------------------------------------------------------------------------
# Internal helper targets
#------------------------------------------------------------------------

# Display the values for make variables
.PHONY: _tell
_tell:
	@echo '------------------'
	@echo ' Build Environment'
	@echo '------------------'
	@echo 'COMP_VER     = $(COMP_VER)'
	@echo 'COMP_ROOT    = $(COMP_ROOT)'
	@echo 'BUILD_NUMBER = $(COMP_VER4)'
	@echo 'BUILD_ROOT   = $(BUILD_ROOT)'
	@echo 'BUILD_DIR    = $(BUILD_DIR)'
	@echo 'FSROOT       = $(FSROOT)'
