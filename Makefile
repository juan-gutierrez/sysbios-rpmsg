#
# Copyright (c) 2011-2012 Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Repo
BIOSTOOLSROOT   ?= /usr/local
REPO            := $(BIOSTOOLSROOT)

# Customizable version variables - export them or pass as arguments to make
XDCVERSION	    ?= xdctools_3_22_03_41
BIOSVERSION	    ?= bios_6_32_01_38
IPCVERSION	    ?= ipc_1_23_01_26

ifeq (bldcfg.mk,$(wildcard bldcfg.mk))
include bldcfg.mk
endif

BIOSPROD	= $(REPO)/$(BIOSVERSION)
IPCPROD		= $(REPO)/$(IPCVERSION)
XDCDIST_TREE	= $(REPO)/$(XDCVERSION)

export XDCROOT	= $(XDCDIST_TREE)

export XDCPATH	= $(BIOSPROD)/packages;$(IPCPROD)/packages;./src;

all:
	$(XDCROOT)/xdc -j $(j) -Pr src
	
clean:
	$(XDCROOT)/xdc clean -Pr src

smp_config:
	@echo export XDCVERSION=xdctools_3_23_01_43 > bldcfg.mk
	@echo export BIOSVERSION=smpbios_1_00_00_16_eng >> bldcfg.mk
	@echo export SMP=1 >> bldcfg.mk
	@touch src/config.bld

unconfig:
ifeq (bldcfg.mk,$(wildcard bldcfg.mk))
	@rm bldcfg.mk
endif
	@touch src/config.bld

.PHONY: tags
tags:
	ctags -R src/
	cscope -R -b -ssrc/
