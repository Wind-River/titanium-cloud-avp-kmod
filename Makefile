#   BSD LICENSE
# 
#   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
#   Copyright(c) 2014 Wind River Systems, Inc. All rights reserved.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Common variables
MODULE = wrs_avp
MODULE_CFLAGS = -Wall -Werror

## Source files
SRCS-y += avp_misc.c
SRCS-y += avp_net.c
SRCS-y += avp_pci.c
SRCS-y += avp_ctrl.c

ifneq ($(RTE_SDK),)
## DPDK build
include $(RTE_SDK)/mk/rte.vars.mk
## DPDK specific flags
MODULE_CFLAGS += -I$(SRCDIR)
MODULE_CFLAGS += -I$(RTE_OUTPUT)/include
## Final DPDK build step
include $(RTE_SDK)/mk/rte.module.mk

else
## Standalone external module build
MPATH := $$PWD
KPATH := /lib/modules/`uname -r`/build

## Kernel specific flags
MODULE_CFLAGS += -O3
MODULE_CFLAGS += -I$(MPATH)
MODULE_CFLAGS += -I$(MPATH)/include/
## Kernel objects
$(MODULE)-objs += $(notdir $(SRCS-y:%.c=%.o))
obj-m := $(MODULE).o

modules modules_install clean help:
	@$(MAKE) -C $(KPATH) M=$(MPATH) EXTRA_CFLAGS="$(MODULE_CFLAGS)" $@

endif
