#
#  GPL LICENSE SUMMARY
#
#    Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
#    Copyright(c) 2013-2015 Wind River Systems, Inc. All rights reserved.
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of version 2 of the GNU General Public License as
#    published by the Free Software Foundation.
#
#    This program is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#    The full GNU General Public License is included in this distribution
#    in the file called LICENSE.GPL.
#
#    Contact Information:
#    Intel Corporation

## Common variables
MODULE = wrs_avp
MODULE_CFLAGS = -Wall -Werror

## Source files
SRCS-y += avp_misc.c
SRCS-y += avp_net.c
SRCS-y += avp_pci.c
SRCS-y += avp_ctrl.c
SRCS-y += avp_ethtool.c

## Standalone external module build
MPATH := $$PWD

## Kernel specific flags
MODULE_CFLAGS += -O3
MODULE_CFLAGS += -I$(MPATH)
MODULE_CFLAGS += -I$(MPATH)/include/
MODULE_CFLAGS += -I$(RTE_SDK)/$(RTE_TARGET)/include

## Kernel objects
$(MODULE)-objs += $(notdir $(SRCS-y:%.c=%.o))
obj-m := $(MODULE).o

modules modules_install clean help:
	@$(MAKE) -C $(KSRC) M=$(MPATH) EXTRA_CFLAGS="$(MODULE_CFLAGS)" $@

