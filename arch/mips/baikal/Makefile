#
# Baikal-T SOC platform support code.
#
# Copyright (C) 2014-2016 Baikal Electronics JSC
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

cflags-y += -I$(srctree)/arch/mips/include/asm/mach-baikal

obj-y				:= baikal-init.o baikal-int.o baikal-memory.o \
				   baikal-setup.o baikal-time.o \
				   baikal-apb.o baikal-axi.o baikal-emc.o

obj-$(CONFIG_EARLY_PRINTK)	+= baikal-console.o
obj-$(CONFIG_OF)		+= baikal-of.o
obj-$(CONFIG_CPU_FREQ)		+= baikal-cpufreq.o
obj-$(CONFIG_KEXEC)		+= baikal-kexec.o
obj-$(CONFIG_MACH_BAIKAL_BFK2)	+= baikal-efuse.o
