dtb-$(CONFIG_DTB_BAIKAL)	+= baikal.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK)	+= baikal_bfk.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK2)	+= baikal_bfk2.dtb
dtb-$(CONFIG_DTB_BAIKAL_FPGA)	+= baikal_fpga.dtb
dtb-$(CONFIG_DTB_BAIKAL_QEMU)	+= baikal_qemu.dtb
dtb-$(CONFIG_DTB_BAIKAL_MODULE)	+= baikal_module.dtb
dtb-$(CONFIG_DTB_BAIKAL_MITX)	+= baikal_mitx.dtb
dtb-$(CONFIG_DTB_BAIKAL_MSBT2)	+= baikal_msbt2.dtb
dtb-$(CONFIG_DTB_TP_RT1MB)	+= tplatforms_rt1mb.dtb

dtb-$(CONFIG_DT_NONE)		+= \
					baikal.dtb	\
					baikal_bfk.dtb	\
					baikal_bfk2.dtb	\
					baikal_fpga.dtb	\
					baikal_qemu.dtb	\
					baikal_module.dtb \
					baikal_mitx.dtb \
					baikal_msbt2.dtb \
					tplatforms_rt1mb.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
