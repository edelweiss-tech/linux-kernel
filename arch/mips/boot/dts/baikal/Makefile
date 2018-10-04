dtb-$(CONFIG_DTB_BAIKAL)	+= baikal.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK)	+= baikal_bfk.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFK3)	+= baikal_bfk3.dtb baikal_bfk2.dtb
dtb-$(CONFIG_DTB_BAIKAL_BFKX)	+= baikal_bfkx.dtb
dtb-$(CONFIG_DTB_BAIKAL_FPGA)	+= baikal_fpga.dtb
dtb-$(CONFIG_DTB_BAIKAL_QEMU)	+= baikal_qemu.dtb
dtb-$(CONFIG_DTB_BAIKAL_MODULE)	+= baikal_module.dtb
dtb-$(CONFIG_DTB_BAIKAL_MITX)	+= baikal_mitx.dtb
dtb-$(CONFIG_DTB_TP_MSBT2)	+= tplatforms_msbt2.dtb
dtb-$(CONFIG_DTB_TP_MRBT1)	+= tplatforms_mrbt1.dtb
dtb-$(CONFIG_DTB_TP_RT1MB)	+= tplatforms_rt1mb.dtb
dtb-$(CONFIG_DTB_TP_BN1BT1)	+= tplatforms_bn1bt1.dtb

dtb-$(CONFIG_DT_NONE)		+= \
					baikal.dtb	\
					baikal_bfk.dtb	\
					baikal_bfk3.dtb	\
					baikal_fpga.dtb	\
					baikal_qemu.dtb	\
					baikal_module.dtb \
					baikal_mitx.dtb \
					tplatrofms_msbt2.dtb \
					tplatrofms_mrbt1.dtb \
					tplatforms_rt1mb.dtb \
					tplatforms_bn1bt1.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
