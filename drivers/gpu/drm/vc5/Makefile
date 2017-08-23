# Please keep these build lists sorted!

# core driver code
vc5-y := \
	vc5_bo.o \
	vc5_drv.o \
	vc5_fence.o \
	vc5_gem.o \
	vc5_irq.o \
	vc5_mmu.o \
	vc5_trace_points.o

vc5-$(CONFIG_DEBUG_FS) += vc5_debugfs.o

obj-$(CONFIG_DRM_VC5)  += vc5.o

CFLAGS_vc5_trace_points.o := -I$(src)
