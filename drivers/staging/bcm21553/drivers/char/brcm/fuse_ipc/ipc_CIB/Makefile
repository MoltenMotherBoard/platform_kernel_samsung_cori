#Makefile for IPC sources
#EXTRA_CFLAGS := -Werror

EXTRA_CFLAGS += -Idrivers/staging/bcm21553/drivers/char/brcm/fuse_ipc/ipc_CIB/public/
EXTRA_CFLAGS += -Idrivers/staging/bcm21553/drivers/char/brcm/fuse_ipc/ipc_CIB/ap/
EXTRA_CFLAGS += -Idrivers/staging/bcm21553/drivers/char/brcm/fuse_ipc/ipc_CIB/
EXTRA_CFLAGS += -Idrivers/staging/bcm21553/drivers/char/brcm/fuse_ipc/
EXTRA_CFLAGS += -Idrivers/staging/bcm21553/drivers/char/brcm/fuse_log/

EXTRA_CFLAGS += -DUNDER_LINUX -DFUSE_APPS_PROCESSOR -DFUSE_DUAL_PROCESSOR_ARCHITECTURE 

obj-$(CONFIG_BRCM_FUSE_IPC_CIB) += \
	ipc_buffer.o	\
	ipc_bufferpool.o		\
	ipc_powersaving.o	\
	ipc_queues.o		\
	ipc_sharedmemory.o	\
	ipc_trace.o
