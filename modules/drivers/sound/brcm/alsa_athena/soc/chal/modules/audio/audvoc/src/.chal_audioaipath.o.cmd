cmd_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o := /opt/toolchains/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/.chal_audioaipath.o.d  -nostdinc -isystem /opt/toolchains/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/mackief/Dev/Cori/kernel/kernel/arch/arm/include -Iinclude  -I../modules/include -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-bcm215xx/include -Iarch/arm/plat-bcmap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -DUNDER_LINUX -D _ATHENA_ -D MSP -D FUSE_DUAL_PROCESSOR_ARCHITECTURE -D FUSE_APPS_PROCESSOR -D PMU_MAX8986 -D CHAL_NDEBUG_BUILD -D CHIP_REVISION=20 -D VPU_INCLUDED -D ATHENA_INCLUDE_VOIP -DENABLE_GPIO -I ../modules/../modules/drivers/sound/brcm/alsa_athena -I ../modules/../modules/drivers/sound/brcm/alsa_athena/common -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_controller -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_controller/public -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_vdriver/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_datadriver/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/audio/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/bsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/inc/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0 -I ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/athena/B0/include/ -D_BSDTYPES_DEFINED -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/public/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/arpc/inc/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/srpc/inc/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/xdr/inc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/cc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/cp/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/ds/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/lcs/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/msc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/pch/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/phonebk/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/sim/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/sms/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/ss/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/xdr/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/dataservices/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/brcm/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/thirdparty/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/debug/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/os/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/csl/bsp/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/cpps/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/em/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/adc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/pmu/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/rtc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/util/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/capirpc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/capirpc/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/stubs -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/debug/public -I ../modules/drivers/char/brcm/fuse_log -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/voif_handler/public -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/gpio_handler/public -I ./arch/arm/plat-bcmap/include/ -I ./arch/arm/plat-bcmap/include/plat/ -I ./drivers/char/broadcom/ -I ./arch/arm/mach-bcm215xx/include/ -I ./arch/arm/plat-bcmap/include/plat/chal/ -I ./arch/arm/plat-bcmap/include/plat/osabstract/   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(chal_audioaipath)"  -D"KBUILD_MODNAME=KBUILD_STR(snd_brcm_omx)" -D"DEBUG_HASH=24" -D"DEBUG_HASH2=1" -c -o ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.c

deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o := \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.c \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/mobcom_types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/audio_consts.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/tones_def.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0/brcm_rdb_sysmap.h \
  arch/arm/mach-bcm215xx/include/mach/io.h \
  arch/arm/mach-bcm215xx/include/mach/hardware.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0/brcm_rdb_dsp_audio.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0/brcm_rdb_util.h \
  arch/arm/plat-bcmap/include/plat/chal/chal_types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/inc/chal_audiomisc.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/inc/chal_audioaipath.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/log.h \
  arch/arm/plat-bcmap/include/plat/osdal_os.h \
  include/linux/kernel.h \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/spinlock/sleep.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
    $(wildcard include/config/ring/buffer.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
  /opt/toolchains/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include/stdarg.h \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/linkage.h \
  include/linux/stddef.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/posix_types.h \
  include/linux/bitops.h \
    $(wildcard include/config/generic/find/first/bit.h) \
    $(wildcard include/config/generic/find/last/bit.h) \
    $(wildcard include/config/generic/find/next/bit.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/bitops.h \
    $(wildcard include/config/smp.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/arch/has/barriers.h) \
    $(wildcard include/config/arm/dma/mem/bufferable.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  include/linux/typecheck.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/irqflags.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/hwcap.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/outercache.h \
    $(wildcard include/config/outer/cache/sync.h) \
    $(wildcard include/config/outer/cache.h) \
  include/asm-generic/cmpxchg-local.h \
  include/asm-generic/bitops/non-atomic.h \
  include/asm-generic/bitops/fls64.h \
  include/asm-generic/bitops/sched.h \
  include/asm-generic/bitops/hweight.h \
  include/asm-generic/bitops/arch_hweight.h \
  include/asm-generic/bitops/const_hweight.h \
  include/asm-generic/bitops/lock.h \
  include/linux/log2.h \
    $(wildcard include/config/arch/has/ilog2/u32.h) \
    $(wildcard include/config/arch/has/ilog2/u64.h) \
  include/linux/dynamic_debug.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/swab.h \
  include/linux/byteorder/generic.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/bug.h \
    $(wildcard include/config/bug.h) \
    $(wildcard include/config/debug/bugverbose.h) \
  include/asm-generic/bug.h \
    $(wildcard include/config/generic/bug.h) \
    $(wildcard include/config/generic/bug/relative/pointers.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/string.h \

../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o: $(deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o)

$(deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/audvoc/src/chal_audioaipath.o):
