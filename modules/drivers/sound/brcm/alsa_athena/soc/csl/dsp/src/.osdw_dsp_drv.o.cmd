cmd_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o := /opt/toolchains/arm-eabi-4.4.3/bin/arm-eabi-gcc -Wp,-MD,../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/.osdw_dsp_drv.o.d  -nostdinc -isystem /opt/toolchains/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include -I/home/mackief/Dev/Cori/kernel/kernel/arch/arm/include -Iinclude  -I../modules/include -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-bcm215xx/include -Iarch/arm/plat-bcmap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -DUNDER_LINUX -D _ATHENA_ -D MSP -D FUSE_DUAL_PROCESSOR_ARCHITECTURE -D FUSE_APPS_PROCESSOR -D PMU_MAX8986 -D CHAL_NDEBUG_BUILD -D CHIP_REVISION=20 -D VPU_INCLUDED -D ATHENA_INCLUDE_VOIP -DENABLE_GPIO -I ../modules/../modules/drivers/sound/brcm/alsa_athena -I ../modules/../modules/drivers/sound/brcm/alsa_athena/common -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_controller -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_controller/public -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_vdriver/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/audio_datadriver/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/audio/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/bsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/chal/modules/audio/inc/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0 -I ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/public/ -I ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/athena/B0/include/ -D_BSDTYPES_DEFINED -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/public/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/arpc/inc/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/srpc/inc/ -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/xdr/inc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/cc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/cp/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/ds/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/lcs/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/msc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/pch/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/phonebk/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/sim/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/sms/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/ss/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2/xdr/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/dataservices/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/brcm/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/peripherals/pmu/public/thirdparty/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/debug/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/os/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/soc/csl/bsp/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/cpps/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/em/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/adc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/pmu/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/hal/rtc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/util/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/capirpc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/modem/capi/capirpc/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/gen/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/sysinterface/sysrpc/public/ -I ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/stubs -I ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/debug/public -I ../modules/drivers/char/brcm/fuse_log -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/voif_handler/public -I ../modules/../modules/drivers/sound/brcm/alsa_athena/audio/gpio_handler/public -I ./arch/arm/plat-bcmap/include/ -I ./arch/arm/plat-bcmap/include/plat/ -I ./drivers/char/broadcom/ -I ./arch/arm/mach-bcm215xx/include/ -I ./arch/arm/plat-bcmap/include/plat/chal/ -I ./arch/arm/plat-bcmap/include/plat/osabstract/   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(osdw_dsp_drv)"  -D"KBUILD_MODNAME=KBUILD_STR(snd_brcm_omx)" -D"DEBUG_HASH=62" -D"DEBUG_HASH2=9" -c -o ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.c

deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o := \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.c \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/mobcom_types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/chip_version.h \
  include/linux/sched.h \
    $(wildcard include/config/sched/debug.h) \
    $(wildcard include/config/prove/rcu.h) \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/no/hz.h) \
    $(wildcard include/config/detect/softlockup.h) \
    $(wildcard include/config/detect/hung/task.h) \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/core/dump/default/elf/headers.h) \
    $(wildcard include/config/virt/cpu/accounting.h) \
    $(wildcard include/config/bsd/process/acct.h) \
    $(wildcard include/config/taskstats.h) \
    $(wildcard include/config/audit.h) \
    $(wildcard include/config/inotify/user.h) \
    $(wildcard include/config/epoll.h) \
    $(wildcard include/config/posix/mqueue.h) \
    $(wildcard include/config/keys.h) \
    $(wildcard include/config/perf/events.h) \
    $(wildcard include/config/schedstats.h) \
    $(wildcard include/config/task/delay/acct.h) \
    $(wildcard include/config/fair/group/sched.h) \
    $(wildcard include/config/rt/group/sched.h) \
    $(wildcard include/config/preempt/notifiers.h) \
    $(wildcard include/config/blk/dev/io/trace.h) \
    $(wildcard include/config/tree/preempt/rcu.h) \
    $(wildcard include/config/cc/stackprotector.h) \
    $(wildcard include/config/sysvipc.h) \
    $(wildcard include/config/auditsyscall.h) \
    $(wildcard include/config/generic/hardirqs.h) \
    $(wildcard include/config/rt/mutexes.h) \
    $(wildcard include/config/debug/mutexes.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/lockdep.h) \
    $(wildcard include/config/task/xacct.h) \
    $(wildcard include/config/cpusets.h) \
    $(wildcard include/config/cgroups.h) \
    $(wildcard include/config/futex.h) \
    $(wildcard include/config/compat.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/fault/injection.h) \
    $(wildcard include/config/latencytop.h) \
    $(wildcard include/config/function/graph/tracer.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/cgroup/mem/res/ctlr.h) \
    $(wildcard include/config/cpumask/offstack.h) \
    $(wildcard include/config/have/unstable/sched/clock.h) \
    $(wildcard include/config/hotplug/cpu.h) \
    $(wildcard include/config/stack/growsup.h) \
    $(wildcard include/config/debug/stack/usage.h) \
    $(wildcard include/config/preempt.h) \
    $(wildcard include/config/cgroup/sched.h) \
    $(wildcard include/config/mm/owner.h) \
    $(wildcard include/config/frame/pointer.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/param.h \
    $(wildcard include/config/hz.h) \
  include/linux/capability.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/posix_types.h \
  include/linux/threads.h \
    $(wildcard include/config/nr/cpus.h) \
    $(wildcard include/config/base/small.h) \
  include/linux/kernel.h \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/spinlock/sleep.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
    $(wildcard include/config/ring/buffer.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
  /opt/toolchains/arm-eabi-4.4.3/bin/../lib/gcc/arm-eabi/4.4.3/include/stdarg.h \
  include/linux/linkage.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/linkage.h \
  include/linux/bitops.h \
    $(wildcard include/config/generic/find/first/bit.h) \
    $(wildcard include/config/generic/find/last/bit.h) \
    $(wildcard include/config/generic/find/next/bit.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/bitops.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/arch/has/barriers.h) \
    $(wildcard include/config/arm/dma/mem/bufferable.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  include/linux/irqflags.h \
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
  include/linux/timex.h \
  include/linux/time.h \
    $(wildcard include/config/arch/uses/gettimeoffset.h) \
  include/linux/cache.h \
    $(wildcard include/config/arch/has/cache/line/size.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/cache.h \
    $(wildcard include/config/arm/l1/cache/shift.h) \
    $(wildcard include/config/aeabi.h) \
  include/linux/seqlock.h \
  include/linux/spinlock.h \
    $(wildcard include/config/debug/spinlock.h) \
    $(wildcard include/config/generic/lockbreak.h) \
    $(wildcard include/config/debug/lock/alloc.h) \
  include/linux/preempt.h \
    $(wildcard include/config/debug/preempt.h) \
  include/linux/thread_info.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/vfpv3.h) \
    $(wildcard include/config/iwmmxt.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
  include/linux/list.h \
    $(wildcard include/config/debug/list.h) \
  include/linux/poison.h \
    $(wildcard include/config/illegal/pointer/value.h) \
  include/linux/prefetch.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/processor.h \
  include/linux/stringify.h \
  include/linux/bottom_half.h \
  include/linux/spinlock_types.h \
  include/linux/spinlock_types_up.h \
  include/linux/lockdep.h \
    $(wildcard include/config/lock/stat.h) \
  include/linux/rwlock_types.h \
  include/linux/spinlock_up.h \
  include/linux/rwlock.h \
  include/linux/spinlock_api_up.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/atomic.h \
    $(wildcard include/config/generic/atomic64.h) \
  include/asm-generic/atomic-long.h \
  include/linux/math64.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/div64.h \
  include/linux/param.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/timex.h \
  arch/arm/mach-bcm215xx/include/mach/timex.h \
  include/linux/jiffies.h \
  include/linux/rbtree.h \
  include/linux/cpumask.h \
    $(wildcard include/config/debug/per/cpu/maps.h) \
    $(wildcard include/config/disable/obsolete/cpumask/functions.h) \
  include/linux/bitmap.h \
  include/linux/string.h \
    $(wildcard include/config/binary/printf.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/string.h \
  include/linux/errno.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/errno.h \
  include/asm-generic/errno.h \
  include/asm-generic/errno-base.h \
  include/linux/nodemask.h \
    $(wildcard include/config/highmem.h) \
  include/linux/numa.h \
    $(wildcard include/config/nodes/shift.h) \
  include/linux/mm_types.h \
    $(wildcard include/config/split/ptlock/cpus.h) \
    $(wildcard include/config/want/page/debug/flags.h) \
    $(wildcard include/config/kmemcheck.h) \
    $(wildcard include/config/aio.h) \
    $(wildcard include/config/proc/fs.h) \
    $(wildcard include/config/mmu/notifier.h) \
  include/linux/auxvec.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/auxvec.h \
  include/linux/prio_tree.h \
  include/linux/rwsem.h \
    $(wildcard include/config/rwsem/generic/spinlock.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/rwsem.h \
  include/linux/completion.h \
  include/linux/wait.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/current.h \
  include/linux/page-debug-flags.h \
    $(wildcard include/config/page/poisoning.h) \
    $(wildcard include/config/page/debug/something/else.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/page.h \
    $(wildcard include/config/cpu/copy/v3.h) \
    $(wildcard include/config/cpu/copy/v4wt.h) \
    $(wildcard include/config/cpu/copy/v4wb.h) \
    $(wildcard include/config/cpu/copy/feroceon.h) \
    $(wildcard include/config/cpu/copy/fa.h) \
    $(wildcard include/config/cpu/xscale.h) \
    $(wildcard include/config/cpu/copy/v6.h) \
    $(wildcard include/config/sparsemem.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/glue.h \
    $(wildcard include/config/cpu/arm610.h) \
    $(wildcard include/config/cpu/arm710.h) \
    $(wildcard include/config/cpu/abrt/lv4t.h) \
    $(wildcard include/config/cpu/abrt/ev4.h) \
    $(wildcard include/config/cpu/abrt/ev4t.h) \
    $(wildcard include/config/cpu/abrt/ev5tj.h) \
    $(wildcard include/config/cpu/abrt/ev5t.h) \
    $(wildcard include/config/cpu/abrt/ev6.h) \
    $(wildcard include/config/cpu/abrt/ev7.h) \
    $(wildcard include/config/cpu/pabrt/legacy.h) \
    $(wildcard include/config/cpu/pabrt/v6.h) \
    $(wildcard include/config/cpu/pabrt/v7.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/memory.h \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/thumb2/kernel.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
  include/linux/const.h \
  arch/arm/mach-bcm215xx/include/mach/memory.h \
    $(wildcard include/config/sdram/base/addr.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
  include/asm-generic/getorder.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/mmu.h \
    $(wildcard include/config/cpu/has/asid.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/cputime.h \
  include/asm-generic/cputime.h \
  include/linux/smp.h \
    $(wildcard include/config/use/generic/smp/helpers.h) \
  include/linux/sem.h \
  include/linux/ipc.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/ipcbuf.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/sembuf.h \
  include/linux/rcupdate.h \
    $(wildcard include/config/rcu/torture/test.h) \
    $(wildcard include/config/tree/rcu.h) \
    $(wildcard include/config/tiny/rcu.h) \
  include/linux/rcutree.h \
  include/linux/signal.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/signal.h \
  include/asm-generic/signal-defs.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/sigcontext.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/siginfo.h \
  include/asm-generic/siginfo.h \
  include/linux/path.h \
  include/linux/pid.h \
  include/linux/percpu.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/need/per/cpu/embed/first/chunk.h) \
    $(wildcard include/config/need/per/cpu/page/first/chunk.h) \
    $(wildcard include/config/have/setup/per/cpu/area.h) \
  include/linux/pfn.h \
  include/linux/init.h \
    $(wildcard include/config/hotplug.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/percpu.h \
  include/asm-generic/percpu.h \
  include/linux/percpu-defs.h \
    $(wildcard include/config/debug/force/weak/per/cpu.h) \
  include/linux/topology.h \
    $(wildcard include/config/sched/smt.h) \
    $(wildcard include/config/sched/mc.h) \
    $(wildcard include/config/use/percpu/numa/node/id.h) \
    $(wildcard include/config/have/memoryless/nodes.h) \
  include/linux/mmzone.h \
    $(wildcard include/config/force/max/zoneorder.h) \
    $(wildcard include/config/zone/dma32.h) \
    $(wildcard include/config/memory/hotplug.h) \
    $(wildcard include/config/compaction.h) \
    $(wildcard include/config/arch/populates/node/map.h) \
    $(wildcard include/config/flat/node/mem/map.h) \
    $(wildcard include/config/no/bootmem.h) \
    $(wildcard include/config/have/memory/present.h) \
    $(wildcard include/config/need/node/memmap/size.h) \
    $(wildcard include/config/need/multiple/nodes.h) \
    $(wildcard include/config/have/arch/early/pfn/to/nid.h) \
    $(wildcard include/config/sparsemem/extreme.h) \
    $(wildcard include/config/nodes/span/other/nodes.h) \
    $(wildcard include/config/holes/in/zone.h) \
    $(wildcard include/config/arch/has/holes/memorymodel.h) \
  include/linux/pageblock-flags.h \
    $(wildcard include/config/hugetlb/page.h) \
    $(wildcard include/config/hugetlb/page/size/variable.h) \
  include/generated/bounds.h \
  include/linux/memory_hotplug.h \
    $(wildcard include/config/have/arch/nodedata/extension.h) \
    $(wildcard include/config/memory/hotremove.h) \
  include/linux/notifier.h \
  include/linux/mutex.h \
  include/linux/srcu.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/topology.h \
  include/asm-generic/topology.h \
  include/linux/proportions.h \
  include/linux/percpu_counter.h \
  include/linux/seccomp.h \
    $(wildcard include/config/seccomp.h) \
  include/linux/rculist.h \
  include/linux/rtmutex.h \
    $(wildcard include/config/debug/rt/mutexes.h) \
  include/linux/plist.h \
    $(wildcard include/config/debug/pi/list.h) \
  include/linux/resource.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/resource.h \
  include/asm-generic/resource.h \
  include/linux/timer.h \
    $(wildcard include/config/timer/stats.h) \
    $(wildcard include/config/debug/objects/timers.h) \
  include/linux/ktime.h \
    $(wildcard include/config/ktime/scalar.h) \
  include/linux/debugobjects.h \
    $(wildcard include/config/debug/objects.h) \
    $(wildcard include/config/debug/objects/free.h) \
  include/linux/hrtimer.h \
    $(wildcard include/config/high/res/timers.h) \
  include/linux/task_io_accounting.h \
    $(wildcard include/config/task/io/accounting.h) \
  include/linux/kobject.h \
  include/linux/sysfs.h \
    $(wildcard include/config/sysfs.h) \
  include/linux/kref.h \
  include/linux/latencytop.h \
  include/linux/cred.h \
    $(wildcard include/config/debug/credentials.h) \
    $(wildcard include/config/security.h) \
  include/linux/key.h \
    $(wildcard include/config/sysctl.h) \
  include/linux/sysctl.h \
  include/linux/selinux.h \
    $(wildcard include/config/security/selinux.h) \
  include/linux/aio.h \
  include/linux/workqueue.h \
    $(wildcard include/config/debug/objects/work.h) \
  include/linux/aio_abi.h \
  include/linux/uio.h \
  include/linux/fs.h \
    $(wildcard include/config/dnotify.h) \
    $(wildcard include/config/quota.h) \
    $(wildcard include/config/fsnotify.h) \
    $(wildcard include/config/inotify.h) \
    $(wildcard include/config/fs/posix/acl.h) \
    $(wildcard include/config/debug/writecount.h) \
    $(wildcard include/config/file/locking.h) \
    $(wildcard include/config/block.h) \
    $(wildcard include/config/fs/xip.h) \
    $(wildcard include/config/migration.h) \
  include/linux/limits.h \
  include/linux/ioctl.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/ioctl.h \
  include/asm-generic/ioctl.h \
  include/linux/kdev_t.h \
  include/linux/dcache.h \
  include/linux/stat.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/stat.h \
  include/linux/radix-tree.h \
  include/linux/semaphore.h \
  include/linux/fiemap.h \
  include/linux/quota.h \
    $(wildcard include/config/quota/netlink/interface.h) \
  include/linux/dqblk_xfs.h \
  include/linux/dqblk_v1.h \
  include/linux/dqblk_v2.h \
  include/linux/dqblk_qtree.h \
  include/linux/nfs_fs_i.h \
  include/linux/nfs.h \
  include/linux/sunrpc/msg_prot.h \
  include/linux/inet.h \
  include/linux/fcntl.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/fcntl.h \
  include/asm-generic/fcntl.h \
  include/linux/err.h \
  include/linux/mount.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/pgtable.h \
    $(wildcard include/config/highpte.h) \
  include/asm-generic/4level-fixup.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/proc-fns.h \
    $(wildcard include/config/cpu/arm7tdmi.h) \
    $(wildcard include/config/cpu/arm720t.h) \
    $(wildcard include/config/cpu/arm740t.h) \
    $(wildcard include/config/cpu/arm9tdmi.h) \
    $(wildcard include/config/cpu/arm920t.h) \
    $(wildcard include/config/cpu/arm922t.h) \
    $(wildcard include/config/cpu/arm925t.h) \
    $(wildcard include/config/cpu/arm926t.h) \
    $(wildcard include/config/cpu/arm940t.h) \
    $(wildcard include/config/cpu/arm946e.h) \
    $(wildcard include/config/cpu/arm1020.h) \
    $(wildcard include/config/cpu/arm1020e.h) \
    $(wildcard include/config/cpu/arm1022.h) \
    $(wildcard include/config/cpu/arm1026.h) \
    $(wildcard include/config/cpu/mohawk.h) \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/cpu/v6.h) \
    $(wildcard include/config/cpu/v7.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/cpu-single.h \
  arch/arm/mach-bcm215xx/include/mach/vmalloc.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/pgtable-hwdef.h \
  include/asm-generic/pgtable.h \
  include/linux/interrupt.h \
    $(wildcard include/config/pm/sleep.h) \
    $(wildcard include/config/generic/irq/probe.h) \
  include/linux/irqreturn.h \
  include/linux/irqnr.h \
  include/linux/hardirq.h \
  include/linux/smp_lock.h \
    $(wildcard include/config/lock/kernel.h) \
  include/linux/ftrace_irq.h \
    $(wildcard include/config/ftrace/nmi/enter.h) \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/hardirq.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/irq.h \
  arch/arm/mach-bcm215xx/include/mach/irqs.h \
    $(wildcard include/config/dpram.h) \
    $(wildcard include/config/arch/bcm21553/b0.h) \
    $(wildcard include/config/arch/bcm21553/b1.h) \
  include/linux/version.h \
  arch/arm/mach-bcm215xx/include/mach/hardware.h \
  include/linux/irq_cpustat.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/consts.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/msconsts.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/log.h \
  arch/arm/plat-bcmap/include/plat/osdal_os.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/sharedmem.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/athena/B0/include/shared.h \
    $(wildcard include/config/fqcr/fcwr.h) \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/common/mobcom_types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/dsp/athena/B0/include/shared_ap.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/sharedmem_comms.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/dsp_feature_def.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/csl_dsp.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/bsp/public/nandsdram_memmap.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0/brcm_rdb_sysmap.h \
  arch/arm/mach-bcm215xx/include/mach/io.h \
  arch/arm/plat-bcmap/include/plat/chal/chal_intc_inc.h \
  arch/arm/plat-bcmap/include/plat/chal/chal_types.h \
  arch/arm/plat-bcmap/include/plat/chal/chal_common.h \
  arch/arm/plat-bcmap/include/plat/chal/chal_common_os.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/io.h \
  include/linux/delay.h \
  /home/mackief/Dev/Cori/kernel/kernel/arch/arm/include/asm/delay.h \
  arch/arm/plat-bcmap/include/plat/types.h \
  arch/arm/plat-bcmap/include/plat/mobcom_types.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/rdb/athena/B0/brcm_rdb_intc.h \
  ../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/public/osdw_dsp_drv.h \

../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o: $(deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o)

$(deps_../modules/../modules/drivers/sound/brcm/alsa_athena/soc/csl/dsp/src/osdw_dsp_drv.o):
