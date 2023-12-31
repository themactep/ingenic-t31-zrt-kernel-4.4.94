#include <asm/asm.h>
#include <asm/regdef.h>

#define CP0_PAGEMASK	$5,0
#define CP0_TLB_SPEC	$5,4
#define CP0_STATUS	$12,0
#define CP0_INTCTL	$12,1
#define CP0_CAUSE	$13,0
#define CP0_EBASE	$15,1
#define CP0_CONFIG	$16,0
#define CP0_CONFIG1	$16,1
#define CP0_CONFIG2	$16,2
#define CP0_CONFIG3	$16,3
#define CP0_CONFIG4	$16,4
#define CP0_CONFIG5	$16,5
#define CP0_CONFIG7	$16,7
#define CP0_LLADDR	$17,0
#define PMON_CSR	$17,7
#define PMON_HIGH	$17,4
#define PMON_LC	$17,5
#define PMON_RC	$17,6
#define CP0_WATCHLo	$18,0
#define CP0_WATCHHI	$19,0
#define CP0_ERRCTL	$26,0
#define CP0_CONTEXT	$4,0

	.data
	.global _regs_stack
_regs_stack:
	.align 5
	.space 272,0

	.text
	.global save_goto
	.align 2
	.ent save_goto,0
save_goto:
	.set	push
	.set	noreorder
	.set	noat

	la	k0, _regs_stack
	sllv	k0, k0, a2
	sw	s0,0(k0)
	sw	s1,4(k0)
	sw	s2,8(k0)
	sw	s3,12(k0)
	sw	s4,16(k0)
	sw	s5,20(k0)
	sw	s6,24(k0)
	sw	s7,28(k0)
	sw	gp,32(k0)
	sw	sp,36(k0)
	sw	fp,40(k0)
	sw	ra,44(k0)

	mfc0	k1,CP0_PAGEMASK
	sw	k1,48(k0)
	mfc0	k1,CP0_TLB_SPEC
	sw	k1,52(k0)
	mfc0	k1,CP0_STATUS
	sw	k1,56(k0)
	mfc0	k1,CP0_INTCTL
	sw	k1,60(k0)
	mfc0	k1,CP0_CAUSE
	sw	k1,64(k0)
	mfc0	k1,CP0_EBASE
	sw	k1,68(k0)
	mfc0	k1,CP0_CONFIG
	sw	k1,72(k0)
	mfc0	k1,CP0_CONFIG1
	sw	k1,76(k0)
	mfc0	k1,CP0_CONFIG2
	sw	k1,80(k0)
	mfc0	k1,CP0_CONFIG3
	sw	k1,84(k0)
	mfc0	k1,CP0_CONFIG4
	sw	k1,88(k0)
	mfc0	k1,CP0_CONFIG5
	sw	k1,92(k0)
	mfc0	k1,CP0_CONFIG7
	sw	k1,96(k0)
	mfc0	k1,CP0_LLADDR
	sw	k1,100(k0)
	mfc0	k1,PMON_CSR
	sw	k1,104(k0)
	mfc0	k1,PMON_HIGH
	sw	k1,108(k0)
	mfc0	k1,PMON_LC
	sw	k1,112(k0)
	mfc0	k1,PMON_RC
	sw	k1,116(k0)
	mfc0	k1,CP0_WATCHLo
	sw	k1,120(k0)
	mfc0	k1,CP0_WATCHHI
	sw	k1,124(k0)
	mfc0	k1,CP0_ERRCTL
	sw	k1,128(k0)
	mfc0	k1,CP0_CONTEXT
	sw	k1,132(k0)

	move	t0, a0
	move	a0, a1
	jr.hb	t0
	nop

	.set	at
	.set	reorder
	.set	pop
	END(save_goto)
	.text
	.global restore_goto
	.align 2
	.ent restore_goto,0
restore_goto:
	.set	push
	.set	noreorder
	.set	noat

	la	k0, _regs_stack
	sllv	k0, k0, a2
	lw	s0,0(k0)
	lw	s1,4(k0)
	lw	s2,8(k0)
	lw	s3,12(k0)
	lw	s4,16(k0)
	lw	s5,20(k0)
	lw	s6,24(k0)
	lw	s7,28(k0)
	lw	gp,32(k0)
	lw	sp,36(k0)
	lw	fp,40(k0)
	lw	ra,44(k0)

	lw	k1,48(k0)
	mtc0	k1,CP0_PAGEMASK
	lw	k1,52(k0)
	mtc0	k1,CP0_TLB_SPEC
	lw	k1,56(k0)
	mtc0	k1,CP0_STATUS
	lw	k1,60(k0)
	mtc0	k1,CP0_INTCTL
	lw	k1,64(k0)
	mtc0	k1,CP0_CAUSE
	lw	k1,68(k0)
	mtc0	k1,CP0_EBASE
	lw	k1,72(k0)
	mtc0	k1,CP0_CONFIG
	lw	k1,76(k0)
	mtc0	k1,CP0_CONFIG1
	lw	k1,80(k0)
	mtc0	k1,CP0_CONFIG2
	lw	k1,84(k0)
	mtc0	k1,CP0_CONFIG3
	lw	k1,88(k0)
	mtc0	k1,CP0_CONFIG4
	lw	k1,92(k0)
	mtc0	k1,CP0_CONFIG5
	lw	k1,96(k0)
	mtc0	k1,CP0_CONFIG7
	lw	k1,100(k0)
	mtc0	k1,CP0_LLADDR
	lw	k1,104(k0)
	mtc0	k1,PMON_CSR
	lw	k1,108(k0)
	mtc0	k1,PMON_HIGH
	lw	k1,112(k0)
	mtc0	k1,PMON_LC
	lw	k1,116(k0)
	mtc0	k1,PMON_RC
	lw	k1,120(k0)
	mtc0	k1,CP0_WATCHLo
	lw	k1,124(k0)
	mtc0	k1,CP0_WATCHHI
	lw	k1,128(k0)
	mtc0	k1,CP0_ERRCTL
	lw	k1,132(k0)
	mtc0	k1,CP0_CONTEXT

	jr.hb	ra
	nop

	.set	at
	.set	reorder
	.set	pop
	END(restore_goto)
