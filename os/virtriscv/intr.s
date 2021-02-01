#include "mem.h"
#include "csrregs.h"

// The mask used to enable/disable interrupts in sstatus
#define INTR_MASK 2

TEXT splhi(SB), $-4
	// Save caller PC, disable interrupts, return previous interrupt enable state
	MOVW $(MACHADDR), R4
	MOVW R1, 0(R4)

	CSRRC CSR(CSR_sstatus), $2, R8
	AND $INTR_MASK, R8, R8
	RET

TEXT spllo(SB), $-4
	// Enable interrupts and return previous interrupt enable state
	CSRRS CSR(CSR_sstatus), $2, R8
	AND $INTR_MASK, R8, R8
	RET

TEXT splx(SB), $-4
	// Save PC
	MOVW $(MACHADDR), R4
	MOVW R1, 0(R4)
TEXT splxpc(SB), $-4
	// Restore interrupt state to R8
	CSRRC CSR(CSR_sstatus), $INTR_MASK, R0 // First clear
	CSRRS CSR(CSR_sstatus), R8, R8 // Then set using R8 as a mask
	RET

TEXT islo(SB), $-4
	// Return non-zero if interrupts are enabled
	MOVW CSR(CSR_sstatus), R8
	AND $INTR_MASK, R8, R8
	RET

#define UREGSIZE		(152)
#define INTREGSIZE		4
#define UREGSTART		8
#define INTREGSTART		(INTREGSIZE)
#define UREGADDR(x)		(UREGSTART + x)(R2)
#define INTREG(x)		UREGADDR(INTREGSTART + (x-1)*INTREGSIZE)
#define UREGCSR(x)		INTREG(x+32)
#define TRAPSTACKSIZE		(UREGSIZE + UREGSTART)
TEXT traphandler(SB), 1, $-4
	// Ignore the current stack, load the trap stack from sscratch instead
	// The stack pointer is temporarily stored in sscratch
	CSRRW CSR(CSR_sscratch), R2, R2
	ADD $-TRAPSTACKSIZE, R2

	// Save registers
	MOVW R1, INTREG(1)
	MOVW R3, INTREG(3)
	MOVW R4, INTREG(4)
	MOVW R5, INTREG(5)
	MOVW R6, INTREG(6)
	MOVW R7, INTREG(7)
	MOVW R8, INTREG(8)
	MOVW R9, INTREG(9)
	MOVW R10, INTREG(10)
	MOVW R11, INTREG(11)
	MOVW R12, INTREG(12)
	MOVW R13, INTREG(13)
	MOVW R14, INTREG(14)
	MOVW R15, INTREG(15)
	MOVW R16, INTREG(16)
	MOVW R17, INTREG(17)
	MOVW R18, INTREG(18)
	MOVW R19, INTREG(19)
	MOVW R20, INTREG(20)
	MOVW R21, INTREG(21)
	MOVW R22, INTREG(22)
	MOVW R23, INTREG(23)
	MOVW R24, INTREG(24)
	MOVW R25, INTREG(25)
	MOVW R26, INTREG(26)
	MOVW R27, INTREG(27)
	MOVW R28, INTREG(28)
	MOVW R29, INTREG(29)
	MOVW R30, INTREG(30)
	MOVW R31, INTREG(31)

	// Save the new trap stack pointer to CSR,
	// and retrieve the program stack pointer
	// to R31, then save it to the Ureg.
	CSRRW CSR(CSR_sscratch), R2, R31
	MOVW R31, INTREG(2)

	// Copy CSRs to the Ureg
	CSRRC CSR(CSR_sstatus), R0, R10
	MOVW R10, UREGCSR(0)

	CSRRC CSR(CSR_sie), R0, R10
	MOVW R10, UREGCSR(1)

	CSRRC CSR(CSR_scause), R0, R10
	MOVW R10, UREGCSR(2)

	CSRRC CSR(CSR_stval), R0, R10
	MOVW R10, UREGCSR(3)

	// Assume we are in supervisor mode
	MOVW $2, R10
	MOVW R10, UREGCSR(4)

	// Copy the address of the instruction where the trap
	// occured to ureg->pc
	CSRRC CSR(CSR_sepc), R0, R10
	MOVW R10, UREGADDR(0)

	// Call the C trap handler with the Ureg
	MOVW R2, R8
	ADD $UREGSTART, R8
	JAL R1, kerneltrap(SB)

	// Get the stack pointer and temporarily save it to sscratch
	MOVW INTREG(2), R31
	CSRRW CSR(CSR_sscratch), R31, R0

	// Restore registers
	MOVW INTREG(1), R1
	MOVW INTREG(3), R3
	MOVW INTREG(4), R4
	MOVW INTREG(5), R5
	MOVW INTREG(6), R6
	MOVW INTREG(7), R7
	MOVW INTREG(8), R8
	MOVW INTREG(9), R9
	MOVW INTREG(10), R10
	MOVW INTREG(11), R11
	MOVW INTREG(12), R12
	MOVW INTREG(13), R13
	MOVW INTREG(14), R14
	MOVW INTREG(15), R15
	MOVW INTREG(16), R16
	MOVW INTREG(17), R17
	MOVW INTREG(18), R18
	MOVW INTREG(19), R19
	MOVW INTREG(20), R20
	MOVW INTREG(21), R21
	MOVW INTREG(22), R22
	MOVW INTREG(23), R23
	MOVW INTREG(24), R24
	MOVW INTREG(25), R25
	MOVW INTREG(26), R26
	MOVW INTREG(27), R27
	MOVW INTREG(28), R28
	MOVW INTREG(29), R29
	MOVW INTREG(30), R30
	MOVW INTREG(31), R31

	// Reset the trap stack pointer, and swap it with the program stack pointer in sscratch
	ADD $TRAPSTACKSIZE, R2
	CSRRW CSR(CSR_sscratch), R2, R2

	// Return from the trap
	WORD $270532723 // SRET
