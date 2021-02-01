#include "mem.h"
#include "csrregs.h"

#define ARG 8

TEXT _start(SB), $-4
	// set static base
	MOVW $setSB(SB), R3

	// set stack pointer
	MOVW $(MACHADDR+MACHSIZE-4), R2

	// set link addr on stack to 0, so debuggers know to stop
	ADD $-4, R2
	MOVW R0, 0(R2)

	// call main
	JAL R1, main(SB)
deadloop:
	ADD R0, R0, R0
	JMP deadloop
	RET

	// hack to load qsort, fmtprint and defont, so the linker doesn't try to optimize them away
	JAL R1, qsort(SB)
	JAL R1, fmtprint(SB)
	JAL R1, _unpackinfo(SB)

TEXT memory_fence(SB), 0, $-4
	WORD $267386895 // Normal FENCE PI PO PR PW SI SO SR SW
	RET

TEXT sbi_ecall(SB), 1, $-4
	// The address of the struct to populate is in R8, the rest are on stack
	MOVW R(ARG), .ret+0(FP)

	// Move the arguments and perform the call
	MOVW a0+4(FP), R10
	MOVW a1+8(FP), R11
	MOVW a3+12(FP), R12
	MOVW num+16(FP), R17
	MOVW num+20(FP), R16
	ECALL

	// Move the result from R10 and R11 to the result struct
	MOVW .ret+0(FP), R(ARG)
	MOVW R10, 0(R(ARG))
	MOVW R11, 4(R(ARG))
	RET

TEXT setlabel(SB), $-4
	MOVW R2, 0(R8)	// Save the stack pointer to the first field
	MOVW R1, 4(R8)	// Save the link register (calling address) to the second field
	MOVW $0, R(ARG)	// Return 0
	RET

TEXT gotolabel(SB), $-4
	MOVW 0(R8), R2
	MOVW 4(R8), R1
	MOVW $1, R(ARG)
	RET

TEXT getcallerpc(SB), $-4
	MOVW 0(SP), R(ARG)
	RET

TEXT getsp(SB), $-4
	MOVW R2, R8
	RET

/* Test-And-Set */
TEXT _tas(SB), $-4
	MOVW $1, R9
	WORD $244589615	// AMOSWAP.W 1 1 R9 R8 R8
	RET

TEXT atomic_inc(SB), $-4
	MOVW $1, R9
	WORD $110371887 // AMOADD.W 1 1 R9 R8 R8
	RET

TEXT wait_for_interrupt(SB), $-4
	WORD $273678451 // WFI
	RET
