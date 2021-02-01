/*
 * The addresses to all CSR registers for all modes.
 * The names are the same as in the RISC-V specification,
 * prefixed with CSR to avoid name clashes.
 *
 * This file is meant to be included in assembly, so it doesn't
 * include C code.
 */

/***************** User mode *****************/

// User trap setup
#define CSR_ustatus				0x000
#define CSR_uie					0x004
#define CSR_utvec				0x005

// User trap handling
#define CSR_uscratch			0x040
#define CSR_uepc				0x041
#define CSR_ucause				0x042
#define CSR_utval				0x043
#define CSR_uip					0x044

// User floating-point
#define CSR_fflags				0x001
#define CSR_frm					0x002
#define CSR_fcsr				0x003

// User counters/timers
#define CSR_cycle				0xC00
#define CSR_time				0xC01
#define CSR_instret				0xC02
#define CSR_hpmcounter3			0xC03
#define CSR_hpmcounter4			0xC04
#define CSR_hpmcounter5			0xC04
#define CSR_hpmcounter6			0xC06
#define CSR_hpmcounter7			0xC07
#define CSR_hpmcounter8			0xC08
#define CSR_hpmcounter9			0xC09
#define CSR_hpmcounter10		0xC0A
#define CSR_hpmcounter11		0xC0B
#define CSR_hpmcounter12		0xC0C
#define CSR_hpmcounter13		0xC0D
#define CSR_hpmcounter14		0xC0E
#define CSR_hpmcounter15		0xC0F
#define CSR_hpmcounter16		0xC10
#define CSR_hpmcounter17		0xC11
#define CSR_hpmcounter18		0xC12
#define CSR_hpmcounter19		0xC13
#define CSR_hpmcounter20		0xC14
#define CSR_hpmcounter21		0xC15
#define CSR_hpmcounter22		0xC16
#define CSR_hpmcounter23		0xC17
#define CSR_hpmcounter24		0xC18
#define CSR_hpmcounter25		0xC19
#define CSR_hpmcounter26		0xC1A
#define CSR_hpmcounter27		0xC1B
#define CSR_hpmcounter28		0xC1C
#define CSR_hpmcounter29		0xC1D
#define CSR_hpmcounter30		0xC1E
#define CSR_hpmcounter31		0xC1F
#define CSR_cycleh				0xC80
#define CSR_timeh				0xC81
#define CSR_instreth			0xC82
#define CSR_hpmcounter3h		0xC83
#define CSR_hpmcounter4h		0xC84
#define CSR_hpmcounter5h		0xC84
#define CSR_hpmcounter6h		0xC86
#define CSR_hpmcounter7h		0xC87
#define CSR_hpmcounter8h		0xC88
#define CSR_hpmcounter9h		0xC89
#define CSR_hpmcounter10h		0xC8A
#define CSR_hpmcounter11h		0xC8B
#define CSR_hpmcounter12h		0xC8C
#define CSR_hpmcounter13h		0xC8D
#define CSR_hpmcounter14h		0xC8E
#define CSR_hpmcounter15h		0xC8F
#define CSR_hpmcounter16h		0xC90
#define CSR_hpmcounter17h		0xC91
#define CSR_hpmcounter18h		0xC92
#define CSR_hpmcounter19h		0xC93
#define CSR_hpmcounter20h		0xC94
#define CSR_hpmcounter21h		0xC95
#define CSR_hpmcounter22h		0xC96
#define CSR_hpmcounter23h		0xC97
#define CSR_hpmcounter24h		0xC98
#define CSR_hpmcounter25h		0xC99
#define CSR_hpmcounter26h		0xC9A
#define CSR_hpmcounter27h		0xC9B
#define CSR_hpmcounter28h		0xC9C
#define CSR_hpmcounter29h		0xC9D
#define CSR_hpmcounter30h		0xC9E
#define CSR_hpmcounter31h		0xC9F

/************** Supervisor mode **************/

// Supervisor trap setup
#define CSR_sstatus				0x100
#define CSR_sedeleg				0x102
#define CSR_sideleg				0x103
#define CSR_sie					0x104
#define CSR_stvec				0x105
#define CSR_scounteren			0x106

// Supervisor trap handling
#define CSR_sscratch			0x140
#define CSR_sepc				0x141
#define CSR_scause				0x142
#define CSR_stval				0x143
#define CSR_sip					0x144

// Supervisor memory protection and translation
#define CSR_satp				0x180

/**************** Machine mode ***************/

// Machine Information Registers
#define CSR_mvendorid			0xF11
#define CSR_marchid				0xF12
#define CSR_mimpid				0xF13
#define CSR_mhartid				0xF14

// Machine trap setup
#define CSR_mstatus				0x300
#define CSR_misa				0x301
#define CSR_medeleg				0x302
#define CSR_mideleg				0x303
#define CSR_mie					0x304
#define CSR_mtvec				0x305
#define CSR_mcounteren			0x306

// Machine trap handling
#define CSR_mscratch			0x340
#define CSR_mepc				0x341
#define CSR_mcause				0x342
#define CSR_mtval				0x343
#define CSR_mip					0x344

// Machine memory protection
#define CSR_pmpcfg0				0x3A0
#define CSR_pmpcfg1				0x3A1
#define CSR_pmpcfg2				0x3A2
#define CSR_pmpcfg3				0x3A3
#define CSR_pmpaddr0			0x3B0
#define CSR_pmpaddr1			0x3B1
#define CSR_pmpaddr2			0x3B2
#define CSR_pmpaddr3			0x3B3
#define CSR_pmpaddr4			0x3B4
#define CSR_pmpaddr5			0x3B5
#define CSR_pmpaddr6			0x3B6
#define CSR_pmpaddr7			0x3B7
#define CSR_pmpaddr8			0x3B8
#define CSR_pmpaddr9			0x3B9
#define CSR_pmpaddr10			0x3BA
#define CSR_pmpaddr11			0x3BB
#define CSR_pmpaddr12			0x3BC
#define CSR_pmpaddr13			0x3BD
#define CSR_pmpaddr14			0x3BE
#define CSR_pmpaddr15			0x3BF

// Machine counters/timers
#define CSR_mcycle				0xB00
#define CSR_mtime				0xB01
#define CSR_minstret			0xB02
#define CSR_mhpmcounter3		0xB03
#define CSR_mhpmcounter4		0xB04
#define CSR_mhpmcounter5		0xB04
#define CSR_mhpmcounter6		0xB06
#define CSR_mhpmcounter7		0xB07
#define CSR_mhpmcounter8		0xB08
#define CSR_mhpmcounter9		0xB09
#define CSR_mhpmcounter10		0xB0A
#define CSR_mhpmcounter11		0xB0B
#define CSR_mhpmcounter12		0xB0C
#define CSR_mhpmcounter13		0xB0D
#define CSR_mhpmcounter14		0xB0E
#define CSR_mhpmcounter15		0xB0F
#define CSR_mhpmcounter16		0xB10
#define CSR_mhpmcounter17		0xB11
#define CSR_mhpmcounter18		0xB12
#define CSR_mhpmcounter19		0xB13
#define CSR_mhpmcounter20		0xB14
#define CSR_mhpmcounter21		0xB15
#define CSR_mhpmcounter22		0xB16
#define CSR_mhpmcounter23		0xB17
#define CSR_mhpmcounter24		0xB18
#define CSR_mhpmcounter25		0xB19
#define CSR_mhpmcounter26		0xB1A
#define CSR_mhpmcounter27		0xB1B
#define CSR_mhpmcounter28		0xB1C
#define CSR_mhpmcounter29		0xB1D
#define CSR_mhpmcounter30		0xB1E
#define CSR_mhpmcounter31		0xB1F
#define CSR_mcycleh				0xB80
#define CSR_mtimeh				0xB81
#define CSR_minstreth			0xB82
#define CSR_mhpmcounter3h		0xB83
#define CSR_mhpmcounter4h		0xB84
#define CSR_mhpmcounter5h		0xB84
#define CSR_mhpmcounter6h		0xB86
#define CSR_mhpmcounter7h		0xB87
#define CSR_mhpmcounter8h		0xB88
#define CSR_mhpmcounter9h		0xB89
#define CSR_mhpmcounter10h		0xB8A
#define CSR_mhpmcounter11h		0xB8B
#define CSR_mhpmcounter12h		0xB8C
#define CSR_mhpmcounter13h		0xB8D
#define CSR_mhpmcounter14h		0xB8E
#define CSR_mhpmcounter15h		0xB8F
#define CSR_mhpmcounter16h		0xB90
#define CSR_mhpmcounter17h		0xB91
#define CSR_mhpmcounter18h		0xB92
#define CSR_mhpmcounter19h		0xB93
#define CSR_mhpmcounter20h		0xB94
#define CSR_mhpmcounter21h		0xB95
#define CSR_mhpmcounter22h		0xB96
#define CSR_mhpmcounter23h		0xB97
#define CSR_mhpmcounter24h		0xB98
#define CSR_mhpmcounter25h		0xB99
#define CSR_mhpmcounter26h		0xB9A
#define CSR_mhpmcounter27h		0xB9B
#define CSR_mhpmcounter28h		0xB9C
#define CSR_mhpmcounter29h		0xB9D
#define CSR_mhpmcounter30h		0xB9E
#define CSR_mhpmcounter31h		0xB9F

// Machine counter setup
#define CSR_mcountinhibit		0x320
#define CSR_mhpmevent3			0x323
#define CSR_mhpmevent4			0x324
#define CSR_mhpmevent5			0x324
#define CSR_mhpmevent6			0x326
#define CSR_mhpmevent7			0x327
#define CSR_mhpmevent8			0x328
#define CSR_mhpmevent9			0x329
#define CSR_mhpmevent10			0x32A
#define CSR_mhpmevent11			0x32B
#define CSR_mhpmevent12			0x32C
#define CSR_mhpmevent13			0x32D
#define CSR_mhpmevent14			0x32E
#define CSR_mhpmevent15			0x32F
#define CSR_mhpmevent16			0x330
#define CSR_mhpmevent17			0x331
#define CSR_mhpmevent18			0x332
#define CSR_mhpmevent19			0x333
#define CSR_mhpmevent20			0x334
#define CSR_mhpmevent21			0x335
#define CSR_mhpmevent22			0x336
#define CSR_mhpmevent23			0x337
#define CSR_mhpmevent24			0x338
#define CSR_mhpmevent25			0x339
#define CSR_mhpmevent26			0x33A
#define CSR_mhpmevent27			0x33B
#define CSR_mhpmevent28			0x33C
#define CSR_mhpmevent29			0x33D
#define CSR_mhpmevent30			0x33E
#define CSR_mhpmevent31			0x33F

// Debug/trace registers
#define CSR_tselect				0x7A0
#define CSR_tdata1				0x7A1
#define CSR_tdata2				0x7A2
#define CSR_tdata3				0x7A3

// Debug mode registers
#define CSR_dcsr				0x7B0
#define CSR_dpc					0x7B1
#define CSR_dscratch0			0x7B2
#define CSR_dscratch1			0x7B3
