#include "lib9.h"
#include "isa.h"
#include "interp.h"
#include "raise.h"

/*
 * JIT compiler to RISC-V.
 * Assumes that processor supports at least rv32mfd.
 *
 * Note that the operand order is different than the JIT compilers
 * for other architectures, both for instructions and functions.
 * The general order is rd, rs, imm. The exception is store instructions,
 * which goes against the instruction operand ordering by having the source
 * register first.
 */

enum {
	R0	= 0,
	R1	= 1,
	R2	= 2,
	R3	= 3,
	R4	= 4,
	R5	= 5,
	R6	= 6,
	R7	= 7,
	R8	= 8,
	R9	= 9,
	R10	= 10,
	R11	= 11,
	R12	= 12,
	R13	= 13,
	R14	= 14,
	R15	= 15,

	Rlink	= 1,
	Rsp	= 2,
	Rarg	= 8,

	// Temporary registers
	Rtmp	= 4, // Used for building constants and other single-instruction values
	Rta	= 5, // Used for intermediate addresses for double indirect

	// Permanent registers
	Rh	= 6, // Contains H, which is used to check if values are invalid

	// Registers for storing arguments and other mid-term values
	RA0	= 8,
	RA1	= 9,
	RA2	= 10,
	RA3	= 11,
	RA4	= 12,

	Rfp	= 13, // Frame pointer
	Rmp	= 14, // Module pointer
	Rreg	= 15, // Pointer to the REG struct

	// Floating-point registers
	F0	= 0,
	F1	= 1,
	F2	= 2,
	F3	= 3,
	F4	= 4,
	F5	= 5,
	F6	= 6,

	// Opcodes
	OP		= 51,		// 0b0110011
	OPimm		= 19,		// 0b0010011
	OPfp		= 83,		// 0b1010011
	OPlui		= 55,		// 0b0110111
	OPauipc		= 23,		// 0b0010111
	OPjal		= 111,		// 0b1101111
	OPjalr		= 103,		// 0b1100111
	OPbranch	= 99,		// 0b1100011
	OPload		= 3,		// 0b0000011
	OPloadfp	= 7,		// 0b0000111
	OPstore		= 35,		// 0b0100011
	OPstorefp	= 39,		// 0b0100111
	OPmiscmem	= 15,		// 0b0001111
	OPsystem	= 115,		// 0b1110011
	OPamo		= 47,		// 0b0101111
	OPmadd		= 67,		// 0b1000011
	OPnmadd		= 79,		// 0b1001111
	OPmsub		= 71,		// 0b1000111
	OPnmsub		= 75,		// 0b1001011

	// Rounding modes
	RNE		= 0, // Round to nearest, ties to even
	RTZ		= 1, // Round towards zero
	RDN		= 2, // Round down
	RUP		= 3, // Round up
	RMM		= 4, // Round to nearest, ties to max magnitude
	RDYN		= 7, // Use default

	RM		= RDYN, // Default rounding mode

	// Flags to mem
	Ldw = 1,	// Load 32-bit word
	Ldh,		// Load 16-bit half-word (with sign-extension)
	Ldb,		// Load 8-bit byte (with sign-extension)
	Ldhu,		// Load 16-bit unsiged half-word
	Ldbu,		// Load 8-bit unsigned byte
	Lds,		// Load 32-bit single-precision float
	Ldd,		// Load 64-bit double-precision float

	Stw,		// Store 32-bit word
	Sth,		// Store 16-bit half-word
	Stb,		// Store 8-bit byte
	Sts,		// Store 32-bit single-precision float
	Std,		// Store 64-bit double-precision float

	Laddr,		// Special flag for operand functions
			// Moves the address of the operand to a register

	// Flags to branch
	EQ = 1,
	NE,
	LT,
	LE,
	GT,
	GE,

	// Flags to punt
	SRCOP	= (1<<0),
	DSTOP	= (1<<1),
	WRTPC	= (1<<2),
	TCHECK	= (1<<3),
	NEWPC	= (1<<4),
	DBRAN	= (1<<5),
	THREOP	= (1<<6),

	// The index of each macro
	MacFRP	= 0,
	MacRET,
	MacCASE,
	MacCOLR,
	MacMCAL,
	MacFRAM,
	MacMFRA,
	MacRELQ,
	NMACRO
};

// Masks for the high and low portions of immidiate values
#define IMMSIGNED	0xFFFFF800
#define IMMH		0xFFFFF000
#define IMML		0x00000FFF

// Check if a immidiate has to be split over multiple instructions
#define SPLITIMM(imm)					((((((ulong)(imm)) & IMMSIGNED) != 0)) && ((((ulong)(imm)) & IMMSIGNED) != IMMSIGNED))

#define SPLITH(imm)					((((ulong)(imm)) + ((((ulong)(imm)) & (1<<11)) ? (1<<12) : 0)) & IMMH)
#define SPLITL(imm)					(((ulong)(imm)) & IMML)

// Extract bits of immidiate values, like imm[11:5]. Basically shifts to the right and masks
// Examples:
// imm[11:5] -> IMM(imm, 11, 5)
// imm[11:0] -> IMM(imm, 11, 0)
// imm[11]   -> IMM(imm, 11, 11)
#define IMM(imm, to, from)				((((ulong)(imm)) >> (from)) & ((1 << ((to)-(from)+1)) - 1))

// All RISC-V instruction encoding variants. Set up with LSB on the left and MSB on the right, opposite to the tables in the RISC-V specification

#define Iimm(imm)					(IMM(imm, 11, 0)<<20)
#define Simm(imm)					((IMM(imm, 4, 0)<<7) | (IMM(imm, 11, 5)<<25))
#define Bimm(imm)					((IMM(imm, 11, 11)<<7) | (IMM(imm, 4, 1)<<8) | (IMM(imm, 10, 5)<<25) | (IMM(imm, 12, 12)<<30))
#define Uimm(imm)					((IMM(imm, 31, 12)<<12))
#define Jimm(imm)					((IMM(imm, 19, 12)<<12) | (IMM(imm, 11, 11)<<20) | (IMM(imm, 10, 1)<<21) | (IMM(imm, 20, 20)<<30))

#define Rtype(op, funct3, funct7, rd, rs1, rs2)		gen((op) | ((rd)<<7) | ((funct3)<<12) | ((rs1)<<15) | ((rs2)<<20) | ((funct7)<<25))
#define R4type(op, funct3, funct2, rd, rs1, rs2, rs3)	gen((op) | ((rd)<<7) | ((funct3)<<12) | ((rs1)<<15) | ((rs2)<<20) | ((funct2)<<25) | ((rs3)<<27))
#define Itype(op, funct3, rd, rs1, imm)			gen((op) | ((rd)<<7) | ((funct3)<<12) | ((rs1)<<15) | Iimm(imm))
#define Stype(op, funct3, rs1, rs2, imm)		gen((op) | ((funct3)<<12) | ((rs1)<<15) | ((rs2)<<20) | Simm(imm))
#define Btype(op, funct3, rs1, rs2, imm)		gen((op) | ((funct3)<<12) | ((rs1)<<15) | ((rs2)<<20) | Bimm(imm))
#define Utype(op, rd, imm)				gen((op) | ((rd)<<7) | Uimm(imm))
#define Jtype(op, rd, imm)				gen((op) | ((rd)<<7) | Jimm(imm))

/* Macros for laying down RISC-V instructions. Uses the instruction name from the specification */

// Upper immediate instructions
#define LUI(dest, imm)					Utype(OPlui, dest, imm)
#define AUIPC(dest, imm)				Utype(OPauipc, dest, imm)

// Jump instructions
#define JAL(dest, offset)				Jtype(OPjal, dest, offset)
#define JALR(dest, base, offset)			Itype(OPjalr, 0, dest, base, offset)

// Branch instructions
#define BEQ(src1, src2, offset)				Btype(OPbranch, 0, src1, src2, offset)
#define BNE(src1, src2, offset)				Btype(OPbranch, 1, src1, src2, offset)
#define BLT(src1, src2, offset)				Btype(OPbranch, 4, src1, src2, offset)
#define BGE(src1, src2, offset)				Btype(OPbranch, 5, src1, src2, offset)
#define BLTU(src1, src2, offset)			Btype(OPbranch, 6, src1, src2, offset)
#define BGEU(src1, src2, offset)			Btype(OPbranch, 7, src1, src2, offset)

// Load instructions
#define LB(dest, base, imm)				Itype(OPload, 0, dest, base, imm)
#define LH(dest, base, imm)				Itype(OPload, 1, dest, base, imm)
#define LW(dest, base, imm)				Itype(OPload, 2, dest, base, imm)
#define LBU(dest, base, imm)				Itype(OPload, 4, dest, base, imm)
#define LHU(dest, base, imm)				Itype(OPload, 5, dest, base, imm)

// Store instructions
#define SB(src, base, imm)				Stype(OPstore, 0, base, src, imm)
#define SH(src, base, imm)				Stype(OPstore, 1, base, src, imm)
#define SW(src, base, imm)				Stype(OPstore, 2, base, src, imm)

// Arithmetic immediate instructions
#define ADDI(dest, src, imm)				Itype(OPimm, 0, dest, src, imm)
#define SLTI(dest, src, imm)				Itype(OPimm, 2, dest, src, imm)
#define SLTIU(dest, src, imm)				Itype(OPimm, 3, dest, src, imm)
#define XORI(dest, src, imm)				Itype(OPimm, 4, dest, src, imm)
#define ORI(dest, src, imm)				Itype(OPimm, 6, dest, src, imm)
#define ANDI(dest, src, imm)				Itype(OPimm, 7, dest, src, imm)

#define SLLI(dest, src, shamt)				Itype(OPimm, 1, dest, src, shamt)
#define SRLI(dest, src, shamt)				Itype(OPimm, 5, dest, src, shamt)
#define SRAI(dest, src, shamt)				Itype(OPimm, 5, dest, src, shamt | (1<<10))

// Arithmetic register instructions
#define ADD(dest, src1, src2)				Rtype(OP, 0, 0, dest, src1, src2)
#define SUB(dest, src1, src2)				Rtype(OP, 0, (1<<5), dest, src1, src2)
#define SLL(dest, src1, src2)				Rtype(OP, 1, 0, dest, src1, src2)
#define SLT(dest, src1, src2)				Rtype(OP, 2, 0, dest, src1, src2)
#define SLTU(dest, src1, src2)				Rtype(OP, 3, 0, dest, src1, src2)
#define XOR(dest, src1, src2)				Rtype(OP, 4, 0, dest, src1, src2)
#define SRL(dest, src1, src2)				Rtype(OP, 5, 0, dest, src1, src2)
#define SRA(dest, src1, src2)				Rtype(OP, 5, (1<<5), dest, src1, src2)
#define OR(dest, src1, src2)				Rtype(OP, 6, 0, dest, src1, src2)
#define AND(dest, src1, src2)				Rtype(OP, 7, 0, dest, src1, src2)

// The M extension for multiplication and division
#define MUL(dest, src1, src2)				Rtype(OP, 0, 1, dest, src1, src2)
#define MULH(dest, src1, src2)				Rtype(OP, 1, 1, dest, src1, src2)
#define MULHSU(dest, src1, src2)			Rtype(OP, 2, 1, dest, src1, src2)
#define MULHU(dest, src1, src2)				Rtype(OP, 3, 1, dest, src1, src2)
#define DIV(dest, src1, src2)				Rtype(OP, 4, 1, dest, src1, src2)
#define DIVU(dest, src1, src2)				Rtype(OP, 5, 1, dest, src1, src2)
#define REM(dest, src1, src2)				Rtype(OP, 6, 1, dest, src1, src2)
#define REMU(dest, src1, src2)				Rtype(OP, 7, 1, dest, src1, src2)

// The F extension for single-precision floating-point. rm is the rounding mode
#define FLW(dest, base, offset)				Itype(OPloadfp, 2, dest, base, offset)
#define FSW(src, base, offset)				Stype(OPstorefp, 2, base, src, offset)

#define FMADDS(rm, dest, src1, src2, src3)		R4type(OPmadd, rm, 0, dest, src1, src2, src3)
#define FMSUBS(rm, dest, src1, src2, src3)		R4type(OPmsub, rm, 0, dest, src1, src2, src3)
#define FNMADDS(rm, dest, src1, src2, src3)		R4type(OPnmadd, rm, 0, dest, src1, src2, src3)
#define FNMSUBS(rm, dest, src1, src2, src3)		R4type(OPnmsub, rm, 0, dest, src1, src2, src3)

#define FADDS(rm, dest, src1, src2)			Rtype(OPfp, rm, 0, dest, src1, src2)
#define FSUBS(rm, dest, src1, src2)			Rtype(OPfp, rm, 1<<2, dest, src1, src2)
#define FMULS(rm, dest, src1, src2)			Rtype(OPfp, rm, 1<<3, dest, src1, src2)
#define FDIVS(rm, dest, src1, src2)			Rtype(OPfp, rm, 3<<2, dest, src1, src2)
#define FSQRTS(rm, dest, src)				Rtype(OPfp, rm, 11<<2, dest, src, 0)

#define FSGNJS(dest, src1, src2)			Rtype(OPfp,  0, 1<<4, dest, src1, src2)
#define FSGNJNS(dest, src1, src2)			Rtype(OPfp,  1, 1<<4, dest, src1, src2)
#define FSGNJXS(dest, src1, src2)			Rtype(OPfp,  2, 1<<4, dest, src1, src2)

#define FMINS(dest, src1, src2)				Rtype(OPfp,  0, 5<<2, dest, src1, src2)
#define FMAXS(dest, src1, src2)				Rtype(OPfp,  1, 5<<2, dest, src1, src2)

#define FMVXW(dest, src)				Rtype(OPfp,  0, 7<<4, dest, src, 0)
#define FMVWX(rm, dest, src)				Rtype(OPfp,  0, 15<<3, dest, src, 0)

#define FEQS(dest, src1, src2)				Rtype(OPfp,  2, 5<<4, dest, src1, src2)
#define FLTS(dest, src1, src2)				Rtype(OPfp,  1, 5<<4, dest, src1, src2)
#define FLES(dest, src1, src2)				Rtype(OPfp,  0, 5<<4, dest, src1, src2)

#define FCLASSS(dest, src)				Rtype(OPfp,  1, 7<<4, dest, src, 0)

#define FCVTWS(rm, dest, src)				Rtype(OPfp, rm, 3<<5, dest, src, 0)
#define FCVTWUS(rm, dest, src)				Rtype(OPfp, rm, 3<<5, dest, src, 1)
#define FCVTSW(rm, dest, src)				Rtype(OPfp, rm, 13<<3, dest, src, 0)
#define FCVTSWU(rm, dest, src)				Rtype(OPfp, rm, 13<<3, dest, src, 1)

// The D extension for double-precision floating-point
#define FLD(dest, base, offset)				Itype(OPloadfp, 3, dest, base, offset)
#define FSD(src, base, offset)				Stype(OPstorefp, 3, base, src, offset)

#define FMADDD(rm, dest, src1, src2, src3)		R4type(OPmadd, rm, 1, dest, src1, src2, src3)
#define FMSUBD(rm, dest, src1, src2, src3)		R4type(OPmsub, rm, 1, dest, src1, src2, src3)
#define FNMADDD(rm, dest, src1, src2, src3)		R4type(OPnmadd, rm, 1, dest, src1, src2, src3)
#define FNMSUBD(rm, dest, src1, src2, src3)		R4type(OPnmsub, rm, 1, dest, src1, src2, src3)

#define FADDD(rm, dest, src1, src2)			Rtype(OPfp, rm, 1, dest, src1, src2)
#define FSUBD(rm, dest, src1, src2)			Rtype(OPfp, rm, 5, dest, src1, src2)
#define FMULD(rm, dest, src1, src2)			Rtype(OPfp, rm, 9, dest, src1, src2)
#define FDIVD(rm, dest, src1, src2)			Rtype(OPfp, rm, 13, dest, src1, src2)
#define FSQRTD(rm, dest, src)				Rtype(OPfp, rm, 45, dest, src, 0)

#define FSGNJD(dest, src1, src2)			Rtype(OPfp,  0, 17, dest, src1, src2)
#define FSGNJND(dest, src1, src2)			Rtype(OPfp,  1, 17, dest, src1, src2)
#define FSGNJXD(dest, src1, src2)			Rtype(OPfp,  2, 17, dest, src1, src2)

#define FMIND(dest, src1, src2)				Rtype(OPfp,  0, 21, dest, src1, src2)
#define FMAXD(dest, src1, src2)				Rtype(OPfp,  1, 21, dest, src1, src2)

#define FEQD(dest, src1, src2)				Rtype(OPfp,  2, 81, dest, src1, src2)
#define FLTD(dest, src1, src2)				Rtype(OPfp,  1, 81, dest, src1, src2)
#define FLED(dest, src1, src2)				Rtype(OPfp,  0, 81, dest, src1, src2)

#define FCLASSD(dest, src)				Rtype(OPfp,  1, 113, dest, src, 0)

#define FCVTSD(rm, dest, src)				Rtype(OPfp, rm, 32, dest, src, 1)
#define FCVTDS(rm, dest, src)				Rtype(OPfp, rm, 32, dest, src, 0)
#define FCVTWD(rm, dest, src)				Rtype(OPfp, rm, 97, dest, src, 0)
#define FCVTWUD(rm, dest, src)				Rtype(OPfp, rm, 97, dest, src, 1)
#define FCVTDW(rm, dest, src)				Rtype(OPfp, rm, 105, dest, src, 0)
#define FCVTDWU(rm, dest, src)				Rtype(OPfp, rm, 105, dest, src, 1)

// Pseudoinstructions
#define MOV(rd, rs)					ADDI(rd, rs, 0)
#define NOT(rd, rs)					XORI(rd, rs, -1)
#define NEG(rd, rs)					SUB(rd, R0, rs)

#define BEQZ(rs, offset)				BEQ(rs, R0, offset)
#define BNEZ(rs, offset)				BNE(rs, R0, offset)
#define BLEZ(rs, offset)				BGE(R0, rs, offset)
#define BGEZ(rs, offset)				BGE(rs, R0, offset)
#define BLTZ(rs, offset)				BLT(rs, R0, offset)
#define BGTZ(rs, offset)				BLT(R0, rs, offset)

#define BGT(rs1, rs2, offset)				BLT(rs2, rs1, offset)
#define BLE(rs1, rs2, offset)				BGE(rs2, rs1, offset)
#define BGTU(rs1, rs2, offset)				BLTU(rs2, rs1, offset)
#define BLEU(rs1, rs2, offset)				BGEU(rs2, rs1, offset)

#define JUMP(offset)					JAL(R0, offset)
#define JL(offset)					JAL(R1, offset)
#define JR(rs, offset)					JALR(R0, rs, offset)
#define JRL(rs, offset)					JALR(R1, rs, offset)

/* Helper macros */

// Used to look up the address of an array element relative to base
#define IA(s, o)			(ulong)(base+s[o])

// The offset from the current code address to the pointer
#define OFF(ptr)			((ulong)(ptr) - (ulong) (code))

// Call a function at the given address
#define CALL(o)				(LUI(Rtmp, SPLITH(o)), JRL(Rtmp, SPLITL(o)))

// Return from a function
#define RETURN				JR(Rlink, 0)

// Call a macro. Takes the macro idx as the argument
#define CALLMAC(idx)			CALL(IA(macro, idx))

// Jump to a specific address
#define JABS(ptr)			(LUI(Rtmp, SPLITH(ptr)), JR(Rtmp, SPLITL(ptr)))

// Jump to a Dis address
#define JDIS(pc)			JABS(IA(patch, pc))

// Jump to an address in the dst field of an instruction
#define JDST(i)				JDIS((i->d.ins - mod->prog))

// Set the offset of a branch instruction at address ptr to the current code address
// The order is opposite from OFF because it is used where the branch should jump to,
// not where it jumps from
#define PATCHBRANCH(ptr)		*ptr |= Bimm((ulong)(code) - (ulong)(ptr))

// Gets the address of a PC relative to the base
#define RELPC(pc)			(ulong)(base+(pc))

// Throw an error if the register is 0
#define NOTNIL(r)			(BNE(r, R0, 12), LUI(Rtmp, nullity), JRL(Rtmp, nullity))

// Array bounds check. Throws an error if the index is out of bounds
#define BCK(rindex, rsize)		(BLTU(rindex, rsize, 8), CALL(bounds))

// Cause an immediate illegal instruction exception, which should
// cause a register dump, stack trace, and a halt.
// Useful for debugging register state
#define CRASH()				gen(0);


static	ulong*	code;
static	ulong*	codestart;
static	ulong*	codeend;
static	ulong*	base;
static	ulong*	patch;
static	ulong	codeoff;
static	int	pass;
static	int	puntpc = 1;
static	Module*	mod;
static	uchar*	tinit;
static	ulong*	litpool;
static	int	nlit;
static	ulong	macro[NMACRO];
	void	(*comvec)(void);
static	void	macfrp(void);
static	void	macret(void);
static	void	maccase(void);
static	void	maccolr(void);
static	void	macmcal(void);
static	void	macfram(void);
static	void	macmfra(void);
static	void	macrelq(void);
static	void movmem(Inst*);
static	void mid(Inst*, int, int);

extern	void	das(ulong*, int);
extern  void    _d2v(vlong *y, double d);

// Float constants
double double05 = 0.5;
double double4294967296 = 4294967296.0;

#define T(r)	*((void**)(R.r))

// The macro table. Macros are long sequences of instructions which come up often, like calls and returns,
// so they are extracted out into separate blocks. The calling convention is separate for each macro.
struct
{
	int	idx;
	void	(*gen)(void);
	char*	name;
} mactab[] =
{
	MacFRP,		macfrp,		"FRP", 	/* decrement and free pointer */
	MacRET,		macret,		"RET",	/* return instruction */
	MacCASE,	maccase,	"CASE",	/* case instruction */
	MacCOLR,	maccolr,	"COLR",	/* increment and color pointer */
	MacMCAL,	macmcal,	"MCAL",	/* mcall bottom half */
	MacFRAM,	macfram,	"FRAM",	/* frame instruction */
	MacMFRA,	macmfra,	"MFRA",	/* punt mframe because t->initialize==0 */
	MacRELQ,	macrelq,	"RELQ",	/* reschedule */
};


/*  Helper functions */

void
urk(char *s)
{
	iprint("urk: %s\n", s);
	error(exCompile);
}

static void
gen(u32int o)
{
	if (code < codestart || code >= codeend) {
		iprint("gen: code out of bounds\n");
		iprint("code:      0x%p\n", code);
		iprint("codestart: 0x%p\n", codestart);
		iprint("codeend:   0x%p\n", codeend);
		//while (1) {}
	}

	*code++ = o;
}

static void
loadi(int reg, ulong val)
{
	// Load a value into a register

	// Check if the upper 20 bits are needed
	if (SPLITIMM(val)) {
		// Check if the lower 12 bits are needed
		LUI(reg, SPLITH(val));
		ADDI(reg, reg, SPLITL(val));
	} else {
		ADDI(reg, R0, val);
	}
}

static void
multiply(int rd, int rs, long c)
{
	// Multiply by a constant, rd = rs * c
	int shamt;

	if (c < 0) {
		NEG(rd, rs);
		rs = rd;
		c = -c;
	}

	switch (c) {
	case 0:
		MOV(rd, R0);
		break;
	case 1:
		if (rd != rs)
			MOV(rd, rs);
		break;
	case 2:
		shamt = 1;
		goto shift;
	case 3:
		shamt = 1;
		goto shiftadd;
	case 4:
		shamt = 2;
		goto shift;
	case 5:
		shamt = 2;
		goto shiftadd;
	case 7:
		shamt = 3;
		goto shiftsub;
	case 8:
		shamt = 3;
		goto shift;
	case 16:
		shamt = 4;
		goto shift;
	case 32:
		shamt = 5;
		goto shift;
	case 64:
		shamt = 6;
		goto shift;
	case 128:
		shamt = 7;
		goto shift;
	case 256:
		shamt = 8;
		goto shift;
	case 512:
		shamt = 9;
		goto shift;
	case 1024:
		shamt = 10;
		goto shift;
	shift:
		SLLI(rd, rs, shamt);
		break;
	shiftadd:
		if (rd == rs) {
			MOV(Rtmp, rs);
			rs = Rtmp;
		}

		SLLI(rd, rs, shamt);
		ADD(rd, rd, rs);
		break;
	shiftsub:
		if (rd == rs) {
			MOV(Rtmp, rs);
			rs = Rtmp;
		}

		SLLI(rd, rs, shamt);
		SUB(rd, rd, rs);
		break;
	default:
		loadi(Rtmp, c);
		MUL(rd, rd, Rtmp);
	}
}

static void
mem(int type, int r, int base, long offset)
{
	// Load or store data at an offset from an address in a register.
	// - type should be one of Ld* or St*.
	// - r is the source or destination register.
	// - base is the register with the base address.
	// - offset is added to the value of rs to get the
	//   address to load/store from/to

	if (SPLITIMM(offset)) {
		// The offset is too long. Add the upper part of offset to rs in the tmp register,
		// and use that as the base instead.
		LUI(Rtmp, SPLITH(offset));
		ADD(Rtmp, Rtmp, base);
		base = Rtmp;
		offset = SPLITL(offset);
	}

	switch (type) {
	case Ldw:
		LW(r, base, offset);
		break;
	case Ldh:
		LH(r, base, offset);
		break;
	case Ldhu:
		LHU(r, base, offset);
		break;
	case Ldb:
		LB(r, base, offset);
		break;
	case Ldbu:
		LBU(r, base, offset);
		break;
	case Lds:
		FLW(r, base, offset);
		break;
	case Ldd:
		FLD(r, base, offset);
		break;
	case Stw:
		SW(r, base, offset);
		break;
	case Sth:
		SH(r, base, offset);
		break;
	case Stb:
		SB(r, base, offset);
		break;
	case Sts:
		FSW(r, base, offset);
		break;
	case Std:
		FSD(r, base, offset);
		break;
	case Laddr:
		ADDI(r, base, offset);
		break;
	default:
		if (cflag > 2)
			iprint("Invalid type argument to mem: %d\n", type);
		urk("mem");
		break;
	}
}

static void
operand(int mtype, int mode, Adr *a, int r, int li)
{
	// Load or store the value from a src or dst operand of an instruction
	// - mtype is the memory access type, as in mem
	// - mode is the mode bits of the operand fields
	// - a is the source or dest struct
	// - r is the register to load the address into
	int base;
	long offset;

	switch (mode) {
	default:
		urk("operand");
	case AIMM:
		// Immediate value
		loadi(r, a->imm);

		if (mtype == Laddr) {
			mem(Stw, r, Rreg, li);
			mem(Laddr, r, Rreg, li);
		}
		return;
	case AFP:
		// Indirect offset from FP
		base = Rfp;
		offset = a->ind;
		break;
	case AMP:
		// Indirect offset from MP
		base = Rmp;
		offset = a->ind;
		break;
	case AIND|AFP:
		// Double indirect from FP
		mem(Ldw, Rta, Rfp, a->i.f);
		base = Rta;
		offset = a->i.s;
		break;
	case AIND|AMP:
		// Double indirect from MP
		mem(Ldw, Rta, Rmp, a->i.f);
		base = Rta;
		offset = a->i.s;
		break;
	}

	mem(mtype, r, base, offset);
}

static void
op1(int mtype, Inst *i, int r)
{
	// Load or store the source operand
	operand(mtype, USRC(i->add), &i->s, r, O(REG, st));
}

static void
op3(int mtype, Inst *i, int r)
{
	// Load or store the dest operand
	operand(mtype, UDST(i->add), &i->d, r, O(REG, dt));
}

static void
op2(int mtype, Inst *i, int r)
{
	// Load or store the middle operand
	int ir;

	switch (i->add&ARM) {
	default:
		return;
	case AXIMM:
		// Short immediate
		loadi(r, (short) i->reg);

		if (mtype == Laddr) {
			mem(Stw, r, Rreg, O(REG, t));
			mem(Laddr, r, Rreg, O(REG, t));
		}
		return;
	case AXINF:
		// Small offset from FP
		ir = Rfp;
		break;
	case AXINM:
		// Small offset from MP
		ir = Rmp;
		break;
	}

	// Load indirect
	mem(mtype, r, ir, i->reg);
}

static void
literal(ulong imm, int roff)
{
	// TODO: Why do this?
	nlit++;

	loadi(Rta, (ulong) litpool);
	mem(Stw, Rta, Rreg, roff);

	if (pass == 0)
		return;

	*litpool = imm;
	litpool++;
}

static void
rdestroy(void)
{
	destroy(R.s);
}

static void
rmcall(void)
{
	// Called by the compiled code to transfer control during an mcall
	Frame *f;
	Prog *p;

	if (R.dt == (ulong) H)
		error(exModule);

	f = (Frame*)R.FP;
	if (f == H)
		error(exModule);

	f->mr = nil;

	((void(*)(Frame*))R.dt)(f);

	R.SP = (uchar*)f;
	R.FP = f->fp;

	if (f->t == nil)
		unextend(f);
	else
		freeptrs(f, f->t);

	p = currun();
	if (p->kill != nil)
		error(p->kill);
}

static void
rmfram(void)
{
	Type *t;
	Frame *f;
	uchar *nsp;

	if(R.d == H)
		error(exModule);
	t = (Type*)R.s;
	if(t == H)
		error(exModule);
	nsp = R.SP + t->size;
	if(nsp >= R.TS) {
		R.s = t;
		extend();
		T(d) = R.s;
		return;
	}
	f = (Frame*)R.SP;
	R.SP = nsp;
	f->t = t;
	f->mr = nil;
	initmem(t, f);
	T(d) = f;
}

static void
bounds(void)
{
	error(exBounds);
}

static void
nullity(void)
{
	error(exNilref);
}

static void
punt(Inst *i, int m, void (*fn)(void))
{
	ulong pc;
	ulong *branch;

	if (m & SRCOP) {
		// Save the src operand in R->s
		op1(Laddr, i, RA1);
		mem(Stw, RA1, Rreg, O(REG, s));
	}

	if (m & DSTOP) {
		// Save the dst operand in R->d
		op3(Laddr, i, RA3);
		mem(Stw, RA3, Rreg, O(REG, d));
	}

	if (m & WRTPC) {
		// Store the PC in R->PC
		loadi(RA0, RELPC(patch[i - mod->prog+1]));
		mem(Stw, RA0, Rreg, O(REG, PC));
	}

	if (m & DBRAN) {
		// TODO: What does this do?
		pc = patch[i->d.ins - mod->prog];
		literal((ulong) (base+pc), O(REG, d));
	}

	if ((i->add & ARM) == AXNON) {
		if (m & THREOP) {
			// R->m = R->d
			mem(Ldw, RA2, Rreg, O(REG, d));
			mem(Stw, RA2, Rreg, O(REG, m));
		}
	} else {
		// R->m = middle operand
		op2(Laddr, i, RA2);
		mem(Stw, RA2, Rreg, O(REG, m));
	}

	// R->FP = Rfp
	mem(Stw, Rfp, Rreg, O(REG, FP));

	CALL(fn);

	loadi(Rreg, (ulong) &R);

	if (m & TCHECK) {
		mem(Ldw, RA0, Rreg, O(REG, t));

		branch = code;
		BEQZ(RA0, 0);

		// If R->t != 0
		mem(Ldw, Rlink, Rreg, O(REG, xpc)); // Rlink = R->xpc
		RETURN;

		PATCHBRANCH(branch); // endif
	}

	mem(Ldw, Rfp, Rreg, O(REG, FP));
	mem(Ldw, Rmp, Rreg, O(REG, MP));

	if (m & NEWPC) {
		// Jump to R->PC
		mem(Ldw, RA0, Rreg, O(REG, PC));
		JR(RA0, 0);
	}
}

static void
movloop(uint s)
{
	// Move a section of memory in a loop.
	// s is the size of each value, and should be 1, 2, or 4.
	// The source address should be in RA1.
	// The destination address should be in RA2.
	// The amount of values to transfer should be in RA3
	// All registers will be altered

	ulong *loop;

	if (s > 4 && s == 3) {
		// Unnatural size. Transfer byte for byte
		s = 1;
	}

	loop = code;
	BEQZ(RA3, 0);

	switch (s) {
	case 0:
		MOV(RA3, R0);
		break;
	case 1:
		mem(Ldb, RA0, RA1, 0);
		mem(Stb, RA0, RA2, 0);
		break;
	case 2:
		mem(Ldh, RA0, RA1, 0);
		mem(Ldh, RA0, RA2, 0);
		break;
	case 4:
		mem(Ldw, RA0, RA1, 0);
		mem(Ldw, RA0, RA2, 0);
		break;
	default:
		urk("movloop");
	}

	ADDI(RA1, RA2, s);
	ADDI(RA1, RA2, s);
	ADDI(RA3, RA3, -s);

	JABS(loop);

	PATCHBRANCH(loop);
}

static void
movmem(Inst *i)
{
	// Move a region of memory. Makes small transfers efficient, while defaulting
	// to a move loop for larger transfers.
	// The source address should be in RA1
	ulong *branch;

	if ((i->add & ARM) != AXIMM) {
		op2(Ldw, i, RA3);

		branch = code;
		BEQ(RA3, R0, 0);

		// if src2 != 0
		movloop(1);
		// endif

		PATCHBRANCH(branch);
		return;
	}

	switch (i->reg) {
	case 0:
		break;
	case 4:
		mem(Ldw, RA2, RA1, 0);
		op3(Stw, i, RA2); // Save directly, don't bother loading the address
		break;
	case 8:
		mem(Ldw, RA2, RA1, 0);
		mem(Ldw, RA3, RA1, 4);

		op3(Laddr, i, RA4);
		mem(Stw, RA2, RA4, 0);
		mem(Stw, RA3, RA4, 4);
		break;
	default:
		op3(Laddr, i, RA2);

		if ((i->reg & 3) == 0) {
			loadi(RA3, i->reg >> 2);
			movloop(4);
		} else if ((i->reg & 1) == 0) {
			loadi(RA3, i->reg >> 1);
			movloop(2);
		} else {
			loadi(RA3, i->reg);
			movloop(1);
		}
		break;
	}
}

static void
movptr(Inst *i)
{
	// Arguments:
	// - RA1: The address to move from
	// - op3: The address to move to

	ulong *branch;

	branch = code;
	BEQ(RA1, Rh, 0);

	// if RA1 != H
	CALLMAC(MacCOLR);		// colour if not H
	// endif

	PATCHBRANCH(branch);

	op3(Laddr, i, RA2);
	NOTNIL(RA2);

	mem(Ldw, RA0, RA2, 0);
	mem(Stw, RA1, RA2, 0);
	CALLMAC(MacFRP);
}

static void
branch(Inst *i, int mtype, int btype)
{
	// Insert a branch comparing integers
	// mtype should be the mtype to pass to mem to get the correct width
	// btype should be a constant like EQ, NE, LT, etc
	ulong *branch;

	op2(mtype, i, RA1);
	op1(mtype, i, RA2);

	branch = code;

	// Invert the condition to skip the jump
	switch (btype) {
	case EQ:
		BNE(RA1, RA2, 0);
		break;
	case NE:
		BEQ(RA1, RA2, 0);
		break;
	case GT:
		BLE(RA1, RA2, 0);
		break;
	case LT:
		BGE(RA1, RA2, 0);
		break;
	case LE:
		BGT(RA1, RA2, 0);
		break;
	case GE:
		BLT(RA1, RA2, 0);
		break;
	}

	JDST(i);

	PATCHBRANCH(branch);
}

static void
branchl(Inst *i, int btype)
{
	// Insert a branch comparing 64-bit integers
	// btype should be a constant like EQ, NE, LT, etc
	ulong *branch;

	op1(Laddr, i, RA0);
	mem(Ldw, RA1, RA0, 0);
	mem(Ldw, RA2, RA0, 4);

	op2(Laddr, i, RA0);
	mem(Ldw, RA3, RA0, 0);
	mem(Ldw, RA4, RA0, 4);

	// Set RA1 and RA2 to 1 if the condition holds
	switch (btype) {
	case EQ:
	case NE:
		// RA1 = RA1 - RA3 == 0
		// RA2 = RA2 - RA4 == 0
		SUB(RA1, RA1, RA3);
		SUB(RA2, RA2, RA4);
		SLTU(RA1, R0, RA1);
		SLTU(RA2, R0, RA2);
		break;
	case LT:
	case GE:
		// RA1 = RA1 < RA3
		// RA2 = RA2 < RA4
		SLT(RA1, RA1, RA3);
		SLT(RA2, RA2, RA4);
		break;
	case GT:
	case LE:
		// RA1 = RA3 < RA1
		// RA2 = RA4 < RA2
		SLT(RA1, RA3, RA1);
		SLT(RA2, RA4, RA2);
		break;
	}

	AND(RA1, RA1, RA2);

	// Insert the branch. Negate to skip the jump
	// Have to negate again for NE, GE and LE
	branch = code;
	switch (btype) {
	case NE:
	case GE:
	case LE:
		// If the negated condition holds, skip the jump
		BNE(RA1, R0, 0);
		break;
	default:
		// If the condition doesn't hold, skip the jump
		BEQ(RA1, R0, 0);
		break;
	}

	JDST(i);

	PATCHBRANCH(branch);
}

static void
branchfd(Inst *i, int btype)
{
	// Insert a branch comparing double-precision floats
	// btype should be a constant like EQ, NE, LT, etc
	ulong *branch;

	op2(Ldd, i, F1);
	op1(Ldd, i, F2);

	// Float compare instructions don't branch, so the branch
	// instruction has to check the result
	switch (btype) {
	case EQ:
	case NE:
		FEQD(RA0, F1, F2);
		break;
	case LT:
	case GE:
		FLTD(RA0, F1, F2);
		break;
	case LE:
	case GT:
		FLED(RA0, F1, F2);
		break;
	}

	// Branch if the result is negative, skipping the jump
	branch = code;
	switch (btype) {
	case NE:
	case GE:
	case GT:
		BNE(RA0, R0, 0);
		break;
	default:
		BEQ(RA0, R0, 0);
		break;
	}

	JDST(i);

	PATCHBRANCH(branch);
}

/* Macros */
static void
macfram(void)
{
	// Allocate a mframe
	// Arguments:
	// - RA3: src1->links[src2]->t

	ulong *branch;

	mem(Ldw, RA2, Rreg, O(REG, SP));	// RA2 = f = R.SP
	mem(Ldw, RA1, RA3, O(Type, size));	// RA1 = src1->links[src2]->t->size
	ADD(RA0, RA2, RA1);			// RA0 = nsp = R.SP + t->size
	mem(Ldw, RA1, Rreg, O(REG, TS));	// RA1 = R->TS

	branch = code;
	BGEU(RA0, RA1, 0);

	// nsp < R.TS
	mem(Stw, RA2, Rreg, O(REG, SP));	// R.SP = nsp

	mem(Stw, RA3, RA2, O(Frame, t));	// f->t = RA3
	mem(Stw, R0, RA2, O(Frame, mr));	// f->mr = 0
	mem(Ldw, Rta, RA3, O(Type, initialize));
	JR(Rta, 0);				// call t->init(RA2)

	// nsp >= R.TS; must expand
	PATCHBRANCH(branch);
	// Call extend. Store registers
	mem(Stw, RA3, Rreg, O(REG, s));
	mem(Stw, Rlink, Rreg, O(REG, st));
	mem(Stw, Rfp, Rreg, O(REG, FP));
	CALL(extend);

	// Restore registers
	loadi(Rreg, (ulong) &R);
	mem(Ldw, Rlink, Rreg, O(REG, st));
	mem(Ldw, Rfp, Rreg, O(REG, FP));
	mem(Ldw, Rmp, Rreg, O(REG, MP));
	mem(Ldw, RA2, Rreg, O(REG, s));
	RETURN;
}

static void
macmfra(void)
{
	mem(Stw, Rlink, Rreg, O(REG, st));
	mem(Stw, RA3, Rreg, O(REG, s)); // Save type
	mem(Stw, RA0, Rreg, O(REG, d)); // Save destination
	mem(Stw, Rfp, Rreg, O(REG, FP));

	CALL(rmfram);

	loadi(Rreg, (ulong)&R);
	mem(Ldw, Rlink, Rreg, O(REG, st));
	mem(Ldw, Rfp, Rreg, O(REG, FP));
	mem(Ldw, Rmp, Rreg, O(REG, MP));

	RETURN;
}

static void
macmcal(void)
{
	// The bottom half of a mcall instruction
	// Calling convention:
	// - RA0: The address of the function to jump to
	// - RA2: The frame address, src1 to mcall
	// - RA3: The module reference, src3 to mcall

	ulong *branch1, *branch2, *branch3;

	branch1 = code;
	BEQ(RA0, Rh, 0);
	// If RA0 != H

	mem(Ldw, RA1, RA3, O(Modlink, prog));	// Load m->prog into RA1

	branch2 = code;
	BNEZ(RA1, 0);
	// If m->prog == 0

	mem(Stw, Rlink, Rreg, O(REG, st));	// Store link register
	mem(Stw, RA2, Rreg, O(REG, FP));	// Store FP register
	mem(Stw, RA0, Rreg, O(REG, dt));	// Store destination address

	CALL(rmcall);

	// After the call has returned
	loadi(Rreg, (ulong)&R);			// Load R
	mem(Ldw, Rlink, Rreg, O(REG, st));	// Load link register
	mem(Ldw, Rfp, Rreg, O(REG, FP));	// Load FP register
	mem(Ldw, Rmp, Rreg, O(REG, MP));	// Load MP register
	RETURN;

	// else
	PATCHBRANCH(branch1);			// If RA0 == H
	PATCHBRANCH(branch2);			// If m->prog != 0

	MOV(Rfp, RA2);				// Rfp = RA2
	mem(Stw, RA3, Rreg, O(REG, M));		// R.M = RA3

	// D2H(RA3)->ref++
	ulong heapref = O(Heap, ref) - sizeof(Heap);
	mem(Ldw, RA1, RA3, heapref);
	ADDI(RA1, RA1, 1);
	mem(Stw, RA1, RA3, heapref);

	mem(Ldw, Rmp, RA3, O(Modlink, MP));	// Rmp = R.M->mp
	mem(Stw, Rmp, Rreg, O(REG, MP));	// R.MP = Rmp

	mem(Ldw, RA1, RA3, O(Modlink, compiled));
	branch3 = code;
	BNEZ(RA1, 0);

	// if M.compiled == 0
	mem(Stw, Rfp, Rreg, O(REG, FP)); // R.FP = Rfp
	mem(Stw, RA0, Rreg, O(REG, PC)); // R.PC = Rpc
	mem(Ldw, Rlink, Rreg, O(REG, xpc));
	RETURN;				// Leave it to the interpreter to handle

	// else
	PATCHBRANCH(branch3);
	JR(RA0, 0);			// Jump to the compiled module
}

static void
maccase(void)
{
	/*
	* RA1 = value (input arg), v
	* RA2 = count, n
	* RA3 = table pointer (input arg), t
	* RA0 = n/2, n2
	* RA4 = pivot element t+n/2*3, l
	*/

	ulong *loop, *found, *branch;

	mem(Ldw, RA2, RA3, 0);		// get count from table
	MOV(Rlink, RA3);		// initial table pointer

	loop = code;
	BLEZ(RA2, 0);			// n <= 0? goto out

	SRAI(RA0, RA2, 1);		// n2 = n>>1

	// l = t + n/2*3
	ADD(RA4, RA0, RA2);		// l = n/2 + n
	ADD(RA4, RA3, RA1);		// l += t

	mem(Ldw, Rta, RA4, 4);		// Rta = l[1]
	branch = code;
	BGE(RA1, Rta, 0);

	// if v < l[1]
	MOV(RA2, RA0);			// n = n2
	JABS(loop);			// continue

	// if v >= l[1]
	PATCHBRANCH(branch);
	mem(Ldw, Rta, RA4, 8);		// Rta = l[2]
	found = code;
	BLT(RA1, Rta, 0);		// branch to found

	// if v >= l[2]
	ADDI(RA3, RA4, 12);		// t = l+3
	SUB(RA2, RA2, RA0);		// n -= n2
	ADDI(RA2, RA2, -1);		// n -= 1

	JABS(loop);			// goto loop

	// endloop

	// found: v >= l[1] && v < l[2]
	// jump to l[3]
	PATCHBRANCH(found);
	JR(RA4, 12);

	// out: Loop ended
	PATCHBRANCH(loop);
	mem(Ldw, RA2, Rlink, 0);	// load initial n
	ADD(Rtmp, RA2, RA2);		// Rtmp = 2*n
	ADD(RA2, RA2, Rtmp);		// n = 3*n

	// goto (initial t)[n*3+1]
	SLLI(RA2, RA2, 2);		// RA2 = n*sizeof(long)
	ADD(Rlink, Rlink, RA2);		// Rlink = t[n*3]
	JR(Rlink, 4);			// goto Rlink+4 = t[n*3+1]
}

static void
maccolr(void)
{
	// Color a pointer
	// Arguments:
	// - RA1: The pointer to color
	ulong *branch;

	// h->ref++
	mem(Ldw, RA0, RA1, O(Heap, ref) - sizeof(Heap));
	ADDI(RA0, RA0, 1);
	mem(Stw, RA0, RA1, O(Heap, ref) - sizeof(Heap));

	// RA0 = mutator
	mem(Ldw, RA0, RA1, O(Heap, color) - sizeof(Heap));

	// RA2 = h->color
	loadi(RA2, (ulong) &mutator);
	mem(Ldw, RA2, RA2, 0);

	branch = code;
	BEQ(RA0, RA2, 0);

	// if h->color != mutator

	// h->color = propagator
	loadi(RA2, propagator);
	mem(Stw, RA2, RA1, O(Heap, color) - sizeof(Heap));

	// nprop = !0
	loadi(RA2, (ulong) &nprop);
	mem(Stw, RA2, RA2, 0);			// TODO: Is this wrong? Should RA1 be stored instead?

	// endif
	PATCHBRANCH(branch);
	RETURN;
}

static void
macfrp(void)
{
	// Destroy a pointer
	// Arguments:
	// - RA0: The pointer to destroy
	ulong *branch1, *branch2;

	branch1 = code;
	BEQ(RA0, Rh, 0);

	// if RA0 != H
	mem(Ldw, RA2, RA0, O(Heap, ref) - sizeof(Heap));
	ADDI(RA2, RA2, -1);

	branch2 = code;
	BEQ(RA2, R0, 0);

	// if --h->ref != 0
	mem(Stw, RA2, RA0, O(Heap, ref) - sizeof(Heap));
	RETURN;
	// endif

	PATCHBRANCH(branch2);
	mem(Stw, Rfp, Rreg, O(REG, FP));
	mem(Stw, Rlink, Rreg, O(REG, st));
	mem(Stw, RA0, Rreg, O(REG, s));
	CALL(rdestroy);

	loadi(Rreg, (ulong) &R);
	mem(Ldw, Rlink, Rreg, O(REG, st));
	mem(Ldw, Rfp, Rreg, O(REG, FP));
	mem(Ldw, Rmp, Rreg, O(REG, MP));

	// endif
	PATCHBRANCH(branch1);
	RETURN;
}

static void
macret(void)
{
	Inst i;
	ulong *branch1, *branch2, *branch3, *branch4, *branch5, *branch6;

	branch1 = code;
	BEQ(RA1, R0, 0);

	// if t(Rfp) != 0
	mem(Ldw, RA0, RA1, O(Type, destroy));
	branch2 = code;
	BEQ(RA0, R0, 0);

	// if destroy(t(fp)) != 0
	mem(Ldw, RA2, Rfp, O(Frame, fp));
	branch3 = code;
	BEQ(RA2, R0, 0);

	// if fp(Rfp) != 0
	mem(Ldw, RA3, Rfp, O(Frame, mr));
	branch4 = code;
	BEQ(RA3, R0, 0);

	// if mr(Rfp) != 0
	mem(Ldw, RA2, Rreg, O(REG, M));
	mem(Ldw, RA3, RA2, O(Heap, ref) - sizeof(Heap));
	ADDI(RA3, RA3, -1);

	branch5 = code;
	BEQ(RA3, R0, 0);

	// if --ref(arg) != 0
	mem(Stw, RA3, RA2, O(Heap, ref) - sizeof(Heap));
	mem(Ldw, RA1, Rfp, O(Frame, mr));
	mem(Stw, RA1, Rreg, O(REG, M));
	mem(Ldw, Rmp, RA1, O(Modlink, MP));
	mem(Stw, Rmp, Rreg, O(REG, MP));

	mem(Ldw, RA3, RA1, O(Modlink, compiled));
	branch6 = code;
	BEQ(RA3, R0, 0);

	// This part is a bit weird, because it should be the innermost
	// if-statement (in C terms), but the else of branch4 also ends up here.
	// This could be a mistake, but it's in at least the ARM and MIPS version.

	// if R.M->compiled != 0
	// if mr(Rfp) == 0
	PATCHBRANCH(branch4);
	JRL(RA0, 0);				// Call destroy(t(fp))

	mem(Stw, Rfp, Rreg, O(REG, SP));	// R->SP = Rfp
	mem(Ldw, RA1, Rfp, O(Frame, lr));	// RA1 = Rfp->lr
	mem(Ldw, Rfp, Rfp, O(Frame, fp));	// Rfp = Rfp->fp
	mem(Stw, Rfp, Rreg, O(REG, FP));	// R->FP = Rfp

	JR(RA1, 0);				// goto RA1, if compiled
	// does not continue past here

	// if R.M->compiled == 0
	PATCHBRANCH(branch6);
	JRL(RA0, 0);				// Call destroy(t(fp))

	mem(Stw, Rfp, Rreg, O(REG, SP));	// R->SP = Rfp
	mem(Ldw, RA1, Rfp, O(Frame, lr));	// RA1 = Rfp->lr
	mem(Ldw, Rfp, Rfp, O(Frame, fp));	// Rfp = Rfp->fp
	mem(Stw, RA1, Rreg, O(REG, PC));	// R.PC = RA1
	mem(Ldw, Rlink, Rreg, O(REG, xpc));	// Rlink = R->xpc
	RETURN;					// return to xec uncompiled code

	// endif
	PATCHBRANCH(branch5);
	PATCHBRANCH(branch3);
	PATCHBRANCH(branch2);
	PATCHBRANCH(branch1);

	i.add = AXNON;
	punt(&i, TCHECK|NEWPC, optab[IRET]);
}

static void
macrelq(void)
{
	// Store frame pointer and link register, then return to xev
	mem(Stw, Rfp, Rreg, O(REG, FP));
	mem(Stw, Rlink, Rreg, O(REG, PC));
	mem(Ldw, Rlink, Rreg, O(REG, xpc));
	RETURN;
}

/* Main compilation functions */
static void
comi(Type *t)
{
	// Compile a type initializer
	int i, j, m, c;

	for (i = 0; i < t->np; i++) {
		c = t->map[i];
		j = i << 5;

		for (m = 0x80; m != 0; m >>= 1) {
			if (c & m)
				mem(Stw, Rh, RA2, j);

			j += sizeof(WORD*);
		}
	}

	RETURN;
}

static void
comd(Type *t)
{
	// Compile a type destructor
	int i, j, m, c;

	mem(Stw, Rlink, Rreg, O(REG, dt));

	for (i = 0; i < t->np; i++) {
		c = t->map[i];
		j = i << 5;

		for (m = 0x80; m != 0; m >>= 1) {
			if (c & m) {
				mem(Ldw, RA0, Rfp, j);
				CALL(base+macro[MacFRP]);
			}

			j += sizeof(WORD*);
		}
	}

	mem(Ldw, Rlink, Rreg, O(REG, dt));
	RETURN;
}

static void
typecom(Type *t)
{
	// Compile a type
	int n;
	ulong *tmp, *start;

	if (t == nil | t->initialize != 0)
		return;

	tmp = mallocz(4096*sizeof(ulong), 0);
	if (tmp == nil)
		error(exNomem);

	codestart = tmp;
	codeend = tmp + 4096;
	iprint("Typecom np %d, size %d\n", t->np, t->size);
	code = tmp;
	comi(t);
	n = code - tmp;
	code = tmp;
	comd(t);
	n += code - tmp;
	free(tmp);

	n *= sizeof(*code);
	code = mallocz(n, 0);
	if (code == nil)
		return;

	codestart = code;
	codeend = code + n;

	start = code;
	t->initialize = code;
	comi(t);
	t->destroy = code;
	comd(t);

	segflush(start, n);

	if (cflag > 3)
		iprint("typ= %.8p %4d i %.8p d %.8p asm=%d\n",
			t, t->size, t->initialize, t->destroy, n);

	if (cflag > 6) {
		das(start, code-start);
	}
}

static void
patchex(Module *m, ulong *p)
{
	// Apply patches for a module. p is the patch array
	Handler *h;
	Except *e;

	for (h = m->htab; h != nil && h->etab != nil; h++) {
		h->pc1 = p[h->pc1];
		h->pc2 = p[h->pc2];

		for (e = h->etab; e->s != nil; e++)
			e->pc = p[e->pc];

		if (e->pc != -1)
			e->pc = p[e->pc];
	}
}

static void
commframe(Inst *i)
{
	// Compile a mframe instruction
	ulong *branch1, *branch2;

	op1(Ldw, i, RA0);
	branch1 = code;
	BEQ(RA0, Rh, 0);

	// if RA0 != H

	// RA3 = src->links[src2]->frame
	if ((i->add & ARM) == AXIMM) {
		mem(Ldw, RA3, RA0, OA(Modlink, links) + i->reg*sizeof(Modl) + O(Modl, frame));
	} else {
		// RA1 = src->links[src2]
		op2(Ldw, i, RA1);
		multiply(RA1, RA1, sizeof(Modl));
		ADD(RA1, RA1, RA0);

		// RA3 = src->links[src2]->frame
		mem(Ldw, RA3, RA1, O(Modl, frame));
	}

	mem(Ldw, RA1, RA3, O(Type, initialize));
	branch2 = code;
	BNEZ(RA1, 0);

	// if frame->initialize == 0
	op3(Laddr, i, RA0);
	// endif

	// if RA0 == H || frame->initialize == 0
	PATCHBRANCH(branch1);
	loadi(Rlink, RELPC(patch[i - mod->prog + 1]));
	CALLMAC(MacMFRA);

	// if frame->inititalize != 0
	PATCHBRANCH(branch2);
	CALLMAC(MacFRAM);
	op3(Stw, i, RA2);
}

static void
commcall(Inst *i)
{
	// Compile a mcall instruction
	ulong *branch;

	op1(Ldw, i, RA2);			// RA2 = src1 = frame
	loadi(RA0, RELPC(patch[i - mod->prog+1])); // RA0 = pc
	mem(Stw, RA0, RA2, O(Frame, lr));	// frame.lr = RA0 = pc
	mem(Stw, Rfp, RA2, O(Frame, fp));	// frame.fp = fp
	mem(Ldw, RA3, Rreg, O(REG, M)); 	// RA3 = R.M
	mem(Stw, RA3, RA2, O(Frame, mr));	// frame.mr = RA3 = R.M

	op3(Ldw, i, RA3); // RA3 = src3 = Modlink

	branch = code;
	BEQ(RA3, Rh, 0);
	// If RA3 != H

	// RA0 = Modlink->links[src2]->pc
	if ((i->add&ARM) == AXIMM) {
		// i->reg contains the immediate of src2, don't have to store it in a register
		mem(Ldw, RA0, RA3, OA(Modlink, links) + i->reg*sizeof(Modl) + O(Modl, u.pc));
	} else {
		op2(Ldw, i, RA1);		// RA1 = src2

		// RA1 *= sizeof(Modl)
		multiply(RA1, RA1, sizeof(Modl));

		ADDI(RA1, RA1, RA3);
		mem(Ldw, RA0, RA1, OA(Modlink, links) + O(Modl, u.pc));
	}

	PATCHBRANCH(branch); // endif

	CALLMAC(MacMCAL);
}

static void
comcase(Inst *i, int w)
{
	// Compile a case instruction
	int l;
	WORD *t, *e;

	if (w != 0) {
		// Use the MacCASE macro
		op1(Ldw, i, RA1);
		op3(Laddr, i, RA3);
		CALLMAC(MacCASE);
	}

	// Get a pointer to the table
	t = (WORD*)(mod->origmp + i->d.ind+4);

	// Get the flag right before the table
	l = t[-1];

	/* have to take care not to relocate the same table twice -
	 * the limbo compiler can duplicate a case instruction
	 * during its folding phase
	 */

	if (pass == 0) {
		if (l >= 0)
			t[-1] = -l-1;	/* Mark it not done */
		return;
	}

	if (l >= 0) {			/* Check pass 2 done */
		return;
	}

	t[-1] = -l-1;			/* Set real count */
	e = t + t[-1]*3;

	while (t < e) {
		t[2] = RELPC(patch[t[2]]);
		t += 3;
	}

	t[0] = RELPC(patch[t[0]]);
}

static void
comcasel(Inst *i)
{
	// Same as comecase, but with double words
	int l;
	WORD *t, *e;

	t = (WORD*) (mod->origmp + i->d.ind + 8);
	l = t[-2];

	if (pass == 0) {
		if (l >= 0)
			t[-2] = -l-1;	/* Mark it not done */
		return;
	}

	if (l >= 0)			/* Check pass 2 done */
		return;

	t[-2] = -l-1;			/* Set real count */
	e = t + t[-2]*6;

	while (t < e) {
		t[4] = RELPC(patch[t[4]]);
		t += 6;
	}

	t[0] = RELPC(patch[t[0]]);
}

static void
comgoto(Inst *i)
{
	// Compile a goto instruction
	WORD *t, *e;

	op1(Ldw, i, RA1);		// RA1 = src
	op3(Laddr, i, RA0);		// RA0 = &dst
	SLLI(RA1, RA1, 2);		// RA1 = src*sizeof(int)
	ADD(RA1, RA1, RA0);		// RA1 += RA0
	mem(Ldw, RA0, RA1, 0);		// RA0 = dst[src]
	JR(RA0, 0);			// goto dst[src]

	if (pass == 0)
		return;

	t = (WORD*)(mod->origmp+i->d.ind);
	e = t + t[-1];
	t[-1] = 0;

	while (t < e) {
		t[0] = RELPC(patch[t[0]]);
		t++;
	}
}

static void
comp(Inst *i)
{
	// Compile a single DIS instruction
	char buf[64];
	ulong *branch1, *branch2, *loop;

	switch (i->op) {
	default:
		snprint(buf, sizeof buf, "%s compile, no '%D'", mod->name, i);
		error(buf);
		break;
	case IMCALL:
		commcall(i);
		break;
	case ISEND:
	case IRECV:
	case IALT:
		punt(i, SRCOP|DSTOP|TCHECK|WRTPC, optab[i->op]);
		break;
	case ISPAWN:
		punt(i, SRCOP|DBRAN, optab[i->op]);
		break;
	case IBNEC:
	case IBEQC:
	case IBLTC:
	case IBLEC:
	case IBGTC:
	case IBGEC:
		punt(i, SRCOP|DBRAN|NEWPC|WRTPC, optab[i->op]);
		break;
	case ICASEC:
		comcase(i, 0);
		punt(i, SRCOP|DSTOP|NEWPC, optab[i->op]);
		break;
	case ICASEL:
		comcasel(i);
		punt(i, SRCOP|DSTOP|NEWPC, optab[i->op]);
		break;
	case IADDC:
	case IMULL:
	case IDIVL:
	case IMODL:
	case IMNEWZ:
	case ILSRW:
	case ILSRL:
		punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case IMODW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA0);
		REM(RA0, RA0, RA1);
		op3(Stw, i, RA0);
		break;
	case IMODB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA0);
		REM(RA0, RA0, RA1);
		op3(Stb, i, RA0);
		break;
	case IDIVW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA0);
		DIV(RA0, RA0, RA1);
		op3(Stw, i, RA0);
		break;
	case IDIVB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA0);
		DIV(RA0, RA0, RA1);
		op3(Stb, i, RA0);
		break;
	case ILOAD:
	case INEWA:
	case INEWAZ:
	case INEW:
	case INEWZ:
	case ISLICEA:
	case ISLICELA:
	case ICONSB:
	case ICONSW:
	case ICONSL:
	case ICONSF:
	case ICONSM:
	case ICONSMP:
	case ICONSP:
	case IMOVMP:
	case IHEADMP:
	case IHEADB:
	case IHEADW:
	case IHEADL:
	case IINSC:
	case ICVTAC:
	case ICVTCW:
	case ICVTWC:
	case ICVTLC:
	case ICVTCL:
	case ICVTFC:
	case ICVTCF:
	case ICVTRF:
	case ICVTFR:
	case ICVTWS:
	case ICVTSW:
	case IMSPAWN:
	case ICVTCA:
	case ISLICEC:
	case INBALT:
		punt(i, SRCOP|DSTOP, optab[i->op]);
		break;
	case INEWCM:
	case INEWCMP:
		punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case IMFRAME:
		if((i->add&ARM) == AXIMM)
			commframe(i);
		else
			punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case ICASE:
		comcase(i, 1);
		break;
	case IGOTO:
		comgoto(i);
		break;
	case IMOVF:
		op1(Ldd, i, F1);
		op3(Std, i, F1);
		break;
	case IMOVL:
		op1(Laddr, i, RA0);
		mem(Ldw, RA1, RA0, 0);
		mem(Ldw, RA2, RA0, 4);

		op3(Laddr, i, RA0);
		mem(Stw, RA1, RA0, 0);
		mem(Stw, RA2, RA0, 4);
		break;
	case IHEADM:
		op1(Laddr, i, RA1);
		NOTNIL(RA1);

		if(OA(List, data) != 0) {
			ADDI(RA1, RA1, OA(List, data));
		}

		movmem(i);
		break;
	case IMOVM:
		op1(Laddr, i, RA1);
		movmem(i);
		break;
	case IFRAME:
		if(UXSRC(i->add) != SRC(AIMM)) {
			punt(i, SRCOP|DSTOP, optab[i->op]);
			break;
		}
		tinit[i->s.imm] = 1;
		loadi(RA3, (ulong) mod->type[i->s.imm]);
		CALL(base+macro[MacFRAM]);
		op3(Stw, i, RA2);
		break;
	case INEWCB:
	case INEWCW:
	case INEWCF:
	case INEWCP:
	case INEWCL:
		punt(i, DSTOP|THREOP, optab[i->op]);
		break;
	case IEXIT:
		punt(i, 0, optab[i->op]);
		break;
	case ICVTBW:
		op1(Ldbu, i, RA0);
		op3(Stw, i, RA0);
		break;
	case ICVTWB:
		op1(Ldw, i, RA0);
		op3(Stb, i, RA0);
		break;
	case ILEA:
		op1(Laddr, i, RA0);
		op3(Stw, i, RA0);
		break;
	case IMOVW:
		op1(Ldw, i, RA0);
		op3(Stw, i, RA0);
		break;
	case IMOVB:
		op1(Ldb, i, RA0);
		op3(Stb, i, RA0);
		break;
	case ITAIL:
		op1(Ldw, i, RA0);
		NOTNIL(RA0);
		mem(Ldw, RA1, RA0, O(List, tail));
		movptr(i);
		break;
	case IMOVP:
		op1(Ldw, i, RA1);
		NOTNIL(RA1);
		movptr(i);
		break;
	case IHEADP:
		op1(Ldw, i, RA0);
		NOTNIL(RA0);
		mem(Ldw, OA(List, data), RA0, RA1);
		movptr(i);
		break;
	case ILENA:
		op1(Ldw, i, RA1);
		MOV(RA0, R0);

		branch1 = code;
		BEQ(RA1, Rh, 0);

		// if src != H
		mem(Ldw, RA0, RA1, O(Array, len));
		// endif

		PATCHBRANCH(branch1);
		op3(Stw, i, RA0);
		break;
	case ILENC:
		op1(Ldw, i, RA1);
		MOV(RA0, R0);

		branch1 = code;
		BEQ(RA1, Rh, 0);

		// if RA1 != H
		mem(Ldw, RA0, RA1, O(String, len));

		branch2 = code;
		BGE(RA0, 0, 0);

		// if string->len < 0

		// RA0 = abs(string->len)
		// TODO: This might be wrong
		NEG(RA0, RA0);

		// endif

		PATCHBRANCH(branch1);
		PATCHBRANCH(branch2);
		op3(Stw, i, RA0);
		break;
	case ILENL:
		MOV(RA0, R0);				// RA0 = 0
		op1(Ldw, i, RA1);			// RA1 = src

		// while RA1 != H
		loop = code;
		BEQ(RA1, Rh, 0);

		mem(Ldw, RA1, RA1, O(List, tail));	// RA0 = RA0->tail
		ADDI(RA0, RA0, 1);			// RA1++
		JABS(loop);
		// endwhile

		PATCHBRANCH(loop);
		op3(Stw, i, RA0);			// return RA1
		break;
	case ICALL:
		op1(Ldw, i, RA0);
		loadi(RA1, RELPC(patch[i - mod->prog + 1]));
		mem(Stw, RA1, RA0, O(Frame, lr));
		mem(Stw, Rfp, RA0, O(Frame, fp));
		MOV(Rfp, RA0);
		JDST(i);
		break;
	case IJMP:
		JDST(i);
		break;
	case IBEQW:
		branch(i, Ldw, EQ);
		break;
	case IBNEW:
		branch(i, Ldw, NE);
		break;
	case IBLTW:
		branch(i, Ldw, LT);
		break;
	case IBLEW:
		branch(i, Ldw, LE);
		break;
	case IBGTW:
		branch(i, Ldw, GT);
		break;
	case IBGEW:
		branch(i, Ldw, GE);
		break;
	case IBEQB:
		branch(i, Ldb, EQ);
		break;
	case IBNEB:
		branch(i, Ldb, NE);
		break;
	case IBLTB:
		branch(i, Ldb, LT);
		break;
	case IBLEB:
		branch(i, Ldb, LE);
		break;
	case IBGTB:
		branch(i, Ldb, GT);
		break;
	case IBGEB:
		branch(i, Ldb, GE);
		break;
	case IBEQF:
		branchfd(i, EQ);
		break;
	case IBNEF:
		branchfd(i, NE);
		break;
	case IBLTF:
		branchfd(i, LT);
		break;
	case IBLEF:
		branchfd(i, LE);
		break;
	case IBGTF:
		branchfd(i, GT);
		break;
	case IBGEF:
		branchfd(i, GE);
		break;
	case IRET:
		mem(Ldw, RA1, Rfp, O(Frame, t));
		CALLMAC(MacRET);
		break;
	case IMULW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA0);
		MUL(RA0, RA0, RA1);
		op3(Stw, i, RA0);
		break;
	case IMULB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA0);
		MUL(RA0, RA0, RA1);
		op3(Stb, i, RA0);
		break;
	case IORW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		OR(RA0, RA1, RA2);
		op3(Stw, i, RA0);
		break;
	case IANDW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		AND(RA0, RA1, RA2);
		op3(Stw, i, RA0);
		break;
	case IXORW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		XOR(RA0, RA1, RA2);
		op3(Stw, i, RA0);
		break;
	case ISUBW:
		op1(Ldw, i, RA2);
		op2(Ldw, i, RA1);
		SUB(RA0, RA1, RA2);
		op3(Stw, i, RA0);
		break;
	case IADDW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		ADD(RA0, RA1, RA2);
		op3(Stw, i, RA0);
		break;
	case ISHRW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		SRL(RA0, RA2, RA1);	// Shift order is switched
		op3(Stw, i, RA0);
		break;
	case ISHLW:
		op1(Ldw, i, RA1);
		op2(Ldw, i, RA2);
		SLL(RA0, RA2, RA1);	// Shift order is switched
		op3(Stw, i, RA0);
	case IORB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		OR(RA0, RA1, RA2);
		op3(Stb, i, RA0);
		break;
	case IANDB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		AND(RA0, RA1, RA2);
		op3(Stb, i, RA0);
		break;
	case IXORB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		XOR(RA0, RA1, RA2);
		op3(Stb, i, RA0);
		break;
	case ISUBB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		SUB(RA0, RA1, RA2);
		op3(Stb, i, RA0);
		break;
	case IADDB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		ADD(RA0, RA1, RA2);
		op3(Stb, i, RA0);
		break;
	case ISHRB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		SRL(RA0, RA2, RA1);	// Shift order is switched
		op3(Stb, i, RA0);
	case ISHLB:
		op1(Ldb, i, RA1);
		op2(Ldb, i, RA2);
		SLL(RA0, RA2, RA1);	// Shift order is switched
		op3(Stb, i, RA0);
	case IINDC:
		op1(Ldw, i, RA1);	// RA1 = src1 = string
		NOTNIL(RA1);

		op2(Ldw, i, RA2);	// RA2 = src2 = index

		mem(Ldw, RA0, RA1, O(String, len));	// RA0 = string->len

		if(bflag){
			MOV(RA3, RA0);
			branch1 = code;
			BGE(RA3, R0, 0);

			// if string->len < 0
			NEG(RA3, RA3);
			// endif

			PATCHBRANCH(branch1);
			BCK(RA2, RA3);
		}

		ADDI(RA1, RA1, O(String, data));

		branch2 = code;
		BGE(RA0, R0, 0);

		// if string->len < 0
		SLLI(RA2, RA2, 2);		// index = index << 2; in words, not bytes
		// endif

		PATCHBRANCH(branch2);
		mem(Ldw, RA3, RA1, RA2);	// RA3 = string[index]
		op3(Stw, i, RA3);
		break;
	case IINDL:
	case IINDF:
	case IINDW:
	case IINDB:
		op1(Ldw, i, RA1);				// RA1 = src1 = array
		NOTNIL(RA1);
		op3(Ldw, i, RA2);				// RA2 = src2 = index

		if(bflag) {
			mem(Ldw, RA3, RA1, O(Array, len));	// RA3 = array->len
			BCK(RA2, RA3);
		}

		mem(Ldw, RA1, RA1, O(Array, data));		// RA1 = array->data

		// Modify the index to match the data width
		switch(i->op) {
		case IINDL:
		case IINDF:
			SLLI(RA2, RA2, 3);
			break;
		case IINDW:
			SLLI(RA2, RA2, 2);
			break;
		}

		ADD(RA1, RA1, RA2);
		op2(Stw, i, RA1);
		break;
	case IINDX:
		op1(Ldw, i, RA1);			// RA1 = src1 = array
		NOTNIL(RA0);
		op3(Ldw, i, RA2);			// RA2 = src2 = index

		if(bflag){
			mem(Ldw, RA3, RA1, O(Array, len));	// RA3 = array->len
			BCK(RA2, RA3);
		}

		mem(Ldw, RA3, RA1, O(Array, t));	// RA3 = array->t
		mem(Ldw, RA3, RA3, O(Type, size));	// RA3 = array->t->size
		mem(Ldw, RA1, RA1, O(Array, data));	// RA1 = array->data

		MUL(RA2, RA2, RA3);			// RA2 = index*size
		ADD(RA1, RA1, RA0);			// RA1 = array->data + index*size
		op2(Stw, i, RA1);
		break;
	case IADDL:
	case ISUBL:
	case IORL:
	case IANDL:
	case IXORL:
		// The Dis instructions uses the format "src3 = src2 op src1",
		// which is opposite to RISC-V. To make the code more intuitive the order
		// is switched here, so the operations are "src3 = RA1.RA2 op RA3.RA4"

		// RA1, RA2 = src2
		op2(Laddr, i, RA0);
		mem(Ldw, RA1, RA0, 0);
		mem(Ldw, RA2, RA0, 4);

		// RA3, RA4 = src1
		op1(Laddr, i, RA0);
		mem(Ldw, RA3, RA0, 0);
		mem(Ldw, RA4, RA0, 4);

		switch (i->op) {
		case IADDL:
			ADD(RA0, RA1, RA3);		// RA0 = src2[31:0] + src1[31:0]
			ADD(RA2, RA2, RA4);		// RA2 = src2[63:32] + src1[63:32]

			// Check for overflow
			SLTU(RA1, RA0, RA1);		// RA1 = RA0 < src2[31:0] ? 1 : 0

			// Add the overflow to the upper bits
			ADD(RA2, RA2, RA1);

			// Move the lower result to RA1
			MOV(RA1, RA0);
			break;
		case ISUBL:
			SUB(RA0, RA1, RA3);		// RA0 = src2[31:0] - src1[31:0]
			SUB(RA2, RA2, RA4);		// RA2 = src2[63:32] - src1[63:32]

			// Check for underflow
			SLTU(RA1, RA1, RA0);		// RA1 = src2[31:0] < RA0 ? 1 : 0

			// Add the underflow to the upper bits
			SUB(RA2, RA2, RA1);

			// Move the lower result to RA1
			MOV(RA1, RA0);
			break;
		case IORL:
			OR(RA1, RA1, RA3);
			OR(RA2, RA2, RA4);
			break;
		case IANDL:
			AND(RA1, RA1, RA3);
			AND(RA2, RA2, RA4);
			break;
		case IXORL:
			XOR(RA1, RA1, RA3);
			XOR(RA2, RA2, RA4);
			break;
		}

		// dst = RA1, RA2
		op3(Laddr, i, RA0);
		mem(Stw, RA1, RA0, 0);
		mem(Stw, RA2, RA0, 4);
		break;
	case ICVTWL:
		op1(Ldw, i, RA1);
		op2(Laddr, i, RA0);
		SRAI(RA2, RA1, 31);			// Shift right 31 places to sign-extend
		mem(Stw, RA1, RA0, 0);
		mem(Stw, RA2, RA0, 4);
		break;
	case ICVTLW:
		op1(Ldw, i, RA0);
		op3(Stw, i, RA0);
		break;
	case IBEQL:
		branchl(i, EQ);
		break;
	case IBNEL:
		branchl(i, NE);
		break;
	case IBLEL:
		branchl(i, LE);
		break;
	case IBGTL:
		branchl(i, GT);
		break;
	case IBLTL:
		branchl(i, LT);
		break;
	case IBGEL:
		branchl(i, GE);
		break;
	case ICVTFL:
		ADDI(Rsp, Rsp, -16);

		op1(Ldd, i, F1);		// Load the double to convert
		op3(Laddr, i, Rarg);		// Load the destination as the first argument to _d2v

		// Round F1 by adding 0.5 or -0.5

		// F2 = 0.5
		LUI(Rta, SPLITH(&double05));
		mem(Ldd, F2, Rta, SPLITL(&double05));

		FSGNJD(F2, F2, F1);		// F2 = F1 >= 0 ? F2 : -F2
		FADDD(RM, F1, F1, F2);		// F1 += F2

		mem(Std, F1, Rsp, 8);		// Store F1 as the second argument, and call _d2v

		// Call _d2v
		mem(Stw, Rfp, Rreg, O(REG, FP));
		CALL(_d2v);
		loadi(Rreg, (ulong) &R);
		mem(Ldw, Rfp, Rreg, O(REG, FP));
		mem(Ldw, Rmp, Rreg, O(REG, MP));

		ADDI(Rsp, Rsp, 16);
		break;
	case ICVTLF:
		op1(Laddr, i, Rta);
		mem(Ldw, RA0, Rta, 0);
		mem(Ldw, RA1, Rta, 4);

		FCVTDWU(RM, F0, RA0);		// F0 = float(unsigned src[0:31])
		FCVTDW(RM, F1, RA1);		// F1 = float(src[32:63])

		// F2 = 4294967296
		LUI(Rta, SPLITH(&double4294967296));
		mem(Ldd, F2, Rta, SPLITL(&double4294967296));

		FMADDD(RM, F0, F1, F2, F0);	// F0 = F1 * F2 + F0

		// Store the result
		op3(Std, i, F0);
		break;
	case IDIVF:
		op1(Ldd, i, F1);
		op2(Ldd, i, F2);
		FDIVD(RM, F1, F2, F1);
		op3(Std, i, F1);
		break;
	case IMULF:
		op1(Ldd, i, F1);
		op2(Ldd, i, F2);
		FMULD(RM, F1, F2, F1);
		op3(Std, i, F1);
		break;
	case ISUBF:
		op1(Ldd, i, F1);
		op2(Ldd, i, F2);
		FSUBD(RM, F1, F2, F1);
		op3(Std, i, F1);
		break;
	case IADDF:
		op1(Ldd, i, F1);
		op2(Ldd, i, F2);
		FADDD(RM, F1, F2, F1);
		op3(Std, i, F1);
		break;
	case INEGF:
		op1(Ldd, i, F1);
		FSGNJND(F1, F1, F1);
		op3(Std, i, F1);
		break;
	case ICVTWF:
		op1(Ldw, i, RA0);
		FCVTDW(RM, F1, RA0);
		op3(Std, i, F1);
		break;
	case ICVTFW:
		op1(Ldd, i, F1);
		FCVTWD(RM, RA0, F1);
		op3(Stw, i, RA0);
		break;
	case ISHLL:
		/* should do better */
		punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case ISHRL:
		/* should do better */
		punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case IRAISE:
		punt(i, SRCOP|WRTPC|NEWPC, optab[i->op]);
		break;
	case IMULX:
	case IDIVX:
	case ICVTXX:
	case IMULX0:
	case IDIVX0:
	case ICVTXX0:
	case IMULX1:
	case IDIVX1:
	case ICVTXX1:
	case ICVTFX:
	case ICVTXF:
	case IEXPW:
	case IEXPL:
	case IEXPF:
		punt(i, SRCOP|DSTOP|THREOP, optab[i->op]);
		break;
	case ISELF:
		punt(i, DSTOP, optab[i->op]);
		break;
	}
}

static void
preamble(void)
{
	if(comvec)
		return;

	comvec = malloc(20 * sizeof(*code));
	if(comvec == nil)
		error(exNomem);
	code = (ulong*)comvec;
	codestart = code;
	codeend = code + 10;

	loadi(Rh, (ulong) H);
	loadi(Rreg, (ulong) &R);
	mem(Stw, Rlink, Rreg, O(REG, xpc));
	mem(Ldw, Rfp,   Rreg, O(REG, FP));
	mem(Ldw, Rmp,   Rreg, O(REG, MP));
	mem(Ldw, RA0,   Rreg, O(REG, PC));
	JR(RA0, 0);

	if (cflag > 4) {
		iprint("preamble\n");
		das(codestart, code-codestart);
	}

	segflush(comvec, ((ulong)code-(ulong)comvec) * sizeof(*code));
}

int
compile(Module *m, int size, Modlink *ml)
{
	Link *l;
	Modl *e;
	int i, n;
	ulong *s, *tmp;

	iprint("compile allocating initial\n");
	base = nil;
	patch = mallocz(size*sizeof(*patch), 0);
	tinit = malloc(m->ntype*sizeof(*tinit));
	tmp = mallocz(2048*sizeof(ulong), 0);
	iprint("compile allocating done\n");
	iprint("tmp address: 0x%p\n", tmp);

	if (patch == nil || tinit == nil || tmp == nil)
		goto bad;

	// Set base so that addresses are at the same order of magnitude in both passes
	base = tmp;

	preamble();
	codestart = tmp;
	codeend = tmp + 2048;

	mod = m;
	n = 0;
	pass = 0;
	nlit = 0;

	// Do the first pass
	iprint("compile first pass\n");
	for (i = 0; i < size; i++) {
		codeoff = n;
		code = tmp;
		comp(&m->prog[i]);
		patch[i] = n;
		n += code - tmp;
	}
	iprint("first pass used %d instructions\n", n);

	// Generate macros at the end
	iprint("compile first macro\n");
	for (i = 0; i < NMACRO; i++) {
		codeoff = n;
		code = tmp;
		mactab[i].gen();
		macro[mactab[i].idx] = n;
		n += code - tmp;
	}

	iprint("first pass and macros used %d instructions\n", n);

	free(tmp);
	iprint("allocate base with size %d\n", (n+nlit)*sizeof(*code));
	base = mallocz((n+nlit)*sizeof(*code), 0);
	codestart = base;
	codeend = base + n + nlit;
	if (base == nil)
		goto bad;
	iprint("allocate base done\n");
	iprint("base address: 0x%p\n", base);

	if (cflag > 3)
		iprint("dis=%5d %5d risc-v=%5d asm=%.8p: %s\n",
			size, size*sizeof(Inst), n, base, m->name);

	// Prepare for the next pass
	pass++;
	nlit = 0;
	litpool = base + n;
	code = base;
	n = 0;
	codeoff = 0;

	// Translate the instructions
	iprint("compile second pass\n");
	for (i = 0; i < size; i++) {
		s = code;
		comp(&m->prog[i]);

		if (patch[i] != n) {
			// The previous instruction used a different number of instructions
			// than in the first pass, messing up the offsets
			if (cflag <= 4)
			    iprint("%3d %D\n", i, &m->prog[i-1]);
			iprint("First and second pass instruction count doesn't match\n");
			iprint("first pass: %lud\nsecond pass: %d\n", patch[i], n);
			urk("phase error");
		}

		if (cflag > 4) {
			iprint("%3d %D\n", i, &m->prog[i]);
			das(s, code-s);
		}

		n += code - s;
	}

	// Insert the macros
	iprint("compile second macro\n");
	for (i = 0; i < NMACRO; i++) {
		s = code;
		mactab[i].gen();

		if (macro[mactab[i].idx] != n) {
			iprint("mac phase err: %lud != %d\n", macro[mactab[i].idx], n);
			urk("phase error");
		}

		n += code - s;

		if (cflag > 4) {
			iprint("%s:\n", mactab[i].name);
			das(s, code-s);
		}
	}

	iprint("compile m->ext types\n");
	for (l = m->ext; l->name; l++) {
		l->u.pc = (Inst*) RELPC(patch[l->u.pc - m->prog]);
		typecom(l->frame);
	}

	if (ml != nil) {
		e = &ml->links[0];

		iprint("compile ml->links types\n");
		for (i = 0; i < ml->nlinks; i++) {
			e->u.pc = (Inst*) RELPC(patch[e->u.pc - m->prog]);
			typecom(e->frame);
			e++;
		}
	}

	iprint("compile m->type types\n");
	for (i = 0; i < m->ntype; i++) {
		if (tinit[i] != 0)
			typecom(m->type[i]);
	}

	iprint("compile patches\n");
	patchex(m, patch);
	m->entry = (Inst*) RELPC(patch[mod->entry - mod->prog]);

	iprint("compile done\n");
	free(patch);
	free(tinit);
	free(m->prog);
	m->prog = (Inst*) base;
	m->compiled = 1;
	segflush(base, n*sizeof(*base));
	return 1;

bad:
	iprint("compile failed\n");
	free(patch);
	free(tinit);
	free(base);
	free(tmp);
	return 0;
}
