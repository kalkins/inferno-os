#include <lib9.h>

#define getrd(w)			(((w) >> 7) & 0x1F)
#define getrs1(w)			(((w) >> 15) & 0x1F)
#define getrs2(w)			(((w) >> 20) & 0x1F)
#define getrs3(w)			(((w) >> 27) & 0x1F)

#define getfunct2(w)			(((w) >> 25) & 0x3)
#define getfunct3(w)			(((w) >> 12) & 0x7)
#define getfunct7(w)			(((w) >> 25) & 0x7F)

#define sign(w)				(((long) (w)) >> 31)
#define imm10(w)			(((w) >> 20) & 0x7FF)
#define imm10t5(w)			(imm10(w) & 0xFE0)
#define imm10t1(w)			(imm10(w) & 0x7FE)
#define imm19t12(w)			((w) & 0xFF000)
#define imm4(w)				(((w) >> 7) & 0x1F)
#define imm4t1(w)			(imm4(w) & 0x1E)
#define imm31(w)			((w) & 0xFFFFF000)

#define getIimm(w)			(imm10(w) | (sign(w) & 0xFFFFF800))
#define getSimm(w)			(imm10t5(w) | imm4(w) | (sign(w) & 0xFFFFF800))
#define getBimm(w)			(imm10t5(w) | imm4t1(w) | (((w) << 4) & 0x800) | (sign(w) & 0xFFFFF000))
#define getUimm(w)			(imm31(w))
#define getJimm(w)			(imm10t1(w) | imm19t12(w) | (((w) >> 9) & 0x800) | (sign(w) & 0xFFF00000))

enum {
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
};

typedef struct {
	ulong w;
	ulong addr;
	ulong op;

	int rd;
	int rs1;
	int rs2;
	int rs3;

	int funct2;
	int funct3;
	int funct7;

	long imm;

	char *buf;
	uint bufsize;
} Instr;

typedef struct {
	char *name;
	int funct2;
	int funct3;
	int funct5;
	int funct7;
	int rd;
	int rs1;
	int rs2;
	int rs3;
} Instrlist;

char *rmodes[] = {
	".rne",
	".rtz",
	".rdn",
	".rup",
	".rmm",
	".reserved-101",
	".reserved-110",
	"",
};

static void
decodeRtype(ulong pc, Instr *i)
{
	i->rd = getrd(i->w);
	i->rs1 = getrs1(i->w);
	i->rs2 = getrs2(i->w);

	i->funct3 = getfunct3(i->w);
	i->funct7 = getfunct7(i->w);
}

static void
decodeR4type(ulong pc, Instr *i)
{
	i->rd = getrd(i->w);
	i->rs1 = getrs1(i->w);
	i->rs2 = getrs2(i->w);
	i->rs3 = getrs3(i->w);

	i->funct2 = getfunct2(i->w);
	i->funct3 = getfunct3(i->w);
}

static void
decodeItype(ulong pc, Instr *i)
{
	i->rd = getrd(i->w);
	i->rs1 = getrs1(i->w);

	i->funct3 = getfunct3(i->w);
	i->imm = getIimm(i->w);
}

static void
decodeStype(ulong pc, Instr *i)
{
	i->rs1 = getrs1(i->w);
	i->rs2 = getrs2(i->w);

	i->funct3 = getfunct3(i->w);
	i->imm = getSimm(i->w);
}

static void
decodeBtype(ulong pc, Instr *i)
{
	i->rs1 = getrs1(i->w);
	i->rs2 = getrs2(i->w);

	i->funct3 = getfunct3(i->w);
	i->imm = getBimm(i->w);
}

static void
decodeUtype(ulong pc, Instr *i)
{
	i->rd = getrd(i->w);
	i->imm = getUimm(i->w);
}

static void
decodeJtype(ulong pc, Instr *i)
{
	i->rd = getrd(i->w);
	i->imm = getJimm(i->w);
}

static int
decodeop(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"add",    .funct3=0, .funct7=0},
		{"sub",    .funct3=0, .funct7=32},
		{"sll",    .funct3=1, .funct7=0},
		{"slt",    .funct3=2, .funct7=0},
		{"sltu",   .funct3=3, .funct7=0},
		{"xor",    .funct3=4, .funct7=0},
		{"srl",    .funct3=5, .funct7=0},
		{"sra",    .funct3=5, .funct7=32},
		{"or",     .funct3=6, .funct7=0},
		{"and",    .funct3=7, .funct7=0},

		{"mul",    .funct3=0, .funct7=1},
		{"mulh",   .funct3=1, .funct7=1},
		{"mulhsu", .funct3=2, .funct7=1},
		{"mulhu",  .funct3=3, .funct7=1},
		{"div",    .funct3=4, .funct7=1},
		{"divu",   .funct3=5, .funct7=1},
		{"rem",    .funct3=6, .funct7=1},
		{"remu",   .funct3=7, .funct7=1},
	};
	char *name = "unknown OP";
	decodeRtype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3 && list[j].funct7 == i->funct7) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, x%d, r%d", name, i->rd, i->rs1, i->rs2);
	return 1;
}

static int
decodeopimm(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"addi",   .funct3=0},
		{"slti",   .funct3=2},
		{"sltiu",  .funct3=3},
		{"xori",   .funct3=4},
		{"ori",    .funct3=6},
		{"andi",   .funct3=7},
	};

	char *name = "unknown OP-IMM";
	decodeItype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, x%d, %d", name, i->rd, i->rs1, i->imm);
	return 1;
}

static int
decodeopload(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"lb",   .funct3=0},
		{"lh",   .funct3=1},
		{"lw",   .funct3=2},
		{"lbu",  .funct3=4},
		{"lhu",  .funct3=5},
	};

	char *name = "unknown LOAD";
	decodeItype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, %d(x%d)", name, i->rd, i->imm, i->rs1);
	return 1;
}

static int
decodeopstore(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"sb",  .funct3=0},
		{"sh",  .funct3=1},
		{"sw",  .funct3=2},
	};

	char *name = "unknown STORE";
	decodeStype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, %d(x%d)", name, i->rs2, i->imm, i->rs1);
	return 1;
}


static int
decodeopfp(ulong pc, Instr *i)
{
	decodeRtype(pc, i);

	// Float instructions with rm, rd, rs1, rs2
	static Instrlist rdrm2rs[] = {
		{"fadd.s",  .funct7=0},
		{"fsub.s",  .funct7=4},
		{"fmul.s",  .funct7=8},
		{"fdiv.s",  .funct7=12},

		{"fadd.d",  .funct7=1},
		{"fsub.d",  .funct7=5},
		{"fmul.d",  .funct7=9},
		{"fdiv.d",  .funct7=13},
	};

	for (int j = 0; j < sizeof(rdrm2rs); j++) {
		if (rdrm2rs[j].funct7 == i->funct7) {
			snprint(i->buf, i->bufsize, "%s%s\tf%d, f%d, f%d",
			        rdrm2rs[j].name, rmodes[i->funct3], i->rd, i->rs1, i->rs2);
			return 1;
		}
	}

	// Float instructions with rd, rs1, rs2
	static Instrlist rd2rs[] = {
		{"fsgnj.s",  .funct3=0, .funct7=16},
		{"fsgnjn.s", .funct3=1, .funct7=16},
		{"fsgnjx.s", .funct3=2, .funct7=16},

		{"fsgnj.d",  .funct3=0, .funct7=17},
		{"fsgnjn.d", .funct3=1, .funct7=17},
		{"fsgnjx.d", .funct3=2, .funct7=17},

		{"fmin.s",   .funct3=0, .funct7=20},
		{"fmax.s",   .funct3=1, .funct7=20},

		{"fmin.d",   .funct3=0, .funct7=21},
		{"fmax.d",   .funct3=1, .funct7=21},

		{"fle.s",    .funct3=0, .funct7=80},
		{"flt.s",    .funct3=1, .funct7=80},
		{"feq.s",    .funct3=2, .funct7=80},

		{"fle.d",    .funct3=0, .funct7=81},
		{"flt.d",    .funct3=1, .funct7=81},
		{"feq.d",    .funct3=2, .funct7=81},
	};

	for (int j = 0; j < sizeof(rd2rs); j++) {
		if (rd2rs[j].funct3 == i->funct3 && rd2rs[j].funct7 == i->funct7) {
			snprint(i->buf, i->bufsize, "%s\tf%d, f%d, f%d", rd2rs[j].name, i->rd, i->rs1, i->rs2);
			return 1;
		}
	}

	// Float instructions with rm, rd, rs1
	static Instrlist rdrmrs[] = {
		{"fsqrt.s\tf%d, f%d",   .funct7=44,  .rs2=0},
		{"fsqrt.d\tf%d, f%d",   .funct7=45,  .rs2=0},

		{"fcvt.w.s\tx%d, f%d",  .funct7=96,  .rs2=0},
		{"fcvt.wu.s\tx%d, f%d", .funct7=96,  .rs2=1},
		{"fcvt.s.w\tf%d, x%d",  .funct7=104, .rs2=0},
		{"fcvt.s.wu\tf%d, x%d", .funct7=104, .rs2=1},

		{"fcvt.s.d\tf%d, f%d",  .funct7=32,  .rs2=1},
		{"fcvt.d.s\tf%d, f%d",  .funct7=33,  .rs2=0},

		{"fcvt.w.d\tx%d, f%d",  .funct7=97,  .rs2=0},
		{"fcvt.wu.d\tx%d, f%d", .funct7=97,  .rs2=1},
		{"fcvt.d.w\tf%d, x%d",  .funct7=105, .rs2=0},
		{"fcvt.d.wu\tf%d, x%d", .funct7=105, .rs2=1},
	};

	for (int j = 0; j < sizeof(rdrmrs); j++) {
		if (rdrmrs[j].funct7 == i->funct7 && rdrmrs[j].rs2 == i->rs2) {
			snprint(i->buf, i->bufsize, rdrmrs[j].name, rmodes[i->funct3], i->rd, i->rs1);
			return 1;
		}
	}

	// Float instructions with rd, rs1
	static Instrlist rdrs[] = {
		{"fmv.x.w\tf%d, x%d",  .funct3=0, .funct7=112, .rs2=0},
		{"fmv.w.x\tx%d, f%d",  .funct3=0, .funct7=120, .rs2=0},

		{"fclass.s\tf%d, f%d", .funct3=1, .funct7=112, .rs2=0},
		{"fclass.d\tf%d, f%d", .funct3=1, .funct7=113, .rs2=0},
	};

	for (int j = 0; j < sizeof(rdrs); j++) {
		if (rdrs[j].funct3 == i->funct3 && rdrs[j].funct7 == i->funct7 && rdrmrs[j].rs2 == i->rs2) {
			snprint(i->buf, i->bufsize, rdrmrs[j].name, i->rd, i->rs1);
			return 1;
		}
	}

	return 0;
}

static int
decodeoploadfp(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"flw", .funct3=2},
		{"fld", .funct3=3},
	};
	char *name = "unknown LOAD-FP";
	decodeItype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, %d(x%d)", name, i->rd, i->imm, i->rs1);
	return 1;
}

static int
decodeopstorefp(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"fsw", .funct3=2},
		{"fsd", .funct3=3},
	};
	char *name = "unknown LOAD-FP";
	decodeStype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, %d(x%d)", name, i->rd, i->imm, i->rs1);
	return 1;
}

static int
decodeopbranch(ulong pc, Instr *i)
{
	static Instrlist list[] = {
		{"beq",  .funct3=0},
		{"bne",  .funct3=1},
		{"blt",  .funct3=4},
		{"bge",  .funct3=5},
		{"bltu", .funct3=6},
		{"bgeu", .funct3=7},
	};
	char *name = "unknown BRANCH";
	decodeBtype(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct3 == i->funct3) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s\tx%d, x%d, 0x%lux", name, i->rs1, i->rs2, i->imm + pc);
	return 1;
}

static int
decodeopmultiplyadd(ulong pc, Instr *i)
{
	// Reuse the funct7 field for the op
	static Instrlist list[] = {
		{"fmadd.s",  .funct2=0, .funct7=OPmadd},
		{"fmadd.d",  .funct2=1, .funct7=OPmadd},

		{"fnmadd.s", .funct2=0, .funct7=OPnmadd},
		{"fnmadd.d", .funct2=1, .funct7=OPnmadd},

		{"fmsub.s",  .funct2=0, .funct7=OPmsub},
		{"fmsub.d",  .funct2=1, .funct7=OPmsub},

		{"fnmsub.s", .funct2=0, .funct7=OPnmsub},
		{"fnmsub.d", .funct2=1, .funct7=OPnmsub},
	};
	char *name = "unknown multiply add";
	decodeR4type(pc, i);

	for (int j = 0; j < sizeof(list); j++) {
		if (list[j].funct2 == i->funct2 && list[j].funct7 == i->op) {
			name = list[j].name;
			break;
		}
	}

	snprint(i->buf, i->bufsize, "%s%s\tx%d, x%d, x%d, x%d", name, rmodes[i->funct3], i->rd, i->rs1, i->rs2, i->rs3);
	return 1;
}

static int
decode(ulong pc, Instr *i)
{
	i->op = i->w & 0x7F;

	switch (i->op) {
	case OP:
		return decodeop(pc, i);
	case OPimm:
		return decodeopimm(pc, i);
	case OPfp:
		return decodeopfp(pc, i);
	case OPlui:
		decodeUtype(pc, i);
		snprint(i->buf, i->bufsize, "lui\tx%d, 0x%lux", i->rd, i->imm);
		return 1;
	case OPauipc:
		decodeUtype(pc, i);
		snprint(i->buf, i->bufsize, "auipc\tx%d, 0x%lux", i->rd, i->imm);
		return 1;
	case OPjal:
		decodeJtype(pc, i);
		snprint(i->buf, i->bufsize, "jal\tx%d, 0x%lux", i->rd, i->imm + pc);
		return 1;
	case OPjalr:
		decodeItype(pc, i);
		snprint(i->buf, i->bufsize, "jalr\tx%d, %d(x%d)", i->rd, i->imm, i->rs1);
		return 1;
	case OPload:
		return decodeopload(pc, i);
	case OPstore:
		return decodeopstore(pc, i);
	case OPloadfp:
		return decodeoploadfp(pc, i);
	case OPstorefp:
		return decodeopstorefp(pc, i);
	case OPbranch:
		return decodeopbranch(pc, i);
	case OPmadd:
	case OPnmadd:
	case OPmsub:
	case OPnmsub:
		return decodeopmultiplyadd(pc, i);
	case OPmiscmem:
	case OPsystem:
	case OPamo:
	default:
		return 0;
	}
}

void
das(ulong *x, int n)
{
	ulong pc;
	Instr i;
	char buf[128];

	pc = (ulong) x;

	while (n > 0) {
		i.w = *(ulong*) pc;
		i.buf = buf;
		i.bufsize = 128;

		if (decode(pc, &i) == 0) {
			sprint(buf, "???");
		}

		iprint("%.8lux %.8lux\t%s\n", pc, i.w, buf);
		pc += 4;
		n--;
	}
}
