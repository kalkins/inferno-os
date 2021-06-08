#include "u.h"
#include "ureg.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

#define SCAN_LIMIT	1024

void
dumplongs(char *msg, ulong *v, int n)
{
	int i, l, pad;

	if (!isvalid_va(v)) {
		return;
	}

	l = 0;
	pad = strlen(msg) + 4;
	iprint("%s at %.8p: ", msg, v);
	for(i=0; i<n; i++){
		if(l >= 4){
			iprint("\n%*c%.8p: ", pad, ' ', v);
			l = 0;
		}
		if(isvalid_va(v)){
			iprint(" %.8lux", *v++);
			l++;
		}else{
			iprint(" invalid");
			break;
		}
	}
	iprint("\n");
}

void
dumparound(ulong addr)
{
	uint addr0 = (addr/16)*16;
	int a_row, a_col;
	uchar ch, *cha;
	uint c;
	/* +-32 bytes to print */
	iprint("%ux:\n", addr0 +(-2)*16);
	for (a_col = 0; a_col<16; ++a_col) {
		iprint("|%.2uX", a_col);
	}
	iprint("\n");

	for (a_row = -2; a_row < 3; ++a_row) {
		for (a_col = 0; a_col<16; ++a_col) {
			cha = (uchar *)(addr0 +a_row*16+a_col);
			ch = *cha;
			c = ch;
			if (cha == (uchar *)addr)
				iprint(">%2.2uX", c);
			else iprint(" %2.2uX", c);
		}
		iprint("\n");
	}
	iprint("\n");
}

int
is_normal_jal(ulong inst)
{
	return (inst & 0x007f) == 0x6f;
}

int
is_compressed_jal(ulong inst)
{
	return (inst & 0xe0030000) == 0x20010000;
}

int
is_compressed_jalr(ulong inst)
{
	return (inst & 0xf07f0000) == 0x90020000;
}

int is_jal(ulong inst)
{
	return is_normal_jal(inst) || is_compressed_jal(inst) || is_compressed_jalr(inst);
}

long
decode_jal(ulong inst)
{
	long offset;

	if (is_normal_jal(inst)) {
		// Normal JAL
		// Load the jump offset
		offset  = (inst & 0x000ff000) >> 0;  // imm[19:12]
		offset |= (inst & 0x00100000) >> 9;  // imm[11]
		offset |= (inst & 0x7fe00000) >> 20; // imm[10:1]

		// Sign extend
		if (inst & (1<<31)) {
			offset |= 0xfff00000;
		}
	} else if (is_compressed_jal(inst)) {
		// Compressed JAL
		inst = inst >> 16;

		offset  = (inst & 0x0004) << 3; // imm[5]
		offset |= (inst & 0x0038) >> 2; // imm[3:1]
		offset |= (inst & 0x0040) << 1; // imm[7]
		offset |= (inst & 0x0080) >> 1; // imm[6]
		offset |= (inst & 0x0100) << 2; // imm[10]
		offset |= (inst & 0x0600) >> 1; // imm[9:8]
		offset |= (inst & 0x0800) >> 7; // imm[4]

		// Sign extend
		if (inst & (1<<12)) {
			offset |= 0xfffff800;
		}

		// The caller address is incorrect, because the instruction was compressed,
		// so we compensate here
		offset += 2;
	} else if (is_compressed_jalr(inst)) {
		// Compressed JALR, but we have no way of knowing
		// where it jumped to
		offset = 0;
	} else {
		offset = -1;
	}

	return offset;
}

int
is_addisp(ulong inst)
{
	// Normal ADDI with R2 as rd and rs1
	return (inst & 0x000fffff) == 0x00010113;
}

int
is_neg_addisp(ulong inst)
{
	// Normal ADDI with R2 as rd and rs1 and negative imm
	return is_addisp(inst) && (inst & 0x80000000);
}

int
is_caddisp(ulong inst)
{
	// Compressed C.ADDIW with R2 as rd and rs1
	return (inst & 0xef83) == 0x0101;
}

int
is_neg_caddisp(ulong inst)
{
	// Compressed C.ADDIW with R2 as rd and rs1 and negative imm
	return is_caddisp(inst) && (inst & 0x1000);
}

int
is_caddi16sp(ulong inst)
{
	// Compressed C.ADDI16SP
	return (inst & 0xef83) == 0x6101;
}

int
is_neg_caddi16sp(ulong inst)
{
	// Compressed C.ADDI16SP with negative imm
	return is_caddi16sp(inst) && (inst & 0x1000);
}

int
is_add_sp(ulong inst)
{
	return is_addisp(inst) || is_caddisp(inst)  || is_caddi16sp(inst);
}

int
is_neg_add_sp(ulong inst)
{
	return is_neg_addisp(inst) || is_neg_caddisp(inst)  || is_neg_caddi16sp(inst);
}

long
decode_add_sp(ulong inst)
{
	long imm;

	if (is_addisp(inst)) {
		// inst[31:20] = imm[11:0]
		imm = (inst & 0xFFF00000) >> 20;

		// Sign extend
		if (imm & (1<<11)) {
			imm |= 0xfffff000;
		}
	} else if (is_caddisp(inst)) {
		// inst[6:2] = imm[4:0]
		imm = (inst & 0x7c) >> 2;

		// Sign extend
		if (inst & (1<<12)) {
			imm |= 0xffffffe0;
		}
	} else if (is_caddi16sp(inst)) {
		imm = 0;
		imm |= (inst & 0x0004) << 3; // nzimm[5]
		imm |= (inst & 0x0018) << 4; // nzimm[8:7]
		imm |= (inst & 0x0020) << 1; // nzimm[6]
		imm |= (inst & 0x0040) >> 2; // nzimm[4]

		// Sign extend
		if (inst & 0x1000) {
			imm |= 0xfffffe00;
		}
	} else {
		return -1;
	}

	return imm;
}

void*
scan_for_stack_dec(void *start_addr)
{
	ulong addr = (ulong) start_addr;
	ulong inst;

	for (int i = 0; i < SCAN_LIMIT; i++) {
		addr -= 2;

		if (!isvalid_va((void*) addr)) {
			break;
		}

		inst = *((ulong*) addr);

		if (is_neg_add_sp(inst)) {
			return (void*) addr;
		}
	}

	return 0;
}

void*
scan_for_stack_link(void *start_addr, void *end_addr)
{
	// Go through the stack and look for link addresses
	ulong *addr;
	ulong *link;

	for (addr = start_addr; (void*) addr <= end_addr; addr++) {
		// Get the address from the stack
		link = (void*) *addr;
		// Decrement to get the jump instruction
		link--;

		// See if it is a valid address, and that address contains a JAL instruction
		if (isvalid_wa(link) && is_jal(*link)) {
			return addr;
		}
	}

	return 0;
}

static void
_dumpstack(Ureg *ureg)
{
	ulong *l;
	ulong inst;
	ulong *estack;
	ulong *caller;
	ulong *prev_caller;
	ulong *callee;
	ulong **link;

	l = (ulong*) ureg->sp;

	iprint("ktrace PC: 0x%lux SP: 0x%lux\n", ureg->pc, ureg->sp);
	if(up != nil && l >= (ulong*) up->kstack && l <= (ulong*)(up->kstack+KSTACK-4)) {
		estack = (ulong*) (up->kstack+KSTACK-4);
		iprint("Process stack:  0x%8.8lux-0x%p\n", up->kstack, estack);
	} else if(l >= (ulong*) m->stack && l <= ((ulong*)m+BY2PG-4)) {
		estack = (ulong*) m+BY2PG-4;
		iprint("System stack: 0x%8.8lux-0x%p\n", (ulong)(m+1), estack);
	} else if(l >= (ulong*) RAMZERO && l <= (ulong*) TRAPSTACK) {
		estack = (ulong*) TRAPSTACK;
		iprint("Trap stack: 0x%8.8lux-0x%p\n", (ulong)RAMZERO, estack);
	} else if(l >= (ulong*) RAMBOOT && l < (ulong*) RAMZERO) {
		estack = (ulong*) TRAPSTACK;
		l = (ulong*) RAMZERO;
		iprint("Stack seems to have overflowed into the bootloader memory. Trying to search at RAMZERO\n");
	} else {
		iprint("unknown stack\n");
		return;
	}

	/*
	link = scan_for_stack_link(l, estack);

	if (link == 0) {
		iprint("Could not find a link address on the stack. Aborting trace\n");
		return;
	} else if ((ulong) *link == ureg->r1) {
		// The link address and link register match, so it's correct
		l = ((ulong*) link) + 1;
		caller = *link - 1;
		iprint("0x%p called the function containing 0x%p\n", caller, ureg->pc);
	} else {
		// The link address and link register doesn't match.
		// This could be because the function is a leaf,
		// or because another function was called and returned.

		// Try to look for a stack decrement instruction
		ulong *decaddr = scan_for_stack_dec((void*) ureg->pc);

		if (isvalid_wa(decaddr)) {
			// Compare the
			long imm = decode_add_sp(*decaddr);

			if (imm == (ulong) link - (ulong) l) {
				// The stack decrement matches the link
				// address on the stack, so it's confirmed
				l = ((ulong*) link) + 1;
				caller = *link - 1;
				iprint("0x%p called the function containing 0x%p\n", caller, ureg->pc);
			} else {

			}
		}
	}
	*/

	iprint("Call stack:\n");

	while (l < estack) {
		if (!isvalid_wa(caller)) {
			iprint("Illegal link address 0x%p\n", caller);
			iprint("Aborting stack trace\n");
			break;
		}

		// Scan through the stack after the next link address
		link = scan_for_stack_link(l, estack);
		if (link != 0) {
			l = ((ulong*) link) + 1;
			caller = *link - 1;

			if (is_compressed_jal(*caller) || is_compressed_jalr(*caller)) {
				caller = (void*) (((char*) caller) + 2);
			}

			iprint("0x%p\n", caller);
		} else {
			iprint("Searching for link on stack failed\n");
			break;
		}
	}
}

/*
 * Fill in enough of Ureg to get a stack trace, and call a function.
 * Used by debugging interface rdb.
 */
void
callwithureg(void (*fn)(Ureg*))
{
	Ureg ureg;
	ureg.pc = (ulong) callwithureg;
	ureg.sp = getsp();
	ureg.r1 = 0;
	fn(&ureg);
}

void
dumpstack(void)
{
	callwithureg(_dumpstack);
}

void
dumpregs(Ureg* ureg)
{
	iprint("Mode  0x%08lux   PC    0x%08lux   status  0x%08lux   cause   0x%08lux   tval     0x%08lux \n",
	      ureg->curmode, ureg->pc, ureg->status, ureg->cause, ureg->tval);
	iprint("R15   0x%08lux   R14   0x%08lux   R13     0x%08lux   R12     0x%08lux   R11      0x%08lux\n",
		ureg->r15, ureg->r14, ureg->r13, ureg->r12, ureg->r11);
	iprint("R10   0x%08lux   R9    0x%08lux   R8      0x%08lux   R7      0x%08lux   R6       0x%08lux\n",
	       ureg->r10, ureg->r9, ureg->r8, ureg->r7, ureg->r6);
	iprint("R5    0x%08lux   R4    0x%08lux   R3      0x%08lux   R2/sp   0x%08lux   R1/link  0x%08lux\n",
	       ureg->r5, ureg->r4, ureg->r3, ureg->r2, ureg->r1);

	dumplongs("stack", (ulong *)(ureg->sp), 16);
	iprint("\n");
	_dumpstack(ureg);
	iprint("\n");
}
