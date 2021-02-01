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

	// If we are in a leaf we have to get the link address from R1.
	// Otherwise, we get the link address from 0(R2).
	// The problem is knowing if we are in a leaf or not.
	int leaf = 0;
	caller = 0;
	ulong *linkreg = ((ulong*) ureg->r1);
	ulong *linkstack = ((ulong*) (*l));

	if (isvalid_wa(linkreg)) {
		inst = *(linkreg-1);
		long offset = decode_jal(inst);

		if (offset != -1) {
			long imm = decode_add_sp(*((ulong*) ((ulong) linkreg + offset)));
			if (imm == -1) {
				caller = linkreg;
				leaf = 1;
			}
		}
	} else if (linkreg != 0) {
		iprint("WARNING: Invalid link register value 0x%p\n", linkreg);
	}

	if (caller == 0) {
		if (isvalid_wa(linkstack)) {
			caller = linkstack;
		} else {
			iprint("ERROR: Both the link register (0x%p) and link on stack (0x%p) are invalid.\n", linkreg, linkstack);
			iprint("Aborting stack trace\n");
			return;
		}
	}

	// Subtract one to get the address of the instruction that called the function
	caller -= 1;

	prev_caller = 0;

	while(l<estack) {
		if (!isvalid_wa(caller)) {
			iprint("Illegal link address 0x%p\n", caller);
			iprint("Aborting stack trace\n");
			break;
		}

		inst = *caller;

		int jaloff = decode_jal(inst);
		if (jaloff == -1) {
			iprint("ERROR:\n");
			iprint("Instruction at link address 0x%p is not a JAL. This might be the top of a proc\n", caller);
			iprint("The instruction was 0x%08lux\n", inst);
			dumplongs("caller", caller-12, 32);
			iprint("Aborting stack trace\n");
			break;
		} else if (jaloff == 0) {
			// The decoding failed without error. The caller is known, but the callee is not.

			// Scan through the stack after the next link address
			ulong **link = scan_for_stack_link(l, estack);
			if (link != 0 && *link == caller) {
				iprint("0x%p called the function containing 0x%p\n", caller, prev_caller);
				l = (ulong*) link;
				prev_caller = caller;
				caller = *link;
				continue;
			}

			// Look through the instructions after the link register being
			// saved to stack. If it doesn't occur after 200 instructions,
			// print an error
			if (leaf == 0) {
			    int i;
			    ulong addr = (ulong) prev_caller;
			    callee = scan_for_stack_dec(prev_caller);

			    if (callee == 0) {
				    iprint("ERROR: The function at 0x%p is longer than the limit of %d instructions\n", prev_caller, SCAN_LIMIT);
				    iprint("Stack scanning failed. Aborting stack trace.\n");
				    break;
			    }
			} else {
				iprint("ERROR:\n");
				iprint("Could not determine jump location\n");
				break;
			}
		} else {
			// The decoding worked. Calculate the address
			callee = (ulong*) (((ulong) caller) + jaloff);
		}

		// Check that the instruction is "ADD $x, R2, R2"
		inst = *callee;
		long addimm = decode_add_sp(inst);
		if (addimm == -1) {
			if (leaf == 1) {
				// Treat it as a zero, to not change l
				addimm = 0;
			} else {
				iprint("ERROR:\n");
				iprint("The first instruction in the function at 0x%p does not grow the stack.\n", callee);
				iprint("The instruction was 0x%08lux\n", inst);
				dumplongs("callee", callee-12, 32);
				iprint("Aborting stack trace\n");
				break;
			}
		}

		iprint("0x%p called from 0x%p\n", callee, caller);

		if (addimm > 0) {
			iprint("ERROR:\n");
			iprint("The stack is incremented, not decremented, at 0x%p.\n", callee);
			dumplongs("callee", callee-12, 32);
			iprint("Aborting stack trace\n");
			break;
		}

		// The next function is not a leaf.
		leaf = 0;
		// Increment the stack. l has to be cast back and forth to avoid pointer alignment.
		// addimm is negative, so it's subtracted to increment.
		l =  (ulong*) (((ulong) l) - addimm);
		// The next link address is stored at 0(R2). Decrement to get the address of the caller.
		prev_caller = caller;
		caller = (ulong*) (*l);

		if (caller == 0) {
			break;
		}

		caller -= 1;
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
	ureg.pc = (ulong) &callwithureg;
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
	iprint("Mode  0x%08lux   PC   0x%08lux   status  0x%08lux   cause    0x%08lux   tval  0x%08lux \n",
	      ureg->curmode, ureg->pc, ureg->status, ureg->cause, ureg->tval);
	iprint("R14   0x%08lux   R13  0x%08lux   R12     0x%08lux   R11      0x%08lux   R10   0x%08lux\n",
		ureg->r14, ureg->r13, ureg->r12, ureg->r11, ureg->r10);
	iprint("R9    0x%08lux   R8   0x%08lux   R7      0x%08lux   R6       0x%08lux   R5    0x%08lux\n",
		ureg->r9, ureg->r8, ureg->r7, ureg->r6, ureg->r5);
	iprint("R4    0x%08lux   R3   0x%08lux   R2/sp   0x%08lux   R1/link  0x%08lux\n",
		ureg->r4, ureg->r3, ureg->r2, ureg->r1);

	dumplongs("stack", (ulong *)(ureg->sp), 16);
	iprint("\n");
	_dumpstack(ureg);
	iprint("\n");
}
