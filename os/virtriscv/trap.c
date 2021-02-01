#include "u.h"
#include "ureg.h"
#include "../port/lib.h"
#include "dat.h"
#include "fns.h"
#include "csr.h"
#include "mem.h"
#include "io.h"
#include "plic.h"

// Bits in SIE and SIP
#define CAT_SOFTWARE	(1 << 1)
#define CAT_TIMER	(1 << 5)
#define CAT_EXCEPTION	(1 << 9)

extern void traphandler(void);

static Lock vctllock;
static Vctl *vctl[256];

enum
{
	Ntimevec = 20		/* number of time buckets for each intr */
};
ulong intrtimes[256][Ntimevec];

static int trap_category(long irq)
{
	if (irq & (1 << 31)) {
	    if (irq == IRQSTimer) {
		    return CAT_TIMER;
	    } else {
		    return CAT_SOFTWARE;
	    }
	} else {
		return CAT_EXCEPTION;
	}
}

static int
getvno(int irq, int tbdf)
{
	switch(tbdf) {
	case BUSCORE:
		if (irq & 1<<31) {
			return 64 + (irq & 0x3f);
		} else {
			return (irq & 0x3f);
		}
	case BUSPLIC:
		return 128 + irq;
	default:
		return -1;
	}
}

static void
enableintr(long irq, int tbdf)
{
	int category = trap_category(irq);
	csr_set_sie(category);

	switch(tbdf) {
	case BUSPLIC:
		plicenable(irq);
		break;
	case BUSCORE:
		break;
	default:
		panic("intrenable: unknown irq bus %d", tbdf);
	}
}

static void
disableintr(long irq, int tbdf)
{
	int category = trap_category(irq);
	csr_clear_sie(category);

	switch(tbdf) {
	case BUSPLIC:
		plicdisable(irq);
		break;
	case BUSCORE:
		break;
	default:
		panic("intrdisable: unknown irq bus %d", tbdf);
	}
}

void
intrenable(long irq, void (*f)(Ureg*, void*), void* a, int tbdf, char *name)
{
	int vno;
	Vctl *v;

	if(f == nil){
		iprint("intrenable: nil handler for %ld, tbdf 0x%uX for %s\n",
			irq, tbdf, name);
		return;
	}

	v = xalloc(sizeof(Vctl));
	v->isintr = 1;
	v->irq = irq;
	v->tbdf = tbdf;
	v->f = f;
	v->a = a;
	strncpy(v->name, name, KNAMELEN-1);
	v->name[KNAMELEN-1] = 0;

	ilock(&vctllock);

	vno = getvno(irq, tbdf);
	if(vno < 0){
		iunlock(&vctllock);
		iprint("intrenable: couldn't enable irq %ld, tbdf 0x%uX for %s\n",
			irq, tbdf, v->name);
		xfree(v);
		return;
	}
	if(vctl[vno]){
		if(vctl[vno]->isr != v->isr || vctl[vno]->eoi != v->eoi)
			panic("intrenable: handler: %s %s %luX %luX %luX %luX\n",
				vctl[vno]->name, v->name,
				vctl[vno]->isr, v->isr, vctl[vno]->eoi, v->eoi);
		v->next = vctl[vno];
	} else {
		// This is the first handler of this type
		enableintr(irq, tbdf);
	}
	vctl[vno] = v;
	iunlock(&vctllock);
}

int
intrdisable(int irq, void (*f)(Ureg *, void *), void *a, int tbdf, char *name)
{
	Vctl **pv, *v;
	int vno;

	vno = getvno(irq, tbdf);
	ilock(&vctllock);

	pv = &vctl[vno];
	while (*pv &&
		  ((*pv)->irq != irq || (*pv)->tbdf != tbdf || (*pv)->f != f || (*pv)->a != a ||
		   strcmp((*pv)->name, name)))
		pv = &((*pv)->next);
	assert(*pv);

	v = *pv;
	*pv = (*pv)->next;	/* Link out the entry */

	if (!vctl[vno]) {
		// This was the last handler of this type
		disableintr(irq, tbdf);
	}

	iunlock(&vctllock);
	xfree(v);
	return 0;
}

static int
handleintr(Ureg *ureg, int vno)
{
	int n = 0;
	//ilock(&vctllock);

	for (Vctl *v = vctl[vno]; v != nil; v = v->next) {
		n++;
		if (v->f != nil) {
			v->f(ureg, v->a);
		} else {
			iprint("WARNING: Intr handler %s function is nil\n", v->name);
		}
	}

	//iunlock(&vctllock);

	return n;
}

static void
devinterrupt(Ureg *ureg)
{
	long irq = plicclaim();
	int n = handleintr(ureg, getvno(irq, BUSPLIC));

	if (n) {
		plic_complete(irq);
	} else {
		iprint("WARNING: Unhandled PLIC interrupt %ld\n", irq);
	}
}

static void
traperror(Ureg *ureg)
{
	static int first = 1;

	// If this function is called multiple times, something is wrong with debug printing.
	// Enter an infinite loop to prevent endless cascades.
	if (first == 1) {
		first = 0;
		iprint("ERROR: 0x%lux=0x%lux: %s\n", ureg->pc, ureg->tval, trapname(ureg->cause));

		iprint("Dumping regs and stack\n");
		dumpregs(ureg);

		iprint("Exiting\n");
		exit(-1);
	}

	for (;;) {}
}

// This is called by the assembly traphandler after registers are saved,
// with an Ureg representing the state when the trap occured.
// It should return normally, traphandler handles SRET
void
kerneltrap(Ureg *ureg)
{
	static int in_error_trap = 0;
	long cause = csr_read_scause();
	long status = csr_read_sstatus();
	long pending = csr_read_sip();
	ulong epc = csr_read_sepc();
	uintptr sp = getsp();

	if (cause == IRQSExternal) {
		devinterrupt(ureg);
	} else if (cause == ERRLoadAccess) {
		traperror(ureg);
	} else {
		int is_error = !(cause & 1<<31);

		if (is_error) {
			if (in_error_trap == 1) {
				iprint("ERROR TRAP WHILE HANDLING ANOTHER ERROR\n");
				traperror(ureg);
			} else {
				in_error_trap = 1;
			}
		}

		int n = handleintr(ureg, getvno(cause, BUSCORE));

		if (n == 0 && is_error) {
			iprint("UNHANDLED ERROR TRAP\n");
			traperror(ureg);
		}

		in_error_trap = 0;
	}

	// Clear pending interrupts
	csr_clear_sip(pending);

	// Restore trap registers, in case traps happened during this function
	csr_write_sepc(epc);
	csr_write_sstatus(status);
}

void
trapinit(void)
{
	// Set trap handler
	// This produces the warning "address of array/func ignored", but it works fine
	ulong handler_addr = (ulong) &traphandler;

	if (handler_addr & 0x00000003) {
		// The compiler should make sure functions are aligned, but
		// just in case
		iprint("WARNING: traphandler not aligned. Rounding address down");
		handler_addr &= 0xfffffffc;
	}

	csr_write_stvec(handler_addr | TRAP_MODE_DIRECT);

	// Set sscratch to the trap stack location
	csr_write_sscratch(TRAPSTACK);

	// Make sure all interrupts are disabled
	csr_write_sie(0);

	// Initialize PLIC interrupts
	plicinit();

	// Enable interrupts globally
	spllo();
}

char*
trapname(long cause)
{
	switch (cause) {
	case IRQUSoftware:
		return "User software interrupt";
	case IRQSSoftware:
		return "Supervisor software interrupt";
	case IRQUTimer:
		return "User timer interrupt";
	case IRQSTimer:
		return "Supervisor timer interrupt";
	case IRQUExternal:
		return "User external interrupt";
	case IRQSExternal:
		return "Supervisor external interrupt";
	case ERRInstrMisaligned:
		return "Instruction misaligned";
	case ERRInstrAccess:
		return "Instruction access fault";
	case ERRInstrIllegal:
		return "Illegal instruction";
	case ERRBreakpoint:
		return "Breakpoint";
	case ERRLoadMisaligned:
		return "Load misaligned";
	case ERRLoadAccess:
		return "Load access fault";
	case ERRStoreMisaligned:
		return "Store misaligned";
	case ERRStoreAccess:
		return "Store access fault";
	case ERRInstrPageFault:
		return "Instruction page fault";
	case ERRLoadPageFault:
		return "Load page fault";
	case ERRStorePageFault:
		return "Store page fault";
	case UCALL:
		return "UCALL";
	case SCALL:
		return "SCALL";
	default:
		if (cause < 0){
			return "Unknown interrupt";
		} else {
			return "Unknown trap";
		}
	}
}

int
isvalid_va(void *v)
{
	return (ulong)v < conf.topofmem && (ulong)v > RAMZERO;
}

int
isvalid_wa(void *v)
{
	return isvalid_va(v) && !((ulong)v & 1);
}
