#include "u.h"
#include "ureg.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "io.h"
#include "plic.h"

static Lock pliclock;

void
plicinit(void)
{
	*(long*)(PLIC_SPRIORITY) = 0;
}

void
plicenable(long irq)
{
	*(long*)(PLIC_BASE + irq*4) = 1;
	*(long*)(PLIC_SENABLE) |= (1<<irq);
}

void
plicdisable(long irq)
{
	*(long*)(PLIC_SENABLE) &= ~(1<<irq);
}

long
plicclaim(void)
{
	return *(long*) PLIC_SCLAIM;
}

void
plic_complete(long irq)
{
	*(long*) PLIC_SCLAIM = irq;
}
