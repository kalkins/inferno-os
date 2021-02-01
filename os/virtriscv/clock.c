#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"
#include "csr.h"
#include "sbi.h"

#define TIME_FREQ	(10000000)

#define TIMER_MASK	(INTR_STIMER)

// The value to set a timer infinitely far in the future.
// For some reason setting a higher value causes
// the interrupt to fire anyway, but this should be good enough
#define TIMER_INF	((uvlong) 1 << 60)

void clockintr(Ureg *ureg, void *a);

void
clockinit(void)
{
	// Enable interrupts, but set time timer to max
	timerset(TIMER_INF);
	intrenable(IRQSTimer, clockintr, nil, BUSCORE, "Timer");

	timersinit();
}

void
clockintr(Ureg *ureg, void *a)
{
	timerset(TIMER_INF);
	timerintr(ureg, 0);
}

void
clockcheck(void)
{
	return;
}

uvlong
fastticks(uvlong *hz)
{
	uvlong now = csr_read_timeh();
	now = now << 32;
	now |= csr_read_time();
	if (hz) {
	    *hz = TIME_FREQ;
	}
	return now;
}

void
timerset(uvlong next)
{
	uvlong now = fastticks(0);
	long error = sbi_set_timer(next);
	if (error != SBI_SUCCESS) {
		panic("SBI error when setting timer: %s\n", get_sbi_error(error));
	}
}

void
microdelay(int microsecond)
{
	uvlong hz;
	uvlong now = fastticks(&hz);
	uvlong target = now + microsecond*(hz / (1000000));

	while (now < target) {
		now = fastticks(nil);
	}
}

void delay(int millisecond)
{
	uvlong hz;
	uvlong now = fastticks(&hz);
	uvlong target = now + millisecond*(hz / (1000));

	int i;
	while (now < target) {
		for (i = 0; i < 1000; i++);
		now = fastticks(nil);
	}
}
