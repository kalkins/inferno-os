#define IOBASE 0x10001000

#define BUSUNKNOWN	(-1)

/*
 * Trap codes.
 *
 * Interrupts (IRQ) have MSB set to 1.
 *
 * The letter following IRQ specifies if it is for
 * supervisor mode (S) or user mode (U).
 *
 * Most other traps are errors, except UCALL and SCALL,
 * which are environment calls.
 */
#define IRQUSoftware		(0 | 1<<31)
#define IRQSSoftware		(1 | 1<<31)
#define IRQUTimer		(4 | 1<<31)
#define IRQSTimer		(5 | 1<<31)
#define IRQUExternal		(8 | 1<<31)
#define IRQSExternal		(9 | 1<<31)
#define ERRInstrMisaligned	0
#define ERRInstrAccess		1
#define ERRInstrIllegal		2
#define ERRBreakpoint		3
#define ERRLoadMisaligned	4
#define ERRLoadAccess		5
#define ERRStoreMisaligned	6
#define ERRStoreAccess		7
#define ERRInstrPageFault	12
#define ERRLoadPageFault	13
#define ERRStorePageFault	15
#define UCALL			8
#define SCALL			9

#define TRAP_MODE_DIRECT	0
#define TRAP_MODE_VECTORED	1

#define INTR_USOFTWARE		(1<<0)
#define INTR_SSOFTWARE		(1<<1)
#define INTR_UTIMER		(1<<4)
#define INTR_STIMER		(1<<5)
#define INTR_UEXTERNAL		(1<<8)
#define INTR_SEXTERNAL		(1<<9)

#define BUSCORE		0
#define BUSPLIC		1


typedef struct Vctl {
	struct Vctl*	next;			/* handlers on this vector */

	char	name[KNAMELEN];		/* of driver */
	int	isintr;			/* interrupt or fault/trap */
	long	irq;
	int	tbdf;
	int	(*isr)(int);		/* get isr bit for this irq */
	int	(*eoi)(int);		/* eoi */

	void	(*f)(Ureg*, void*);	/* handler to call */
	void*	a;			/* argument to call it with */
} Vctl;
