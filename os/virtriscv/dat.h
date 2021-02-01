#define HZ          (100)       /*! clock frequency */
#define MS2HZ       (1000/HZ)   /*! millisec per clock tick */
#define TK2SEC(t)   ((t)/HZ)    /*! ticks to seconds */
#define MS2TK(t)    ((t)/MS2HZ) /*! milliseconds to ticks */

#define MACHP(n)    (n == 0 ? (Mach*)(MACHADDR) : (Mach*)0)

typedef struct Lock Lock;
typedef struct Ureg Ureg;
typedef struct Label Label;
typedef struct FPenv FPenv;
typedef struct Mach Mach;
typedef struct FPU FPU;
typedef ulong Instr;
typedef struct Conf Conf;
typedef struct ISAConf ISAConf;
typedef struct TrapStack TrapStack;

struct Lock
{
    ulong   key;
    ulong   sr;
    ulong   pc;
    int pri;
};

struct Label
{
	ulong sp;
	ulong pc;
};

enum /* FPenv.status */
{
    FPINIT,
    FPACTIVE,
    FPINACTIVE
};

struct FPenv
{
    int x;
};

struct  FPU
{
    FPenv env;
};

struct Conf
{
	ulong   nmach;      /* processors */
	ulong   nproc;      /* processes */
	ulong   npage;      /* total physical pages of memory */
	ulong   npage0;     /* total physical pages of memory */
	ulong   npage1;     /* total physical pages of memory */
	ulong   base0;      /* base of bank 0 */
	ulong   base1;      /* base of bank 1 */
	ulong   ialloc;     /* max interrupt time allocation in bytes */
	ulong   topofmem;   /* top addr of memory */
	int	monitor;    /* flag for monitor available */
};

struct TrapStack
{
	ulong	regs[32];	/* integer registers */
	double	fregs[32];	/* floating point registers. Assumed to be double */
};

#include "../port/portdat.h"

struct Mach
{
	ulong	splpc;		/* pc of last caller to splhi */
	int     machno;     /* physical id of processor */
	ulong   ticks;      /* of the clock since boot time */
	Proc*   proc;       /* current process on this processor */
	Label   sched;      /* scheduler wakeup */

	// Stacks for exceptions
	ulong	fiqstack[4];
	ulong	irqstack[4];
	ulong	abtstack[4];
	ulong	undstack[4];

	// Detect if kernel stack reaches Mach
	int stack[1];
};

extern Mach *m;
extern Proc *up;

struct
{
	Lock;
	int	machs;			/* bitmap of active CPUs */
	int	exiting;		/* shutdown */
	int	ispanic;		/* shutdown in response to a panic */
	int	thunderbirdsarego;	/* lets the added processors continue to schedinit */
} active;

/*
 *  hardware info about a device
 */
typedef struct {
	ulong	port;
	int	size;
} Devport;

struct DevConf
{
	ulong	intnum;			/* interrupt number */
	char	*type;			/* card type, malloced */
	int	nports;			/* Number of ports */
	Devport	*ports;			/* The ports themselves */
};

/*
 *  a parsed .ini line
 */
#define NISAOPT		8

struct ISAConf {
	char	*type;
	ulong	port;
	int	irq;
	ulong	dma;
	ulong	mem;
	ulong	size;
	ulong	freq;

	int	nopt;
	char	*opt[NISAOPT];
};
