#define KADDR(p) ((void*) p)
#define PADDR(p) ((ulong) p)
#define coherence()     /* nothing needed for uniprocessor */
#define procsave(p)     /* Save the mach part of the current */
						/* process state, no need for one cpu */

void (*screenputs)(char*, int);

#include "../port/portfns.h"

#define	waserror()	(up->nerrlab++, setlabel(&up->errlab[up->nerrlab-1]))

ulong getsp(void);

u32int atomic_inc(u32int* addr);

void wait_for_interrupt(void);

void memory_fence(void);

int inb(ulong);
int ins(ulong);
ulong inl(ulong);
void outb(ulong, int);
void outs(ulong, int);
void outl(ulong, ulong);
void inss(ulong, void*, int);
void outss(ulong, void*, int);
void insb(ulong, void*, int);
void outsb(ulong, void*, int);
char* getconf(char*);

void i8250console(void);

void delay(int);
void trapinit(void);
void intrenable(long irq, void (*f)(Ureg*, void*), void* a, int tbdf, char *name);
int intrdisable(int irq, void (*f)(Ureg *, void *), void *a, int tbdf, char *name);
char *trapname(long scause);
void links(void);
int isvalid_wa(void*);
int isvalid_va(void*);
void dumpregs(Ureg* ureg);

#define	kmapinval()
uintptr mmukmap(uintptr, uintptr, uint);

void clockinit(void);

void screeninit(void);
void* fbinit(int set, int *width, int *height, int *depth);
void fbflush(uint min_x, uint min_y, uint max_x, uint max_y);
int fbblank(int blank);
