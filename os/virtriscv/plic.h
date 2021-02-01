/*
 * Platform-level interrupt controller
 */
#define PLIC_BASE		0x0c000000
#define PLIC_PRIORITY	(PLIC_BASE + 0x0)
#define PLIC_PENDING	(PLIC_BASE + 0x1000)
#define PLIC_SENABLE	(PLIC_BASE + 0x2080)
#define PLIC_SPRIORITY	(PLIC_BASE + 0x201000)
#define PLIC_SCLAIM		(PLIC_BASE + 0x201004)

#define PLICUART	10

void plicinit(void);
void plicenable(long irq);
void plicdisable(long irq);
long plicclaim(void);
void plic_complete(long irq);
