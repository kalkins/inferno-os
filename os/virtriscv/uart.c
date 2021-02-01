#include "u.h"
#include "../port/lib.h"
#include "dat.h"
#include "fns.h"
#include "uart.h"

#define UART_BASE 0x10000000
#define UART_SIZE 0x8
#define FIFO_SIZE 16

#define Reg(p, reg)		((volatile unsigned char *)(p->regs + reg))
#define ReadReg(p, reg)		(*(Reg(p, reg)))
#define WriteReg(p, reg, v)	(*(Reg(p, reg)) = (v))
#define SetReg(p, reg, v)	WriteReg(p, reg, ReadReg(p, reg) | v)
#define ClearReg(p, reg, v)	WriteReg(p, reg, ReadReg(p, reg) & ~v)

enum {
	Nuart	= 1,
	Bspeed	= 115200,

	RBR	= 0,	// input
	THR	= 0,	// output
	DLL	= 0,	// divisor LSB
	DLM	= 1,	// divisor MSB
	IER	= 1,	// interrupt enable
	FCR	= 2,	// FIFO control
	ISR	= 2,	// interrupt status
	LCR	= 3,	// line control
	LSR	= 5,	// line status

	// LCR fields
	Sone	= 0x00, // 1 stop bit
	Stwo	= 0x04, // 1.5 or 2 stop bit (5 bit or 6-8 bit word)
	Pno	= 0x00,	// No parity
	Podd	= 0x08,	// Odd parity
	Peven	= 0x18,	// Even parity
	Phigh	= 0x28,	// High parity
	Plow	= 0x38,	// Low parity
	Brsig	= 0x40, // Break signal
	DLAB	= 1 << 7,
};

typedef struct Uart {
	QLock;

	char *name;
	void *regs;

	int baud;
	int bits;
	int parity;
	int brsig;
	int stopb;
	int dlab;

	int enabled;

	/* FIFO */
	Lock	flock;
	int	fifoon;
	int	fifotrigger;

	/* Buffers */
	Queue	*iq;
	Queue	*oq;

	/* Read */
	Lock	rlock;

	/* Write */
	Lock	wlock;
} Uart;

static Uart *uart[Nuart];
static Uart *consuart;
static int nuart = 0;

/*
 * Enable or disable DLAB
 */
static void
setdlab(Uart *p, int on) {
	if (on) {
		SetReg(p, LCR, DLAB);
	} else {
		ClearReg(p, LCR, DLAB);
	}

	p->dlab = (on != 0);
}
/*
 * Set baud rate
 */
static void
setbaud(Uart *p, int rate)
{
	int divisor;
	int dlab;

	if (rate < 0) {
		return;
	}

	if (p->dlab) {
		dlab = 1;
	} else {
		setdlab(p, 1);
		dlab = 0;
	}

	// Get the divisor and write it to the registers
	divisor = Bspeed / rate;
	WriteReg(p, DLL, divisor & 0xff);
	WriteReg(p, DLM, (divisor >> 8) & 0xff);
	p->baud = rate;

	// Restore DLAB state
	if (dlab == 0) {
		setdlab(p, 0);
	}
}

/*
 * Set parity type
 */
static void
setparity(Uart *p, int type)
{
	int lcr = ReadReg(p, LCR);
	lcr = (lcr & 0xc7) | type;
	p->parity = type;
}

/*
 * Set data word length. Must be 5-8 bits
 */
static void
setbits(Uart *p, int bits)
{
	if (bits < 5 || bits > 8) {
		return;
	}

	int lcr = ReadReg(p, LCR);
	lcr = (lcr & 0xfffc) | (bits - 5);
	WriteReg(p, LCR, lcr);
	p->bits = bits;
}

/*
 * Enable or disable break signal
 */
static void
setbrsig(Uart *p, int on) {
	if (on) {
		SetReg(p, LCR, Brsig);
	} else {
		ClearReg(p, LCR, Brsig);
	}

	p->brsig = (on != 0);
}

/*
 * Set the number of stop bits. The options are Sone or Stwo
 */
static void
setstopb(Uart *p, int type)
{
	int lcr = ReadReg(p, LCR);
	lcr = (lcr & 0x04) | type;
	p->stopb = type;
}

/*
 * Enable or disable FIFO.
 *
 * The trigger attribute is how many bytes has to be received before an
 * interrupt is triggered. Legal values are 1, 4, 8, and 14.
 */
static void
setfifo(Uart *p, int on, int trigger, int clearin, int clearout)
{
	int v = 0;
	v |= (on != 0);
	v |= (clearin != 0) << 1;
	v |= (clearout != 0) << 2;

	switch (trigger) {
	case 1:
		break;
	case 4:
		v |= 1 << 6;
		break;
	case 8:
		v |= 1 << 7;
		break;
	case 14:
		v |= 3 << 6;
	default:
		// Default to 1 byte
		trigger = 1;
		break;
	}

	WriteReg(p, FCR, v);
	p->fifoon = (on != 0);
	p->fifotrigger = trigger;
}

static void
uartenable(Uart *p)
{
	// Enable interrupts
	WriteReg(p, IER, 0x01);
	p->enabled = 1;
}

static void
uartdisable(Uart *p)
{
	// Disable interrupts
	WriteReg(p, IER, 0x00);
	p->enabled = 0;
}

void
uartsetup(ulong port, ulong baud, char *name)
{
	Uart *p = uart[port];

	p->name = name;
	p->regs = (void*) (UART_BASE + UART_SIZE*port);

	setdlab(p, 0);
	setbaud(p, 9600);
	setbits(p, 8);
	setstopb(p, Sone);
	setparity(p, Pno);
	setbrsig(p, 0);

	// Enable interrupts
	uartenable(p);

	// Enable and clear FIFO
	setfifo(p, 1, 1, 1, 1);
}

void
uartputc(int c)
{
	// Wait until the FIFO is empty.
	// This is marked by a 1 in bit 5 of LSR.
	while ((ReadReg(LSR) & 1<<5) == 0) {}

	// Write the byte to THR
	WriteReg(THR, c);
}

void
uartputs(char *s, int len)
{
	int i = 0; // The index in the string
	int f; // The number of bytes written to the FIFO

	while (i < len) {
		// Wait until the FIFO is empty.
		while ((ReadReg(LSR) & 1<<5) == 0) {}

		// Write up to 16 bytes to the FIFO
		for (f = 0; f < FIFO_SIZE && i < len; f++) {
			WriteReg(THR, s[i]);
			i++;
		}
	}
}

int
uartgetc(void)
{
	// Wait until there is data in the FIFO.
	// This is marked by a 1 in bit 0 of LSR.
	while ((ReadReg(LSR) & 1) == 0) {}

	// Read the byte from RBR.
	return ReadReg(RBR);
}

void
uartintr(void)
{
	char c;

	// Read incoming characters
	while ((ReadReg(LSR) & 1) == 1) {
		c = ReadReg(RBR);
	}
}
