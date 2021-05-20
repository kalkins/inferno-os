#include "u.h"
#include "../port/lib.h"
#include "dat.h"
#include "mem.h"
#include "fns.h"
#include "../port/uart.h"
#include "sbi.h"
#include "virtio.h"
#include "version.h"

#define	MAXCONF		32

Conf conf;
Mach *m = (Mach*)MACHADDR;
Proc *up = 0;

char *confname[MAXCONF];
char *confval[MAXCONF];
int nconf;

extern int main_pool_pcnt;
extern int heap_pool_pcnt;
extern int image_pool_pcnt;

/* Unimplemented functions */
void    fpinit(void) {}
void    FPsave(void*) {}
void    FPrestore(void*) {}
int     segflush(void*, ulong) { return 0; }
void    idlehands(void) { return; }
void    setpanic(void) { return; }

int
pcmspecial(char *idstr, ISAConf *isa)
{
	return -1;
}

void
exit(int panic)
{
	if (panic) {
		print("PANIC\n");
	}

	SBI_SHUTDOWN();
	for (;;);
}

void
reboot(void)
{
	// Don't know how to talk to QEMU, but illegal write does the trick
	spllo();
	print("Rebooting\n");
	(*(volatile unsigned char*)(0x0000)) = 1;
}

void
halt(void)
{
	spllo();
	print("CPU halted\n");
	while (0) {}
}

void
addconf(char *name, char *val)
{
	if(nconf >= MAXCONF)
		return;
	confname[nconf] = name;
	confval[nconf] = val;
	nconf++;
}

char*
getconf(char *name)
{
	int i;

	for(i = 0; i < nconf; i++)
		if(cistrcmp(confname[i], name) == 0)
			return confval[i];
	return 0;
}

void
confinit(void)
{
	ulong base;
	conf.topofmem = 64*MiB + RAMZERO;

	base = PGROUND((ulong)end);
	conf.base0 = base;

	conf.npage1 = 0;
	conf.npage0 = (conf.topofmem - base)/BY2PG;
	conf.npage = conf.npage0 + conf.npage1;
	conf.ialloc = (((conf.npage*(main_pool_pcnt))/100)/2)*BY2PG;

	conf.nproc = 100 + ((conf.npage*BY2PG)/MB)*5;
	conf.nmach = MAXMACH;

	print("Conf: top=0x%lux, npage0=0x%lux, ialloc=0x%lux, nproc=0x%lux\n",
			conf.topofmem, conf.npage0,
			conf.ialloc, conf.nproc);
}

void
poolsizeinit(void)
{
	ulong nb;
	nb = conf.npage*BY2PG;
	poolsize(mainmem, (nb*main_pool_pcnt)/100, 0);
	poolsize(heapmem, (nb*heap_pool_pcnt)/100, 0);
	poolsize(imagmem, (nb*image_pool_pcnt)/100, 1);
}

void
init0(void)
{
	Osenv *o;
	char buf[2*KNAMELEN];

	up->nerrlab = 0;

	print("Starting init0()\n");
	spllo();

	if(waserror())
		panic("init0 %r");

	o = up->env;
	o->pgrp->slash = namec("#/", Atodir, 0, 0);
	cnameclose(o->pgrp->slash->name);
	o->pgrp->slash->name = newcname("/");
	o->pgrp->dot = cclone(o->pgrp->slash);

	chandevinit();

	if(!waserror()){
		ksetenv("cputype", "riscv", 0);
		snprint(buf, sizeof(buf), "riscv %s", conffile);
		ksetenv("terminal", buf, 0);
		poperror();
	}

	poperror();

	disinit("/osinit.dis");
}

void
userinit(void)
{
	Proc *p;
	Osenv *o;

	p = newproc();
	o = p->env;

	o->fgrp = newfgrp(nil);
	o->pgrp = newpgrp();
	o->egrp = newegrp();
	kstrdup(&o->user, eve);

	strcpy(p->text, "interp");

	p->fpstate = FPINIT;

	p->sched.pc = (ulong)init0;
	p->sched.sp = (ulong)p->kstack+KSTACK-8;

	ready(p);
}

int
main() {
	char input;

	memset(edata, 0, end-edata);
	memset(m, 0, sizeof(Mach));

	confinit();
	xinit();
	poolinit();
	poolsizeinit();
	printinit();

	trapinit();
	clockinit();
	i8250console();
	serwrite = uartputs;
	virtio_init();
	input_init();
	screeninit();

	print("\nRISC-V QEMU\n");
	print("Inferno OS %s Vita Nuova\n\n", VERSION);

	procinit();
	links();
	chandevreset();

	eve = strdup("inferno");

	userinit();
	schedinit();

	while (1) {}
	halt();
	return 0;
}
