#define KiB     1024u       /*! Kibi 0x0000000000000400 */
#define MiB     1048576u    /*! Mebi 0x0000000000100000 */
#define GiB     1073741824u /*! Gibi 000000000040000000 */

/*
 * Sizes
 */
#define	BI2BY		8			/* bits per byte */
#define	BI2WD		32			/* bits per word */
#define	BY2WD		4			/* bytes per word */
#define	BY2V		8			/* bytes per double word */
#define	BY2PG		4096			/* bytes per page */
#define	WD2PG		(BY2PG/BY2WD)		/* words per page */
#define	PGSHIFT		12			/* log(BY2PG) */
#define	ROUND(s, sz)	(((s)+((sz)-1))&~((sz)-1))
#define	PGROUND(s)	ROUND(s, BY2PG)

#define	MAXMACH		1			/* max # cpus system can run */

#define KSTKSIZE    (8*KiB)
#define KSTACK      KSTKSIZE

/*
 * Fundamental addresses
 */
#define	MACHADDR	0x800f0000		/* as seen by current processor */
#define	MACHSIZE	BY2PG
#define TRAPSTACK	0x800d0000		/* Top of the stack used by the trap handler */

/*
 *  Address spaces
 */
#define RAMBOOT		0x80000000		/* Start of bootloader space. Not usable by the OS */
#define RAMZERO		0x80020000		/* Start of usable RAM */
#define	KZERO		0x80300000		/* base of kernel address space */
#define	L2		(KZERO+0x3000)		/* L2 ptes for vectors etc */
#define	VCBUFFER	(KZERO+0x3400)		/* videocore mailbox buffer */
#define	FIQSTKTOP	(KZERO+0x4000)		/* FIQ stack */
#define	L1		(KZERO+0x4000)		/* tt ptes: 16KiB aligned */
#define	KTZERO		(KZERO+0x8000)		/* kernel text start */
#define	UTZERO		(UZERO+BY2PG)		/* first address in user text */
#define	USTKTOP		(KZERO-BY2PG)		/* byte just beyond user stack */
#define	USTKSIZE	(1*MiB)			/* size of user stack */
#define	TSTKTOP		(USTKTOP-USTKSIZE)	/* end of new stack in sysexec */
#define	TSTKSIZ 	100

/*
 * CLINT (core local interruptor)
 */
#define CLINT 0x2000000L
#define CLINT_MTIMECMP(hartid) (CLINT + 0x4000 + 8*(hartid))
#define CLINT_MTIME (CLINT + 0xBFF8) // cycles since boot.

/*
 * VIRTIO
 */
#define VIRTIO_START	0x10001000
#define VIRTIO_END	0x10008000
#define VIRTIO_OFFSET	0x1000
#define NUM_VIRTIO	((VIRTIO_END - VIRTIO_START) / VIRTIO_OFFSET)
