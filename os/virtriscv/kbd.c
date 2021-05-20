#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

enum {
	Data=		0x60,		/* data port */

	Status=		0x64,		/* status port */
	Inready=	0x01,		/*  input character ready */
	Outbusy=	0x02,		/*  output busy */
	Sysflag=	0x04,		/*  system flag */
	Cmddata=	0x08,		/*  cmd==0, data==1 */
	Inhibit=	0x10,		/*  keyboard/mouse inhibited */
	Minready=	0x20,		/*  mouse character ready */
	Rtimeout=	0x40,		/*  general timeout */
	Parity=	0x80,

	Cmd=		0x64,		/* command port (write only) */

	Spec=		0x80,

	PF=		Spec|0x20,	/* num pad function key */
	View=		Spec|0x00,	/* view (shift window up) */
	KF=		0xF000,		/* function key (begin Unicode private space) */
	Shift=		Spec|0x60,
	Break=		Spec|0x61,
	Ctrl=		Spec|0x62,
	Latin=		Spec|0x63,
	Caps=		Spec|0x64,
	Num=		Spec|0x65,
	Middle=		Spec|0x66,
	No=		0x00,		/* peter */

	Home=		KF|13,
	Up=		KF|14,
	Pgup=		KF|15,
	Print=		KF|16,
	Left=		KF|17,
	Right=		KF|18,
	End=		'\r',
	Down=		View,
	Pgdown=		KF|19,
	Ins=		KF|20,
	Del=		0x7F,
	Scroll=		KF|21,
};

Rune kbtab[] =
{
[0x00]	No,	0x1b,	'1',	'2',	'3',	'4',	'5',	'6',
[0x08]	'7',	'8',	'9',	'0',	'-',	'=',	'\b',	'\t',
[0x10]	'q',	'w',	'e',	'r',	't',	'y',	'u',	'i',
[0x18]	'o',	'p',	'[',	']',	'\n',	Ctrl,	'a',	's',
[0x20]	'd',	'f',	'g',	'h',	'j',	'k',	'l',	';',
[0x28]	'\'',	'`',	Shift,	'\\',	'z',	'x',	'c',	'v',
[0x30]	'b',	'n',	'm',	',',	'.',	'/',	Shift,	'*',
[0x38]	Latin,	' ',	Ctrl,	KF|1,	KF|2,	KF|3,	KF|4,	KF|5,
[0x40]	KF|6,	KF|7,	KF|8,	KF|9,	KF|10,	Num,	Scroll,	'7',
[0x48]	'8',	'9',	'-',	'4',	'5',	'6',	'+',	'1',
[0x50]	'2',	'3',	'0',	'.',	No,	No,	No,	KF|11,
[0x58]	KF|12,	No,	No,	No,	No,	No,	No,	No,
[0x60]	No,	No,	No,	No,	No,	No,	No,	No,
[0x68]	No,	No,	No,	No,	No,	No,	No,	No,
[0x70]	No,	No,	No,	No,	No,	No,	No,	No,
[0x78]	No,	View,	No,	Up,	No,	No,	No,	No,
};

Rune kbtabshift[] =
{
[0x00]	No,	0x1b,	'!',	'@',	'#',	'$',	'%',	'^',
[0x08]	'&',	'*',	'(',	')',	'_',	'+',	'\b',	'\t',
[0x10]	'Q',	'W',	'E',	'R',	'T',	'Y',	'U',	'I',
[0x18]	'O',	'P',	'{',	'}',	'\n',	Ctrl,	'A',	'S',
[0x20]	'D',	'F',	'G',	'H',	'J',	'K',	'L',	':',
[0x28]	'"',	'~',	Shift,	'|',	'Z',	'X',	'C',	'V',
[0x30]	'B',	'N',	'M',	'<',	'>',	'?',	Shift,	'*',
[0x38]	Latin,	' ',	Ctrl,	KF|1,	KF|2,	KF|3,	KF|4,	KF|5,
[0x40]	KF|6,	KF|7,	KF|8,	KF|9,	KF|10,	Num,	Scroll,	'7',
[0x48]	'8',	'9',	'-',	'4',	'5',	'6',	'+',	'1',
[0x50]	'2',	'3',	'0',	'.',	No,	No,	No,	KF|11,
[0x58]	KF|12,	No,	No,	No,	No,	No,	No,	No,
[0x60]	No,	No,	No,	No,	No,	No,	No,	No,
[0x68]	No,	No,	No,	No,	No,	No,	No,	No,
[0x70]	No,	No,	No,	No,	No,	No,	No,	No,
[0x78]	No,	Up,	No,	Up,	No,	No,	No,	No,
};

Rune kbtabesc1[] =
{
[0x00]	No,	No,	No,	No,	No,	No,	No,	No,
[0x08]	No,	No,	No,	No,	No,	No,	No,	No,
[0x10]	No,	No,	No,	No,	No,	No,	No,	No,
[0x18]	No,	No,	No,	No,	'\n',	Ctrl,	No,	No,
[0x20]	No,	No,	No,	No,	No,	No,	No,	No,
[0x28]	No,	No,	Shift,	No,	No,	No,	No,	No,
[0x30]	No,	No,	No,	No,	No,	'/',	No,	Print,
[0x38]	Latin,	No,	No,	No,	No,	No,	No,	No,
[0x40]	No,	No,	No,	No,	No,	No,	Break,	Home,
[0x48]	Up,	Pgup,	No,	Left,	No,	Right,	No,	End,
[0x50]	Down,	Pgdown,	Ins,	Del,	No,	No,	No,	No,
[0x58]	No,	No,	No,	No,	No,	No,	No,	No,
[0x60]	No,	No,	No,	No,	No,	No,	No,	No,
[0x68]	No,	No,	No,	No,	No,	No,	No,	No,
[0x70]	No,	No,	No,	No,	No,	No,	No,	No,
[0x78]	No,	Up,	No,	No,	No,	No,	No,	No,
};

void
kbd_handle_key(virtio_input_event *event)
{
	int c, i;
	static int esc1, esc2;
	static int alt, caps, ctl, num, shift;
	static int collecting, nk;
	static Rune kc[5];
	int keyup;

	if (event->type != EV_KEY) {
		return;
	}

	/*
	 *  get the character
	 */
	c = event->code;
	keyup = event->value == 0;

	/*
	 *  e0's is the first of a 2 character sequence
	 */
	if(c == 0xe0){
		esc1 = 1;
		return;
	} else if(c == 0xe1){
		esc2 = 2;
		return;
	}

	if(c > sizeof kbtab){
		if(c != 0xFF)	/* these come fairly often: CAPSLOCK U Y */
			print("unknown key %ux\n", c);
		return;
	}

	if(esc1){
		c = kbtabesc1[c];
		esc1 = 0;
	} else if(esc2) {
		esc2--;
		return;
	} else if(shift) {
		c = kbtabshift[c];
	} else {
		c = kbtab[c];
	}

	if(caps && c<='z' && c>='a'){
		c += 'A' - 'a';
	}

	/*
	 *  keyup only important for shifts
	 */
	if(keyup){
		switch(c){
		case Latin:
			alt = 0;
			break;
		case Shift:
			shift = 0;
			break;
		case Ctrl:
			ctl = 0;
			break;
		}
		return;
	}

	/*
 	 *  normal character
	 */
	if(!(c & (Spec|KF))){
		if(ctl){
			if(alt && c == Del)
				exit(0);
			c &= 0x1f;
		}
		if(!collecting){
			kbdputc(kbdq, c);
			return;
		}
		kc[nk++] = c;
		c = latin1(kc, nk);
		if(c < -1)	/* need more keystrokes */
			return;
		if(c != -1)	/* valid sequence */
			kbdputc(kbdq, c);
		else	/* dump characters */
			for(i=0; i<nk; i++)
				kbdputc(kbdq, kc[i]);
		nk = 0;
		collecting = 0;
		return;
	} else {
		switch(c){
		case Caps:
			caps ^= 1;
			return;
		case Num:
			num ^= 1;
			return;
		case Shift:
			shift = 1;
			return;
		case Latin:
			alt = 1;
			/*
			 * VMware uses Ctl-Alt as the key combination
			 * to make the VM give up keyboard and mouse focus.
			 * This has the unfortunate side effect that when you
			 * come back into focus, Plan 9 thinks you want to type
			 * a compose sequence (you just typed alt).
			 *
			 * As a clumsy hack around this, we look for ctl-alt
			 * and don't treat it as the start of a compose sequence.
			 */
			if(!ctl){
				collecting = 1;
				nk = 0;
			}
			return;
		case Ctrl:
			collecting = 0;
			nk = 0;
			ctl = 1;
			return;
		}
	}
	kbdputc(kbdq, c);
}
