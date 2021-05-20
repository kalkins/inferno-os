typedef struct Cursor Cursor;
typedef struct Cursorinfo Cursorinfo;

#define CURSWID 16
#define CURSHGT 16

struct Cursor {
	Point   offset;
	uchar   clr[CURSWID/BI2BY*CURSHGT];
	uchar   set[CURSWID/BI2BY*CURSHGT];
};

struct Cursorinfo {
	Cursor;
	Lock;
};

/* devmouse.c */
extern void mousetrack(int, int, int, int);
extern Point mousexy(void);

extern void mouseaccelerate(int);
extern int m3mouseputc(Queue*, int);
extern int m5mouseputc(Queue*, int);
extern int mouseputc(Queue*, int);

extern Cursorinfo cursor;
extern Cursor arrow;

/* mouse.c */
extern void mousectl(Cmdbuf*);
extern void mouseresize(void);

/* screen.c */
extern void	blankscreen(int);
extern void	flushmemscreen(Rectangle);
extern uchar*	attachscreen(Rectangle*, ulong*, int*, int*, int*);
extern int	cursoron(int);
extern void	cursoroff(int);
extern void	setcursor(Cursor*);

extern void	drawqlock(void);
extern void	drawqunlock(void);
extern int	candrawqlock(void);
extern void	swcursorinit(void);

/* devdraw.c */
//extern QLock	drawlock;

#define ishwimage(i)	1		/* for ../port/devdraw.c */
