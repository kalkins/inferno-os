#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

#define BTN_LEFT		0x110
#define BTN_RIGHT		0x111

#define REL_X			0x00
#define REL_Y			0x01
#define REL_Z			0x02
#define REL_RX			0x03
#define REL_RY			0x04
#define REL_RZ			0x05
#define REL_HWHEEL		0x06
#define REL_DIAL		0x07
#define REL_WHEEL		0x08

// Conversion from evdev codes to Plan9 codes.
// Left, Right, Middle, Side, Extra, Forward, Back, Task
static uchar button_array[] = {1, 4, 2};

int buttons = 0;

void
mouse_handle_buttons(virtio_input_event *event)
{
	int code = event->code - 0x110;
	if (code > sizeof button_array) {
		return;
	}

	int b = button_array[code];

	if (event->value == 0) {
		buttons |= b;
	} else {
		buttons ^= b;
	}

	mousetrack(buttons, 0, 0, 0);
}

void
mouse_handle_event(virtio_input_event *event)
{
	int dx=0, dy=0, abs=0;

	switch (event->type) {
	case EV_REL:
		abs = 0;
	case EV_ABS:
		abs = 1;
	case EV_KEY:
		mouse_handle_buttons(event);
		return;
	default:
		return;
	}

	switch (event->code) {
	case REL_X:
		dx = event->value;
		break;
	case REL_Y:
		dy = event->value;
		break;
	default:
		print("Unknown mouse rel event %d\n", event->type);
		return;
	}

	mousetrack(buttons, dx, dy, abs);
}
