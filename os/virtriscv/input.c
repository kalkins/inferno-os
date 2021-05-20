#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

extern void kbd_handle_key(virtio_input_event *event);
extern void mouse_handle_event(virtio_input_event *event);

void
input_event_handler(virtq *queue, void *dev, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_input_event *event = (void*) desc->addr;

	if (event != 0) {
		switch (event->type) {
		case EV_SYN:
			switch (event->code) {
			case SYN_REPORT:
				break;
			case SYN_CONFIG:
				break;
			case SYN_MT_REPORT:
				break;
			case SYN_DROPPED:
				break;
			default:
				iprint("VIRTIO Input: Unknown SYN %d\n", event->code);
			}
			break;
		default:
			kbd_handle_key(event);
			mouse_handle_event(event);
		}
	} else {
		iprint("VIRTIO Input: used elem pointed to NULL descriptor\n");
	}

	// Make the event available again
	u16int avail_idx = queue->avail->idx;
	u16int new_avail_idx = (avail_idx + 1) % queue->size;
	memory_fence();
	queue->avail->idx = avail_idx + 1;
	virtq_notify(dev, 0, 1, idx + 1);
}

void
input_init_events(virtio_dev *dev)
{
	// Alloc all events in a big block, then fill the ring with addresses to the block
	virtq *q = &dev->queues[0];
	int num = q->size;
	virtio_input_event *events = malloc(num * sizeof(*events));

	if (events == 0) {
		panic("Can't allocate memory\n");
	}

	for (int i = 0; i < num; i++) {
		virtq_add_desc_chain(q, 0, 0, 1,
		                     &events[i], sizeof(*events), 1);

		dev->queues[0].avail->ring[i] = i;
	}

	memory_fence();
	dev->queues[0].avail->idx = q->size - 1;
	virtq_notify(dev, 0, 1, 0);
}

void
input_query_config(virtio_dev *dev)
{
	virtio_input_config *config = (void*) &dev->regs->config;
	config->select = VIRTIO_INPUT_CFG_UNSET;
}

int
input_virtq_init(virtio_dev *dev)
{
	if (dev->queues == 0) {
		dev->queues = malloc(2*sizeof(virtq));
		dev->numqueues = 2;

		if (dev->queues == 0) {
			panic("Could not allocate queues. Malloc failed\n");
		}

		if (virtq_alloc(dev, 0, 0) != 0) {
			panic("Virtio Input: Failed to create event queue");
			return -1;
		}
		if (virtq_alloc(dev, 1, 0) != 0) {
			panic("Virtio Input: Failed to create status queue");
			return -1;
		}

		dev->queues[0].default_handler = input_event_handler;
		dev->queues[0].default_handler_data = dev;
	}

	return 0;
}

int
input_init(void)
{
	if (kbdq == 0) {
		kbdq = qopen(4*1024, 0, 0, 0);
	}

	virtio_dev *dev;

	while ((dev = virtio_get_device(VIRTIO_DEV_INPUT)) != 0) {
		int err = virtio_setup(dev, "INPUT", input_virtq_init, VIRTIO_F_ANY_LAYOUT
		                       | VIRTIO_F_RING_INDIRECT_DESC | VIRTIO_F_RING_EVENT_IDX);
		switch (err) {
		case 0:
			virtio_enable_interrupt(dev, 0);

			input_init_events(dev);
			input_query_config(dev);

			break;
		case -1:
			iprint("Virtio input rejected features\n");
			return -1;
		case -2:
			iprint("Virtio input queue error\n");
			return -1;
		default:
			iprint("Virtio input setup unknown error %d\n", err);
			return -1;
		}
	}

	return 0;
}
