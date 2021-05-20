#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

#define VIRTQ_ALIGN	BY2PG
#define ALIGN(x, align) (((x) + (align)) & ~((align)-1))

typedef struct virtio_dev_list{
	virtio_dev *dev;
	struct virtio_dev_list *next;
} virtio_dev_list;

static virtio_dev_list *devices = 0;

// Used for debugging address offsets and values
void
print_virtio_regs(virtio_regs *regs)
{
	if (regs != 0) {
		iprint("Magic\t\t  0x%p\t%d\n", &regs->magic, regs->magic);
		iprint("Version\t\t  0x%p\t%d\n", &regs->version, regs->version);
		iprint("DeviceId\t  0x%p\t%d\n", &regs->deviceId, regs->deviceId);
		iprint("VendorId\t  0x%p\t%d\n", &regs->vendorId, regs->vendorId);

		iprint("DeviceFeatures\t  0x%p\t%d\n", &regs->deviceFeatures, regs->deviceFeatures);
		iprint("DeviceFeaturesSel 0x%p\n", &regs->deviceFeaturesSel);
		iprint("DriverFeatures\t  0x%p\n", &regs->driverFeatures);
		iprint("DriverFeaturesSel 0x%p\n", &regs->driverFeaturesSel);
		iprint("GuestPageSize\t  0x%p\n", &regs->guestPageSize);

		iprint("QueueSel\t  0x%p\n", &regs->queueSel);
		iprint("QueueNumMax\t  0x%p\t%d\n", &regs->queueNumMax, regs->queueNumMax);
		iprint("QueueNum\t  0x%p\n", &regs->queueNum);
		iprint("QueueReady\t  0x%p\t%d\n", &regs->queueReady, regs->queueReady);
		iprint("QueueAlign\t  0x%p\n", &regs->queueAlign);
		iprint("QueuePfn\t  0x%p\t%d\n", &regs->queuePfn, regs->queuePfn);
		iprint("QueueNotify\t  0x%p\n", &regs->queueNotify);

		iprint("InterruptStatus\t  0x%p\t%d\n", &regs->interruptStatus, regs->interruptStatus);
		iprint("InterruptAck\t  0x%p\n", &regs->interruptAck);

		iprint("Status\t\t  0x%p\t%d\n", &regs->status, regs->status);

		iprint("QueueDescLow\t  0x%p\n", &regs->queueDescLow);
		iprint("QueueDescHigh\t  0x%p\n", &regs->queueDescHigh);
		iprint("QueueDriverLow\t  0x%p\n", &regs->queueDriverLow);
		iprint("QueueDriverHigh\t  0x%p\n", &regs->queueDriverHigh);
		iprint("QueueDeviceLow\t  0x%p\n", &regs->queueDeviceLow);
		iprint("QueueDeviceHigh\t  0x%p\n", &regs->queueDeviceHigh);

		iprint("ConfigGeneration  0x%p\t%d\n", &regs->configGeneration, regs->configGeneration);
		iprint("Config\t\t  0x%p\t%d\n", &regs->config, regs->config);
	}
}

static uint
virtqsize(uint qsz, uint align)
{
	return ALIGN(sizeof(virtq_desc)*qsz + sizeof(u16int)*(3 + qsz), align)
		+ sizeof(u16int)*3 + sizeof(virtq_used_elem)*qsz;
}

int
virtq_alloc(virtio_dev *dev, uint queueIdx, ulong size)
{
	virtq *queue = &dev->queues[queueIdx];
	dev->regs->queueSel = queueIdx;

	if (dev->regs->queuePfn != 0) {
		// Queue is already configured
		return 0;
	}

	int queueMax = dev->regs->queueNumMax;
	if (size == 0) {
		size = queueMax;
	}

	if (queueMax == 0 || size > queueMax) {
		return -1;
	}
	dev->regs->queueNum = queueMax;
	queue->used_idx = 0;

	if (dev->version == 1) {
		dev->regs->guestPageSize = BY2PG;

		uint qsize = virtqsize(size, VIRTQ_ALIGN); // The size of the whole queue
		void *buf = xspanalloc(qsize, BY2PG, 0);
		int pfn = (uintptr) buf >> PGSHIFT;
		dev->regs->queuePfn = pfn;
		dev->regs->queueAlign = VIRTQ_ALIGN;
		queue->size = size;
		queue->desc = buf;
		queue->avail = (void*) ((uintptr) queue->desc + size*sizeof(virtq_desc));
		queue->used = (void*) ALIGN((uintptr) queue->avail + sizeof(virtq_avail) + size*sizeof(u16int), VIRTQ_ALIGN);
	} else if (dev->version == 2) {
		ulong qsize = 26*size + 12;
		void *buf = xspanalloc(qsize, 16, 0);
		queue->size = size;
		queue->desc = buf;
		queue->driver = (void*) ((uintptr) buf + 16*size);
		queue->device = (void*) ((uintptr) buf + 18*size + 6);

		dev->regs->queueDescLow = (u64int) queue->desc & 0xffffffff;
		dev->regs->queueDescHigh = (u64int) queue->desc >> 32;
		dev->regs->queueDriverLow = (u64int) queue->driver & 0xffffffff;
		dev->regs->queueDriverHigh = (u64int) queue->driver >> 32;
		dev->regs->queueDeviceLow = (u64int) queue->device & 0xffffffff;
		dev->regs->queueDeviceHigh = (u64int) queue->device >> 32;
	} else {
		iprint("ERROR: Unknown virtio version %d for device 0x%lux\n", dev->version, dev->regs);
		return -1;
	}

	return 0;
}

void
virtio_init(void)
{
	virtio_regs *start = (void*) VIRTIO_START;
	virtio_regs *end = (void*) VIRTIO_END;
	ulong offset = VIRTIO_OFFSET;
	virtio_dev_list **list = &devices;

	for (virtio_regs* virtio = start; virtio <= end; virtio = (void*) ((ulong) virtio + offset)) {
		if (virtio->magic == 0x74726976 && virtio->deviceId != 0) {
			// Reset the device
			virtio->status = 0;

			// Allocate space for the device
			virtio_dev *dev = malloc(sizeof(virtio_dev));
			memset(dev, 0, sizeof(virtio_dev));
			dev->regs = virtio;
			dev->type = virtio->deviceId;
			dev->version = virtio->version;
			dev->index = ((ulong) virtio - (ulong) start) >> 12;

			// Add the device to the linked list
			*list = malloc(sizeof(virtio_dev_list));
			(*list)->dev = dev;
			list = &((*list)->next);
			*list = 0;
		}
	}
}

int
virtio_setup(virtio_dev *dev, char *name, virtq_dev_specific_init virtq_init, u64int features)
{
	dev->name = name;

	// Reset the device
	int status = 0;
	dev->regs->status = status;
	dev->enabled = 0;

	// Set ACKNOWLEDGE bit
	status |= VIRTIO_STATUS_ACK;
	dev->regs->status = status;

	// Set DRIVER bit
	status |= VIRTIO_STATUS_DRIVER;
	dev->regs->status = status;

	// Negotiate features
	// In this case we just accept all the default features
	dev->regs->deviceFeaturesSel = 0;
	u64int deviceFeatures = dev->regs->deviceFeatures;
	dev->regs->deviceFeaturesSel = 1;
	deviceFeatures |= ((u64int) dev->regs->deviceFeatures) << 32;

	dev->features = features & deviceFeatures;
	dev->regs->driverFeaturesSel = 0;
	dev->regs->driverFeatures = dev->features & 0xffffffff;

	if (dev->version != 1) {
		// Write upper 32 bits (not used in legacy mode)
		dev->regs->driverFeaturesSel = 1;
		dev->regs->driverFeatures = (dev->features >> 32) & 0xffffffff;

		status |= VIRTIO_STATUS_FEATURES_OK;
		dev->regs->status = status;

		// Check that the features are accepted
		if ((dev->regs->status & VIRTIO_STATUS_FEATURES_OK) != 1) {
			dev->regs->status = VIRTIO_STATUS_FAILED;
			return -1;
		}
	}

	if (virtq_init(dev) != 0) {
		dev->regs->status = VIRTIO_STATUS_FAILED;
		return -2;
	}

	status |= VIRTIO_STATUS_DRIVER_OK;
	dev->regs->status = status;

	dev->enabled = 1;

	return 0;
}

void
virtio_disable(virtio_dev *dev)
{
	dev->regs->status = 0;
	dev->enabled = 0;
	dev->features = 0;
}

virtio_dev*
virtio_get_device(int type)
{
	for (virtio_dev_list *list = devices; list != 0; list = list->next) {
		if (list->dev->type == type && list->dev->enabled == 0) {
			return list->dev;
		}
	}

	return 0;
}

static void
virtq_add_handler(virtq *queue, int idx, virtq_intr_handler handler, void *handler_data)
{
	if (handler == 0) return;

	virtq_handler_list **list = &queue->intr_handlers;
	virtq_handler_list *new;

	while (*list != 0) {
		if ((*list)->idx == idx) {
			(*list)->handler = handler;
			(*list)->data = handler_data;
			return;
		}

		list = &(*list)->next;
	}

	new = malloc(sizeof(virtq_handler_list));
	new->idx = idx;
	new->handler = handler;
	new->data = handler_data;
	new->next = 0;
	*list = new;
}

static virtq_handler_list*
virtq_pop_handler(virtq *queue, int idx)
{
	if (queue == 0) {
		iprint("virtq_pop_handler: queue is 0\n");
		return 0;
	}

	virtq_handler_list **list = &queue->intr_handlers;

	while (*list != 0) {
		if ((*list)->idx == idx) {
			virtq_handler_list *entry = *list;
			virtq_intr_handler handler = entry->handler;

			*list = entry->next;
			return entry;
		}

		list = &(*list)->next;
	}

	return 0;
}

static int
virtq_add_desc_helper(virtq *queue, void* addr, u32int len, int device_writable)
{
	int flags = device_writable == 1 ? VIRTQ_DESC_F_WRITE : 0;
	int idx = atomic_inc(&queue->desc_idx) % queue->size;

	queue->desc[idx].addr = (u64int) addr;
	queue->desc[idx].len = len;
	queue->desc[idx].flags = flags;
	queue->desc[idx].next = 0;

	return idx;
}

int
virtq_add_desc_chain(virtq *queue, virtq_intr_handler handler, void *handler_data, uint num, ...)
{
	va_list va;
	va_start(va, num);

	int head = -1;
	int prev = -1;

	for (int i = 0; i < num; i++) {
		void* addr = va_arg(va, void*);
		u32int len = va_arg(va, u32int);
		int device_writable = va_arg(va, int);

		if (i == 0) {
			head = virtq_add_desc_helper(queue, addr, len, device_writable);
			prev = head;
		} else {
			uint idx = virtq_add_desc_helper(queue, addr, len, device_writable);

			int prev_flags = queue->desc[prev].flags | VIRTQ_DESC_F_NEXT;
			queue->desc[prev].flags = prev_flags;
			queue->desc[prev].next = idx;
			prev = idx;
		}
	}

	if (head != -1) {
		// Add to the available ring, but don't update avail->idx yet
		uint avail_idx = queue->avail->idx + queue->avail_pending;
		queue->avail->ring[avail_idx % queue->size] = head;
		atomic_inc(&queue->avail_pending);

		virtq_add_handler(queue, head, handler, handler_data);
	}

	va_end(va);

	return head;
}

void
virtq_make_available(virtq *queue)
{
	memory_fence();
	queue->avail->idx += queue->avail_pending;
	queue->avail_pending = 0;
}

virtq_used_elem*
virtq_get_next_used(virtq *queue)
{
	if (queue->used_idx != (u16int) queue->used->idx) {
		virtq_used_elem *ring = (void*) ((uintptr) queue->used + 4);
		virtq_used_elem *elem = &ring[queue->used_idx % queue->size];
		atomic_inc(&queue->used_idx);

		return elem;
	}

	return 0;
}

static void
virtio_dev_intr_handler(Ureg*, void *arg)
{
	virtio_dev *dev = arg;

	memory_fence();

	int pending = dev->regs->interruptStatus;

	if (pending & 1) {
		for (int i = 0; i < dev->numqueues; i++) {
			virtq *queue = &dev->queues[i];
			virtq_used_elem *elem;

			while ((elem = virtq_get_next_used(queue)) != 0) {
				consoleprint=0;
				consoleprint=1;

				if (elem->id >= queue->size) {
					iprint("virtq_intr_handler: %s: idx %ud out of bounds\n", dev->name, (u16int)elem->id);
					iprint("mod size: %d\n", ((u16int)elem->id) % queue->size);

					if ((u16int) elem->id == 32639) {
						while(1) {}
					}
					continue;
				}

				virtq_handler_list *entry = virtq_pop_handler(queue, elem->id);
				if (entry) {
					virtq_intr_handler handler = entry->handler;
					void *data = entry->data;
					free(entry);

					if (handler) {
						handler(queue, data, elem->id, elem->len);
					}
				} else if (queue->default_handler) {
					queue->default_handler(queue, queue->default_handler_data, elem->id, elem->len);
				}
			}
		}
	}

	if (pending & 2) {
		if (dev->config_change_handler) {
			dev->config_change_handler(dev);
		}
	}

	dev->regs->interruptAck = pending;
}

void
virtio_enable_interrupt(virtio_dev *dev, virtio_config_change_handler config_change_handler)
{
	dev->config_change_handler = config_change_handler;
	intrenable(dev->index+1, virtio_dev_intr_handler, dev, BUSPLIC, dev->name);
}

void
virtio_disable_interrupt(virtio_dev *dev)
{
	intrenable(dev->index+1, virtio_dev_intr_handler, dev, BUSPLIC, dev->name);
}

void
virtq_free_chain(virtq *queue, virtq_desc *head)
{
	virtq_desc *desc = head;

	while (desc != 0) {
		if (desc->flags & VIRTQ_DESC_F_INDIRECT) {
			virtq_desc *ind = (void*) desc->addr;

			while (ind != 0) {
				free((void*) ind->addr);

				if (ind->flags & VIRTQ_DESC_F_NEXT) {
					ind = &((virtq_desc*) desc->addr)[desc->next];
				} else {
					break;
				}
			}
		}

		free((void*) desc->addr);

		if (desc->flags & VIRTQ_DESC_F_NEXT) {
			desc = &queue->desc[desc->next];
		} else {
			break;
		}
	}
}

void
virtq_notify(virtio_dev *dev, int queuenum, int notify_response, int avail_idx)
{
	if (queuenum < dev->numqueues) {
		virtq *queue = &dev->queues[queuenum];
		int event_enabled = dev->features & VIRTIO_F_RING_EVENT_IDX;

		if (avail_idx < 0) {
			avail_idx = queue->avail->idx - 1;
		}

		if (event_enabled) {
			u16int *used_event = (void*) ((uintptr) queue->avail + 4 + 2*queue->size);
			u16int *avail_event = (void*) ((uintptr) queue->used + 4 + 8*queue->size);

			queue->avail->flags = 0;

			if (notify_response) {
				*used_event = avail_idx;
			} else {
				*used_event = (u16int) -1;
			}

			if (*avail_event <= avail_idx && *avail_event >= queue->used_idx) {
				dev->regs->queueNotify = queuenum;
			}
		} else {
			queue->avail->flags = !notify_response;

			if (queue->used->flags == 0) {
				dev->regs->queueNotify = queuenum;
			}
		}
	}
}
