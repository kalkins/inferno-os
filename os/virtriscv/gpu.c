#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

#define GPU_MAX_DISPLAYS 1

#define GPU_F_RES	(1<<0)
#define GPU_F_ATTACH	(1<<1)
#define GPU_F_SCANOUT	(1<<2)

#define FRAMERATE	(60)
#define FRAMENANO	(1000000000 / FRAMERATE)

struct display {
	int flags;
	uint scanout;
	uint resource;
	uint width;
	uint height;
	uint format;
	usize fb_length;
	void *framebuffer;
};

struct display displays[GPU_MAX_DISPLAYS];

static virtio_dev *gpu = 0;
static uint gpu_resource_id = 1; // Starts at 1, so 0 means no resource
static virtio_gpu_pmode pmodes[VIRTIO_GPU_MAX_SCANOUTS];

struct gpu_rect_elem {
	virtio_gpu_rect r;
	struct gpu_rect_elem *next;
};

static struct gpu_rect_elem *gpu_rect_queue;
static QLock gpu_invalidate_lock;

/*
 * Virtio GPU
 */
void
gpu_info_handler(virtq *queue, void *data, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_gpu_ctrl_hdr *request = (void*) desc->addr;

	if (request != 0) {
		if (request->type == VIRTIO_GPU_CMD_GET_DISPLAY_INFO) {
			virtio_gpu_resp_display_info *resp = (void*) queue->desc[idx+1].addr;

			if (resp != 0) {
				if (resp->hdr.type == VIRTIO_GPU_RESP_OK_DISPLAY_INFO) {
					memmove(&pmodes, resp->pmodes, len - offsetof(virtio_gpu_resp_display_info, pmodes));
				} else {
					iprint("Unknown response %d\n", resp->hdr.type);
				}
			} else {
				iprint("No response\n");
			}
		} else {
			iprint("VIRTIO GPU unknown request type %d\n", request->type);
		}
	} else {
		iprint("Used elem pointed to NULL descriptor\n");
	}

	virtq_free_chain(queue, desc);
}

void
gpu_create_resource_handler(virtq *queue, void *data, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_gpu_resource_create_2d *request = (void*) desc->addr;
	struct display *d = data;

	if (request != 0) {
		if (request->hdr.type == VIRTIO_GPU_CMD_RESOURCE_CREATE_2D) {
			virtio_gpu_ctrl_hdr *resp = (void*) queue->desc[idx+1].addr;

			if (resp != 0) {
				if (resp->type == VIRTIO_GPU_RESP_OK_NODATA) {
					d->resource = request->resource_id;
					d->flags |= GPU_F_RES;
				} else {
					iprint("Unknown response %d\n", resp->type);
				}
			} else {
				iprint("No response\n");
			}
		} else {
			iprint("VIRTIO GPU unknown request type %d\n", request->hdr.type);
		}
	} else {
		iprint("Used elem pointed to NULL descriptor\n");
	}

	virtq_free_chain(queue, desc);
}

void
gpu_attach_handler(virtq *queue, void *data, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_gpu_resource_attach_backing *request = (void*) desc->addr;
	struct display *d = data;

	if (request != 0) {
		if (request->hdr.type == VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING) {
			virtio_gpu_ctrl_hdr *resp = (void*) queue->desc[idx+2].addr;

			if (resp != 0) {
				if (resp->type == VIRTIO_GPU_RESP_OK_NODATA) {
					d->flags |= GPU_F_ATTACH;
				} else {
					iprint("Unknown response %d\n", resp->type);
				}
			} else {
				iprint("No response\n");
			}
		} else {
			iprint("VIRTIO GPU unknown request type %d\n", request->hdr.type);
		}
	} else {
		iprint("Used elem pointed to NULL descriptor\n");
	}

	virtq_free_chain(queue, desc);
}

void
gpu_set_scanout_handler(virtq *queue, void *data, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_gpu_resource_attach_backing *request = (void*) desc->addr;
	struct display *d = data;

	if (request != 0) {
		if (request->hdr.type == VIRTIO_GPU_CMD_SET_SCANOUT) {
			virtio_gpu_ctrl_hdr *resp = (void*) queue->desc[idx+1].addr;

			if (resp != 0) {
				if (resp->type == VIRTIO_GPU_RESP_OK_NODATA) {
					d->flags |= GPU_F_SCANOUT;
				} else {
					iprint("Unknown response %d\n", resp->type);
				}
			} else {
				iprint("No response\n");
			}
		} else {
			iprint("VIRTIO GPU unknown request type %d\n", request->hdr.type);
		}
	} else {
		iprint("Used elem pointed to NULL descriptor\n");
	}

	virtq_free_chain(queue, desc);
}

void
gpu_display_info(void)
{
	virtio_gpu_ctrl_hdr *request;
	virtio_gpu_resp_display_info *response;

	request = malloc(sizeof(*request));
	response = malloc(sizeof(*response));

	if (request == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

	virtq_add_desc_chain(&gpu->queues[0], gpu_info_handler, 0, 2,
	                     request, sizeof(*request), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);
	virtq_notify(gpu, 0, 1, -1);
}

void
gpu_create_resource(struct display *d)
{
	virtio_gpu_resource_create_2d *request;
	virtio_gpu_ctrl_hdr *response;

	request = malloc(sizeof(*request));
	response = malloc(sizeof(*response));

	if (request == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
	request->format = d->format;
	request->width = d->width;
	request->height = d->height;
	request->resource_id = atomic_inc(&gpu_resource_id);

	virtq_add_desc_chain(&gpu->queues[0], gpu_create_resource_handler, d, 2,
	                     request, sizeof(*request), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);
	virtq_notify(gpu, 0, 1, -1);
}

void
gpu_attach_backing(struct display *d)
{
	virtio_gpu_resource_attach_backing *request = malloc(sizeof(*request));
	virtio_gpu_mem_entry *entry = malloc(sizeof(*entry));
	virtio_gpu_ctrl_hdr *response = malloc(sizeof(*response));

	if (request == 0 || entry == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
	request->resource_id = d->resource;
	request->nr_entries = 1;
	entry->addr = (uintptr) d->framebuffer;
	entry->length = d->fb_length;
	entry->padding = 0;

	virtq_add_desc_chain(&gpu->queues[0], gpu_attach_handler, d, 3,
	                     request, sizeof(*request), 0,
	                     entry, sizeof(*entry), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);
	virtq_notify(gpu, 0, 1, -1);
}

void
gpu_set_scanout(struct display *d, int enable)
{
	virtio_gpu_set_scanout *request = malloc(sizeof(*request));
	virtio_gpu_ctrl_hdr *response = malloc(sizeof(*response));

	if (request == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
	request->scanout_id = enable == 1 ? d->scanout : 0;
	request->resource_id = d->resource;
	request->r.x = 0;
	request->r.y = 0;
	request->r.width = d->width;
	request->r.height = d->height;

	virtq_add_desc_chain(&gpu->queues[0], gpu_set_scanout_handler, d, 2,
	                     request, sizeof(*request), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);
	virtq_notify(gpu, 0, 1, -1);
}

void
gpu_setup_fb(int width, int height, int depth, int format)
{
	struct display *d = &displays[0];

	if (depth != 32) {
		panic("VIRTIO GPU only supports 32 bit depth\n");
	}

	d->flags = 0;
	d->width = width;
	d->height = height;
	d->format = format;
	d->scanout = 0;

	gpu_create_resource(d);

	while ((d->flags & GPU_F_RES) == 0) {
	}

	usize fb_len = height*width*4;
	d->fb_length = fb_len;
	d->framebuffer = malloc(fb_len);

	if (d->framebuffer == 0) {
		panic("Malloc returned 0 when allocating framebuffer");
	}

	gpu_attach_backing(d);

	while ((d->flags & GPU_F_ATTACH) == 0) {
	}

	gpu_set_scanout(d, 1);

	while ((d->flags & GPU_F_SCANOUT) == 0) {
	}
}

void
gpu_transfer(struct display *d, virtio_gpu_rect r, int notify)
{
	virtio_gpu_transfer_to_host_2d *request;
	virtio_gpu_ctrl_hdr *response;

	request = malloc(sizeof(*request));
	response = malloc(sizeof(*response));

	if (request == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
	request->resource_id = d->resource;
	request->offset = (r.x + r.y * d->width)*4;
	request->r = r;

	virtq_add_desc_chain(&gpu->queues[0], 0, 0, 2,
	                     request, sizeof(*request), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);

	if (notify)
		virtq_notify(gpu, 0, 1, -1);
}

void
gpu_flush(struct display *d, virtio_gpu_rect r, int notify)
{
	virtio_gpu_resource_flush *request;
	virtio_gpu_ctrl_hdr *response;

	request = malloc(sizeof(*request));
	response = malloc(sizeof(*response));

	if (request == 0 || response == 0) {
		panic("Can't allocate memory\n");
	}

	request->hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
	request->resource_id = d->resource;
	request->r = r;

	virtq_add_desc_chain(&gpu->queues[0], 0, 0, 2,
	                     request, sizeof(*request), 0,
	                     response, sizeof(*response), 1);

	virtq_make_available(&gpu->queues[0]);

	if (notify)
		virtq_notify(gpu, 0, 1, -1);
}

void
gpu_config_handler(virtio_dev *dev)
{
	virtio_gpu_config *config = (void*) &dev->regs->config;
	int events = config->events_read;

	if (events & VIRTIO_GPU_EVENT_DISPLAY) {
		gpu_display_info();
	} else if (events != 0) {
		iprint("GPU unknown event %d\n", events);
	}

	config->events_clear = events;
}

void
gpu_default_handler(virtq *queue, void *dev, int idx, int len)
{
	virtq_desc *desc = &queue->desc[idx];
	virtio_gpu_ctrl_hdr *request = (void*) desc->addr;

	if (request != 0) {
		virtq_desc *tmp = desc;
		virtio_gpu_ctrl_hdr *resp;

		while (tmp != 0) {
			if (resp == 0 && tmp->flags & VIRTQ_DESC_F_WRITE) {
				resp = (void*) tmp->addr;
			}

			if (tmp->flags & VIRTQ_DESC_F_WRITE) {
				tmp = &queue->desc[tmp->next];
			} else {
				tmp = 0;
			}
		}

		switch (request->type) {
		case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
		case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
			if (resp != 0) {
				virtio_gpu_ctrl_hdr *resp;
				switch (resp->type) {
				case VIRTIO_GPU_RESP_OK_NODATA:
					iprint("VIRTIO GPU: OK NODATA\n");
					break;
				default:
					iprint("VIRTIO GPU: Transfer or flush, unknown response %d\n", resp->type);
				}
			} else {
				iprint("VIRTIO GPU: No response for request %d\n", request->type);
			}
			break;
		default:
			iprint("VIRTIO GPU: unknown request type %d\n", request->type);
		}
	} else {
		iprint("VIRTIO GPU: Used elem pointed to NULL descriptor\n");
	}

	virtq_free_chain(queue, desc);

}

void
gpu_invalidate(uint x, uint y, uint width, uint height)
{
	int thresh = 20;
	struct gpu_rect_elem **elem;
	virtio_gpu_rect new = {
		.x = x,
		.y = y,
		.width = width,
		.height = height,
	};
	virtio_gpu_rect old;

	qlock(&gpu_invalidate_lock);

	// Try to merge with an existing rect, if it's close enough
#define min(x, y) (x < y ? x : y)
#define max(x, y) (x >= y ? x : y)
#define is_close(x, y) (x >= y-thresh && x <= y+thresh)
	for (elem = &gpu_rect_queue; *elem != nil; elem = &(*elem)->next) {
		old = (*elem)->r;

		int close_left   = is_close(new.x, old.x + old.width);
		int close_right  = is_close(new.x + new.width, old.x);
		int close_top    = is_close(new.y, old.y + old.height);
		int close_bottom = is_close(new.y + new.height, old.y);

		int close_horizontal = is_close(new.y, old.y) && is_close(new.height, old.height);
		int close_vertical   = is_close(new.x, old.x) && is_close(new.width, old.width);

		if (((close_left || close_right) && close_horizontal) || ((close_top || close_bottom) && close_vertical)) {
			(*elem)->r.x      = min(new.x, old.x);
			(*elem)->r.width  = max(new.x+new.width, old.x+old.width) - old.x;
			(*elem)->r.y      = min(new.y, old.y);
			(*elem)->r.height = max(new.y+new.height, old.y+old.height) - old.y;
			break;
		}
	}

	if (*elem == nil) {
		// Did not merge, add it at the end
		*elem = malloc(sizeof(**elem));
		(*elem)->r = new;
		(*elem)->next = nil;
	}

	qunlock(&gpu_invalidate_lock);
}

void
gpu_timer(Ureg*, Timer*)
{
	struct display *d = &displays[0];
	struct gpu_rect_elem *elem = gpu_rect_queue;
	struct gpu_rect_elem *prev;

	qlock(&gpu_invalidate_lock);
	memory_fence();

	while (elem != nil) {
		// Transfer to GPU
		gpu_transfer(d, elem->r, 0);

		// Flush GPU
		gpu_flush(d, elem->r, 0);

		prev = elem;
		elem = elem->next;
		free(prev);

		if (elem == nil) {
			virtq_notify(gpu, 0, 1, -1);
			break;
		}
	}

	gpu_rect_queue = nil;

	qunlock(&gpu_invalidate_lock);
}

int
gpu_virtq_init(virtio_dev *dev)
{
	if (dev->queues == 0) {
		dev->queues = malloc(2*sizeof(virtq));
		dev->numqueues = 2;

		if (dev->queues == 0) {
			panic("Could not allocate queues. Malloc failed\n");
		}

		if (virtq_alloc(dev, 0, 0) != 0) {
			panic("Virtio GPU: Failed to create control queue");
			return -1;
		}
		if (virtq_alloc(dev, 1, 0) != 0) {
			panic("Virtio GPU: Failed to create cursor queue");
			return -1;
		}
	}

	return 0;
}

int
gpu_init(void)
{
	gpu = virtio_get_device(VIRTIO_DEV_GPU);

	if (gpu != 0) {
		int err = virtio_setup(gpu, "GPU", gpu_virtq_init, VIRTIO_F_ANY_LAYOUT
		                      | VIRTIO_F_RING_INDIRECT_DESC | VIRTIO_F_RING_EVENT_IDX
		                      | VIRTIO_GPU_F_EDID);
		switch (err) {
		case 0:
			virtio_enable_interrupt(gpu, gpu_config_handler);

			if ((((virtio_gpu_config*) &gpu->regs->config)->events_read & VIRTIO_GPU_EVENT_DISPLAY) == 0) {
				// Only update display info if the gpu hasn't already marked it.
				// Then it will be updated by the config handler

				gpu_display_info();
			}

			// Run the config handler right away, don't wait for interrupt
			gpu_config_handler(gpu);

			while (pmodes[0].enabled != 1) {}

			// Set the framebuffer to invalidate (only changed regions) at a given framerate
			Timer *frametimer = malloc(sizeof(*frametimer));
			frametimer->tmode = Tperiodic;
			frametimer->tns = FRAMENANO;
			frametimer->tf = gpu_timer;
			timeradd(frametimer);
			break;
		case -1:
			iprint("Virtio GPU rejected features\n");
			return -1;
		case -2:
			iprint("Virtio GPU queue error\n");
			return -1;
		default:
			iprint("Virtio GPU setup unknown error %d\n", err);
			return -1;
		}

		return 0;
	} else {
		return -1;
	}
}

/*
 * Framebuffer
 */

static int
fbdefault(int *width, int *height, int *depth)
{
	*width = pmodes[0].r.width;
	*height = pmodes[0].r.height;
	*depth = 32;
	return 0;
}

void*
fbinit(int set, int *width, int *height, int *depth)
{
	if (gpu_init() != 0) {
		return 0;
	}

	if(!set) {
		fbdefault(width, height, depth);
	}

	gpu_setup_fb(*width, *height, *depth, VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM);
	void *fb = displays[0].framebuffer;

	if(fb) {
		memset((char*)fb, 0x7F, displays[0].fb_length);
		gpu_invalidate(0, 0, *width, *height);
	} else {
		iprint("Could not set up framebuffer\n");
	}

	return fb;
}

void
fbflush(uint min_x, uint min_y, uint max_x, uint max_y)
{
	uint x = min_x;
	uint y = min_y;
	uint width = max_x - x;
	uint height = max_y - y;

	gpu_invalidate(x, y, width, height);
}

int
fbblank(int blank)
{
	// Set scanout to 0
	gpu_set_scanout(&displays[0], blank);

	return 0;
}
