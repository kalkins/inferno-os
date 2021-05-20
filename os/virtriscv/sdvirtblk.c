#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "virtio.h"

#include "../port/sd.h"

extern SDifc sdvirtblkifc;

SDev *head;

static int
blk_virtq_init(virtio_dev *dev)
{
	if (dev->queues == 0) {
		dev->queues = malloc(sizeof(virtq));
		dev->numqueues = 1;

		if (dev->queues == 0) {
			panic("Virtio blk: Could not allocate queues. Malloc failed\n");
		}

		if (virtq_alloc(dev, 0, 0) != 0) {
			panic("Virtio blk: Failed to create event queue");
			return -1;
		}

		dev->queues[0].default_handler = nil;
		dev->queues[0].default_handler_data = dev;
	}

	return 0;
}

static int
blk_enable(SDev* sdev)
{
	virtio_enable_interrupt(sdev->ctlr, nil);

	return 1;
}

static int
blk_disable(SDev* sdev)
{
	virtio_disable_interrupt(sdev->ctlr);

	return 1;
}

static SDev*
blk_pnp(void)
{
	virtio_dev *dev;
	SDev *sdev;
	SDev **next;

	for (next = &head; *next != 0; *next = (*next)->next) {}

	while ((dev = virtio_get_device(VIRTIO_DEV_BLOCK)) != 0) {
		int err = virtio_setup(dev, "BLK", blk_virtq_init, VIRTIO_F_ANY_LAYOUT
		                       | VIRTIO_F_RING_INDIRECT_DESC | VIRTIO_F_RING_EVENT_IDX
		                       | VIRTIO_BLK_F_RO | VIRTIO_BLK_F_SIZE_MAX | VIRTIO_BLK_F_SEG_MAX);

		switch (err) {
		case 0:
			sdev = malloc(sizeof(SDev));
			sdev->ctlr = dev;
			sdev->ifc = &sdvirtblkifc;
			sdev->nunit = 1;

			*next = sdev;
			next = &sdev->next;

			blk_enable(sdev);

			break;
		case -1:
			iprint("Virtio blk rejected features\n");
			break;
		case -2:
			iprint("Virtio blk queue error\n");
			break;
		default:
			iprint("Virtio blk unknown error during setup %d\n", err);
			break;
		}
	}

	return head;
}

static SDev*
blk_id(SDev* sdev)
{
	char name[16];
	virtio_dev *dev;
	static char idno[16] = "0123456789";

	for (int i = 0; sdev != nil; sdev = sdev->next) {
		if (sdev->ifc == &sdvirtblkifc) {
			sdev->idno = idno[i++];

			snprint(name, sizeof(name), "virtblk%c", sdev->idno);
			kstrdup(&sdev->name, name);
		}
	}

	return nil;
}

static int
blk_verify(SDunit *unit)
{
	virtio_dev *dev = unit->dev->ctlr;

	snprint((void*) &unit->inquiry[8], sizeof(unit->inquiry)-8,
	        "VIRTIO port %d Block Device", dev->index);

	unit->inquiry[4] = sizeof(unit->inquiry)-4;

	return 1;
}

static int
blk_online(SDunit *unit)
{
	virtio_dev *dev = (virtio_dev*) unit->dev->ctlr;
	virtio_blk_config *config = (virtio_blk_config*) &dev->regs->config;

	if (dev->features & VIRTIO_BLK_F_BLK_SIZE) {
		unit->secsize = config->blk_size;
	} else {
		unit->secsize = 512;
	}

	unit->sectors = config->capacity;

	return 1;
}

static long
blk_bio(SDunit* unit, int lun, int write, void* data, long nb, long bno)
{
	ulong len = nb * unit->secsize;
	virtio_dev *dev = (virtio_dev*) unit->dev->ctlr;
	virtio_blk_config *config = (virtio_blk_config*) &dev->regs->config;
	virtio_blk_req *req;
	uchar *status;

	if (write && dev->features & VIRTIO_BLK_F_RO) {
		// The drive is read-only
		iprint("VIRTIO block write of read only device\n");
		return -1;
	} else if (bno + nb > config->capacity) {
		// Out of bounds
		iprint("VIRTIO block device %s out of bounds\n", write ? "Write" : "Read");
		return -1;
	}

	req = malloc(sizeof(*req));
	req->type = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
	req->sector = (bno * unit->secsize) / 512; // VIRTIO always uses sectors of 512, though the device might not
	status = &req->status;
	*status = 255;

	virtq_add_desc_chain(&dev->queues[0], nil, nil, 3,
	                     req, VIRTIO_BLK_HDR_SIZE, 0,
	                     data, len, write ? 0 : 1,
	                     status, VIRTIO_BLK_STATUS_SIZE, 1);

	virtq_make_available(&dev->queues[0]);
	virtq_notify(dev, 0, 0, -1);

	// Block until the drive responds
	while (*status == 255) {}

	switch (*status) {
	case VIRTIO_BLK_S_OK:
		free(req);
		return len;
		break;
	case VIRTIO_BLK_S_IOERR:
		iprint("VIRTIO block device IO error\n");
		break;
	case VIRTIO_BLK_S_UNSUPP:
		iprint("VIRTIO block device unsupported operation\n");
		break;
	default:
		iprint("VIRTIO block device returned %d\n", status);
		error("Unknown VIRTIO block device return code\n");
	}

	free(req);
	return -1;
}

SDifc sdvirtblkifc = {
	"virtblk",			/* name */

	blk_pnp,			/* pnp */
	nil,				/* legacy */
	blk_id,				/* id */
	blk_enable,			/* enable */
	blk_disable,			/* disable */

	blk_verify,			/* verify */
	blk_online,			/* online */
	nil,				/* rio */
	nil,				/* rctl */
	nil,				/* wctl */

	blk_bio,			/* bio */
};
