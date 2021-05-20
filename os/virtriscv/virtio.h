// The VIRTIO specification defines these types to specify the width and endianess.
// These types will be used in this file to be consistent with the standard.
typedef u8int	u8;
typedef u16int	u16;
typedef u16int	le16;
typedef u32int	u32;
typedef u32int	le32;
typedef u64int	u64;
typedef u64int	le64;

enum {
	VIRTIO_DEV_NET			= 1,
	VIRTIO_DEV_BLOCK		= 2,
	VIRTIO_DEV_GPU			= 16,
	VIRTIO_DEV_INPUT		= 18,

	VIRTIO_STATUS_ACK		= 1,
	VIRTIO_STATUS_DRIVER		= 2,
	VIRTIO_STATUS_FAILED		= 128,
	VIRTIO_STATUS_FEATURES_OK	= 8,
	VIRTIO_STATUS_DRIVER_OK		= 4,
	VIRTIO_STATUS_NEEDS_RESET	= 64,

	VIRTIO_F_NOTIFY_ON_EMPTY	= (le64) 1<<24,
	VIRTIO_F_ANY_LAYOUT		= (le64) 1<<27,
	VIRTIO_F_RING_INDIRECT_DESC	= (le64) 1<<28,
	VIRTIO_F_RING_EVENT_IDX		= (le64) 1<<29,
	VIRTIO_F_VERSION_1		= (le64) 1<<32,
	VIRTIO_F_ACCESS_PLATFORM	= (le64) 1<<33,
	VIRTIO_F_RING_PACKED		= (le64) 1<<34,
	VIRTIO_F_IN_ORDER		= (le64) 1<<35,
	VIRTIO_F_ORDER_PLATFORM		= (le64) 1<<36,
	VIRTIO_F_SR_IOV			= (le64) 1<<37,
	VIRTIO_F_NOTIFICATION_DATA	= (le64) 1<<38,

	VIRTQ_DESC_F_NEXT		= 1,
	VIRTQ_DESC_F_WRITE		= 2,
	VIRTQ_DESC_F_INDIRECT		= 4,
};

typedef struct virtq virtq;
typedef struct virtq_desc virtq_desc;
typedef struct virtq_avail virtq_avail;
typedef struct virtq_used_elem virtq_used_elem;
typedef struct virtq_used virtq_used;
typedef struct virtq_handler_list virtq_handler_list;

typedef struct virtio_regs virtio_regs;
typedef struct virtio_dev virtio_dev;

typedef int(*virtq_dev_specific_init)(virtio_dev *dev);
typedef void(*virtq_intr_handler)(virtq*, void* data, int idx, int len);
typedef void(*virtio_config_change_handler)(virtio_dev*);

// A struct matching the register in virtio-mmio.
// There are some undefined regions, here defined with underscore.
struct virtio_regs {
	le32 magic;
	le32 version;
	le32 deviceId;
	le32 vendorId;

	union {
		le32 hostFeatures;		// legacy
		le32 deviceFeatures;
	};
	union {
		le32 hostFeaturesSel;		// legacy
		le32 deviceFeaturesSel;
	};
	le32 _1[2];
	union {
		le32 guestFeatures;		// legacy
		le32 driverFeatures;
	};
	union {
		le32 guestFeaturesSel;		// legacy
		le32 driverFeaturesSel;
	};
	le32 guestPageSize;			// legacy
	le32 _2[1];

	le32 queueSel;
	le32 queueNumMax;
	le32 queueNum;
	le32 queueAlign;				// legacy
	le32 queuePfn;				// legacy
	le32 queueReady;
	le32 _3[2];
	le32 queueNotify;
	le32 _4[3];

	le32 interruptStatus;
	le32 interruptAck;
	le32 _5[2];

	le32 status;
	le32 _6[3];

	le32 queueDescLow;
	le32 queueDescHigh;
	le32 _7[2];
	le32 queueDriverLow;
	le32 queueDriverHigh;
	le32 _8[2];
	le32 queueDeviceLow;
	le32 queueDeviceHigh;
	le32 _9[2];

	le32 _10[19];
	le32 configGeneration;
	le32 config;
};

struct virtq_desc {
	le64 addr;
	le32 len;
	le16 flags;
	le16 next;
};

struct virtq_avail {
	le16 flags;
	le16 idx;
	le16 ring[1];
	//le16 used_event;
};

struct virtq_used_elem {
	le32 id;
	le32 len;
};

struct virtq_used {
	le16 flags;
	le16 idx;
	virtq_used_elem ring[1];
	//le16 avail_event;
};

struct virtq_handler_list {
	le16				idx;
	void				*data;
	virtq_intr_handler		handler;
	virtq_handler_list		*next;
};

struct virtq {
	le32		size;
	le32		desc_idx;
	le32		used_idx;
	le32		avail_pending;

	virtq_handler_list *intr_handlers;
	virtq_intr_handler default_handler;
	void		*default_handler_data;

	virtq_desc*	desc;

	union {
		virtq_avail*	avail;
		virtq_avail*	driver;
	};

	union {
		virtq_used*	used;
		virtq_used*	device;
	};
};

struct virtio_dev {
	virtio_regs	*regs;
	virtq		*queues;
	int		numqueues;
	int		index;
	int		type;
	int		version;
	int		enabled;
	char		*name;
	le64		features;
	virtio_config_change_handler config_change_handler;
	QLock;
};

void virtio_init(void);

int virtio_setup(virtio_dev *dev, char *name, virtq_dev_specific_init virtq_init, le64 features);

void virtio_disable(virtio_dev *dev);

virtio_dev *virtio_get_device(int type);

void virtio_enable_interrupt(virtio_dev *dev, virtio_config_change_handler config_change_handler);

void virtio_disable_interrupt(virtio_dev *dev);

int virtq_alloc(virtio_dev *dev, uint queueIdx, ulong size);

int virtq_add_desc_chain(virtq *queue, virtq_intr_handler handler, void *handler_data, uint num, ...);

void virtq_free_chain(virtq *queue, virtq_desc *head);

void virtq_make_available(virtq *queue);

virtq_used_elem *virtq_get_next_used(virtq *queue);

void virtq_notify(virtio_dev *dev, int queuenum, int notify_response, int avail_idx);


/************************* GPU *************************/

#define VIRTIO_GPU_F_VIRGL		(1<<0)
#define VIRTIO_GPU_F_EDID		(1<<1)
#define VIRTIO_GPU_FLAG_FENCE		(1<<0)

#define VIRTIO_GPU_MAX_SCANOUTS 16
#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)

enum {
        /* 2d commands */
        VIRTIO_GPU_CMD_GET_DISPLAY_INFO = 0x0100,
        VIRTIO_GPU_CMD_RESOURCE_CREATE_2D,
        VIRTIO_GPU_CMD_RESOURCE_UNREF,
        VIRTIO_GPU_CMD_SET_SCANOUT,
        VIRTIO_GPU_CMD_RESOURCE_FLUSH,
        VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D,
        VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING,
        VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING,
        VIRTIO_GPU_CMD_GET_CAPSET_INFO,
        VIRTIO_GPU_CMD_GET_CAPSET,
        VIRTIO_GPU_CMD_GET_EDID,

        /* cursor commands */
        VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
        VIRTIO_GPU_CMD_MOVE_CURSOR,

        /* success responses */
        VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
        VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
        VIRTIO_GPU_RESP_OK_CAPSET_INFO,
        VIRTIO_GPU_RESP_OK_CAPSET,
        VIRTIO_GPU_RESP_OK_EDID,

        /* error responses */
        VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
        VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
        VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
        VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
        VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
        VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,

        /* GPU formats */
        VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM  = 1,
        VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM  = 2,
        VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM  = 3,
        VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM  = 4,

        VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM  = 67,
        VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM  = 68,

        VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM  = 121,
        VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM  = 134,
};

typedef struct {
        le32 events_read;
        le32 events_clear;
        le32 num_scanouts;
        le32 reserved;
} virtio_gpu_config;

typedef struct {
	le32 type;
	le32 flags;
	le32 fence_id;
	le32 ctx_id;
	le32 padding;
} virtio_gpu_ctrl_hdr;

typedef struct {
        le32 x;
        le32 y;
        le32 width;
        le32 height;
} virtio_gpu_rect;

typedef struct {
	virtio_gpu_rect r;
	le32 enabled;
	le32 flags;
} virtio_gpu_pmode;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        virtio_gpu_pmode pmodes[VIRTIO_GPU_MAX_SCANOUTS];
} virtio_gpu_resp_display_info;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        le32 resource_id;
        le32 format;
        le32 width;
        le32 height;
} virtio_gpu_resource_create_2d;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        le32 resource_id;
        le32 nr_entries;
} virtio_gpu_resource_attach_backing;

typedef struct {
        le64 addr;
        le32 length;
        le32 padding;
} virtio_gpu_mem_entry;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        virtio_gpu_rect r;
        le32 scanout_id;
        le32 resource_id;
} virtio_gpu_set_scanout;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        virtio_gpu_rect r;
        le64 offset;
        le32 resource_id;
        le32 padding;
} virtio_gpu_transfer_to_host_2d;

typedef struct {
        virtio_gpu_ctrl_hdr hdr;
        virtio_gpu_rect r;
        le32 resource_id;
        le32 padding;
} virtio_gpu_resource_flush;


/*********************** Input ***********************/

enum {
	EV_SYN				= 0x00,
	EV_KEY				= 0x01,
	EV_REL				= 0x02,
	EV_ABS				= 0x03,
	EV_MSC				= 0x04,
	EV_SW				= 0x05,
	EV_LED				= 0x11,
	EV_SND				= 0x12,
	EV_REP				= 0x14,
	EV_FF				= 0x15,
	EV_PWR				= 0x16,
	EV_FF_STATUS			= 0x17,

	SYN_REPORT			= 0,
	SYN_CONFIG			= 1,
	SYN_MT_REPORT			= 2,
	SYN_DROPPED			= 3,

	VIRTIO_INPUT_CFG_UNSET		= 0x00,
	VIRTIO_INPUT_CFG_ID_NAME	= 0x01,
	VIRTIO_INPUT_CFG_ID_SERIAL	= 0x02,
	VIRTIO_INPUT_CFG_ID_DEVIDS	= 0x03,
	VIRTIO_INPUT_CFG_PROP_BITS	= 0x10,
	VIRTIO_INPUT_CFG_EV_BITS	= 0x11,
	VIRTIO_INPUT_CFG_ABS_INFO	= 0x12,
};

typedef struct {
	le32  min;
	le32  max;
	le32  fuzz;
	le32  flat;
	le32  res;
} virtio_input_absinfo;

typedef struct {
	le16  bustype;
	le16  vendor;
	le16  product;
	le16  version;
} virtio_input_devids;

typedef struct {
	u8    select;
	u8    subsel;
	u8    size;
	u8    reserved[5];
	union {
		char string[128];
		u8   bitmap[128];
		virtio_input_absinfo abs;
		virtio_input_devids ids;
	};
} virtio_input_config;

typedef struct {
	le16 type;
	le16 code;
	le32 value;
} virtio_input_event;


/********************* Block device *********************/

enum {
	/* Feature flags */
	VIRTIO_BLK_F_BARRIER		= (le64) 1<<0,
	VIRTIO_BLK_F_SIZE_MAX		= (le64) 1<<1,
	VIRTIO_BLK_F_SEG_MAX		= (le64) 1<<2,
	VIRTIO_BLK_F_GEOMETRY		= (le64) 1<<4,
	VIRTIO_BLK_F_RO			= (le64) 1<<5,
	VIRTIO_BLK_F_BLK_SIZE		= (le64) 1<<6,
	VIRTIO_BLK_F_SCSI		= (le64) 1<<7,
	VIRTIO_BLK_F_FLUSH		= (le64) 1<<9,
	VIRTIO_BLK_F_TOPOLOGY		= (le64) 1<<10,
	VIRTIO_BLK_F_CONFIG_WCE		= (le64) 1<<11,
	VIRTIO_BLK_F_DISCARD		= (le64) 1<<13,
	VIRTIO_BLK_F_WRITE_ZEROES	= (le64) 1<<14,

	/* Request types */
	VIRTIO_BLK_T_IN			= 0,
	VIRTIO_BLK_T_OUT		= 1,
	VIRTIO_BLK_T_SCSI_CMD		= 2,
	VIRTIO_BLK_T_SCSI_CMD_OUT	= 3,
	VIRTIO_BLK_T_FLUSH		= 4,
	VIRTIO_BLK_T_DISCARD		= 11,
	VIRTIO_BLK_T_WRITE_ZEROES	= 13,

	/* Status codes */
	VIRTIO_BLK_S_OK			= 0,
	VIRTIO_BLK_S_IOERR		= 1,
	VIRTIO_BLK_S_UNSUPP		= 2,
};

typedef struct {
	le64 capacity;
	le32 size_max;
	le32 seg_max;
	struct virtio_blk_geometry {
		le16 cylinders;
		u8 heads;
		u8 sectors;
	} geometry;
	le32 blk_size;
	struct virtio_blk_topology {
		u8 physical_block_exp;
		u8 alignment_offset;
		le16 min_io_size;
		le32 opt_io_size;
	} topology;
	u8 writeback;
	u8 unused0[3];
	le32 max_discard_sectors;
	le32 max_discard_seg;
	le32 discard_sector_alignment;
	le32 max_write_zeroes_sectors;
	le32 max_write_zeroes_seg;
	u8 write_zeroes_may_unmap;
	u8 unused1[3];
} virtio_blk_config;

#define VIRTIO_BLK_HDR_SIZE	(sizeof(le32)+sizeof(le32)+sizeof(le64))
#define VIRTIO_BLK_STATUS_SIZE	(sizeof(u8))

typedef struct {
        le32 type;
        le32 reserved;
        le64 sector;
	//u8 data[1]; // This is sent in a separate descriptor
	u8 status;
} virtio_blk_req;

typedef struct {
       le64 sector;
       le32 num_sectors;
       struct {
               le32 unmap:1;
               le32 reserved:31;
       } flags;
} virtio_blk_discard_write_zeroes;
