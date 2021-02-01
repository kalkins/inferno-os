/* SBI Extension IDs */
#define SBI_EXT_SET_TIMER				0x0
#define SBI_EXT_CONSOLE_PUTCHAR			0x1
#define SBI_EXT_CONSOLE_GETCHAR			0x2
#define SBI_EXT_CLEAR_IPI				0x3
#define SBI_EXT_SEND_IPI				0x4
#define SBI_EXT_REMOTE_FENCE_I			0x5
#define SBI_EXT_REMOTE_SFENCE_VMA		0x6
#define SBI_EXT_REMOTE_SFENCE_VMA_ASID	0x7
#define SBI_EXT_SHUTDOWN				0x8
#define SBI_EXT_BASE					0x10
#define SBI_EXT_TIME					0x54494D45
#define SBI_EXT_IPI						0x735049
#define SBI_EXT_RFENCE					0x52464E43
#define SBI_EXT_HSM						0x48534D

/* SBI function IDs for BASE extension*/
#define SBI_BASE_GET_SPEC_VERSION	0x0
#define SBI_BASE_GET_IMPL_ID		0x1
#define SBI_BASE_GET_IMPL_VERSION	0x2
#define SBI_BASE_PROBE_EXT			0x3
#define SBI_BASE_GET_MVENDORID		0x4
#define SBI_BASE_GET_MARCHID		0x5
#define SBI_BASE_GET_MIMPID			0x6

/* SBI Implementation IDs */
#define SBI_IMPL_BBL				0
#define SBI_IMPL_OPENSBI			1
#define SBI_IMPL_Xvisor				2
#define SBI_IMPL_KVM				3
#define SBI_IMPL_RUSTSBI			4
#define SBI_IMPL_DIOSIX				5

/* SBI function IDs for RFENCE extension*/
#define SBI_RFENCE_REMOTE_FENCE_I			0x0
#define SBI_RFENCE_REMOTE_SFENCE_VMA		0x1
#define SBI_RFENCE_REMOTE_SFENCE_VMA_ASID	0x2
#define SBI_RFENCE_REMOTE_HFENCE_GVMA		0x3
#define SBI_RFENCE_REMOTE_HFENCE_GVMA_VMID	0x4
#define SBI_RFENCE_REMOTE_HFENCE_VVMA		0x5
#define SBI_RFENCE_REMOTE_HFENCE_VVMA_ASID	0x6

/* SBI function IDs for HSM extension */
#define SBI_HSM_HART_START			0x0
#define SBI_HSM_HART_STOP			0x1
#define SBI_HSM_HART_GET_STATUS		0x2

#define SBI_HSM_HART_STATUS_STARTED			0x0
#define SBI_HSM_HART_STATUS_STOPPED			0x1
#define SBI_HSM_HART_STATUS_START_PENDING	0x2
#define SBI_HSM_HART_STATUS_STOP_PENDING	0x3

#define SBI_VERSION_MAJOR_OFFSET	24
#define SBI_VERSION_MAJOR_MASK		0x7f
#define SBI_VERSION_MINOR_MASK		0xffffff
#define SBI_VENDOR_START			0x09000000
#define SBI_VENDOR_END				0x09FFFFFF
#define SBI_FIRMWARE_START			0x0A000000
#define SBI_FIRMWARE_END			0x0AFFFFFF

/* SBI function IDs for time extension */
#define SBI_TIME_SET_TIMER			0x0

/* SBI return error codes */
#define SBI_SUCCESS					0
#define SBI_ERR_FAILED				-1
#define SBI_ERR_NOT_SUPPORTED		-2
#define SBI_ERR_INVALID_PARAM		-3
#define SBI_ERR_DENIED				-4
#define SBI_ERR_INVALID_ADDRESS		-5
#define SBI_ERR_ALREADY_AVAILABLE	-6

/* SBI call shorthands for different numbers of arguments */
#define SBI_ECALL(__num, __func, __a0, __a1, __a2)	sbi_ecall(__a0, __a1, __a2, __num, __func)
#define SBI_ECALL_0(__num, __func) 					SBI_ECALL(__num, __func, 0, 0, 0)
#define SBI_ECALL_1(__num, __func, __a0)			SBI_ECALL(__num, __func, __a0, 0, 0)
#define SBI_ECALL_2(__num, __func, __a0, __a1)		SBI_ECALL(__num, __func, __a0, __a1, 0)

/* SBI call shorthands for common functions */
#define SBI_GET_SPEC_VERSION()		SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_SPEC_VERSION)
#define SBI_GET_IMPL_ID()			SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_IMPL_ID)
#define SBI_GET_IMPL_VERSION()		SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_IMPL_VERSION)
#define SBI_PROBE_EXTENSION(ID)		SBI_ECALL_1(SBI_EXT_BASE, SBI_BASE_PROBE_EXT, ID)
#define SBI_GET_MVENDORID()			SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_MVENDORID)
#define SBI_GET_MARCHID()			SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_MARCHID)
#define SBI_GET_MIMPID()			SBI_ECALL_0(SBI_EXT_BASE, SBI_BASE_GET_MIMPID)

/* Legacy extensions */
#define SBI_SEND_IPI(VAL)			SBI_ECALL_1(SBI_EXT_SEND_IPI, 0, VAL)
#define SBI_CLEAR_IPI(VAL)			SBI_ECALL_1(SBI_EXT_CLEAR_IPI, 0, VAL)
#define SBI_GETCHAR()				SBI_ECALL_0(SBI_EXT_CONSOLE_GETCHAR, 0)
#define SBI_PUTCHAR(VAL)			SBI_ECALL_1(SBI_EXT_CONSOLE_PUTCHAR, 0, VAL)
#define SBI_SHUTDOWN()				SBI_ECALL_0(SBI_EXT_SHUTDOWN, 0)

typedef struct {
	long error;
	long value;
} sbiret;

sbiret sbi_ecall(long a0, long a1, long a2, long num, long func);

char* get_sbi_error(int error);
sbiret sbi_console_puts(char *str, int len);
sbiret sbi_console_print(char *str);
char sbi_console_getchar(void);

/*
 * Set and interrupt for when the clock reaches the given time.
 * Return an SBI error code.
 */
long sbi_set_timer(u64int time);
