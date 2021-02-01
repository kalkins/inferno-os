#include "u.h"
#include "sbi.h"

char*
get_sbi_error(int error)
{
	switch(error) {
	case SBI_SUCCESS:
		return "Success";
		break;
	case SBI_ERR_FAILED:
		return "Request failed";
		break;
	case SBI_ERR_NOT_SUPPORTED:
		return "Not supported";
		break;
	case SBI_ERR_INVALID_PARAM:
		return "Invalid parameter";
		break;
	case SBI_ERR_DENIED:
		return "Request denied";
		break;
	case SBI_ERR_INVALID_ADDRESS:
		return "Invalid address";
		break;
	case SBI_ERR_ALREADY_AVAILABLE:
		return "Already available";
		break;
	default:
		return "Unknown error";
		break;
	}
}

sbiret
sbi_console_puts(char *str, int len)
{
	int i;
	sbiret ret;

	for (i=0; i<len; i++) {
		ret = SBI_PUTCHAR((ulong) str[i]);

		if (ret.error != SBI_SUCCESS) {
			break;
		}
	}


	return ret;
}

sbiret
sbi_console_print(char *str)
{
	sbiret ret;

	while (str && *str) {
		ret = SBI_PUTCHAR((ulong) *str);
		str++;

		if (ret.error != SBI_SUCCESS) {
			break;
		}
	}

	return ret;
}

char
sbi_ecall_console_getchar(void)
{
	sbiret result = SBI_GETCHAR();

	if (result.error == 0) {
		return result.value;
	} else {
		sbi_console_print("Could not get char: ");
		sbi_console_print(get_sbi_error(result.error));
		sbi_console_print("\n");
		return -1;
	}
}

long
sbi_set_timer(u64int time)
{
	ulong timel = time & 0xffffffff;
	ulong timeh = time >> 32;

	// Check if the timer extension is available
	sbiret ret = SBI_PROBE_EXTENSION(SBI_EXT_TIME);
	if (ret.error == SBI_SUCCESS && ret.value != 0) {
		// The extension is available
		ret = SBI_ECALL_2(SBI_EXT_TIME, SBI_TIME_SET_TIMER, timel, timeh);
	} else {
		// Default to the legacy extension
		ret = SBI_ECALL_2(SBI_EXT_SET_TIMER, 0, timel, timeh);
	}

	return ret.error;
}
