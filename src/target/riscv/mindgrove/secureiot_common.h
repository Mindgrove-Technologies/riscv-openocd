#ifndef SOC_COMMON_H
#define SOC_COMMON_H

#include <helper/command.h>

/* Use two underscores to declare the signature for external use */

__COMMAND_HANDLER(secure_iot_handle_flash_erase);
__COMMAND_HANDLER(secure_iot_handle_flash_write);
__COMMAND_HANDLER(secure_iot_handle_flash_xip);
__COMMAND_HANDLER(secure_iot_handle_sector_erase);
__COMMAND_HANDLER(secure_iot_handle_reset);
__COMMAND_HANDLER(secure_iot_handle_flash_write_length);
__COMMAND_HANDLER(secure_iot_handle_flash_write_data);



#endif