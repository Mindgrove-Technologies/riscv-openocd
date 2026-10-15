
#include "s2401_common.h"


/* ---- s2401 command leaf table ---- */
const struct command_registration s2401_command_handlers[] = {

    {
        .name = "psram_quad_init",
        .handler = s2401_handle_psram_quad_init,
        .mode = COMMAND_EXEC,
        .help = "Initialize PSRAM in quad lines",
        .usage = "<instance> [prescaler]",
    },

    {
        .name = "psram_single_init",
        .handler = s2401_handle_psram_single_init,
        .mode = COMMAND_EXEC,
        .help = "Initialize PSRAM in single line",
        .usage = "<instance> [prescaler]",
    },
    
    {
        .name = "flash_erase",
        .handler = s2401_handle_flash_erase,
        .mode = COMMAND_EXEC,
        .help = "Flash erase",
        .usage = "<instance> [prescaler]",
    },

    {
        .name = "flash_quad_write",
        .handler = s2401_handle_flash_quad_write,
        .mode = COMMAND_EXEC,
        .help = "Flash quad lines write",
        .usage = "<file> <address> [prescaler]",
    },

    {
        .name = "flash_single_write",
        .handler = s2401_handle_flash_single_write,
        .mode = COMMAND_EXEC,
        .help = "Flash single line write",
        .usage = "<file> <address> [prescaler]",
    },
    {
        .name = "flash_quad_xip_init",
        .handler = s2401_handle_flash_quad_xip,
        .mode = COMMAND_EXEC,
        .help = "Enable flash XIP in four lines",
        .usage = "<instance> [prescaler]",
    },
    
    {
        .name = "flash_single_xip_init",
        .handler = s2401_handle_flash_single_xip,
        .mode = COMMAND_EXEC,
        .help = "Enable flash XIP in single line",
        .usage = "<instance> [prescaler]",
    },

    {
        .name = "flash_sector_erase",
        .handler = s2401_handle_sector_erase,
        .mode = COMMAND_EXEC,
        .help = "Erase flash sector",
        .usage = "<mode> <address> <num_sectors> [prescaler]",

    },

    {
        .name = "reset",
        .handler = s2401_handle_reset,
        .mode = COMMAND_EXEC,
        .help = "Reset s2401",
        .usage = "",
    },

    {
        .name = "flash_write_length",
        .handler = s2401_handle_flash_write_length,
        .mode = COMMAND_EXEC,
        .usage = "<instance> <file> <address> <no_of_lines> [prescaler]",
    },

    {
        .name = "flash_write_data",
        .handler = s2401_handle_flash_write_data,
        .mode = COMMAND_EXEC,
        .help = "Write the data in the mentioned address location  ",
        .usage = "<address> <no_of_lines> <data> [prescaler]",
    },

    {
        .name = "flash_read_status",
        .handler = s2401_handle_flash_read_status_register,
        .mode = COMMAND_EXEC,
        .help = "Read the status register",
        .usage = "<instance> <register_no> [prescaler]",
    },

    {
        .name = "flash_write_status",
        .handler = s2401_handle_flash_write_status_register,
        .mode = COMMAND_EXEC,
        .help = "Write the status register",
        .usage = "<instance> <register_no> <status_data> [prescaler]",
    },

    COMMAND_REGISTRATION_DONE
};


int s2401_register_commands(struct command_context *cmd_ctx)
{
    return register_commands(cmd_ctx, NULL, s2401_command_handlers);
}
