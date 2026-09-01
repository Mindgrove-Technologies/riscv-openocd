
#include <helper/types.h>       /* Basic types like uint32_t */
#include <helper/command.h>     /* Command registration and parsing macros */
#include <helper/log.h>         /* LOG_INFO, LOG_ERROR, etc. */
#include <target/target.h>       /* target_write_u32 and target struct definition */
#include "v2500_psram_driver.h"


#define POW2_MINUS1(n)   ((1U << (n)) - 1U)

/* Reset */
#define PSRAM_CMD_GLOBAL_RESET         0xFFFF    /**< Global Reset */

uint16_t V2500_PSRAM_Init(struct target *target,uint8_t xspinum) {

    xspi_msg global_reset_config ={.PRESCALER=6,.FTIE=1,.FMEM_SIZE=25,.CLK_MODE=0, .sioo=0, .TCEN=0, .TEIE=0,\
        .TOIE=0, .TCIE=0, .SMIE=0, .APMS=0, .PMM=0, .ABDTR=0, .csht=0,  .DDTR=0\
        ,.alternate_byte_size=0,.alternate_byte_mode=0,.address_size=0,.address_mode=0,.alternate_byte=0,.abort=0, .mm_mode=0, 
        .sshift=0, .dhqc=0, .status_mask=0, .status_match=0, .pir=0, .timeout=0, .rd_instr=0,
        .wr_instr=0, .rd_dcyc=0, .wr_dcyc=0,\
        .dual_mem=0, .FTF=0, .hw_protect=0,.enable=1 ,.ADDTR=1 ,.dqse=1 ,.dummy_cycles=0 ,.instruction_size=1 ,.instruction_mode=4 ,.data_mode=0,.IDTR=1 };
    
    global_reset_config.dummy_cycles=3;
    global_reset_config.functional_mode=CCR_FMODE_INDIRECT_WRITE;
    // global_reset_config.data_buffer=value;
    global_reset_config.instruction=PSRAM_CMD_GLOBAL_RESET;
    global_reset_config.address=0x0;
    global_reset_config.length=0x0;
    global_reset_config.dsize=0x0;
    global_reset_config.fthresh= POW2_MINUS1(0);
    

    XSPI_Transaction(target,xspinum,&global_reset_config);


    xspi_msg psram_ram_mode_config ={.PRESCALER=6,.FTIE=1,.FMEM_SIZE=25,.CLK_MODE=0, .sioo=0, .TCEN=0, .TEIE=0, .TOIE=0, .TCIE=0, .SMIE=0, .APMS=0, .PMM=0, .ABDTR=0, .csht=0,  .DDTR=1
        ,.alternate_byte_size=0,.alternate_byte_mode=0,.address_size=3,.address_mode=4,.alternate_byte=0,.abort=0,
        .mm_mode=1, 
        .sshift=0, .dhqc=0,.status_mask=0, .status_match=0, .pir=0, .timeout=0,
        .rd_instr=0x20, //Linear burst read
        .wr_instr=0xA0, //Linear burst write
        .rd_dcyc=2,
        .wr_dcyc=4,
        .dual_mem=0, .FTF=0, .hw_protect=0,.enable=1 ,.ADDTR=1 ,
        .dqse=1 ,.dummy_cycles=0 ,.instruction_size=0 ,.instruction_mode=4 ,.data_mode=4 ,
        .IDTR=0, .functional_mode=CCR_FMODE_MMM,.fthresh= POW2_MINUS1(0)};
    
    XSPI_Transaction(target,xspinum,&psram_ram_mode_config);

    return ERROR_OK;

}

int v2500_handle_psram_init(struct command_invocation *cmd)
{
    // 1. Check if an argument was actually provided
    if (CMD_ARGC != 1) {
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    struct target *target = get_current_target(CMD_CTX);
    if (!target) {
        return ERROR_FAIL;
    }

    // 2. Parse the argument (expecting a number/hex)
    uint8_t xspi_number;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], xspi_number);

    // 3. Strict 0 or 1 check
    if (xspi_number != 0 && xspi_number != 1) {
        command_print(CMD, "Error: Invalid value '%d'. Only 0 (Disable) or 1 (Enable) are allowed.", xspi_number);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }
    V2500_PSRAM_Init(target,xspi_number);
    command_print(CMD, "Ram mode is configured with xspi %x",xspi_number);
    return ERROR_OK;
}




