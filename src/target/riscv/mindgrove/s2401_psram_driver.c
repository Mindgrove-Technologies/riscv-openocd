#include <helper/types.h>       /* Basic types like uint32_t */
#include <helper/command.h>     /* Command registration and parsing macros */
#include <helper/log.h>         /* LOG_INFO, LOG_ERROR, etc. */
#include <target/target.h>       /* target_write_u32 and target struct definition */
#include "s2401_psram_driver.h"
#include "helper/types.h"

#define DEFAULT_FLASH_PRESCALER  10
#define FLASH_PRESCALER_MIN      6
#define FLASH_PRESCALER_MAX      0x3FFF
#define DEFAULT_FLASH_PRESCALER  10
#define POW2_MINUS1(n)   ((1U << (n)) - 1U)

qspi_msg psram_msg={.PRESCALER=6,.CLK_MODE=0,.FMEM_SIZE = 27,.FTIE = 0, \
    .TCEN=0,.TEIE=0,.TOIE=0,.SMIE = 0,.APMS= 0,.PMM=0,.csht = 7};

uint16_t PSRAM_Quad_Init(struct target *target,uint8_t qspinum) {
    /*Reset Enable*/
    psram_msg.address = 0x0U;
    psram_msg.address_mode = CCR_ADMODE_NIL;
    psram_msg.address_size = CCR_ADSIZE_24_BIT;
    psram_msg.instruction = 0x66;
    psram_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    psram_msg.dummy_mode = 0;
    psram_msg.data_mode = CCR_DMODE_NO_DATA;
    psram_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    psram_msg.dummy_cycles = 1;
    psram_msg.dummy_bit = 0;
    psram_msg.mm_mode = CCR_MM_MODE_XIP;
    psram_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    psram_msg.fthresh = POW2_MINUS1(0);
    psram_msg.length = 0;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);

    /*RESET command*/
    psram_msg.instruction = 0x99;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);
    
    /*Enter QAUD_MODE command*/
    psram_msg.instruction = 0x35;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);
    
    /*Enter RAM MODE 4 lines*/
    psram_msg.mm_mode = CCR_MM_MODE_RAM;
    psram_msg.functional_mode = CCR_FMODE_MMM;
    psram_msg.instruction_mode = CCR_IMODE_FOUR_LINE;
    psram_msg.address_mode = CCR_ADMODE_FOUR_LINE;
    psram_msg.data_mode = CCR_DMODE_FOUR_LINE;
    psram_msg.wr_instr=0x38;
    psram_msg.wr_dcyc=0;
    psram_msg.rd_instr=0xEB;
    psram_msg.rd_dcyc=6;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);
    return ERROR_OK;

}

uint16_t PSRAM_Single_Init(struct target *target,uint8_t qspinum) {
    /*Reset Enable*/
    psram_msg.address = 0x0U;
    psram_msg.address_mode = CCR_ADMODE_NIL;
    psram_msg.address_size = CCR_ADSIZE_24_BIT;
    psram_msg.instruction = 0x66;
    psram_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    psram_msg.dummy_mode = 0;
    psram_msg.data_mode = CCR_DMODE_NO_DATA;
    psram_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    psram_msg.dummy_cycles = 1;
    psram_msg.dummy_bit = 0;
    psram_msg.mm_mode = CCR_MM_MODE_XIP;
    psram_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    psram_msg.fthresh = POW2_MINUS1(0);
    psram_msg.length = 0;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);

    /*RESET command*/
    psram_msg.instruction = 0x99;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);
    

    
    /*Enter RAM MODE single line*/
    psram_msg.mm_mode = CCR_MM_MODE_RAM;
    psram_msg.functional_mode = CCR_FMODE_MMM;
    psram_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    psram_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    psram_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    psram_msg.wr_instr=0x02;
    psram_msg.wr_dcyc=0;
    psram_msg.rd_instr=0x0B;
    psram_msg.rd_dcyc=8;
    s2401_QSPI_Transaction(target,qspinum,&psram_msg);
    return ERROR_OK;

}

int s2401_handle_psram_quad_init(struct command_invocation *cmd)
{
    if (CMD_ARGC != 1) { // Instance is required
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: psram_quad_init <instance> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    psram_msg.PRESCALER = prescaler;

    struct target *target = get_current_target(CMD_CTX);
    if (!target) {
        return ERROR_FAIL;
    }

    uint8_t qspi_number;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);

    if (qspi_number != 0 && qspi_number != 1) {
        command_print(CMD, "Error: Invalid instance '%d'. Only 0 or 1  are allowed.", qspi_number);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }
    PSRAM_Quad_Init(target,qspi_number);
    command_print(CMD, "Ram mode is configured with qspi %x",qspi_number);
    return ERROR_OK;
}

int s2401_handle_psram_single_init(struct command_invocation *cmd)
{
    if (CMD_ARGC != 1) { // Instance is required
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: psram_quad_init <instance> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    psram_msg.PRESCALER = prescaler;

    struct target *target = get_current_target(CMD_CTX);
    if (!target) {
        return ERROR_FAIL;
    }

    uint8_t qspi_number;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);

    if (qspi_number != 0 && qspi_number != 1) {
        command_print(CMD, "Error: Invalid instance '%d'. Only 0 or 1  are allowed.", qspi_number);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }
    PSRAM_Single_Init(target,qspi_number);
    command_print(CMD, "Ram mode is configured with qspi %x",qspi_number);
    return ERROR_OK;
}