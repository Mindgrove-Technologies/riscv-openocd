#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include<elf.h>
#include <helper/log.h>
#include"s2401_flash_driver.h"
#include "target/riscv/riscv.h"
#include "target/riscv/riscv_reg.h"
#include <inttypes.h> 
#include "target/target.h"
#include "target/register.h"
#include "helper/types.h"
#include "helper/binarybuffer.h"

#define CHUNK_SIZE 128

#define DEFAULT_FLASH_PRESCALER  10
#define FLASH_PRESCALER_MIN      6
#define FLASH_PRESCALER_MAX      0x3FFF
#define DEFAULT_FLASH_PRESCALER  10
qspi_msg s2401_flash_msg={.PRESCALER=DEFAULT_FLASH_PRESCALER,.CLK_MODE=0,.FMEM_SIZE = 27,.FTIE = 0,.TCEN=0,.TEIE=0,.TOIE=0,.SMIE = 0,.APMS= 0,.PMM=0};


/**
 * @fn uint32_t fastReadQuad(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length)
 * 
 * @brief Used to read data from flash through all the four data lines and instruction through one address lines.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data is a pointer to array which contains data to be read from flash.
 * @param address The parameter \a address specifies from which start address data to be read from flash through QSPI.
 * @param data_length The parameter \a data_length specifies how many bytes of data should be read from flash.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
void s2401_print_progress_bar(int progress, int total) {
    int bar_width = 50;  // Width of the progress bar
    float progress_percentage = (float)progress / total;
    int filled = (int)(progress_percentage * bar_width);

    // Print the progress bar
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\r[");
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) {
            log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "#");  // Filled part of the bar
        } else {
            log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, " ");  // Empty part of the bar
        }
    }
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "] %.2f%%", progress_percentage * 100);
    // fflush(stdout);  // Ensure that the output is immediately displayed
}

/**
 * @brief s2401_handle_change_pc handler to force the target's Program Counter (PC) to 0x80000000.
 *
 * This function retrieves the current active target from the command context and verifies
 * that it has been fully examined. To safely handle diverse architectures, it prioritizes 
 * the RISC-V Debug Program Counter (@c dpc) before falling back to the standard standard 
 * Program Counter (@c pc). 
 *
 * It validates the register's existence and bit-width to prevent buffer overflow/assertion 
 * crashes, seamlessly writing to either 32-bit or 64-bit layouts. The register cache entry 
 * is then flagged as valid and dirty, forcing OpenOCD to synchronize this new address 
 * (0x80000000) directly into the hardware core execution register when the target resumes.
 * 
 * Purpose of this function is to abort the active QSPI instance, if the pc 
 * in the 0x90000000 range or 0xb0000000 range we cannot abort the same QSPI instance that is active.
 * If this case happens, we cannot erase or re-program the flash when there is already an application in it.
 * This is the reason PC has been changed to 0x80000000 and then abort to that QSPI instance
 */

int s2401_handle_change_pc(struct command_invocation *cmd) {
struct target *target = get_current_target(cmd->ctx);
    
    if (!target) {
        LOG_ERROR("No target selected or available");
        return ERROR_FAIL;
    }

    if (!target_was_examined(target)) {
        LOG_ERROR("Target not examined yet.");
        return ERROR_FAIL;
    }

    /* 1. Try to get RISC-V 'dpc' first; fall back to standard 'pc' if not found */
    struct reg *pc_reg = register_get_by_name(target->reg_cache, "dpc", 1);
    if (!pc_reg) {
        pc_reg = register_get_by_name(target->reg_cache, "pc", 1);
    }

    if (!pc_reg) {
        LOG_ERROR("Could not find PC or DPC register in the current target cache");
        return ERROR_FAIL;
    }

    /* 2. Clear register size check to avoid the assertion crash */
    if (pc_reg->size == 0) {
        LOG_ERROR("Register size is uninitialized");
        return ERROR_FAIL;
    }

    /* 3. Apply the address safely depending on bit-width */
    if (pc_reg->size <= 32) {
        buf_set_u32(pc_reg->value, 0, pc_reg->size, 0x80000000);
    } else {
        buf_set_u64(pc_reg->value, 0, pc_reg->size, 0x80000000ULL);
    }

    /* 4. Flush to OpenOCD cache */
    pc_reg->dirty = true;
    pc_reg->valid = true;

    LOG_DEBUG("Successfully forced execution vector target to 0x80000000");
    return ERROR_OK;
}

uint32_t s2401_fastReadQuad(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x6B;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 8;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = data_length;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}

/**
 * @fn uint32_t fastReadQuadIO(struct target *target,uint8_t qspinum,uint8_t *data,uint32_t address,uint8_t data_length)
 * 
 * @brief Used to write data into flash through all the four data lines and instruction through two address lines.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data is a pointer to array which contains data to written in flash.
 * @param address The parameter \a address specifies from which start address data to be read from flash through QSPI.
 * @param data_length The parameter \a data_length specifies how many bytes of data should be written to flash from array.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
uint32_t s2401_fastReadQuadIO(struct target *target,uint8_t qspinum,uint8_t *data,uint32_t address,uint8_t data_length){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0xEB;
    s2401_flash_msg.instruction_mode = CCR_IMODE_TWO_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 4;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_FOUR_LINE;
    s2401_flash_msg.alternate_byte = 0x20;
    s2401_flash_msg.length = data_length;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    
}
/**
 * @fn uint32_t fastReadSingle(struct target *target,uint8_t qspinum,uint8_t *data,uint32_t address,uint8_t data_length)
 * 
 * @brief Used to write data into flash through one data line and one instruction line.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data is a pointer to array which contains data read from flash.
 * @param address The parameter \a address specifies from which start address data should be read from flash.
 * @param data_length The parameter \a data_length specifies how many bytes of data should be read from flash.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
uint32_t s2401_fastReadSingle(struct target *target,uint8_t qspinum,uint8_t *data,uint32_t address,uint8_t data_length){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x03;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = data_length;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}

/**
 * @fn uint32_t inputpageQuad(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length)
 * 
 * @brief Used to write data to flash with one address line,one instruction line and four data lines.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data is a pointer to array which contains data to be read from flash.
 * @param address The parameter \a address specifies from which start address data should be read from flash.
 * @param data_length The parameter \a data_length specifies how many bytes of data should be read from flash.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
uint32_t s2401_inputpageQuad(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x32;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = data_length;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    uint8_t temp;
    while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return SUCCESS;
    
}

/**
 * @fn uint32_t inputpageSingle(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length)
 * 
 * @brief Used to write to flash through single line.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data is a pointer to array which contains data to be written to flash.
 * @param address The parameter \a address specifies from which start address data to be written to flash through QSPI.
 * @param data_length The parameter \a data_length specifies how many bytes of data should be written to flash from array.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
uint32_t s2401_inputpageSingle(struct target *target,uint8_t qspinum,uint8_t* data,uint32_t address,uint8_t data_length){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x02;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = data_length;
     s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    uint8_t temp;
    while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return SUCCESS;
}
/**
 * @fn void sector4KErase(struct target *target,uint8_t qspinum,uint32_t address)
 * 
 * @brief Used to erase QSPI flash for 4KB of space from mentioned starting address.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param address The parameter \a address specifies from which start address data should be erased from flash through QSPI.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_sector4KErase(struct target *target,uint8_t qspinum,uint32_t address){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x20;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
	s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	uint8_t temp;
    while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return SUCCESS;
    
}
/**
 * @fn void sector32KErase(struct target *target,uint8_t qspinum,uint32_t address)
 * 
 * @brief Used to erase QSPI flash for 32KB of space from mentioned starting address.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param address The parameter \a address specifies from which start address data should be erased from flash through QSPI.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_sector32KErase(struct target *target,uint8_t qspinum,uint32_t address){
    s2401_flash_msg.address = address;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x52;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
	s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);    
    uint8_t temp;
    while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return SUCCESS;
}
/**
 * @fn void chipErase(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to erase whole chip through QSPI.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_chipErase(struct target *target,uint8_t qspinum){
    //currently not working so used 32Kchip erase as patch
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0xC7;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    uint8_t temp;
        while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x01;
        if(temp != 0x01)
        break;
    }
    return SUCCESS;

}
/**
 * @fn void writeEnable(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to enable qspi write transactions.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeEnable(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0x06;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 0;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    uint8_t temp;
    while(1){
        temp = s2401_readStatusRegister1(target,qspinum);
        temp = temp & 0x02;
        if(temp == 0x02)
        break;
    }
    return SUCCESS;

}
/**
 * @fn void writeDisable(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to disable qspi write transactions.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeDisable(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0x04;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.length = 0;

    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;  
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    
}
/**
 * @fn void suspend(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to suspend current qspi transactions.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_suspend(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0x75;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}


/**
 * @fn void resume(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to resume suspended qspi transactions.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_resume(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0x7A;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint32_t power_down(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to enter power down mode.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_power_down(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0xB9;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint32_t release_power_down(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to exit power down mode.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_release_power_down(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_8_BIT;
    s2401_flash_msg.instruction = 0xAB;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint8_t readStatusRegister1(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to read value in Status register 1.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return Returns value in Status Register 1.
 */
uint8_t s2401_readStatusRegister1(struct target *target,uint8_t qspinum){// status reg 1
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x05;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = &data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	return data;
}
/**
 * @fn uint8_t readStatusRegister2(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to read value in Status register 2.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return Returns value in Status Register 2.
 */
uint8_t s2401_readStatusRegister2(struct target *target,uint8_t qspinum){// status reg 1
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x35;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = &data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	return data;
}
/**
 * @fn uint8_t readStatusRegister3(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to read value in Status register 3.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return Returns value in Status Register 3.
 */
uint8_t s2401_readStatusRegister3(struct target *target,uint8_t qspinum){// status reg 1
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x15;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = &data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	return data;
}
/**
 * @fn uint8_t readFlagStatusRegister(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to read value in Flag Status Register.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return Returns value in Flag Status Register.
 */
uint8_t s2401_readFlagStatusRegister(struct target *target,uint8_t qspinum){
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x70;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.data_buffer = &data;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
    return data;
}
/**
 * @fn void writeEnableStatusRegister(struct target *target,uint8_t qspinum)
 * 
 * @brief Used to enable write operation to status register.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeEnableStatusRegister(struct target *target,uint8_t qspinum){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.instruction = 0x50;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_NO_DATA;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 0;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn void writeStatusRegister1(struct target *target,uint8_t qspinum,uint8_t* statusData)
 * 
 * @brief Used to write value in Status Register 1.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param statusData The parameter \a statusData is value that has to be written to status register.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeStatusRegister1(struct target *target,uint8_t qspinum,uint8_t* statusData){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.instruction = 0x01;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_flash_msg.data_buffer = statusData;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn void writeStatusRegister2(struct target *target,uint8_t qspinum,uint8_t* statusData)
 * 
 * @brief Used to write value in Status Register 2.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param statusData The parameter \a statusData is value that has to be written to status register.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeStatusRegister2(struct target *target,uint8_t qspinum,uint8_t* statusData){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.instruction = 0x31;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_flash_msg.data_buffer = statusData;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn void writeStatusRegister3(struct target *target,uint8_t qspinum,uint8_t* statusData)
 * 
 * @brief Used to write value in Status Register 2.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param statusData The parameter \a statusData is value that has to be written to status register.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeStatusRegister3(struct target *target,uint8_t qspinum,uint8_t* statusData){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.instruction = 0x11;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_flash_msg.data_buffer = statusData;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint8_t readGlobalFreezeBit(struct target *target,uint8_t qspinum)
 *
 * @brief Used to read global freeze bit.
 *
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 *
 * @return 1 if global freeze bit is set otherwise 0.
 */
uint8_t s2401_readGlobalFreezeBit(struct target *target,uint8_t qspinum){
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0xA7;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_flash_msg.data_buffer = &data;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	return data;
}
/**
 * @fn uint32_t writeGlobalFreezeBit(struct target *target,uint8_t qspinum)
 *
 * @brief Used to set or clear global freeze bit.
 *
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 *
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeGlobalFreezeBit(struct target *target,uint8_t qspinum,uint8_t *data){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0xA6;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 1;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.data_buffer = data;
    s2401_flash_msg.length = 1;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint8_t readFlashSFDP(struct target *target,uint8_t qspinum,uint32_t address)
 * 
 * @brief Used to read value of Serial Flash Discoverable Parameter.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param address The parameter \a address specifies from which address we have to read data.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint8_t s2401_readFlashSFDP(struct target *target,uint8_t qspinum,uint32_t address){
    uint8_t data;
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.address = address;
    s2401_flash_msg.instruction = 0x5A;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 7;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 1;
    s2401_flash_msg.data_buffer = &data;
    s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
	return data;
}
/**
 * @fn uint32_t readNVCR(struct target *target,uint8_t qspinum, uint8_t* data)
 * 
 * @brief Used to read NVCR(Non volatile configuration register)
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data specifies pointer to location which has NVCR contents.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_readNVCR(struct target *target,uint8_t qspinum, uint8_t* data){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0xB5;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 2;
    s2401_flash_msg.data_buffer = data;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint32_t readJedecID(struct target *target,uint8_t qspinum, uint8_t *id)
 * 
 * @brief Used to write to NVCR(Non volatile configuration register)
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param id The parameter \a id specifies pointer which has pointer of variable which has JedecID.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_readJedecID(struct target *target,uint8_t qspinum, uint8_t *id){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0x9F;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_READ;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 3;
    s2401_flash_msg.data_buffer = id;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint32_t writeNVCR(struct target *target,uint8_t qspinum, uint8_t* data)
 * 
 * @brief Used to write to NVCR(Non volatile configuration register)
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param data The parameter \a data specifies pointer which has data to be written to NVCR.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_writeNVCR(struct target *target,uint8_t qspinum, uint8_t* data){
    s2401_flash_msg.address_mode = CCR_ADMODE_NIL;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.instruction = 0xB1;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_INDIRECT_WRITE;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 0;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 16;
    s2401_flash_msg.data_buffer = data;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}
/**
 * @fn uint32_t flash_xip_init(struct target *target,uint8_t qspinum, int flash_size)
 * 
 * @brief Used to change QSPI mode to Excute In Place mode,used to run code from flash memory.
 * 
 * @param qspinum The parameter \a qspinum is an unsigned integer that represents the QSPI instance number.
 * @param flash_size The parameter \a flash_size specifies flash size.
 * 
 * @return SUCCESS if operation is successful and ENODEV if invalid instance number.
 */
uint32_t s2401_flash_xip_init(struct target *target,uint8_t qspinum, int flash_size){
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.FMEM_SIZE = flash_size;
    s2401_flash_msg.instruction = 0x6B;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_FOUR_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_MMM;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 8;
    s2401_flash_msg.dummy_bit = 1;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 0;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}

uint32_t s2401_flash_single_xip_init(struct target *target,uint8_t qspinum, int flash_size){
    s2401_flash_msg.address_mode = CCR_ADMODE_SINGLE_LINE;
    s2401_flash_msg.address_size = CCR_ADSIZE_24_BIT;
    s2401_flash_msg.FMEM_SIZE = flash_size;
    s2401_flash_msg.instruction = 0x03;
    s2401_flash_msg.instruction_mode = CCR_IMODE_SINGLE_LINE;
    s2401_flash_msg.data_mode = CCR_DMODE_SINGLE_LINE;
    s2401_flash_msg.functional_mode = CCR_FMODE_MMM;
    s2401_flash_msg.dummy_mode = 0;
    s2401_flash_msg.dummy_cycles = 0;
    s2401_flash_msg.dummy_bit = 1;
    s2401_flash_msg.mm_mode = CCR_MM_MODE_XIP;
    s2401_flash_msg.alternate_byte_mode = CCR_ABMODE_NIL;
    s2401_flash_msg.length = 0;
    return s2401_QSPI_Transaction(target,qspinum,&s2401_flash_msg);
}

static bool _has_bin_extension(const char *path)
{
    const char *ext = strrchr(path, '.');
    return ext && strcasecmp(ext, ".bin") == 0;
}

int s2401_handle_flash_quad_write(struct command_invocation *cmd) {
    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: file path missing");
        command_print(CMD, "Usage: flash_quad_write <file.bin> <address> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    const char *filepath = CMD_ARGV[0];

    if (!_has_bin_extension(filepath)) {
        command_print(CMD, "Error: '%s' is not a .bin file", filepath);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }

    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: start address missing");
        command_print(CMD, "Usage: flash_quad_write <file.bin> <address> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t start_address;
   COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], start_address);

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 3) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    s2401_flash_msg.PRESCALER = prescaler;
    s2401_handle_change_pc(cmd);
    unsigned int qspi_number = 0;
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
    FILE *file = fopen(CMD_ARGV[0], "rb");

    command_print(CMD, "Start address: 0x%x\n", start_address);
    if((start_address>=0x90000000) &&(start_address<=0xAFFFFFFF)){
       qspi_number = 0;
    }else if((start_address>=0xB0000000) &&(start_address<=0xCFFFFFFF)){
       qspi_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
    uint32_t mask_address =start_address&~(0xF<<28);
    // command_print(CMD, "mask address: 0x%x\n", mask_address);
    // command_print(CMD, "QSPI number: 0x%x\n", qspi_number);
    // Variable to accumulate the total length of binary data for executable sections
    size_t executable_binary_length = 0;
    // Seek to the end to find the length of the file
    fseek(file, 0, SEEK_END);
    executable_binary_length  = ftell(file);
    rewind(file);  // Rewind to the beginning of the file

    // Print the length of the executable binary data (excluding the ELF header)
    command_print(CMD, "Length of executable binary data (excluding ELF header): %ld bytes\n", executable_binary_length);

    /*handling sector erase operation*/
    uint32_t progressed_length = 0;
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nWriting code to flash in progress:\n");	

    unsigned char buffer[CHUNK_SIZE];
    uint8_t bytesReadInChunk;
    size_t offset = 0x000;
    size_t remaining_bytes = executable_binary_length;
    uint8_t to_read;
    uint32_t erase_start_address = mask_address & ~(0xFFF);
    uint32_t erase_end_address = (mask_address+executable_binary_length) & ~(0xFFF);
    for(uint32_t s = erase_start_address;s<=erase_end_address;s+=0x1000){
    log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, "Erasing sector:%x\n",s);	
    s2401_writeEnable(target,qspi_number);/*Enable write operation*/
    s2401_sector4KErase(target,qspi_number,s);
    s2401_writeDisable(target,qspi_number);/*Enable write operation*/
    }
    while (remaining_bytes > 0) {
        to_read = (remaining_bytes>CHUNK_SIZE)?CHUNK_SIZE:remaining_bytes;
        bytesReadInChunk = fread(buffer, 1, to_read, file);
        s2401_writeEnable(target,qspi_number);/*Enable write operation*/

        s2401_inputpageQuad(target,qspi_number,buffer,mask_address+offset,bytesReadInChunk);/*To write data to flash*/
        //writeDisable(target,qspi_number);/*Enable write operation*/

        //log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, "\nWriting at offset:%lx\n",start_address+offset);
        //for(uint8_t i = 0;i<to_read;i++){
        //log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__,  "%x ",buffer[i]);
        //}
        progressed_length +=bytesReadInChunk;
        s2401_print_progress_bar(progressed_length, executable_binary_length);
        offset += bytesReadInChunk;
        remaining_bytes-=bytesReadInChunk;
    }
    // Close the file
    fclose(file);

    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCompleted writing!!");	
    command_print(CMD, "Completed writing");
    return ERROR_OK;
}



int s2401_handle_flash_write_length(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: file path missing");
        command_print(CMD, "Usage: flash_write_length <file.bin> <address> <no_of_lines> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    const char *filepath = CMD_ARGV[0];

    if (!_has_bin_extension(filepath)) {
        command_print(CMD, "Error: '%s' is not a .bin file", filepath);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }

    
    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: address missing");
        command_print(CMD, "Usage: flash_write_length <file.bin> <address> <no_of_lines> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    if (CMD_ARGC < 3) {
        command_print(CMD, "Error: no_of_lines missing");
        command_print(CMD, "Usage: flash_write_length <file.bin> <address> <no_of_lines> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 4) {
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

    s2401_flash_msg.PRESCALER = prescaler;

    s2401_handle_change_pc(cmd);
    unsigned int qspi_number = 0;
    //  uint32_t address = 0x30;
    //  uint8_t data[16];
    //  COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], qspi_number);
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
    FILE *file = fopen(CMD_ARGV[0], "rb");
    if (file == NULL) {
        // perror("Error opening file");
        command_print(CMD, "File doesnt exist");
        return -1;
    }
    uint8_t no_of_lines;
    uint32_t start_address;
    
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], no_of_lines);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], start_address);
    // Print the entry point address (start address) in hexadecimal
    command_print(CMD, "(start address): 0x%x\n", start_address);
    if((start_address>=0x90000000) &&(start_address<=0xAFFFFFFF)){
       qspi_number = 0;
    }else if((start_address>=0xB0000000) &&(start_address<=0xCFFFFFFF)){
       qspi_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
    uint32_t mask_address =start_address&~(0xF<<28);
    command_print(CMD, "mask address: 0x%x\n", mask_address);
    command_print(CMD, "QSPI number: 0x%x\n", qspi_number);
    // Variable to accumulate the total length of binary data for executable sections
    size_t executable_binary_length = 0;
    // Seek to the end to find the length of the file
    fseek(file, 0, SEEK_END);
    executable_binary_length  = ftell(file);
    rewind(file);  // Rewind to the beginning of the file
    
    uint64_t executable_binary_length_copy = (uint64_t)executable_binary_length;
    uint8_t *ptr;     
    ptr =(uint8_t*)&executable_binary_length_copy;
    // writeEnable(target,qspi_number);/*Enable write operation*/
    // sector4KErase(target,qspi_number,mask_address & ~(0xFFF));
    // writeDisable(target,qspi_number);/*Enable write operation*/
    s2401_writeEnable(target,qspi_number);/*Enable write operation*/
    if(no_of_lines == 4){
        s2401_inputpageQuad(target,qspi_number,ptr,mask_address,8);/*To write data to flash in four lines*/
    } else if( no_of_lines == 1) {
        s2401_inputpageSingle(target,qspi_number,ptr,mask_address,8);/*To write data to flash in single line*/
    } else {
        command_print(CMD, "Error: Invalid no of lines");
        return ERROR_OK;
    }
    s2401_writeDisable(target,qspi_number);/*Enable write operation*/
    // Print the length of the executable binary data (excluding the ELF header)
    command_print(CMD, "Length of executable binary data (excluding ELF header): 0x%lx bytes\n", executable_binary_length_copy);
    command_print(CMD, "Completed writing length at mask address :%x",mask_address);
    return ERROR_OK;
}


int s2401_handle_flash_write_data(struct command_invocation *cmd) {
/*
 * argv[1] = address
 * argv[2 ... n-1] = data
 */
    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: address missing");
        command_print(CMD, "Usage: flash_write_data <address> <no_of_lines> <data1>....<datan>");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: no_of_lines missing");
        command_print(CMD, "Usage: flash_write_data <address> <no_of_lines> <data1>....<datan>");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    if (CMD_ARGC < 3) {
        command_print(CMD, "Error: data missing");
        command_print(CMD, "Usage: flash_write_data <address> <no_of_lines> <data1>....<datan>");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 4) {
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

    s2401_flash_msg.PRESCALER = prescaler;

    s2401_handle_change_pc(cmd);
    uint32_t start_address,length=0;
    uint32_t total_length = (CMD_ARGC)-2;
    uint8_t qspi_number=0;
    uint8_t __attribute__((unused)) data[CMD_ARGC-2];
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], start_address);
    if((start_address>=0x90000000) &&(start_address<=0xAFFFFFFF)){
        qspi_number = 0;
    }else if((start_address>=0xB0000000) &&(start_address<=0xCFFFFFFF)){
        qspi_number = 1;
    }
    else{
    command_print(CMD, "Not in the expected address range");
    return ERROR_OK;
    }
    uint8_t no_of_lines;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], no_of_lines);
    // printf("Start address :%x",start_address);
    // log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "\nqspi_number :%x ,total_length :%x",qspi_number,total_length);
    // Read the ELF header
    for(uint32_t i = 0;i<total_length;i++){
        COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i+2], data[i]);
    }
    // for(uint8_t j = 0;j<total_length;j++){
    //       log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "%x ",data[j]);
    // }
    uint8_t *ptr = data;
    for(uint32_t address __attribute__((unused)) = start_address&~(0xF<<28),l=0,remaining_length=total_length;remaining_length;address+=length){
        length = (remaining_length>16)?16:remaining_length;
            s2401_writeEnable(target,qspi_number);/*Enable write operation*/
            if(no_of_lines == 4){
                s2401_inputpageQuad(target,qspi_number,ptr+l,address,length);/*To write data to flash in four lines*/
            } else if( no_of_lines == 1) {
                s2401_inputpageSingle(target,qspi_number,ptr+l,address,length);/*To write data to flash in single line*/
            } else {
                command_print(CMD, "Error: Invalid no of lines");
                return ERROR_OK;
            }
            s2401_writeDisable(target,qspi_number);/*Enable write operation*/
        //     printf("Every iteration data: %x\n",address);
        //     log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "Address : %x\n",address);
        //     for(uint8_t k = 0;k<length;k++){
        //     log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "%x ",(ptr+l)[k]);
        // }
        remaining_length-=length;
        l+=length;
    }
    return ERROR_OK;
}

int s2401_handle_sector_erase(struct command_invocation *cmd)
{

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: mode missing");
        command_print(CMD, "Usage: flash_sector_erase <mode> <address> <no_of_sectors> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: address missing");
        command_print(CMD, "Usage: flash_sector_erase <mode> <address> <no_of_sectors> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    if (CMD_ARGC < 3) {
        command_print(CMD, "Error: no_of_sectors missing");
        command_print(CMD, "Usage: flash_sector_erase <mode> <address> <no_of_sectors> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 4) {
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

    s2401_flash_msg.PRESCALER = prescaler;
    s2401_handle_change_pc(cmd);
    uint32_t start_address,no_of_sectors,mode,qspi_number=0;
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], mode);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], start_address);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], no_of_sectors);
    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Requested sectors are erased");
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Sector Erase in Progress!!");
    if((start_address>=0x90000000) &&(start_address<=0xAFFFFFFF)){
       qspi_number = 0;
    }else if((start_address>=0xB0000000) &&(start_address<=0xCFFFFFFF)){
       qspi_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
    uint32_t mask_value =(mode==4)?(~(0xFFF)):((mode==32)?~(0x7FFF):0);
    uint32_t increment=(mode==4)?(0x1000):((mode==32)?(0x8000):0);
    uint32_t mask_address =start_address&~(0xF<<28);
    uint32_t erase_start_address = mask_address & mask_value;
    for(uint32_t s = erase_start_address,i=0;i<no_of_sectors;s+=increment,i++){
        log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Erasing sector:%x\n",s);
        s2401_writeEnable(target,qspi_number);/*Enable write operation*/
        if(mode==4)
            s2401_sector4KErase(target,qspi_number,s);
        else if (mode==32)
            s2401_sector32KErase(target,qspi_number,s);
        else {
            command_print(CMD, "Error: Invalid mode");
            return ERROR_OK;
        }
        s2401_writeDisable(target,qspi_number);/*Enable write operation*/
    }
    return 0;
}

int s2401_handle_reset(struct command_invocation *cmd) {
    
    struct target *target = get_current_target(CMD_CTX);
    uint32_t wd_cntrl_reg = 0x40508;
    uint16_t wd_cntrl = 5; // Enable the wdtimer and sets soft reset bit in control reg
    target_write_u16(target, wd_cntrl_reg, wd_cntrl);
    return 0;
}


int s2401_handle_flash_erase(struct command_invocation *cmd)
{
    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: flash_erase <instance> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    s2401_flash_msg.PRESCALER = prescaler;

    s2401_handle_change_pc(cmd);
    uint8_t qspi_number;
// uint8_t sr;
    if (qspi_number > 2) {
        command_print(CMD, "Error: Invalid instance");
        return ERROR_OK;
    }
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);
    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Flash erase is invoked with qspi %x",qspi_number);
    command_print(CMD, "Wait Chip Erase in Progress!!");
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Wait Chip Erase in Progress!!");
    s2401_writeEnable(target,qspi_number);/*Enable write operation*/
    s2401_chipErase(target,qspi_number);/*Send chip erase command*/
    s2401_writeDisable(target,qspi_number);/*Disable write operation*/
    command_print(CMD, "Chip Erase Complete!"); 
    return ERROR_OK;
}



int s2401_handle_flash_single_xip(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: flash_xip_init <instance> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    s2401_flash_msg.PRESCALER = prescaler;
    s2401_handle_change_pc(cmd);
    uint8_t qspi_number;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);
    if (qspi_number > 2) {
        command_print(CMD, "Error: Invalid instance");
        return ERROR_OK;
    }

    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Flash xip is configured with qspi %x",qspi_number);
    s2401_flash_single_xip_init(target,qspi_number,27);
    return ERROR_OK;
}

int s2401_handle_flash_quad_xip(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: flash_xip_init <instance> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 2) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    s2401_flash_msg.PRESCALER = prescaler;
    s2401_handle_change_pc(cmd);
    uint8_t qspi_number;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);
    if (qspi_number > 2) {
        command_print(CMD, "Error: Invalid instance");
        return ERROR_OK;
    }

    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Flash xip is configured with qspi %x",qspi_number);
    s2401_flash_xip_init(target,qspi_number,27);
    return ERROR_OK;
}

int s2401_handle_flash_single_write(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: file path missing");
        command_print(CMD, "Usage: flash_single_write <file.bin> <address> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    const char *filepath = CMD_ARGV[0];

    if (!_has_bin_extension(filepath)) {
        command_print(CMD, "Error: '%s' is not a .bin file", filepath);
        return ERROR_COMMAND_ARGUMENT_INVALID;
    }

    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: start address missing");
        command_print(CMD, "Usage: flash_single_write <file.bin> <address> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t start_address;
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], start_address);

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 3) {
        COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], prescaler);
        if (prescaler < FLASH_PRESCALER_MIN || prescaler > FLASH_PRESCALER_MAX) {
            command_print(CMD, "Error: invalid prescaler 0x%" PRIx32
                          " (valid: 0x%x - 0x%x)",
                          prescaler, FLASH_PRESCALER_MIN, FLASH_PRESCALER_MAX);
            return ERROR_COMMAND_ARGUMENT_INVALID;
        }
    } else {
        command_print(CMD, "Prescaler not given, using default: %u", DEFAULT_FLASH_PRESCALER);
    }

    s2401_flash_msg.PRESCALER = prescaler;
    s2401_handle_change_pc(cmd);
    unsigned int qspi_number = 0;
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
    FILE *file = fopen(CMD_ARGV[0], "rb");

    command_print(CMD, "Start address: 0x%x\n", start_address);
    if((start_address>=0x90000000) &&(start_address<=0xAFFFFFFF)){
       qspi_number = 0;
    }else if((start_address>=0xB0000000) &&(start_address<=0xCFFFFFFF)){
       qspi_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
    uint32_t mask_address =start_address&~(0xF<<28);
    command_print(CMD, "mask address: 0x%x\n", mask_address);
    command_print(CMD, "QSPI number: 0x%x\n", qspi_number);
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "mask address: 0x%x\n", mask_address);	
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "QSPI number: 0x%x\n", qspi_number);	


    // Variable to accumulate the total length of binary data for executable sections
    size_t executable_binary_length = 0;
    // Seek to the end to find the length of the file
    fseek(file, 0, SEEK_END);
    executable_binary_length  = ftell(file);
    rewind(file);  // Rewind to the beginning of the file

    // Print the length of the executable binary data (excluding the ELF header)
    command_print(CMD, "Length of executable binary data (excluding ELF header): %ld bytes\n", executable_binary_length);

    /*handling sector erase operation*/
    uint32_t progressed_length = 0;
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nWriting code to flash in progress:\n");	

    unsigned char buffer[CHUNK_SIZE];
    uint8_t bytesReadInChunk;
    size_t offset = 0x000;
    size_t remaining_bytes = executable_binary_length;
    uint8_t to_read;
    uint32_t erase_start_address = mask_address & ~(0xFFF);
    uint32_t erase_end_address = (mask_address+executable_binary_length) & ~(0xFFF);
    for(uint32_t s = erase_start_address;s<=erase_end_address;s+=0x1000){
    log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, "Erasing sector:%x\n",s);	
    s2401_writeEnable(target,qspi_number);/*Enable write operation*/
    s2401_sector4KErase(target,qspi_number,s);
    s2401_writeDisable(target,qspi_number);/*Enable write operation*/
    }
    while (remaining_bytes > 0) {
        to_read = (remaining_bytes>CHUNK_SIZE)?CHUNK_SIZE:remaining_bytes;
        bytesReadInChunk = fread(buffer, 1, to_read, file);
        log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, "mask_address+offset: 0x%lx\n", mask_address+offset);

        // for(uint8_t i = 0;i<bytesReadInChunk;i++){
        //     log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "\n%x ",buffer[i]);
        // }
        s2401_writeEnable(target,qspi_number);/*Enable write operation*/
        s2401_inputpageSingle(target,qspi_number,buffer,mask_address+offset,bytesReadInChunk);/*To write data to flash*/
        //writeDisable(target,qspi_number);/*Enable write operation*/
        // log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "\nto_read = %d ",to_read);
        //log_printf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, "\nWriting at offset:%lx\n",start_address+offset);

        progressed_length +=bytesReadInChunk;
        s2401_print_progress_bar(progressed_length, executable_binary_length);
        offset += bytesReadInChunk;
        remaining_bytes-=bytesReadInChunk;
    }
    // Close the file
    fclose(file);

    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCompleted writing!!");	
    command_print(CMD, "Completed writing");
    return ERROR_OK;
}
 
int s2401_handle_flash_read_status_register(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: flash_read_status_register <instance> <register_no> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    
    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: register_no missing");
        command_print(CMD, "Usage: flash_read_status_register <instance> <register_no> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 3) {
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

    s2401_flash_msg.PRESCALER = prescaler;

    s2401_handle_change_pc(cmd);
    uint8_t qspi_number = 0;
    //  uint32_t address = 0x30;
    //  uint8_t data[16];
    //  COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], qspi_number);
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);

    uint8_t register_no;

    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);
    if (qspi_number > 2) {
        command_print(CMD, "Error: Invalid instance");
        return ERROR_OK;
    }
    
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], register_no);
    uint8_t status_data;


    if(register_no == 1) {
        status_data =  s2401_readStatusRegister1(target, qspi_number);
    } else if(register_no == 2) {
        status_data =  s2401_readStatusRegister2(target, qspi_number);
    } else if(register_no == 3) {
        status_data =  s2401_readStatusRegister3(target, qspi_number);
    } else {
        command_print(CMD, "Error: Invalid register no");
        return ERROR_OK;
    }

    command_print(CMD, "Status register%d = %d\n", register_no, status_data);

    return ERROR_OK;
}



int s2401_handle_flash_write_status_register(struct command_invocation *cmd) {

    if (CMD_ARGC < 1) {
        command_print(CMD, "Error: instance missing");
        command_print(CMD, "Usage: flash_read_status_register <instance> <register_no> <status_data> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    
    if (CMD_ARGC < 2) {
        command_print(CMD, "Error: register_no missing");
        command_print(CMD, "Usage: flash_read_status_register <instance> <register_no> <status_data> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    if (CMD_ARGC < 3) {
        command_print(CMD, "Error: status_data missing");
        command_print(CMD, "Usage: flash_read_status_register <instance> <register_no> <status_data> [prescaler]");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }

    uint32_t prescaler = DEFAULT_FLASH_PRESCALER;
    if (CMD_ARGC == 4) {
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

    s2401_flash_msg.PRESCALER = prescaler;

    s2401_handle_change_pc(cmd);
    uint8_t qspi_number = 0;
    //  uint32_t address = 0x30;
    //  uint8_t data[16];
    //  COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], qspi_number);
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);

    uint8_t register_no;

    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], qspi_number);
    if (qspi_number > 2) {
        command_print(CMD, "Error: Invalid instance");
        return ERROR_OK;
    }
    
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], register_no);
    uint8_t status_data;
    COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], status_data);

    s2401_writeEnable(target,qspi_number);/*Enable write operation*/

    if(register_no == 1) {
        s2401_writeStatusRegister1(target, qspi_number, &status_data);
    } else if(register_no == 2) {
        s2401_writeStatusRegister2(target, qspi_number, &status_data);
    } else if(register_no == 3) {
        s2401_writeStatusRegister3(target, qspi_number, &status_data);
    } else {
        s2401_writeDisable(target,qspi_number);
        command_print(CMD, "Error: Invalid no of lines");
        return ERROR_OK;
    }

    command_print(CMD, "Write to the status register is completed\n");

    return ERROR_OK;
}


FLASH_BANK_COMMAND_HANDLER (s2401_flash_bank_command)
{


	return ERROR_OK;
}


static int s2401_erase(struct flash_bank *bank, unsigned int first,
    unsigned int last)
{
    // Perform erase operation and return ERROR_OK.
    return ERROR_OK;
}

static int s2401_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
    // Perform protect operation and return ERROR_OK.
    return ERROR_OK;
}

static int s2401_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
    // Perform write operation and return ERROR_OK.
    return ERROR_OK;
}


static int s2401_probe(struct flash_bank *bank)
{
    // Perform probe operation and return ERROR_OK.
    return ERROR_OK;
}

static int s2401_auto_probe(struct flash_bank *bank)
{
    // Perform auto probe operation and return ERROR_OK.
    return ERROR_OK;
}


static int s2401_protect_check(struct flash_bank *bank)
{
    // Perform protect check operation and return ERROR_OK.
    return ERROR_OK;
}

static int get_s2401_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    // Retrieve s2401 information and return ERROR_OK.
    return ERROR_OK;
}
const struct flash_driver s2401_flash = {
	.name = "s2401",
	.commands = NULL,
	.flash_bank_command = s2401_flash_bank_command,
	.erase = s2401_erase,
	.protect = s2401_protect,
	.write = s2401_write,
	.read = default_flash_read,
	.probe = s2401_probe,
	.auto_probe = s2401_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = s2401_protect_check,
	.info = get_s2401_info,
	.free_driver_priv = default_flash_free_driver_priv,
};