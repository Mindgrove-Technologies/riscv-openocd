#ifndef XSPI_FLASH_H
#define XSPI_FLASH_H
#include "target/riscv/mindgrove/v2500_xspi.h"
#include "stdint.h"


typedef struct {
    uint32_t ccr;
    uint8_t  dummy;
    uint8_t  wel_wip;
    uint32_t  opcode;
    uint8_t rw;
} FlashCommand;

typedef struct {
    uint8_t instance_number;
    uint32_t address;
    uint32_t data_length;   // Number of data to be retrieved
    uint8_t  data_size;
    void *data_buffer;  // Number of bytes to be sent or received from the external device
    FlashCommand *cmd;
}FlashTransaction;

uint8_t XSPI_Flash_Transaction(struct target *target,FlashTransaction* flash_transaction) ;

void Set_WEL(struct target *target, uint8_t instance_number, FlashCommand *cmd) ;
void Check_WIP(struct target *target, uint8_t instance_number, FlashCommand *cmd);
void Enter_four_byte_mode(struct target *target, uint8_t instance_number);
void Exit_four_byte_mode(struct target *target, uint8_t instance_number);
#endif