
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include<elf.h>
#include <helper/log.h>
#include"v2500_xspi_flash.h"
#include"v2500_xspi_issi_flash.h"
#include "target/riscv/riscv.h"

#define CHUNK_SIZE 64
#define POW2_MINUS1(n)   ((1U << (n)) - 1U)

#define FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE                (&(FlashCommand){0x00000001, 0x00, 0x00, 0x000000B7,0x0})  // CCR=0x00000001, Dummy=0, WEL_WIP=0, OPCODE=0x000000B7,R/W=0x0
#define FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE                 (&(FlashCommand){0x00000001, 0x00, 0x00, 0x000000E9,0x0})  // CCR=0x00000001, Dummy=0, WEL_WIP=0, OPCODE=0x000000E9,R/W=0x0
#define FLASH_CMD_EXT_WRITE_ENABLE                             (&(FlashCommand){0x00000001, 0x00, 0x00, 0x00000006,0x0})  // CCR=0x00000001, Dummy=0, WEL_WIP=0, OPCODE=0x00000006,R/W=0x0
#define FLASH_CMD_EXT_WRITE_DISABLE                            (&(FlashCommand){0x00000001, 0x00, 0x00, 0x00000004,0x0})  // CCR=0x00000001, Dummy=0, WEL_WIP=0, OPCODE=0x00000004,R/W=0x0
#define FLASH_CMD_EXT_READ_STATUS_REGISTER                     (&(FlashCommand){0x00040001, 0x00, 0x00, 0x00000005,0x1})  // CCR=0x00040001, Dummy=0, WEL_WIP=0, OPCODE=0x00000005,R/W=0x1
#define FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER    (&(FlashCommand){0x00040041, 0x00, 0x01, 0x00000081,0x0})  // CCR=0x00040041, Dummy=0, WEL_WIP=1, OPCODE=0x00000081,R/W=0x0
#define FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER     (&(FlashCommand){0x00040041, 0x08, 0x00, 0x00000085,0x1})  // CCR=0x00040041, Dummy=8, WEL_WIP=0, OPCODE=0x00000085,R/W=0x1




#define FLASH_CMD_ODDR_WRITE_ENABLE                             (&(FlashCommand){0x0060021C, 0x00, 0x00, 0x00000606,0x0})  // CCR=0x0060021C, Dummy=0, WEL_WIP=0, OPCODE=0x00000606,R/W=0x0
#define FLASH_CMD_ODDR_WRITE_DISABLE                            (&(FlashCommand){0x0060021C, 0x00, 0x00, 0x00000404,0x0})  // CCR=0x0060021C, Dummy=0, WEL_WIP=0, OPCODE=0x00000404,R/W=0x0
#define FLASH_CMD_ODDR_READ_STATUS_REGISTER                     (&(FlashCommand){0x0070021C, 0x08, 0x00, 0x00000505,0x1})  // CCR=0x0070021C, Dummy=8, WEL_WIP=0, OPCODE=0x00000505,R/W=0x1
#define FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER    (&(FlashCommand){0x0070031C, 0x00, 0x01, 0x00008181,0x0})  // CCR=0x0070031C, Dummy=0, WEL_WIP=1, OPCODE=0x00008181,R/W=0x0
#define FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER     (&(FlashCommand){0x0070031C, 0x08, 0x00, 0x00008585,0x1})  // CCR=0x0070031C, Dummy=8, WEL_WIP=0, OPCODE=0x00008585,R/W=0x1

#define GET_WIP(cmd) (((cmd).wel_wip >> 1) & 0x1)
#define GET_WEL(cmd) (((cmd).wel_wip >> 0) & 0x1)

#define BYTES_REQUIRED(addr) ( \
    (((addr) & 0x0FFFFFFF) >> 24) ? 3 : /* upper 4 bytes → return 3 */ \
    (((addr) & 0x0FFFFFFF) >> 16) ? 2 : /* 3-byte range → return 2 */ \
    (((addr) & 0x0FFFFFFF) >>  8) ? 2 : /* 2-byte range → return 2 */ \
    2 )
#define WILL_CROSS_3RD_BYTE(addr, data_type, length) \
    ((((addr) & 0xFFFFFF) + ((data_type) * (length))) > 0xFFFFFF)
#define DDR_BITS_MASK   ((1U << 3) | (1U << 9) | (1U << 21))
#define CHECK_DDR(val)   (((val) & DDR_BITS_MASK) != DDR_BITS_MASK)

// CCR field extract macros
#define CCR_GET_DQSEN(val)    (((val) >> 22) & 0x1)
#define CCR_GET_DDTR(val)     (((val) >> 21) & 0x1)
#define CCR_GET_DMODE(val)    (((val) >> 18) & 0x7)
#define CCR_GET_ABSIZE(val)   (((val) >> 16) & 0x3)
#define CCR_GET_ABDTR(val)    (((val) >> 15) & 0x1)
#define CCR_GET_ABMODE(val)   (((val) >> 12) & 0x7)
#define CCR_GET_ADSIZE(val)   (((val) >> 10) & 0x3)
#define CCR_GET_ADDTR(val)    (((val) >> 9) & 0x1)
#define CCR_GET_ADMODE(val)   (((val) >> 6) & 0x7)
#define CCR_GET_ISIZE(val)    (((val) >> 4) & 0x3)
#define CCR_GET_IDTR(val)     (((val) >> 3) & 0x1)
#define CCR_GET_IMODE(val)    (((val) >> 0) & 0x7)


xspi_msg WEL_WIP_VCR ={.PRESCALER=11,.FTIE=1,.FMEM_SIZE=25,.CLK_MODE=0, .sioo=0, .TCEN=0, .TEIE=0, .TOIE=0, .TCIE=0, .SMIE=0, .APMS=0, .PMM=0, .ABDTR=0, .csht=0,  .DDTR=0,.alternate_byte_size=0,.alternate_byte_mode=0,.address_size=2,.address_mode=0,.alternate_byte=0,.abort=0, .mm_mode=0, .sshift=0, .dhqc=0, .status_mask=0, .status_match=0, .pir=0, .timeout=0, .rd_instr=0, .wr_instr=0, .rd_dcyc=0, .wr_dcyc=0, .dual_mem=0, .FTF=0, .hw_protect=0,.enable=1};
xspi_msg _VCR ={.PRESCALER=11,.FTIE=1,.FMEM_SIZE=25,.CLK_MODE=0, .sioo=0, .TCEN=0, .TEIE=0, .TOIE=0, .TCIE=0, .SMIE=0, .APMS=0, .PMM=0, .ABDTR=0, .csht=0,  .DDTR=0,.alternate_byte_size=0,.alternate_byte_mode=0,.address_size=2,.address_mode=0,.alternate_byte=0,.abort=0, .mm_mode=0, .sshift=0, .dhqc=0, .status_mask=0, .status_match=0, .pir=0, .timeout=0, .rd_instr=0, .wr_instr=0, .rd_dcyc=0, .wr_dcyc=0, .dual_mem=0, .FTF=0, .hw_protect=0,.enable=1};

uint8_t DDR=0;


inline void Set_WEL(struct target *target,uint8_t instance_number, FlashCommand *cmd) {
    uint8_t value[2];
    WEL_WIP_VCR.data_buffer=value;
    if(!(CHECK_DDR(cmd->ccr))){
        WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.IDTR=CCR_GET_IDTR((FLASH_CMD_ODDR_WRITE_ENABLE->ccr));
        WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.functional_mode=FLASH_CMD_ODDR_WRITE_ENABLE->rw;
        WEL_WIP_VCR.dummy_cycles=FLASH_CMD_ODDR_WRITE_ENABLE->dummy;
        WEL_WIP_VCR.instruction=FLASH_CMD_ODDR_WRITE_ENABLE->opcode; 
        WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.address=0x0;
        WEL_WIP_VCR.length=0x0;
        WEL_WIP_VCR.dsize=0x0;
        WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
        WEL_WIP_VCR.instruction_size=1;
        WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_ODDR_WRITE_ENABLE->ccr);
        XSPI_Transaction(target,instance_number,&WEL_WIP_VCR);
    }
    else{    
        WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_WRITE_ENABLE->rw;
        WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_WRITE_ENABLE->dummy;
        WEL_WIP_VCR.instruction=FLASH_CMD_EXT_WRITE_ENABLE->opcode; 
        WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        WEL_WIP_VCR.address=0x0;
        WEL_WIP_VCR.length=0x0;
        WEL_WIP_VCR.dsize=0x0;
        WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
         WEL_WIP_VCR.instruction_size=0;
        WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_WRITE_ENABLE->ccr);
        XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
    }

    while(1)
    {
        if(!(CHECK_DDR(cmd->ccr))){
            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_ODDR_READ_STATUS_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_ODDR_READ_STATUS_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_ODDR_READ_STATUS_REGISTER->opcode;
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr); 
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=2;
            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.address_size=3;
            WEL_WIP_VCR.instruction_size=1;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
        }
        else{
            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_READ_STATUS_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_READ_STATUS_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_EXT_READ_STATUS_REGISTER->opcode; 
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=1;
            WEL_WIP_VCR.address_size=2;
            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.instruction_size=0;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
        }
        if(value[0]&0x2){
            break; 
    }   
    }
}

inline void Check_WIP(struct target *target, uint8_t instance_number,FlashCommand *cmd) {
    uint8_t value[2];
    WEL_WIP_VCR.data_buffer=value;
    while(1)
    {
        if(!(CHECK_DDR(cmd->ccr))){
            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_ODDR_READ_STATUS_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_ODDR_READ_STATUS_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_ODDR_READ_STATUS_REGISTER->opcode; 
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=2;
            WEL_WIP_VCR.address_size=3;

            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.instruction_size=1;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_ODDR_READ_STATUS_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
        }
        else{
            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);

            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_READ_STATUS_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_READ_STATUS_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_EXT_READ_STATUS_REGISTER->opcode; 
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=1;
            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.address_size=2;

            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.instruction_size=0;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_READ_STATUS_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
        }
        if(value[0]&0x1)
            continue;
        else
        {
            break;
        }
    }
}

inline void Enter_four_byte_mode(struct target *target,uint8_t instance_number) {


    WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->rw;
    WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->dummy;
    WEL_WIP_VCR.instruction=FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->opcode; 
    WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.address=0x0;
    WEL_WIP_VCR.length=0x0;
    WEL_WIP_VCR.dsize=0;
    WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
    WEL_WIP_VCR.instruction_size=1;
    WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_ENTER_4_BYTE_ADDRESS_MODE->ccr);    
    XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
}

inline void Exit_four_byte_mode(struct target *target,uint8_t instance_number) {
    
    WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->rw;
    WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->dummy;
    WEL_WIP_VCR.instruction=FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->opcode; 
    WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);
    WEL_WIP_VCR.address=0x0;
    WEL_WIP_VCR.length=0x0;
    WEL_WIP_VCR.dsize=0;
    WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
    WEL_WIP_VCR.instruction_size=1;
    WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_EXIT_4_BYTE_ADDRESS_MODE->ccr);    
    XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
}


uint8_t Write_VCR(struct target *target,uint8_t instance_number, uint8_t ddr_mode)
{
    uint8_t write_value[2];
    WEL_WIP_VCR.data_buffer=write_value;

    if(DDR){
        WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.IDTR=CCR_GET_IDTR((FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr));
        WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.functional_mode=FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->rw;
        WEL_WIP_VCR.dummy_cycles=FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->dummy;
        WEL_WIP_VCR.instruction=FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->opcode; 
        WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.address=0x0;
        WEL_WIP_VCR.length=0x2;
        WEL_WIP_VCR.dsize=0x0;
        WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
        WEL_WIP_VCR.instruction_size=1;
        write_value[0] = 0xFF;
        WEL_WIP_VCR.address_size=3;
        WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
    }
    else{    
        WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->rw;
        WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->dummy;
        WEL_WIP_VCR.instruction=FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->opcode; 
        WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        WEL_WIP_VCR.address=0x0;
        WEL_WIP_VCR.length=0x1;
        WEL_WIP_VCR.dsize=0x0;
        WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
        WEL_WIP_VCR.instruction_size=0;
        write_value[0] = 0xE7; 
        WEL_WIP_VCR.address_size=2;
        WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER->ccr);
        XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);

    }
    uint8_t read_value[2];
    WEL_WIP_VCR.data_buffer=read_value;

    while(1)
    {
        if(!(DDR)){
            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->opcode;
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr); 
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=2;
            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.address_size=3;
            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.instruction_size=1;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_ODDR_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
            uint8_t check = (read_value[0]&0xE7);
            if(check){
                break; 
            }
        }
        else{

            WEL_WIP_VCR.DDTR=CCR_GET_DDTR(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.IDTR=CCR_GET_IDTR(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.ADDTR=CCR_GET_ADDTR(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.dqse=CCR_GET_DQSEN(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.functional_mode=FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->rw;
            WEL_WIP_VCR.dummy_cycles=FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->dummy;
            WEL_WIP_VCR.instruction=FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->opcode; 
            WEL_WIP_VCR.data_mode=CCR_GET_DMODE(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.address_mode=CCR_GET_ADMODE(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            WEL_WIP_VCR.address=0x0;
            WEL_WIP_VCR.length=1;
            WEL_WIP_VCR.address_size=2;
            WEL_WIP_VCR.dsize=0;
            WEL_WIP_VCR.fthresh= POW2_MINUS1(0);
            WEL_WIP_VCR.instruction_size=0;
            WEL_WIP_VCR.instruction_mode=CCR_GET_IMODE(FLASH_CMD_EXT_READ_VOLATILE_CONFIGURATION_REGISTER->ccr);
            XSPI_Transaction(target, instance_number,&WEL_WIP_VCR);
            if(read_value[0]&0xFF){
                break; 
        } 
        }
     
    } 
    return 0;  
}


uint8_t XSPI_Flash_Transaction(struct target *target,FlashTransaction* flash_transaction) {   
    uint8_t exit = 0;
    xspi_msg msg = {.PRESCALER=11,.CLK_MODE=0,.sioo=0,.FTIE=1,.TCEN=0,.TEIE=0,.TOIE=0,.TCIE=0,\
        .SMIE=0,.APMS=0,.PMM=0,.IDTR=CCR_GET_IDTR(flash_transaction->cmd->ccr),\
        .ADDTR=CCR_GET_ADDTR(flash_transaction->cmd->ccr),.ABDTR=0,\
        .dqse=CCR_GET_DQSEN(flash_transaction->cmd->ccr),.csht=0,\
        .dual_mem=0,.FMEM_SIZE=25,.functional_mode=flash_transaction->cmd->rw,.abort=0,.data_mode=CCR_GET_DMODE(flash_transaction->cmd->ccr),\
        .mm_mode=0,.sshift=0,.dhqc=0,.dummy_cycles=flash_transaction->cmd->dummy,\
        .instruction=flash_transaction->cmd->opcode, .address=flash_transaction->address,\
        .status_mask=0, .status_match=0,.pir=0,.length=flash_transaction->data_length,\
        .data_buffer=flash_transaction->data_buffer,.dsize=flash_transaction->data_size,\
        .timeout=0,.rd_instr = 0, .wr_instr = 0, .rd_dcyc = 0, .wr_dcyc = 0, \
        .DDTR=    CCR_GET_DDTR(flash_transaction->cmd->ccr),.alternate_byte_size=0,.alternate_byte_mode=0,\
        .address_size=CCR_GET_ADSIZE(flash_transaction->cmd->ccr),.address_mode=CCR_GET_ADMODE(flash_transaction->cmd->ccr),\
        .instruction_size=0,.instruction_mode=CCR_GET_IMODE(flash_transaction->cmd->ccr),.alternate_byte=0,\
        .FTF = 0, .hw_protect = 0, .fthresh= POW2_MINUS1(flash_transaction->data_size), .enable=1};

    if(!(CHECK_DDR(flash_transaction->cmd->ccr)))
    {
        msg.instruction_size=1;
    }

    if(WILL_CROSS_3RD_BYTE(msg.address,msg.dsize,msg.length))
    {  
        return -1;  //ERROR 
    }

    if((CHECK_DDR(flash_transaction->cmd->ccr)==0)&(DDR==0))
    {
        Set_WEL(target,flash_transaction->instance_number,FLASH_CMD_EXT_WRITE_VOLATILE_CONFIGURATION_REGISTER);
        Write_VCR(target,flash_transaction->instance_number,0xE7);
        DDR=1;


    }
    else if((CHECK_DDR(flash_transaction->cmd->ccr)==1)&(DDR==1))
    {   
        Set_WEL(target,flash_transaction->instance_number,FLASH_CMD_ODDR_WRITE_VOLATILE_CONFIGURATION_REGISTER);
        Write_VCR(target,flash_transaction->instance_number,0xFF);
        DDR=0; 

    }

    if(GET_WEL(*(flash_transaction->cmd)))
    {
        Set_WEL(target,flash_transaction->instance_number,(flash_transaction->cmd)); 
    }

    if(msg.address_mode == 4)
         msg.address_size=3;
    else
        msg.address_size=2;


    if(((msg.address_size) == 3) & ((CHECK_DDR(flash_transaction->cmd->ccr))))
    {
        Enter_four_byte_mode(target,flash_transaction->instance_number);
        exit=1;
    }

    XSPI_Transaction(target, flash_transaction->instance_number,&msg);
    
    if (GET_WIP(*(flash_transaction->cmd)) )
    {
        Check_WIP(target, flash_transaction->instance_number,(flash_transaction->cmd));
        
    }
    if(exit)
    {
        Exit_four_byte_mode(target, flash_transaction->instance_number);
    }

    return 0;

}
void v2500_print_progress_bar(int progress, int total) {
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
 * @brief v2500_handle_change_pc handler to force the target's Program Counter (PC) to 0x80000000.
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

int v2500_handle_change_pc(struct command_invocation *cmd) {
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

int v2500_handle_flash_write(struct command_invocation *cmd)
{
    v2500_handle_change_pc(cmd);

    FlashTransaction flash_transaction;
/*
 * argv[1] = xSPI number
 * argv[2] = filename
 * argv[3] = size if code.bin is fed
 */
// flash_transaction.instance_number = 0;
//  uint32_t address = 0x30;
//  uint8_t data[16];
//  COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], flash_transaction.instance_number);
struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
FILE *file = fopen(CMD_ARGV[0], "rb");
if (file == NULL) {
    // perror("Error opening file");
    command_print(CMD, "File ");
    return -1;
}
uint32_t start_address;
Elf64_Ehdr elf_header;
// command_print(CMD, "Argc:%d\n", CMD_ARGC);
if(CMD_ARGC == 1){
// Read the ELF header

size_t bytesRead = fread(&elf_header, 1, sizeof(Elf64_Ehdr), file);
if (bytesRead != sizeof(Elf64_Ehdr)) {
    fclose(file);
    return -1;
}

// Check if the file is a valid ELF file by checking the magic number
if (elf_header.e_ident[EI_MAG0] != ELFMAG0 || 
    elf_header.e_ident[EI_MAG1] != ELFMAG1 || 
    elf_header.e_ident[EI_MAG2] != ELFMAG2 || 
    elf_header.e_ident[EI_MAG3] != ELFMAG3) {
    // fprintf(stderr, "This is not a valid ELF file\n");
    fclose(file);
    return -1;
}
   start_address = elf_header.e_entry;
}
else if(CMD_ARGC == 2){
   COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], start_address);
}
   // Print the entry point address (start address) in hexadecimal
command_print(CMD, "Start address: 0x%x\n", start_address);
if ((start_address & 0xF0000000) == 0xB0000000){
   flash_transaction.instance_number = 0;
}else if ((start_address & 0xF0000000) == 0xD0000000){
   flash_transaction.instance_number = 1;
}
else{
    command_print(CMD, "Not in the expected address range");
    return ERROR_OK;
}

uint32_t mask_address =start_address&~(0xF<<28);
// command_print(CMD, "mask address: 0x%x\n", mask_address);
// command_print(CMD, "xSPI number: 0x%x\n", flash_transaction.instance_number);
// Variable to accumulate the total length of binary data for executable sections
size_t executable_binary_length = 0;
Elf64_Phdr phdr;
if(CMD_ARGC==1){
// Get the program header table offset and number of entries
fseek(file, elf_header.e_phoff, SEEK_SET);


for (int l = 0; l < elf_header.e_phnum; l++) {
    // Read the program header
    uint32_t val = fread(&phdr, sizeof(Elf64_Phdr), 1, file);
    if (val != 1) {
        fclose(file);
        return -1;  // Error reading program header
    }
    // Check if the current program header is of type PT_LOAD (executable segment)
    if (phdr.p_type == PT_LOAD) {
       executable_binary_length+=phdr.p_filesz;
    }
}
}
else if(CMD_ARGC==2){
       // Seek to the end to find the length of the file
   fseek(file, 0, SEEK_END);
   executable_binary_length  = ftell(file);
   rewind(file);  // Rewind to the beginning of the file
}
// Print the length of the executable binary data (excluding the ELF header)
printf("Length of executable binary data (excluding ELF header): %ld bytes\n", executable_binary_length);

/*handling sector erase operation*/
uint32_t progressed_length = 0;
log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nWriting code to flash in progress:\n");	
if(CMD_ARGC==1){
   
// Now read and process the program headers for executable content
FILE *fileheader = fopen(CMD_ARGV[0], "rb");
fseek(fileheader, elf_header.e_phoff, SEEK_SET);
for (int l = 0; l < elf_header.e_phnum; l++) {
   log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "PROGRAM HEADER:%x\n",l);	
    // Read the program header
    uint32_t val = fread(&phdr, sizeof(Elf64_Phdr), 1, fileheader);
    if (val != 1) {
        fclose(file);
        return -1;  // Error reading program header
    }
    // Check if the current program header is of type PT_LOAD (executable segment)
    if (phdr.p_type == PT_LOAD) {
        log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Segment start virtual address: 0x%lx\n",phdr.p_paddr);
        fseek(file, phdr.p_offset, SEEK_SET); // Move to the segment's start
            unsigned char buffer[CHUNK_SIZE];
            uint8_t bytesReadInChunk;
            size_t remaining_bytes = phdr.p_filesz;
            log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nSection length:%lx\n",remaining_bytes);
            uint8_t to_read;
            mask_address=phdr.p_paddr&~(0xF<<28);
            uint32_t erase_start_address = mask_address & ~(0xFFF);
            uint32_t erase_end_address = (mask_address+remaining_bytes) & ~(0xFFF);
            
            flash_transaction.cmd = FLASH_CMD_EXT_4KB_SUBSECTOR_ERASE;
            for(uint32_t s = erase_start_address;s<=erase_end_address;s+=0x1000){

               flash_transaction.address=s;
               XSPI_Flash_Transaction(target,&flash_transaction);

            }
        

            size_t offset = 0x000;
            
            while (remaining_bytes > 0) {
               to_read = (remaining_bytes>CHUNK_SIZE)?CHUNK_SIZE:remaining_bytes;
               bytesReadInChunk = fread(buffer, 1, to_read, file);
               for(uint8_t i = 0;i<bytesReadInChunk;i++){
                   log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "%x ",buffer[i]);
               }
               if(((mask_address+offset)&(~(0xFF))) == (((mask_address+offset)+bytesReadInChunk-1)&(~(0xFF)))){//check if start address and end address in same sector
                flash_transaction.cmd = FLASH_CMD_EXT_PAGE_PROGRAM;
                flash_transaction.address = mask_address+offset;
                flash_transaction.data_length =bytesReadInChunk ;
                flash_transaction.data_buffer=buffer;
                flash_transaction.data_size = 1 ;
                XSPI_Flash_Transaction(target,&flash_transaction);
            }
            else
            {
                log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCrossing sector");
                uint32_t part1_address,part2_address;
                uint8_t part1_length,part2_length;
                part1_address = (mask_address+offset);
                part2_address = ((mask_address+offset)+bytesReadInChunk-1)&(~(0xFF));
                part1_length = (part2_address-(mask_address+offset));
                part2_length = (mask_address+offset)+bytesReadInChunk-part2_address;
                log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCrossing sector part1 addr:%x",part1_address);
                log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCrossing sector part2 addr:%x",part2_address);
                
                flash_transaction.cmd = FLASH_CMD_EXT_PAGE_PROGRAM;
                flash_transaction.address = part1_address;
                flash_transaction.data_length =part1_length ;
                flash_transaction.data_size = 1 ;
                flash_transaction.data_buffer=buffer;
                XSPI_Flash_Transaction(target,&flash_transaction);
                flash_transaction.data_length =part2_length ;
                flash_transaction.address = part2_address;
                flash_transaction.data_buffer=buffer+part1_length;
                XSPI_Flash_Transaction(target,&flash_transaction);
            }
                progressed_length +=bytesReadInChunk;
                v2500_print_progress_bar(progressed_length, executable_binary_length);
                offset += bytesReadInChunk;
                remaining_bytes-=bytesReadInChunk;
        }
    }
}
fclose(file);


}else if(CMD_ARGC==2){
   uint8_t buffer[CHUNK_SIZE];
   uint8_t bytesReadInChunk;
   size_t offset = 0x000;
   size_t remaining_bytes = executable_binary_length;
   uint8_t to_read;
   uint32_t erase_start_address = mask_address & ~(0xFFF);
   uint32_t erase_end_address = (mask_address+executable_binary_length) & ~(0xFFF);
   flash_transaction.cmd = FLASH_CMD_ODDR_4KB_SUBSECTOR_ERASE;
   flash_transaction.data_length = 0;

   for(uint32_t s = erase_start_address;s<=erase_end_address;s+=0x1000) {
        
        log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\n erasing addr:%x",s);
        flash_transaction.data_length = 0;
        flash_transaction.address=s;
        
        XSPI_Flash_Transaction(target,&flash_transaction);

   }
   while (remaining_bytes > 0) {
      to_read = (remaining_bytes>CHUNK_SIZE)?CHUNK_SIZE:remaining_bytes;
      bytesReadInChunk = fread(buffer, 1, to_read, file);
      flash_transaction.cmd = FLASH_CMD_ODDR_PAGE_PROGRAM;
      flash_transaction.address = mask_address+offset;
      flash_transaction.data_length =bytesReadInChunk ;
      flash_transaction.data_buffer=buffer;
      XSPI_Flash_Transaction(target,&flash_transaction);
      progressed_length +=bytesReadInChunk;
      v2500_print_progress_bar(progressed_length, executable_binary_length);
      offset += bytesReadInChunk;
      remaining_bytes-=bytesReadInChunk;
}
// Close the file
fclose(file);
}
log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "\nCompleted writing!!");	
command_print(CMD, "Completed writing");
return ERROR_OK;
}

int v2500_handle_flash_erase(struct command_invocation *cmd) {
    /*
     * argv[1] = xSPI number
     * 
     */
    v2500_handle_change_pc(cmd);

    FlashTransaction flash_transaction;
    unsigned int xspi_number;
    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], xspi_number);
    if(xspi_number>1){
       return ERROR_OK; 
    }
    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Flash erase is invoked with xSPI %x",xspi_number);
    command_print(CMD, "Wait Chip Erase in Progress!!");
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Wait Chip Erase in Progress!!");
    flash_transaction.cmd=FLASH_CMD_ODDR_CHIP_ERASE;
    flash_transaction.instance_number=xspi_number;
    flash_transaction.address=0;
    flash_transaction.data_length=0;
    XSPI_Flash_Transaction(target,&flash_transaction);
    command_print(CMD, "Chip Erase Complete!"); 
    return ERROR_OK;
}

int v2500_handle_flash_xip(struct command_invocation *cmd)
{
/*
 * argv[1] = xSPI number
 * 
 */
    v2500_handle_change_pc(cmd);

    FlashTransaction flash_transaction;
    unsigned int xspi_number;
    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], xspi_number);
    if(xspi_number>1){
       return ERROR_OK; 
    }
    flash_transaction.instance_number=xspi_number;
    flash_transaction.cmd=FLASH_CMD_ODDR_XIP_FAST_READ;
    flash_transaction.data_length=0;
    flash_transaction.address=0;
    struct target *target = get_current_target(CMD_CTX);
    command_print(CMD, "Flash xip is configured with xSPI %x",xspi_number);
    XSPI_Flash_Transaction(target,&flash_transaction);
    return ERROR_OK;
}

int v2500_handle_reset(struct command_invocation *cmd)
{
    // uint32_t val;
    struct target *target = get_current_target(cmd->ctx);
    struct riscv_info *r = target->arch_info;

    if (!r || !r->dmi_write) {
        command_print(cmd, "Error: RISC-V DMI interface not initialized.");
        return ERROR_FAIL;
    }

    // Step 1: Assert NDM Reset (Keep dmactive set)
    // We don't necessarily need to read first if we want a clean state
    uint32_t assert_reset = (1 << 1) | (1 << 0); // ndmreset=1, dmactive=1
    if (r->dmi_write(target, 0x10, assert_reset) != ERROR_OK) {
        command_print(cmd, "Error: Failed to assert ndmreset.");
        return ERROR_FAIL;
    }

    // // Small delay to allow reset logic to propagate
    usleep(1000); 

    // Step 2: De-assert NDM Reset (Release the cores)
    // If you don't do this, the SoC might stay in reset!
    uint32_t release_reset = (0 << 1) | (1 << 0); // ndmreset=0, dmactive=1
    if (r->dmi_write(target, 0x10, release_reset) != ERROR_OK) {
        command_print(cmd, "Error: Failed to release ndmreset.");
        return ERROR_FAIL;
    }

    command_print(cmd, "NDM Reset pulsed successfully. System booting...");
    return ERROR_OK;
}

int v2500_handle_sector_erase(struct command_invocation *cmd)
{
/*
 * argv[1] = QSPI number
 * 
 */
v2500_handle_change_pc(cmd);
unsigned int start_address,no_of_sectors,mode,xspi_number=0;
COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], mode);
COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], start_address);
COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], no_of_sectors);
struct target *target = get_current_target(CMD_CTX);
FlashTransaction flash_transaction;
command_print(CMD, "Requested sectors are erased");
log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Sector Erase in Progress!!");

if ((start_address & 0xF0000000) == 0xB0000000){
   flash_transaction.instance_number = 0;
}else if ((start_address & 0xF0000000) == 0xD0000000){
   flash_transaction.instance_number = 1;
}
else{
    command_print(CMD, "Not in the expected address range");
    return ERROR_OK;
}

uint32_t mask_value =(mode==4)?(~(0xFFF)):((mode==32)?~(0x7FFF):0);
uint32_t increment=(mode==4)?(0x1000):((mode==32)?(0x8000):0);
uint32_t mask_address =start_address&~(0xF<<28);
uint32_t erase_start_address = mask_address & mask_value;
flash_transaction.instance_number=xspi_number;
flash_transaction.address=0;
for(uint32_t s = erase_start_address,i=0;i<no_of_sectors;s+=increment,i++){
    log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, "Erasing sector:%x\n",s);
    if(mode==4){
        flash_transaction.cmd = FLASH_CMD_ODDR_4KB_SUBSECTOR_ERASE;
        flash_transaction.address=s;
        XSPI_Flash_Transaction(target,&flash_transaction);
    }
    else if (mode==32){
        flash_transaction.cmd = FLASH_CMD_ODDR_32KB_SUBSECTOR_ERASE;
        flash_transaction.address=s;
        XSPI_Flash_Transaction(target,&flash_transaction);
    }
}
return 0;
}

int v2500_handle_flash_write_data(struct command_invocation *cmd)
{
/*
 * argv[1] = address
 * argv[2 ... n-1] = data
 */
    v2500_handle_change_pc(cmd);
    uint32_t start_address,length=0;
    uint32_t total_length = (CMD_ARGC)-1;
    FlashTransaction flash_transaction;    
    uint8_t __attribute__((unused)) data[CMD_ARGC-1];
    struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
    COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], start_address);
    if ((start_address & 0xF0000000) == 0xB0000000){
       flash_transaction.instance_number = 0;
    }else if ((start_address & 0xF0000000) == 0xD0000000){
       flash_transaction.instance_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
    // printf("Start address :%x",start_address);
    // log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "\nxspi_number :%x ,total_length :%x",xspi_number,total_length);
    // Read the ELF header
    for(uint32_t i = 0;i<total_length;i++){
        COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i+1], data[i]);
    }
    // for(uint8_t j = 0;j<total_length;j++){
    //       log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__,  "%x ",data[j]);
    // }
    uint8_t *ptr = data;
    for(uint32_t address __attribute__((unused)) = start_address&~(0xF<<28),l=0,remaining_length=total_length;remaining_length;address+=length){
        length = (remaining_length>16)?16:remaining_length;

        flash_transaction.cmd = FLASH_CMD_ODDR_PAGE_PROGRAM;
        flash_transaction.address = address;
        flash_transaction.data_length =length ;
        flash_transaction.data_buffer=ptr+l;
        XSPI_Flash_Transaction(target,&flash_transaction);

        remaining_length-=length;
        l+=length;
    }
return ERROR_OK;
}

int v2500_handle_flash_write_length(struct command_invocation *cmd)
{
/*
 * argv[1] = QSPI number
 * argv[2] = filename
 * argv[3] = size if code.bin is fed
 */
v2500_handle_change_pc(cmd);
uint8_t flag=0;
//  uint32_t address = 0x30;
//  uint8_t data[16];
//  COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], xspi_number);
struct target *target __attribute__((unused)) = get_current_target(CMD_CTX);
const char *ext = strrchr(CMD_ARGV[0], '.');
    // Check if an extension exists
    if (ext != NULL) {
        // Compare the extension with the desired formats
        if (strcmp(ext, ".bin") == 0) {
            flag = 2;
        } else if (strcmp(ext, ".elf") == 0 || strcmp(ext, ".shakti") == 0) {
            flag = 1;
        } else {
            command_print(CMD,"Invalid file format\n");
        }
    } else {
        command_print(CMD,"No extension found\n");
    }

FILE *file = fopen(CMD_ARGV[0], "rb");
if (file == NULL) {
    // perror("Error opening file");
    command_print(CMD, "File doesnt exist");
    return -1;
}
Elf64_Ehdr elf_header;
if(flag==1){
    size_t bytesRead = fread(&elf_header, 1, sizeof(Elf64_Ehdr), file);
if (bytesRead != sizeof(Elf64_Ehdr)) {
    fclose(file);
    return -1;
}
}


uint32_t start_address;
FlashTransaction flash_transaction;    


// Read the ELF header
COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], start_address);
// Print the entry point address (start address) in hexadecimal
command_print(CMD, "(start address): 0x%x\n", start_address);
    if ((start_address & 0xF0000000) == 0xB0000000){
       flash_transaction.instance_number = 0;
    }else if ((start_address & 0xF0000000) == 0xD0000000){
       flash_transaction.instance_number = 1;
    }
    else{
        command_print(CMD, "Not in the expected address range");
        return ERROR_OK;
    }
uint32_t mask_address =start_address&~(0xF<<28);
command_print(CMD, "mask address: 0x%x\n", mask_address);
command_print(CMD, "xSPI number: 0x%x\n", flash_transaction.instance_number);
// Variable to accumulate the total length of binary data for executable sections
size_t executable_binary_length = 0;
Elf64_Phdr phdr;
if(flag==1){
// Get the program header table offset and number of entries
fseek(file, elf_header.e_phoff, SEEK_SET);


for (int l = 0; l < elf_header.e_phnum; l++) {
    // Read the program header
    uint32_t val = fread(&phdr, sizeof(Elf64_Phdr), 1, file);
    if (val != 1) {
        fclose(file);
        return -1;  // Error reading program header
    }
    // Check if the current program header is of type PT_LOAD (executable segment)
    if (phdr.p_type == PT_LOAD) {
       executable_binary_length+=phdr.p_filesz;
    }
}
}
else if(flag==2){
       // Seek to the end to find the length of the file
   fseek(file, 0, SEEK_END);
   executable_binary_length  = ftell(file);
   rewind(file);  // Rewind to the beginning of the file
}
uint64_t executable_binary_length_copy = (uint64_t)executable_binary_length;
uint8_t *ptr;     
ptr =(uint8_t*)&executable_binary_length_copy;
// writeEnable(target,flash_transaction.instance_number);/*Enable write operation*/
// sector4KErase(target,flash_transaction.instance_number,mask_address & ~(0xFFF));
// writeDisable(target,flash_transaction.instance_number);/*Enable write operation*/
flash_transaction.cmd = FLASH_CMD_ODDR_PAGE_PROGRAM;
flash_transaction.address = mask_address;
flash_transaction.data_length = 8 ;
flash_transaction.data_buffer=ptr;
XSPI_Flash_Transaction(target,&flash_transaction);
// Print the length of the executable binary data (excluding the ELF header)
command_print(CMD, "Length of executable binary data (excluding ELF header): 0x%lx bytes\n", executable_binary_length_copy);
command_print(CMD, "Completed writing length at mask address :%x",mask_address);
return ERROR_OK;
}
