#ifndef XSPI_H
#define XSPI_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

 #include <helper/time_support.h>
#include<target/target.h>

#define XSPI_MAX 2
#define WRITE 0
#define READ 1

#define FIFO_FULL 32
#define FIFO_EMPTY 0

/*Macros for Instruction MODE*/
#define CCR_IMODE_NIL             0x0
#define CCR_IMODE_SINGLE_LINE     0x1
#define CCR_IMODE_TWO_LINE        0x2
#define CCR_IMODE_FOUR_LINE       0x3
#define CCR_IMODE_EIGHT_LINE      0x4
/*Macros for Instruction Size*/
#define CCR_ISIZE_8_BIT           0x0
#define CCR_ISIZE_16_BIT          0x1
#define CCR_ISIZE_24_BIT          0x2
#define CCR_ISIZE_32_BIT          0x3
/*Macros for Address MODE*/
#define CCR_ADMODE_NIL            0x0
#define CCR_ADMODE_SINGLE_LINE    0x1
#define CCR_ADMODE_TWO_LINE       0x2
#define CCR_ADMODE_FOUR_LINE      0x3
#define CCR_ADMODE_EIGHT_LINE     0x4
/*Macros for Address Size*/
#define CCR_ADSIZE_8_BIT          0x0
#define CCR_ADSIZE_16_BIT         0x1
#define CCR_ADSIZE_24_BIT         0x2
#define CCR_ADSIZE_32_BIT         0x3
/*Macros for Alternate Byte mode*/
#define CCR_ABMODE_NIL            0x0
#define CCR_ABMODE_SINGLE_LINE    0x1
#define CCR_ABMODE_TW0_LINE       0x2
#define CCR_ABMODE_FOUR_LINE      0x3
#define CCR_ABMODE_EIGHT_LINE     0x4
/*Macros for Alternate Byte size*/
#define CCR_ABSIZE_8_BIT          0x0
#define CCR_ABSIZE_16_BIT         0x1
#define CCR_ABSIZE_24_BIT         0x2
#define CCR_ABSIZE_32_BIT         0x3
/*Macros for Data mode*/
#define CCR_DMODE_NO_DATA         0x0
#define CCR_DMODE_SINGLE_LINE     0x1
#define CCR_DMODE_TWO_LINE        0x2
#define CCR_DMODE_FOUR_LINE       0x3
#define CCR_DMODE_EIGHT_LINE      0x4
/*Macros for Functional mode*/
#define CCR_FMODE_INDIRECT_WRITE  0x0
#define CCR_FMODE_INDIRECT_READ   0x1
#define CCR_FMODE_APM             0x2
#define CCR_FMODE_MMM             0x3
/*Macros for Memory map mode*/
#define CCR_MM_MODE_XIP           0x0
#define CCR_MM_MODE_RAM           0x1
/*Macros for OR and AND PMM*/
#define PMM_AND                   0
#define PMM_OR                    1
/*Macros for Datatype*/
#define DATA_TYPE_8BIT            0x0 
#define DATA_TYPE_16BIT           0x1
#define DATA_TYPE_32BIT           0x2
#define DATA_TYPE_64BIT           0x3

#define CR_HW_PROTECT(x)      (x << 20)    // Bit 20 - Hardware protection enable
#define CR_FMODE(x)       (x<<18)//2bit
#define CR_PMM(x)         (x<<17)
#define CR_APMS(x)        (x<<16)
#define CR_TOIE(x)        (x<<15)//1bit
#define CR_SMIE(x)        (x<<14)
#define CR_FTIE(x)        (x<<13)
#define CR_TCIE(x)        (x<<12)
#define CR_TEIE(x)        (x<<11)
#define CR_FTHRES(x)      (x<<6 )//4bit
#define CR_FSEL(x)        (x<<5 )//Not used
#define CR_DMM(x)         (x<<4 )//Not used 1bit
#define CR_TCEN(x)        (x<<3 )
#define CR_DMAEN(x)       (x<<2 )//Not used
#define CR_ABORT(x)       (x<<1 )
#define CR_EN(x)          (x<<0 )

//Bit vectors for DCR1 
#define DCR1_MTYP(x)      (x<<10)//2bit
#define DCR1_DEVSIZE(x)   (x<<5 )//5bit 
#define DCR1_CSHT(x)      (x<<2 )//3bit Not used
#define DCR1_FRCK(x)      (x<<1 )//3bit Not used
#define DCR1_CKMODE(x)    (x<<0 )//1bit 

//Bit vectors for DCR2
#define DCR2_WRAPSIZE(x)  (x<<8)//3bit
#define DCR2_PRESCALER(x) (x<<0)//8bit

//Bit vectors for DCR3
#define DCR3_CSBOUND(x)   (x<<8)//5bit
#define DCR3_MAXTRAN(x)   (x<<0)//8bit

//Bit vectors for status register
#define SR_FLEVEL(x)      (x<<6)//6bit
#define SR_BUSY           (1<<5)//1bit
#define SR_TOF            (1<<4)
#define SR_SMF            (1<<3)
#define SR_FTF            (1<<2)
#define SR_TCF            (1<<1)
#define SR_TEF            (1<<0)

//Bit vectors for flag clear register 
#define FCR_CTOF (1<<3)
#define FCR_CSMF (1<<2)
#define FCR_CTCF (1<<1)//1bit
#define FCR_CTEF (1<<0)

//Bit vectors for CCR
#define CCR_MM_MODE(x)    (x<<24) //memory Map mode XIP=0;RAM=1;
#define CCR_SIOO(x)       (x<<23)
#define CCR_DQSE(x)       (x<<22)
#define CCR_DDTR(x)       (x<<21)
#define CCR_DMODE(x)      (x<<18)
#define CCR_ABSIZE(x)     (x<<16)
#define CCR_ABDTR(x)      (x<<15)
#define CCR_ABMODE(x)     (x<<12)
#define CCR_ADSIZE(x)     (x<<10)
#define CCR_ADDTR(x)      (x<<9 )
#define CCR_ADMODE(x)     (x<<6 )
#define CCR_ISIZE(x)      (x<<4 )
#define CCR_IDTR(x)       (x<<3 )
#define CCR_IMODE(x)      (x<<0 )

//Bit vectors for TCR
#define TCR_SSHIFT(x)     (x<<6)
#define TCR_DHQC(x)       (x<<5)
#define TCR_DCYC(x)       (x<<0)

//Bit vectors for RMC
#define RMC_WDCYC(x)      (x<<21)
#define RMC_RDCYC(x)      (x<<16)
#define RMC_WINSTR(x)     (x<<8 )
#define RMC_RINSTR(x)     (x<<0 )

typedef struct{
    uint32_t functional_mode     : 2;            /**< Functional mode                                                           */
    uint32_t instruction;                        /**< Instruction                                                               */
    uint32_t instruction_mode    : 3;            /**< Instruction mode                                                          */
    uint32_t instruction_size    : 2;            /**< Instruction size                                                          */
    uint32_t address_mode        : 3;            /**< Address mode                                                              */
    uint32_t address_size        : 2;            /**< Address size                                                              */
    uint32_t address;                           /**< Address                                                                   */
    uint32_t alternate_byte_mode : 3;            /**< Alternate byte mode                                                       */
    uint32_t alternate_byte;                    /**< Alternate byte                                                            */
    uint32_t alternate_byte_size : 2;            /**< Alternate byte size                                                       */
    uint32_t dummy_cycles        : 5;            /**< Dummy Cycles                                                              */
    uint32_t sioo                : 1;            /**< Send instruction only once                                                */
    uint32_t mm_mode             : 1;            /**< Memory map mode enable                                                    */
    uint32_t data_mode           : 3;            /**< Data mode                                                                 */
    uint32_t length;                            /**< Data length                                                               */
    void *data_buffer;                       /**< Pointer to data buffer                                                    */
    uint32_t FMEM_SIZE           : 5;            /**< Flash memory size                                                         */
    uint32_t CLK_MODE            : 1;            /**< Clock mode                                                                */
    uint32_t fthresh             : 5;
    uint32_t csht                : 3;
    uint32_t abort               : 1;
    uint32_t TCEN               : 1;            /**< Timeout counter enable                                                    */
    uint32_t TEIE               : 1;            /**< Transfer error interrupt enable                                           */
    uint32_t TCIE               : 1;            /**< Transfer complete interrupt enable                                        */
    uint32_t FTIE               : 1;            /**< FIFO threshold interrupt enable                                           */
    uint32_t SMIE               : 1;            /**< Status match interrupt enable                                             */
    uint32_t TOIE               : 1;            /**< TimeOut interrupt enable                                                  */
    uint32_t APMS               : 1;            /**< Automatic poll mode stop                                                  */
    uint32_t PMM                : 1;            /**< Polling match mode                                                        */
    uint32_t PRESCALER          : 7;            /**< Clock prescaler                                                           */
    uint32_t status_mask;
    uint32_t status_match;
    uint32_t dhqc;
    uint32_t sshift;
    uint32_t rd_instr;
    uint32_t wr_instr;
    uint32_t rd_dcyc             : 5;
    uint32_t wr_dcyc             : 5;
    uint32_t enable;
    uint32_t IDTR;
    uint32_t ADDTR;
    uint32_t ABDTR;
    uint32_t DDTR;
    uint32_t dqse;
    uint32_t dual_mem;
    uint32_t FTF;
    uint32_t dsize;
    uint32_t pir;
    uint32_t timeout;
    uint8_t hw_protect;
    uint8_t dmaen;
}xspi_msg;
/**
 * @fn XSPI_Transaction(uint32_t instance_number,xspi_msg *msg)
 * 
 * @brief Used to perfom XSPI transaction with necessary setting.
 * @param xspinum The parameter \a xspinum is an unsigned integer that represents the XSPI instance number.
 * @param msg The parameter \a msg is a structure which has all settings to initiate a transaction.
 * 
 * @return SUCCESS if operation is successful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
uint32_t XSPI_Transaction(struct target *target,uint32_t instance_number,xspi_msg *msg);
uint8_t XSPI_SMF_Flag_Check(uint8_t instance_number,uint8_t wait_till_match);

#ifdef __cplusplus
}
#endif
#endif
