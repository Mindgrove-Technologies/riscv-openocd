#include<stdint.h>
#include "v2500_xspi.h"

// #define OCTOSPI_Reg(x) ((OCTOSPI_Type*)(OCTOSPI0_BASE + ((x) * XSPI_OFFSET)))

// #define OCTOSPI_Reg(x) 

// volatile xspi_msg msg={.PRESCALER=6,.FMEM_SIZE=27,.FTIE=1,.TCEN=0,.TEIE=0,.TOIE=0,.SMIE=0,.APMS=0,.PMM=0};
/**
 * @fn XSPI_Transaction(uint32_t xspinum,xspi_msg *msg)
 * 
 * @brief Used to perfom XSPI transaction with necessary setting.
 * @param xspinum The parameter \a xspinum is an unsigned integer that represents the XSPI instance number.
 * @param msg The parameter \a msg is a structure which has all settings to initiate a transaction.
 * 
 * @return 0 if operation is 0ful,ENODEV if invalid instance number and ELENEXCEED is length of read and write parameters exceeded.
 */
#define OCTOSPI0_BASE               0x00060300UL
#define XSPI_OFFSET                 0x200
uint32_t XSPI_Transaction(struct target *target,uint32_t xspinum,xspi_msg *msg){

    if(xspinum > XSPI_MAX-1)
      return ENODEV;
    uint32_t xspi_base = OCTOSPI0_BASE+(xspinum*XSPI_OFFSET);
    uint32_t CR    =xspi_base+0x00;
    uint32_t DCR1  =xspi_base+0x04;
    uint32_t DCR2  =xspi_base+0x08;
    uint32_t DCR3  =xspi_base+0x0c;
    uint32_t FCR   =xspi_base+0x14;
    uint32_t DLR   =xspi_base+0x18;
    uint32_t AR    =xspi_base+0x1c;
    uint32_t PSMKR =xspi_base+0x20;
    uint32_t PSMAR =xspi_base+0x24;
    uint32_t PIR   =xspi_base+0x28;
    uint32_t CCR   =xspi_base+0x2c;
    uint32_t TCR   =xspi_base+0x30;
    uint32_t IR    =xspi_base+0x34;
    uint32_t ABR   =xspi_base+0x38;
    uint32_t DR    =xspi_base+0x4c;
    uint32_t SR    =xspi_base+0x54;
    uint32_t LPTR  =xspi_base+0x58;
    uint32_t RMC   =xspi_base+0x5c;
    
    uint32_t temp;
    temp = (DCR1_DEVSIZE(msg->FMEM_SIZE) | DCR1_CKMODE(msg->CLK_MODE) | DCR1_CSHT(msg->csht));
    target_write_u32(target,DCR1,temp);

    temp = (DCR2_PRESCALER(msg->PRESCALER));
    target_write_u32(target,DCR2,temp);

    temp = (DCR3_MAXTRAN(1)) | (DCR3_CSBOUND(1));
    target_write_u32(target,DCR3,temp);

    temp=(CR_FMODE(msg->functional_mode) |CR_PMM(msg->PMM) | CR_APMS(msg->APMS) | CR_TOIE(msg->TOIE) | CR_SMIE(msg->SMIE) | CR_FTIE(msg->FTIE) | CR_TCIE(msg->TCIE) | CR_TEIE(msg->TEIE) | CR_TCEN(msg->TCEN) | CR_FTHRES(msg->fthresh) | CR_ABORT(msg->abort) | CR_EN(msg->enable)|  CR_HW_PROTECT(msg->hw_protect) | CR_DMAEN(msg->dmaen));
    target_write_u32(target,CR,temp);

    temp =(FCR_CTOF|FCR_CSMF|FCR_CTCF|FCR_CTEF);//clear flags
    target_write_u32(target,FCR,temp);

    temp = msg->length-1;
    target_write_u32(target,DLR,temp);

    if((msg->functional_mode == CCR_FMODE_APM)){
        temp =msg->status_mask;
        target_write_u32(target,PSMKR,temp);

        temp =msg->status_match;
        target_write_u32(target,PSMAR,temp);

        temp =msg->pir;
        target_write_u32(target,PIR,temp);

    }
    temp = TCR_SSHIFT(msg->sshift) | TCR_DHQC(msg->dhqc) | TCR_DCYC(msg->dummy_cycles);
    target_write_u32(target,TCR,temp);

    temp = (CCR_IMODE(msg->instruction_mode) | CCR_ISIZE(msg->instruction_size) | CCR_ADMODE(msg->address_mode) | CCR_ADSIZE(msg->address_size) | CCR_ABMODE(msg->alternate_byte_mode) | CCR_ABSIZE(msg->alternate_byte_size) | CCR_DMODE(msg->data_mode) | CCR_SIOO(msg->sioo) | CCR_MM_MODE(msg->mm_mode) | CCR_IDTR(msg->IDTR) | CCR_ADDTR(msg->ADDTR) | CCR_ABDTR(msg->ABDTR) | CCR_DDTR(msg->DDTR) | CCR_DQSE(msg->dqse));
    target_write_u32(target,CCR,temp);
    if(msg->TCEN == 1)
      target_write_u32(target,LPTR,msg->timeout);
    if(msg->instruction_mode)
      target_write_u32(target,IR,msg->instruction);
    if((msg->functional_mode == CCR_FMODE_MMM) && (msg->mm_mode == CCR_MM_MODE_RAM))
    {
      temp = RMC_WDCYC(msg->wr_dcyc) | RMC_RDCYC(msg->rd_dcyc) | RMC_WINSTR(msg->wr_instr) | RMC_RINSTR(msg->rd_instr);
      target_write_u32(target,RMC,temp);
    }
    if(msg->alternate_byte_mode){
      target_write_u32(target,ABR,msg->alternate_byte);
    }
    if(msg->address_mode){
        target_write_u32(target,AR,msg->address);
    }

    uint8_t i = 0;
     uint32_t status_reg;
    
    if(msg->functional_mode == CCR_FMODE_INDIRECT_WRITE && msg->length!= 0)
    {
      uint64_t *word_64 = (uint64_t *)msg->data_buffer;

    // //
    // // ---------------- 32-bit writes ----------------
    // //
    uint32_t *word_32 = (uint32_t *)word_64;


    target_read_u32(target, CR, &temp);
    temp &= ~CR_FTHRES(31);
    temp |= CR_FTHRES(3);
    target_write_u32(target, CR, temp);
 
    while ((msg->length - i) >= 4)
    {
      // wait until FIFO has space
      do
      {
        target_read_u32(target, SR, &status_reg);
      } while ((!(status_reg& SR_FTF)));

      target_write_u32(target, DR, *word_32);
      word_32++;
      i += 4;
    }

    // //
    // // ---------------- 16-bit writes ----------------
    // //

    uint16_t *word_16 = (uint16_t *)word_32;
    target_read_u32(target, CR, &temp);
    temp &= ~CR_FTHRES(31);
    temp |= CR_FTHRES(1);
    target_write_u32(target, CR, temp);
    while ((msg->length - i) >= 2)
    {
      // wait until FIFO has space
      do
      {
        target_read_u32(target, SR, &status_reg);
      } while (!(status_reg & SR_FTF));
      target_write_u16(target, DR, *word_16);
      word_16++;
      i += 2;
    }

    //
    // ---------------- 8-bit writes ----------------
    //
    uint8_t *word_8 = (uint8_t *)word_16;
    target_read_u32(target, CR, &temp);
    temp &= ~CR_FTHRES(31);
    temp |= CR_FTHRES(0);
    target_write_u32(target, CR, temp);
    while ((msg->length - i) >= 1)
    {
      // wait until FIFO has space
      do
      {
        target_read_u32(target, SR, &status_reg);
      } while (!(status_reg & SR_FTF));
      target_write_u8(target, DR, *word_8);
      word_8++;
      i += 1;
    // }
    }
  }


    else if(msg->functional_mode == CCR_FMODE_INDIRECT_READ)
    {
      if(msg->data_mode){
            //   QUADSPI_Reg(instance_number)->CR&= ~(CR_FTHRES(31));
        target_read_u32(target, CR, &temp);
        temp &= ~(CR_FTHRES(31));
        target_write_u32(target, CR, temp);
        target_read_u32(target, CR, &temp);
        while (1)
        {
          // status_reg = QUADSPI_Reg(instance_number)->SR;
          target_read_u32(target, SR, &status_reg);
          // printf("status_reg %x", status_reg);
          status_reg &= SR_FTF;
          if (status_reg)
          {
            
            target_read_u8(target, DR, &((uint8_t*)msg->data_buffer)[i]);
            
            //   msg->data_buffer[i] = QUADSPI_Reg(instance_number)->DR.data_8;
            i++;
            if (i == msg->length)
              break;
          }
        }
      }
    }
    
    if(msg->functional_mode == CCR_FMODE_MMM)
      return 0;
    
    if(msg->functional_mode == CCR_FMODE_APM){
      if(msg->address_mode)
        target_write_u32(target,AR,msg->address);
      return 0;
    }
    
    if((msg->functional_mode == CCR_FMODE_INDIRECT_READ) ||(msg->functional_mode == CCR_FMODE_INDIRECT_WRITE) )
    {
      do{
        target_read_u32(target,SR,&temp);
      temp &= SR_TCF;
    }while(temp == 0);
        target_read_u32(target,CR,&temp);
        temp&=~~CR_EN(1);
        target_write_u32(target, CR,temp);
    } 
    return 0;
}

