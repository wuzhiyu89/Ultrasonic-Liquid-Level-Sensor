/**
 * @file storage.c
 * @brief common 片内Flash读写工具 N32G031
 */
#include "storage.h"
#include "n32g031_flash.h"

bool storage_flash_read(uint32_t flash_addr, uint8_t *buf, uint16_t len)
{
    if(buf == NULL)
    {
        return false;
    }
    for(uint16_t i = 0U; i < len; i++)
    {
        buf[i] = *(volatile uint8_t*)(flash_addr + i);
    }
    return true;
}

bool storage_flash_erase_page(uint32_t page_addr)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_STS_CLRFLAG);

    FLASH_STS sts = FLASH_EraseOnePage(page_addr);

    FLASH_Lock();

    if(primask == 0U)
    {
        __enable_irq();
    }
    return (sts == FLASH_COMPL);
}

/**
 * @brief N32G031 Flash 写，最小写入单元为单字(4 字节)
 * @param flash_addr 写入起始地址，须 4 字节对齐；len 为字节长度，须为 4 的倍数
 */
bool storage_flash_write(uint32_t flash_addr, const uint8_t *data, uint16_t len)
{
    if((data == NULL)
        || ((flash_addr & 0x3U) != 0U)   /* G031 Flash单字，4字节对齐 */
        || ((len & 0x3U) != 0U))          /* 长度必须4字节对齐 */
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_STS_CLRFLAG);

    uint16_t word_cnt = len / 4U;
    FLASH_STS sts = FLASH_COMPL;

    for(uint16_t i = 0U; i < word_cnt; i++)
    {
        uint32_t data_word = *( (const uint32_t*)(data + i * 4U) );

        sts = FLASH_ProgramWord(flash_addr + i * 4U, data_word);
        if(sts != FLASH_COMPL)
        {
            break;
        }
    }

    FLASH_Lock();

    if(primask == 0U)
    {
        __enable_irq();
    }

    return (sts == FLASH_COMPL);
}

