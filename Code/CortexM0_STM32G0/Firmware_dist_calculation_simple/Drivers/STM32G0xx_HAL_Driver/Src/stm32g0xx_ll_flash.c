//Taken from here - https://github.com/STMicroelectronics/stm32g0xx-hal-driver/blob/master/Src/stm32g0xx_hal_flash.c

#if defined(USE_FULL_LL_DRIVER)

#include "stm32g0xx_ll_flash.h"
#ifdef  USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

static ErrorStatus FLASH_WaitForLastOperation(void);
static void        FLASH_MassErase           (uint32_t Banks);
static void        FLASH_PageErase           (uint32_t Banks, uint32_t Page);

static ErrorStatus FLASH_WaitForLastOperation(void)
{
    uint32_t error;

#if defined(FLASH_DBANK_SUPPORT)
    error = (FLASH_SR_BSY1 | FLASH_SR_BSY2);
#else
    error = FLASH_SR_BSY1;
#endif /* FLASH_DBANK_SUPPORT */
    while ((FLASH->SR & error) != 0x00U) {
        __NOP();
    }

    error = (FLASH->SR & FLASH_SR_ERRORS);
    FLASH->SR = FLASH_SR_CLEAR;
    if (error != 0x00U) {
        return ERROR;
    }

    while ((FLASH->SR & FLASH_SR_CFGBSY) != 0x00U) {
        __NOP();
    }

    return SUCCESS;
}

ErrorStatus  LL_FLASH_Unlock(void)
{
    ErrorStatus status = ERROR;

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U) {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* verify Flash is unlock */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) == 0x00U) {
            status = SUCCESS;
        }
    }

    return status;
}

ErrorStatus  LL_FLASH_Lock(void)
{
    ErrorStatus status = ERROR;

    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    /* verify Flash is locked */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00u) {
        status = SUCCESS;
    }

    return status;
}

ErrorStatus LL_FLASH_Program_DoubleWord(const uint32_t *Address, uint64_t Data)
{
    ErrorStatus status = ERROR;
    uint32_t *dest = (uint32_t *)Address;
    uint32_t primask_bit;

    status = FLASH_WaitForLastOperation();
    if (status == ERROR) {
        return ERROR;
    }

    /* Set PG bit */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    primask_bit = __get_PRIMASK();
    __disable_irq();

    /* Program first word */
    *dest++ = (uint32_t)Data;

    /* Barrier to ensure programming is performed in 2 steps, in right order
        (independently of compiler optimization behavior) */
    __ISB();

    /* Program second word */
    *dest = (uint32_t)(Data >> 32U);

    __set_PRIMASK(primask_bit);

    status = FLASH_WaitForLastOperation();
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

    return status;
}

//ErrorStatus __RAM_FUNC LL_FLASH_Program_Fast(const uint32_t *Address, uint32_t *DataAddress)
ErrorStatus LL_FLASH_Program_Fast(const uint32_t *Address, uint32_t *DataAddress)
{
    ErrorStatus status = ERROR;
    uint32_t primask_bit;
    uint32_t *dest = (uint32_t *)Address;
    uint32_t *src = DataAddress;

    status = FLASH_WaitForLastOperation();
    if (status == ERROR) {
        return ERROR;
    }

    /* Set FSTPG bit */
    SET_BIT(FLASH->CR, FLASH_CR_FSTPG);

    /* Enter critical section: row programming should not be longer than 7 ms */
    primask_bit = __get_PRIMASK();
    __disable_irq();

    /* Fast Program : 64 words */
    for (uint8_t i = 0; i < 64; i++) {
        *dest++ = *src++;
    }

    /* wait for BSY1 in order to be sure that flash operation is ended befoire
        allowing prefetch in flash. Timeout does not return status, as it will
        be anyway done later */
#if defined(FLASH_DBANK_SUPPORT)
    while ((FLASH->SR & (FLASH_SR_BSY1 | FLASH_SR_BSY2)) != 0x00U)
#else
    while ((FLASH->SR & FLASH_SR_BSY1) != 0x00U)
#endif /* FLASH_DBANK_SUPPORT */
    {
        __NOP();
    }

    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);

    status = FLASH_WaitForLastOperation();
    CLEAR_BIT(FLASH->CR, FLASH_CR_FSTPG);

    return status;
}

static void FLASH_MassErase(uint32_t Banks)
{
    /* Set the Mass Erase Bit and start bit */
    FLASH->CR |= (FLASH_CR_STRT | Banks);
}

static void FLASH_PageErase(uint32_t Banks, uint32_t Page)
{
    uint32_t tmp;

    /* Get configuration register, then clear page number */
    tmp = (FLASH->CR & ~FLASH_CR_PNB);

#if defined(FLASH_DBANK_SUPPORT)
    /* Check if page has to be erased in bank 1 or 2 */
    if (Banks != FLASH_BANK_1) {
        tmp |= FLASH_CR_BKER;
    } else {
        tmp &= ~FLASH_CR_BKER;
    }
#endif /* FLASH_DBANK_SUPPORT */

    /* Set page number, Page Erase bit & Start bit */
    FLASH->CR = (tmp | (FLASH_CR_STRT | (Page <<  FLASH_CR_PNB_Pos) | FLASH_CR_PER));
}

ErrorStatus LL_FLASH_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
    ErrorStatus status;
    uint32_t index;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    if (status == SUCCESS) {
#if !defined(FLASH_DBANK_SUPPORT)
        /* For single bank product force Banks to Bank 1 */
        pEraseInit->Banks = FLASH_BANK_1;
#endif /* FLASH_DBANK_SUPPORT */

        if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASS) {
            /* Proceed to Mass Erase */
            FLASH_MassErase(pEraseInit->Banks);

            /* Wait for last operation to be completed */
            status = FLASH_WaitForLastOperation();
        } else {
            /*Initialization of PageError variable*/
            *PageError = 0xFFFFFFFFU;

            for (index = pEraseInit->Page; index < (pEraseInit->Page + pEraseInit->NbPages); index++) {
                /* Start erase page */
                FLASH_PageErase(pEraseInit->Banks, index);

                /* Wait for last operation to be completed */
                status = FLASH_WaitForLastOperation();

                if (status != SUCCESS) {
                    /* In case of error, stop erase procedure and return the faulty address */
                    *PageError = index;
                    break;
                }
            }

            /* If operation is completed or interrupted, disable the Page Erase Bit */
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
        }
    }

    /* return status */
    return status;
}

#endif /* USE_FULL_LL_DRIVER */
