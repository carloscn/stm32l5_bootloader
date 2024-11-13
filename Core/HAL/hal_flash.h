#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
   FLASH Hierarchical Structure:

   - Each HAL_FLASH_CHIP can contain multiple HAL_FLASH_BANKs.
   - Each HAL_FLASH_BANK can contain multiple HAL_FLASH_PAGEs.
   - This structure represents a hierarchical organization of memory.

   Example Hierarchy:
   HAL_FLASH_CHIP_1
       |
       +-- HAL_FLASH_BANK_1
       |       |
       |       +-- HAL_FLASH_PAGE_1
       |       +-- HAL_FLASH_PAGE_2
       |       +-- HAL_FLASH_PAGE_3
       |
       +-- HAL_FLASH_BANK_2
               |
               +-- HAL_FLASH_PAGE_1
               +-- HAL_FLASH_PAGE_2
               +-- HAL_FLASH_PAGE_3
*/

typedef enum HAL_FLASH_CHIP_T {
    HAL_FLASH_CHIP_0 = 0x0,
    HAL_FLASH_CHIP_1,
    HAL_FLASH_CHIP_2,
    HAL_FLASH_CHIP_3,
    HAL_FLASH_CHIP_4,
    HAL_FLASH_CHIP_5,
    HAL_FLASH_CHIP_NUM_MAX
} HAL_FLASH_CHIP_NUM;

typedef struct hal_flash_chip_t {
    HAL_FLASH_CHIP_NUM chip_num; // The specific flash chip being accessed
} HAL_FLASH_CHIP;

int32_t hal_flash_init(HAL_FLASH_CHIP *flash_ctx);
void hal_flash_free(HAL_FLASH_CHIP *flash_ctx);
int32_t hal_flash_write(HAL_FLASH_CHIP *flash_ctx, uint32_t addr, uint8_t* data, size_t bytes);
int32_t hal_flash_erase(HAL_FLASH_CHIP *flash_ctx, uint32_t addr, size_t bytes);
int32_t hal_flash_read(HAL_FLASH_CHIP *flash_ctx, uint32_t addr, uint8_t* data, size_t bytes);
int32_t hal_flash_page_write(HAL_FLASH_CHIP *flash_ctx, size_t page_num, uint8_t* data, size_t bytes);
int32_t hal_flash_enable_secure(HAL_FLASH_CHIP *flash_ctx, uint32_t addr, size_t bytes);
int32_t hal_flash_disable_secure(HAL_FLASH_CHIP *flash_ctx, uint32_t addr, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif // HAL_FLASH_H