// Add these includes
#include <stdio.h>
#include <errno.h>
#include "retarget.h"

extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len) {
   // Send all characters via UART
   HAL_StatusTypeDef hstatus = HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
   return (hstatus == HAL_OK) ? len : -1;
}

// You can add read implementation if needed
int _read(int file, char *ptr, int len) {
    // Simple implementation - just read one character at a time via UART
    HAL_StatusTypeDef hstatus = HAL_UART_Receive(&huart2, (uint8_t*)ptr, 1, HAL_MAX_DELAY);
    return (hstatus == HAL_OK) ? 1 : -1;
}
