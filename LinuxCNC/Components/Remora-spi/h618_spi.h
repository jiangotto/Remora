/* h618_spi.h
 * Header file for H618 SPI interface using spidev
 * Used for Remora LinuxCNC driver
 */

#ifndef H618_SPI_H
#define H618_SPI_H

#include <stdint.h>

// SPI模式定义（与Linux spidev兼容）
#define H618_SPI_MODE_0    0  // CPOL=0, CPHA=0
#define H618_SPI_MODE_1    1  // CPOL=0, CPHA=1
#define H618_SPI_MODE_2    2  // CPOL=1, CPHA=0
#define H618_SPI_MODE_3    3  // CPOL=1, CPHA=1

// 函数声明
// 检测是否运行在H618平台上
extern int h618_detect(void);

// GPIO相关函数（使用sysfs接口）
extern int h618_gpio_init(void);
extern void h618_gpio_set_fsel(uint8_t pin, uint8_t fsel);  // fsel=1表示输出模式，其他表示输入模式
extern void h618_gpio_set(uint8_t pin);
extern void h618_gpio_clear(uint8_t pin);

// SPI相关函数（使用spidev接口）
extern int h618_spi_init(uint8_t spi_num, uint8_t cs_num, uint8_t mode, uint32_t freq);
extern void h618_spi_transfer(const void *txbuf, void *rxbuf, uint8_t len);
extern void h618_spi_cleanup(void);

#endif /* H618_SPI_H */