/* h618_spi.c
 * Implementation file for H618 SPI interface using spidev
 * Used for Remora LinuxCNC driver
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include "rtapi.h"

#include "h618_spi.h"

// 全局变量
static int h618_spi_fd = -1;
static uint8_t current_spi_num = 0;
static uint8_t current_cs_num = 0;
static int h618_gpio_fd = -1;  // GPIO文件描述符，用于通过libgpiod操作GPIO

// 检测是否运行在H618平台上
int h618_detect(void)
{
    FILE *fp;
    char buf[256];
    ssize_t buflen;
    
    // 尝试读取设备树信息来检测平台
    if ((fp = fopen("/proc/device-tree/compatible", "rb")) != NULL) {
        buflen = fread(buf, 1, sizeof(buf), fp);
        fclose(fp);
        
        // 检查是否包含h618相关字符串
        if (buflen > 0 && strstr(buf, "h616") != NULL) {
            rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected\n");
            return 1;
        }
    }
    
    // 如果设备树检测失败，尝试其他方法（如读取/proc/cpuinfo）
    if ((fp = fopen("/proc/cpuinfo", "r")) != NULL) {
        while (fgets(buf, sizeof(buf), fp) != NULL) {
            if (strstr(buf, "h618") != NULL) {
                fclose(fp);
                rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected from cpuinfo\n");
                return 1;
            }
        }
        fclose(fp);
    }
    
    // 直接检查spidev1.0设备是否存在
    if (access("/dev/spidev1.0", F_OK) == 0) {
        rtapi_print_msg(RTAPI_MSG_INFO, "H618 platform detected (spidev1.0 exists)\n");
        return 1;
    }
    
    return 0;
}

// 通过sysfs设置GPIO方向
static int set_gpio_direction(int pin, const char *direction)
{
    char path[64];
    FILE *fp;
    
    // 导出GPIO引脚
    snprintf(path, sizeof(path), "/sys/class/gpio/export");
    fp = fopen(path, "w");
    if (fp == NULL) {
        // 如果导出失败，可能是因为已经导出，忽略错误
        rtapi_print_msg(RTAPI_MSG_DBG, "Failed to export GPIO %d: %s\n", pin, strerror(errno));
    } else {
        fprintf(fp, "%d", pin);
        fclose(fp);
    }
    
    // 设置GPIO方向
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    fp = fopen(path, "w");
    if (fp == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set GPIO %d direction: %s\n", pin, strerror(errno));
        return -1;
    }
    fprintf(fp, "%s", direction);
    fclose(fp);
    
    return 0;
}

// 通过sysfs设置GPIO值
static int set_gpio_value(int pin, int value)
{
    char path[64];
    FILE *fp;
    
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    fp = fopen(path, "w");
    if (fp == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set GPIO %d value: %s\n", pin, strerror(errno));
        return -1;
    }
    fprintf(fp, "%d", value);
    fclose(fp);
    
    return 0;
}

// 初始化GPIO（使用sysfs）
int h618_gpio_init(void)
{
    // GPIO通过sysfs操作，不需要额外的初始化
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 GPIO initialized using sysfs\n");
    return 1;
}

// 设置GPIO功能（对于sysfs，我们只需要设置方向）
void h618_gpio_set_fsel(uint8_t pin, uint8_t fsel)
{
    // fsel=1表示输出模式
    if (fsel == 1) {
        set_gpio_direction(pin, "out");
    } else {
        set_gpio_direction(pin, "in");
    }
}

// 设置GPIO为高电平
void h618_gpio_set(uint8_t pin)
{
    set_gpio_value(pin, 1);
}

// 设置GPIO为低电平
void h618_gpio_clear(uint8_t pin)
{
    set_gpio_value(pin, 0);
}

// 初始化SPI（使用spidev）
int h618_spi_init(uint8_t spi_num, uint8_t cs_num, uint8_t mode, uint32_t freq)
{
    char spi_dev_path[32];
    uint8_t bits = 8;
    uint16_t delay = 0;
    
    // 记录当前配置
    current_spi_num = spi_num;
    current_cs_num = cs_num;
    
    // 关闭之前打开的SPI设备
    if (h618_spi_fd >= 0) {
        close(h618_spi_fd);
        h618_spi_fd = -1;
    }
    
    // 构建SPI设备路径
    // 用户提到有spidev1.0设备，我们优先使用这个
    if (spi_num == 1 && cs_num == 0) {
        snprintf(spi_dev_path, sizeof(spi_dev_path), "/dev/spidev1.0");
    } else {
        snprintf(spi_dev_path, sizeof(spi_dev_path), "/dev/spidev%d.%d", spi_num, cs_num);
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "Trying to open SPI device: %s\n", spi_dev_path);
    
    // 打开SPI设备
    h618_spi_fd = rtapi_open_as_root(spi_dev_path, O_RDWR);
    if (h618_spi_fd < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to open SPI device %s: %s\n", spi_dev_path, strerror(errno));
        return 0;
    }
    
    // 设置SPI模式
    if (ioctl(h618_spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set SPI mode: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证SPI模式设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_MODE, &mode) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get SPI mode: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 设置位长度
    if (ioctl(h618_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set bits per word: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证位长度设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get bits per word: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 设置SPI频率
    if (ioctl(h618_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to set max speed Hz: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    // 验证频率设置
    if (ioctl(h618_spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &freq) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to get max speed Hz: %s\n", strerror(errno));
        close(h618_spi_fd);
        h618_spi_fd = -1;
        return 0;
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 SPI%d.%d initialized successfully, mode=%d, freq=%d Hz, bits=%d\n",
                   spi_num, cs_num, mode, freq, bits);
    
    return 1;
}

// SPI数据传输（使用spidev）
void h618_spi_transfer(const void *txbuf, void *rxbuf, uint8_t len)
{
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)txbuf,
        .rx_buf = (unsigned long)rxbuf,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 0,  // 使用之前设置的速度
        .bits_per_word = 8,
    };
    
    if (h618_spi_fd < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SPI device not initialized\n");
        return;
    }
    
    ret = ioctl(h618_spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SPI transfer failed: %s\n", strerror(errno));
    }
}

// 清理资源
void h618_spi_cleanup(void)
{
    if (h618_spi_fd >= 0) {
        close(h618_spi_fd);
        h618_spi_fd = -1;
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "H618 SPI resources cleaned up\n");
}