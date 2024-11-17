#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdlib.h>

#define SPI_PATH "/dev/spidev0.0"  // SPI device path
#define SPI_SPEED 1350000          // SPI speed in Hz

int spi_fd;  // File descriptor for SPI device

// Function to initialize the SPI connection
int init_spi() {
    int ret;
    uint8_t mode = 0;
    uint8_t bits_per_word = 8;
    uint32_t speed = SPI_SPEED;  // Store the speed in a variable

    // Open SPI device
    spi_fd = open(SPI_PATH, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }

    // Set SPI mode
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        perror("Failed to set SPI mode");
        return -1;
    }

    // Set bits per word
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    if (ret == -1) {
        perror("Failed to set bits per word");
        return -1;
    }

    // Set SPI speed
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);  // Use the variable `speed` here
    if (ret == -1) {
        perror("Failed to set SPI speed");
        return -1;
    }

    return 0;
}

// Function to read from MCP3008 Channel 0
int read_channel_0() {
    uint8_t tx[] = {0x01, 0b10000000, 0x00}; // Start bit, single-ended mode, Channel 0
    uint8_t rx[3] = {0, 0, 0};

    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };

    // Perform SPI transaction
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        perror("Failed to perform SPI transaction");
        return -1;
    }

    // Extract 10-bit ADC value from response
    int adc_value = ((rx[1] & 0x03) << 8) | rx[2];
    return adc_value;
}

int main() {
    if (init_spi() == -1) {
        exit(1);  // Exit if SPI initialization fails
    }

    while (1) {
        int adc_value = read_channel_0();
        if (adc_value == -1) {
            break;  // Exit if SPI transaction fails
        }

        // Print ADC value to the console
        printf("Channel 0 ADC Value: %d\n", adc_value);
        usleep(500000);  // Delay 0.5 seconds
    }

    // Close SPI device
    close(spi_fd);
    return 0;
}
