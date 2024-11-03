#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>

#define PAGE_SIZE 4096
#define PERI_BUS_BASE 0x7E000000
#define PERI_PHYS_BASE 0x3F000000
#define BUS_TO_PHYS(x) ((x) & ~0xC0000000)

#define CM_BASE 0x00101000
#define PWM_BASE 0x0020C000
#define SPI0_BASE 0x204000
#define SYST_BASE 0x00003000
#define SYST_CLO 0x04
#define DMA_BASE 0x00007000
#define DMA_CHANNEL 6

#define SPI_FIFO (PERI_BUS_BASE + 0x204004)
#define DMA_OFFSET 0x100
#define DMA_ADDR (DMA_BASE + DMA_OFFSET * DMA_CHANNEL)
#define DMA_NO_WIDE_BURSTS (1 << 26)
#define DMA_PERIPHERAL_MAPPING(x) ((x) << 16)
#define DMA_WAIT_ON_WRITES (1 << 28)
#define DMA_PRIORITY(x) ((x) << 16)
#define DMA_PANIC_PRIORITY(x) ((x) << 20)
#define DMA_DEST_DREQ (1 << 6)

#define MEM_FLAG_L1_NONALLOCATING 0x04
#define MAILBOX_IOC_MAGIC 100
#define IOCTL_MBOX_PROPERTY _IOWR(MAILBOX_IOC_MAGIC, 0, char*)

#define TICK_CNT 20
#define CB_CNT (TICK_CNT * 2)
#define CLK_MICROS 1
#define DMA_CHANNEL_ABORT (1 << 30)
#define DMA_CHANNEL_RESET (1 << 31)
#define DMA_ACTIVE (1 << 0)
#define PWM_DMAC_ENAB (1 << 31)
#define PWM_DMAC_PANIC(x) ((x) << 8)
#define PWM_DMAC_DREQ(x) (x)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

typedef struct DMAControlBlock {
    uint32_t tx_info;
    uint32_t src;
    uint32_t dest;
    uint32_t tx_len;
    uint32_t stride;
    uint32_t next_cb;
    uint32_t padding[2]; // Padding to align to 32 bytes
} DMAControlBlock;

typedef struct DMAMemHandle {
    void *virtual_addr;
    uint32_t bus_addr;
    uint32_t mb_handle;
    uint32_t size;
} DMAMemHandle;

int mailbox_fd = -1;
DMAMemHandle *dma_cbs;
DMAMemHandle *spi_data_buffer;
DMAMemHandle *adc_result_buffer;  // New buffer for storing ADC read results

// Mailbox functions for memory allocation and locking
int mbox_open() {
    int fd = open("/dev/vcio", 0);
    if (fd < 0) {
        perror("Failed to open /dev/vcio");
        exit(-1);
    }
    return fd;
}

void mbox_close(int fd) {
    close(fd);
}

int mem_alloc(int fd, unsigned int size, unsigned int align, unsigned int flags) {
    unsigned int mem_alloc[] = {0, 0x3000c, 12, 12, size, align, flags, 0};
    ioctl(fd, IOCTL_MBOX_PROPERTY, mem_alloc);
    return mem_alloc[5];
}

int mem_lock(int fd, unsigned int handle) {
    unsigned int mem_lock[] = {0, 0x3000d, 4, 4, handle, 0};
    ioctl(fd, IOCTL_MBOX_PROPERTY, mem_lock);
    return mem_lock[4];
}

int mem_unlock(int fd, unsigned int handle) {
    unsigned int mem_unlock[] = {0, 0x3000e, 4, 4, handle, 0};
    ioctl(fd, IOCTL_MBOX_PROPERTY, mem_unlock);
    return mem_unlock[4];
}

int mem_free(int fd, unsigned int handle) {
    unsigned int mem_free[] = {0, 0x3000f, 4, 4, handle, 0};
    ioctl(fd, IOCTL_MBOX_PROPERTY, mem_free);
    return mem_free[4];
}

void *mapmem(unsigned int base, unsigned int size) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("Failed to open /dev/mem");
        exit(-1);
    }

    void *mem = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base);
    close(mem_fd);

    if (mem == MAP_FAILED) {
        perror("mmap error");
        exit(-1);
    }
    return mem;
}

void unmapmem(void *addr, unsigned int size) {
    munmap(addr, size);
}

// DMA memory allocation
DMAMemHandle *dma_malloc(unsigned int size) {
    size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

    DMAMemHandle *mem = (DMAMemHandle *)malloc(sizeof(DMAMemHandle));
    mem->mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    mem->bus_addr = mem_lock(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = mapmem(BUS_TO_PHYS(mem->bus_addr), size);
    mem->size = size;

    assert(mem->bus_addr != 0);
    return mem;
}

void dma_free(DMAMemHandle *mem) {
    if (mem->virtual_addr == NULL) return;
    unmapmem(mem->virtual_addr, mem->size);
    mem_unlock(mailbox_fd, mem->mb_handle);
    mem_free(mailbox_fd, mem->mb_handle);
    free(mem);
}

// Define helper functions for control block and tick buffer addresses
static inline DMAControlBlock *ith_cb_virt_addr(int i) {
    return (DMAControlBlock *)((uint8_t *)dma_cbs->virtual_addr + i * sizeof(DMAControlBlock));
}

static inline uint32_t ith_cb_bus_addr(int i) {
    return dma_cbs->bus_addr + i * sizeof(DMAControlBlock);
}

// Setup SPI control register
void setup_spi() {
    volatile uint32_t *spi_control_reg = (uint32_t *)mapmem(SPI0_BASE, PAGE_SIZE);
    spi_control_reg[0] = (0 << 7) | (0 << 6) | (1 << 5) | (0 << 4) | (0 << 3) | (1 << 2);
    spi_control_reg[1] = 32;
    spi_control_reg[2] = 0;
}

// Initialize DMA control blocks for SPI
void dma_init_cbs() {
    DMAControlBlock *cb;
    for (int i = 0; i < TICK_CNT; i++) {
        // Configure DMA control block for ADC read command
        cb = ith_cb_virt_addr(2 * i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_ON_WRITES;
        cb->src = dma_cbs->bus_addr + i * 4;  // Start of `spi_data_buffer`
        cb->dest = SPI_FIFO;
        cb->tx_len = 3;  // 3-byte command for MCP3008
        cb->next_cb = ith_cb_bus_addr(2 * i + 1);

        // Configure DMA control block for ADC read result
        cb = ith_cb_virt_addr(2 * i + 1);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_ON_WRITES | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(6);
        cb->src = SPI_FIFO;
        cb->dest = adc_result_buffer->bus_addr + i * 4;  // Store result in `adc_result_buffer`
        cb->tx_len = 2;  // MCP3008 sends back 10-bit result in 2 bytes
        cb->next_cb = ith_cb_bus_addr((2 * i + 2) % CB_CNT);
    }
}

// Configure the PWM as a clock source
void setup_pwm_for_dma() {
    volatile uint32_t *pwm_reg = mapmem(PWM_BASE, PAGE_SIZE);
    pwm_reg[0] = 0;
    usleep(10);
    pwm_reg[1] = -1;
    usleep(10);
    pwm_reg[2] = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
    usleep(10);
    pwm_reg[4] = 100 * CLK_MICROS;
    pwm_reg[0] = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

// Allocate memory for DMA control blocks and data
void dma_alloc_buffers() {
    dma_cbs = dma_malloc(CB_CNT * sizeof(DMAControlBlock));
    spi_data_buffer = dma_malloc(TICK_CNT * sizeof(uint32_t));
    adc_result_buffer = dma_malloc(TICK_CNT * sizeof(uint32_t));

    // Load ADC read commands into `spi_data_buffer`
    for (int i = 0; i < TICK_CNT; i++) {
        ((uint32_t *)spi_data_buffer->virtual_addr)[i] = 0x018000;  // MCP3008 read command for channel 0
    }
}

// Start the DMA
void dma_start() {
    volatile uint32_t *dma_reg = mapmem(DMA_ADDR, PAGE_SIZE);
    dma_reg[0] = DMA_CHANNEL_ABORT;
    dma_reg[0] = 0;
    dma_reg[0] = DMA_CHANNEL_RESET;
    dma_reg[1] = 0;
    dma_reg[1] = BUS_TO_PHYS((uint32_t)ith_cb_virt_addr(0));
    dma_reg[0] = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_WAIT_ON_WRITES | DMA_ACTIVE;
}

// Main function
int main() {
    mailbox_fd = mbox_open();
    setup_spi();
    dma_alloc_buffers();
    dma_init_cbs();
    setup_pwm_for_dma();
    dma_start();

    // Monitor DMA transfer
    uint32_t results[TICK_CNT];
    memcpy(results, adc_result_buffer->virtual_addr, TICK_CNT * sizeof(uint32_t));
    for (size_t i = 0; i < TICK_CNT; i++) {
        printf("ADC Result %d: %04X\n", i, results[i] & 0x3FF);  // Mask for 10-bit result
    }

    dma_free(dma_cbs);
    dma_free(spi_data_buffer);
    dma_free(adc_result_buffer);
    mbox_close(mailbox_fd);
    return 0;
}
