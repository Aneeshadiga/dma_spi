import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS0)
spi.max_speed_hz = 1350000  # Set SPI speed (1.35 MHz is suitable for MCP3008)

# Function to read from MCP3008 Channel 0
def read_channel_0():
    # Send the start bit, single-ended mode, and channel 0 selection
    response = spi.xfer2([0x01, 0b10000000, 0x00])
    # Extract the 10-bit result from the response
    adc_value = ((response[1] & 0x03) << 8) | response[2]
    return adc_value

try:
    while True:
        # Read the value from Channel 0
        value = read_channel_0()
        # Log the value to the console
        print("Channel 0 ADC Value:", value)
        # Wait for 0.5 seconds before the next reading
        time.sleep(0.5)

except KeyboardInterrupt:
    # Close SPI connection on exit
    spi.close()
    print("SPI connection closed.")
