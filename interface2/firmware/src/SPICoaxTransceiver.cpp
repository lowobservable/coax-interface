#include <Arduino.h>

#include <stm32l4xx_ll_spi.h>

#include <SPICoaxTransceiver.h>

#define COAX_COMMAND_READ_REGISTER 0x2
#define COAX_COMMAND_TX 0x4
#define COAX_COMMAND_RX 0x5
#define COAX_COMMAND_RESET 0xff

SPICoaxTransceiver::SPICoaxTransceiver(const int csPin) : _csPin(csPin)
{
}

bool SPICoaxTransceiver::begin()
{
    spiSetup();

    uint8_t deviceId = readRegister(COAX_REGISTER_DEVICE_ID);

    if (deviceId != 0xa5) {
        return false;
    }

    reset();

    return true;
}

void SPICoaxTransceiver::reset()
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

    uint8_t transmitBuffer[1] = { COAX_COMMAND_RESET };
    uint8_t receiveBuffer[1];

    spiTransfer(transmitBuffer, receiveBuffer, 1);

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
}

uint8_t SPICoaxTransceiver::readRegister(const uint8_t index)
{
    //SPI.beginTransaction(spiSettings);

    //digitalWrite(_csPin, LOW);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

    uint8_t transmitBuffer[2] = { COAX_COMMAND_READ_REGISTER | (index << 4), 0x00 };
    uint8_t receiveBuffer[2];

    spiTransfer(transmitBuffer, receiveBuffer, 2);

    //digitalWrite(_csPin, HIGH);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    //SPI.endTransaction();

    return receiveBuffer[1];
}

int SPICoaxTransceiver::transmit(const uint16_t *buffer, const size_t bufferCount)
{
    //SPI.beginTransaction(spiSettings);

    //digitalWrite(_csPin, LOW);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

    uint8_t transmitBuffer[2] = { COAX_COMMAND_TX };
    uint8_t receiveBuffer[2];

    spiTransfer(transmitBuffer, receiveBuffer, 1);

    size_t count = 0;
    uint8_t error = 0;

    do {
        transmitBuffer[0] = (buffer[count] >> 8) | 0x03;
        transmitBuffer[1] = buffer[count] & 0xff;

        spiTransfer(transmitBuffer, receiveBuffer, 2);

        uint8_t value = receiveBuffer[1];

        if (value == 0) {
            count++;
        } else if (value == 0x81) {
            // Overflow... we'll just try again
            //continue;
            // TODO!!!!
            error = 99;
            break;
        } else if (value == 0x82) {
            // Underflow...
            error = 1;
            break;
        }
    } while (count < bufferCount);

    //digitalWrite(_csPin, HIGH);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    //SPI.endTransaction();

    if (error != 0) {
        return (-1) * error;
    }
    
    return count;
}

int SPICoaxTransceiver::receive(uint16_t *buffer, const size_t bufferSize)
{
    //SPI.beginTransaction(spiSettings);

    //digitalWrite(_csPin, LOW);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

    uint8_t transmitBuffer[2] = { COAX_COMMAND_RX };
    uint8_t receiveBuffer[2];

    spiTransfer(transmitBuffer, receiveBuffer, 1);

    size_t count = 0;
    uint16_t error = 0;

    transmitBuffer[0] = 0x00;
    transmitBuffer[1] = 0x00;

    do {
        spiTransfer(transmitBuffer, receiveBuffer, 2);

        uint16_t value = (receiveBuffer[0] << 8) | receiveBuffer[1];

        if (value & 0x8000) {
            error = value & 0x03ff;

            // eek... it can be an "invalid" error
            if (error == 0) {
                error = 999;
            }

            break;
        } else if (value & 0x4000) {
            break;
        }

        // XXX: overflow
        if (count >= bufferSize) {
            error = 888;
            break;
        }

        buffer[count] = value & 0x03ff;

        count++;
    } while (count < bufferSize);

    //digitalWrite(_csPin, HIGH);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    //SPI.endTransaction();

    if (error != 0) {
        return (-1) * error;
    }

    return count;
}

// TODO: move these to the class?
static SPI_TypeDef *spi = SPI1;

static LL_SPI_InitTypeDef spiConfig = {
    .TransferDirection = LL_SPI_FULL_DUPLEX,
    .Mode = LL_SPI_MODE_MASTER,
    .DataWidth = LL_SPI_DATAWIDTH_8BIT,
    .ClockPolarity = LL_SPI_POLARITY_LOW,
    .ClockPhase = LL_SPI_PHASE_1EDGE,
    .NSS = LL_SPI_NSS_SOFT, /* TODO: consider LL_SPI_NSS_HARD_INPUT */
    .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32, // 2.5 MHz
    .BitOrder = LL_SPI_MSB_FIRST,
    .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
    .CRCPoly = 0
};

static GPIO_InitTypeDef gpioInit = {
    .Pin = /*LL_GPIO_PIN_4 |*/ LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = GPIO_AF5_SPI1
};

bool SPICoaxTransceiver::spiSetup()
{
    // TODO: Move this to STM32 LL...
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_Init(GPIOA, &gpioInit);

    // pin setup...
    LL_SPI_Disable(spi);
    LL_SPI_DeInit(spi);

    // ...
    LL_SPI_SetRxFIFOThreshold(spi, LL_SPI_RX_FIFO_TH_QUARTER);

    LL_SPI_Init(spi, &spiConfig);
    LL_SPI_Enable(spi);

    return true;
}

bool SPICoaxTransceiver::spiTransfer(const uint8_t *transmitBuffer,
        uint8_t *receiveBuffer, const size_t count)
{
    // flush
    /*
    while (LL_SPI_IsActiveFlag_RXNE(spi)) {
         LL_SPI_ReceiveData8(spi);
    }
    */

    for (size_t index = 0; index < count; index++) {
        while (!(SPI1->SR & SPI_SR_TXE)) {
            // timeout...
        }

        LL_SPI_TransmitData8(spi, transmitBuffer[index]);

        // Wait until the transmission is complete
        //while (SPI1->SR & SPI_SR_BSY) {
            // timeout...
        //}

        while (!LL_SPI_IsActiveFlag_RXNE(spi)) {
            // tiemout...
        }

        receiveBuffer[index] = LL_SPI_ReceiveData8(spi);
    }


    return true;
}
