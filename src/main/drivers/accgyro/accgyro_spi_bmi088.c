/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "accgyro.h"
#include "accgyro_spi_bmi088.h"

#define BMI088_MAX_SPI_CLK_HZ 10000000
#define GYRO_EXTI_DETECT_THRESHOLD 1000

#define BMI088_REG_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID 0x0F
#define BMI088_REG_GYRO_RANGE 0x0F
#define BMI088_REG_GYRO_BANDWIDTH 0x10
#define BMI088_REG_GYRO_RATE_DATA 0x02
#define BMI088_REG_GYRO_SOFTRESET 0x14
#define BMI088_REG_GYRO_INT_CTRL 0x15
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_REG_GYRO_SELF_TEST 0x3C
#define BMI088_EN_DRDY_INT 0x80

#define BMI088_REG_ACC_CHIP_ID 0x00
#define BMI088_ACC_CHIP_ID 0x1E
#define BMI088_REG_ACC_CONF 0x40
#define BMI088_REG_ACC_RANGE 0x41
#define BMI088_REG_ACC_PWR_CONF 0x7C
#define BMI088_REG_ACC_PWR_CTRL 0x7D
#define BMI088_REG_ACC_SOFTRESET 0x7E
#define BMI088_REG_ACC_DATA 0x12
#define BMI088_TRIGGER_SOFTRESET 0xB6

enum bmi088_accc_range{
    BMI088_A_RANGE_3G = 0x00,
    BMI088_A_RANGE_6G = 0x01,
    BMI088_A_RANGE_12G = 0x02,
    BMI088_A_RANGE_24G = 0x03,
};
enum bmi088_acc_pwr_save{
    BMI088_A_ACTIVE = 0x00,
    BMI088_A_SUSPEND = 0x03,
};
enum bmi088_acc_enable{
    BMI088_A_OFF = 0x00,
    BMI088_A_ON = 0x04,
};
enum bmi088_acc_bwp{
    BMI088_A_BWP_OSR4 = 0x00,
    BMI088_A_BWP_OSR2 = 0x01,
    BMI088_A_BWP_NORMAL = 0x02,
};
enum bmi088_acc_odr{
    BMI088_A_ODR_12_5 = 0x05,
    BMI088_A_ODR_25 = 0x06,
    BMI088_A_ODR_50 = 0x07,
    BMI088_A_ODR_100 = 0x08,
    BMI088_A_ODR_200 = 0x09,
    BMI088_A_ODR_400 = 0x0A,
    BMI088_A_ODR_800 = 0x0B,
    BMI088_A_ODR_1600 = 0x0C,
};
enum bmi088_gyro_bandwidth{
    BMI088_G_ODR_2000HZ_BW_532HZ = 0x80,
    BMI088_G_ODR_2000HZ_BW_230HZ = 0x81,
    BMI088_G_ODR_1000HZ_BW_116HZ = 0x82,
    BMI088_G_ODR_400HZ_BW_47HZ = 0x83,
    BMI088_G_ODR_200HZ_BW_23HZ = 0x84,
    BMI088_G_ODR_100HZ_BW_12HZ = 0x85,
    BMI088_G_ODR_200HZ_BW_64HZ = 0x86,
    BMI088_G_ODR_100HZ_BW_32HZ = 0x87,
};
enum bmi088_gyro_range {
    BMI088_G_RANGE_125DPS = 0x04,
    BMI088_G_RANGE_250DPS = 0x03,
    BMI088_G_RANGE_500DPS = 0x02,
    BMI088_G_RANGE_1000DPS = 0x01,
    BMI088_G_RANGE_2000DPS = 0x00,
};

void bmi088ExtiHandler(extiCallbackRec_t *cb);
void bmi088SpiGyroInit(gyroDev_t *gyro);
bool bmi088GyroRead(gyroDev_t *gyro);
bool bmi088SpiGyroDetect(gyroDev_t *gyro);
bool bmi088AccRead(accDev_t *acc);
uint8_t bmi088spiBusReadRegisterAcc(const extDevice_t *dev, const uint8_t reg);
bool bmi088SpiAccDetect(accDev_t *acc);
void bmi088SpiAccInit(accDev_t *acc);
static volatile bool BMI088GyroDetected = false;
static volatile bool BMI088AccDetected = false;
static DMA_DATA uint8_t accBuf[32];

/*
 * Gyro interrupt service routine - called when DRDY pin goes high
 * This is the critical path for achieving low latency gyro reads
 */
void bmi088ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    extDevice_t *dev = &gyro->dev;

    // Record timestamps for gyro synchronization
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    // In DMA mode, start the SPI transfer immediately in the ISR
    // This gives the lowest latency between data ready and data capture
    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(dev, gyro->segments);
    }

    // Count interrupts for detection during initialization
    gyro->detectedEXTI++;
}

void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    
    // softreset
    spiWriteReg(dev, BMI088_REG_GYRO_SOFTRESET, BMI088_TRIGGER_SOFTRESET);
    delay(50);  // Wait 50ms after soft reset per datasheet

    // config sensor range
    spiWriteReg(dev, BMI088_REG_GYRO_RANGE, BMI088_G_RANGE_2000DPS);
    delay(1);

    uint8_t odrConfig = 0;

    switch (gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            // ODR: 2kHz, BW: 230Hz
            odrConfig = BMI088_G_ODR_2000HZ_BW_230HZ;
            break;
        case GYRO_HARDWARE_LPF_OPTION_1:
            // ODR: 2kHz, BW: 532Hz
            odrConfig = BMI088_G_ODR_2000HZ_BW_532HZ;
            break;
        case GYRO_HARDWARE_LPF_OPTION_2:
            // ODR: 1kHz, BW: 116Hz
            odrConfig = BMI088_G_ODR_1000HZ_BW_116HZ; 
            break;
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            // ODR: 400Hz, BW: 47Hz
            odrConfig = BMI088_G_ODR_400HZ_BW_47HZ;
            break; 
#endif        
        default:
            odrConfig = BMI088_G_ODR_2000HZ_BW_230HZ;
            break;
    }
    spiWriteReg(dev, BMI088_REG_GYRO_BANDWIDTH, odrConfig);
    delay(1);

    // enable dataready interrupt
    spiWriteReg(dev, BMI088_REG_GYRO_INT_CTRL, BMI088_EN_DRDY_INT);

    // INT3: push-pull, active high
    spiWriteReg(dev, BMI088_REG_GYRO_INT3_INT4_IO_CONF, 0x01);

    // DRDY int is mapped to INT3 pin
    spiWriteReg(dev, BMI088_REG_GYRO_INT3_INT4_IO_MAP, 0x01);

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi088ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING); // TODO - maybe pullup / pulldown ?
    EXTIEnable(mpuIntIO);
}

extiCallbackRec_t bmi088IntCallbackRec;

busStatus_e bmi088Intcallback(uintptr_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

bool bmi088GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(dev->txBuf, 0x00, 8);

        // Check that minimum number of interrupts have been detected
        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uint32_t)gyro;
                dev->txBuf[1] = BMI088_REG_GYRO_RATE_DATA | 0x80;
                gyro->segments[0].len = 8;
                gyro->segments[0].callback = bmi088Intcallback;
                gyro->segments[0].u.buffers.txData = dev->txBuf;
                gyro->segments[0].u.buffers.rxData = dev->rxBuf;
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = BMI088_REG_GYRO_RATE_DATA | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);

        // Wait for completion
        spiWait(dev);

        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        int16_t *gyroData = (int16_t *)&dev->rxBuf[1];//first byte is the register address
        gyro->gyroADCRaw[X] = gyroData[0];
        gyro->gyroADCRaw[Y] = gyroData[1];
        gyro->gyroADCRaw[Z] = gyroData[2];
        break;
    }

    default:
        break;
    }

    return true;
}

uint8_t bmi088SpiDetect(const extDevice_t *dev)
{
	if (BMI088GyroDetected) {
        return BMI_088_SPI;
    }

	spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));

	if (spiReadReg(dev, BMI088_REG_GYRO_CHIP_ID | 0x80) != BMI088_GYRO_CHIP_ID) {
		return MPU_NONE;
	}

	BMI088GyroDetected = true;

	return BMI_088_SPI;
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_SPI) {
        return false;
    }

    // Skip self-test for now - just verify chip ID and configure
    // Self-test can cause issues during detection phase
    
    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}


bool bmi088AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        // BMI088 ACC SPI read format:
        // TX: [addr|0x80][dummy][dummy][dummy][dummy][dummy][dummy][dummy]
        // RX: [???][dummy][X_L][X_H][Y_L][Y_H][Z_L][Z_H]
        // BMI088 accelerometer requires an extra dummy byte after address
        memset(dev->txBuf, 0x00, 9);

        dev->txBuf[0] = BMI088_REG_ACC_DATA | 0x80;

        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 9, true, NULL},  // 1(addr) + 1(dummy) + 1(acc dummy) + 6(data)
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = dev->rxBuf;

        spiSequence(dev, &segments[0]);

        // Wait for completion
        spiWait(dev);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // BMI088 ACC data starts at rxBuf[2] (after addr echo + dummy byte)
        int16_t *accData = (int16_t *)&dev->rxBuf[2];
        acc->ADCRaw[X] = accData[0];
        acc->ADCRaw[Y] = accData[1];
        acc->ADCRaw[Z] = accData[2];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

uint8_t bmi088spiBusReadRegisterAcc(const extDevice_t *dev, const uint8_t reg)
{
    uint8_t data[2] = { 0 };

    if (spiReadRegMskBufRB(dev, reg, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

void bmi088SpiAccInit(accDev_t *acc)
{
    // Step 1: Soft reset
    spiWriteReg(&acc->dev, BMI088_REG_ACC_SOFTRESET, BMI088_TRIGGER_SOFTRESET);
    delay(50);  // Wait 50ms after soft reset

    // Step 2: Dummy read to switch to SPI mode (required after reset)
    bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID);
    delay(1);

    // Step 3: Enable accelerometer (write 0x04 to ACC_PWR_CTRL)
    // From datasheet: After power up, write 0x04 to enable sensor
    spiWriteReg(&acc->dev, BMI088_REG_ACC_PWR_CTRL, BMI088_A_ON);
    delay(5);  // Wait for power on

    // Step 4: Enter active mode (write 0x00 to ACC_PWR_CONF)
    spiWriteReg(&acc->dev, BMI088_REG_ACC_PWR_CONF, BMI088_A_ACTIVE);
    delay(50);  // Wait 50ms after mode change per datasheet

    // Verify accelerometer is enabled
    for (uint8_t i = 0; i < 5; i++) {
        if (bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_PWR_CTRL) == BMI088_A_ON) {
            break;
        }
        delay(5);
    }

    // Step 5: Configure range (12G for flight controller use)
    spiWriteReg(&acc->dev, BMI088_REG_ACC_RANGE, BMI088_A_RANGE_12G);
    delay(1);

    // Step 6: Configure ODR and bandwidth
    // Format: 0x80 | (BWP << 4) | ODR
    // BWP=0x02 (normal), ODR=0x0B (800Hz) -> 0x80 | 0x20 | 0x0B = 0xAB
    spiWriteReg(&acc->dev, BMI088_REG_ACC_CONF, 0x80 | (BMI088_A_BWP_NORMAL << 4) | BMI088_A_ODR_800);
    delay(1);

    // acc_1G = 32768 / 12 = 2731 (for 12G range)
    acc->acc_1G = 2731;
}

bool bmi088SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_088_SPI){
      return false;
    }
    
    //ACC part uses the same SPI bus as the gyro, so we can just use the gyro's spi instance
    spiSetBusInstance(&acc->dev, SPI_DEV_TO_CFG(spiDeviceByInstance(acc->gyro->dev.bus->busType_u.spi.instance)));
    acc->dev.busType_u.spi.csnPin = acc->gyro->csnAccPin; //get the CS pin from the gyro device config
    acc->dev.txBuf = accBuf;
    acc->dev.rxBuf = &accBuf[32 / 2];

    spiSetClkDivisor(&acc->dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));

    // perform dummy-read to switch the accel to SPI-mode
    bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID);
    delay(5);

    if (bmi088spiBusReadRegisterAcc(&acc->dev, BMI088_REG_ACC_CHIP_ID) != BMI088_ACC_CHIP_ID) {
      return false;
    }

    //TODO: check  ACC_ERR_REG

    BMI088AccDetected = true;

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = bmi088AccRead;

    busDeviceRegister(&acc->dev);

    return true;
}

#endif // USE_ACCGYRO_BMI088