/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU        AT32F435G

#define BOARD_NAME           AOCODAF435V2MPU6500
#define MANUFACTURER_ID      SJET

#define SYSTEM_HSE_MHZ 8

/**********************************************************************
 * SPI Bus                                                            *
 **********************************************************************/

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN         PA4
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN         PA13
#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN         PC0
#define SPI3_SCK_PIN         PB3
#define SPI3_SDI_PIN         PB4
#define SPI3_SDO_PIN         PB5

/**********************************************************************
 * I2C Bus                                                            *
 **********************************************************************/

#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7

/**********************************************************************
 * UART Bus                                                           *
 **********************************************************************/

#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10

#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3
#define RX_PPM_PIN           PA3

#define UART3_TX_PIN         PC10
#define UART3_RX_PIN         PC11
#define ESCSERIAL_PIN        PC11

#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1

#define UART5_TX_PIN         PC12
#define UART5_RX_PIN         PD2

/**********************************************************************
 * Motor                                                              *
 **********************************************************************/

#define MOTOR1_PIN           PC6
#define MOTOR2_PIN           PC7
#define MOTOR3_PIN           PC8
#define MOTOR4_PIN           PC9
#define MOTOR5_PIN           PA15
#define MOTOR6_PIN           PA8
#define MOTOR7_PIN           PB10
#define MOTOR8_PIN           PB11

/**********************************************************************
 * USB                                                                *
 **********************************************************************/

#define USB_DETECT_PIN       PB12
 
/**********************************************************************
 * LED & LED strip & Buzzer                                           *
 **********************************************************************/

#define LED0_PIN             PC13
#define LED_STRIP_PIN        PB1

#define BEEPER_PIN           PB8
#define BEEPER_INVERTED

/**********************************************************************
 * ADC                                                                *
 **********************************************************************/

#define ADC_VBAT_PIN         PC2
#define ADC_RSSI_PIN         PC3
#define ADC_CURR_PIN         PC1

#define ADC_INSTANCE         ADC3
#define ADC1_DMA_OPT         11

/**********************************************************************
 * Timer                                                              *
 **********************************************************************/

/* Reserve for debug
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN    , 2,  0 )  \
    TIMER_PIN_MAP( 1, MOTOR2_PIN    , 2,  2 )  \
    TIMER_PIN_MAP( 2, MOTOR3_PIN    , 2,  1 )  \
    TIMER_PIN_MAP( 3, MOTOR4_PIN    , 2,  3 )  \
    TIMER_PIN_MAP( 4, MOTOR5_PIN    , 1,  11)  \
    TIMER_PIN_MAP( 5, MOTOR6_PIN    , 1,  10)  \
    TIMER_PIN_MAP( 6, MOTOR7_PIN    , 1,  8 )  \
    TIMER_PIN_MAP( 7, MOTOR8_PIN    , 1,  9 )  \
    TIMER_PIN_MAP( 8, LED_STRIP_PIN , 2,  7 )  \
    TIMER_PIN_MAP( 9, RX_PPM_PIN    , 2,  6 )
*/

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PC6     , 2,  0)  \
    TIMER_PIN_MAP( 1, PC7     , 2,  2)  \
    TIMER_PIN_MAP( 2, PC8     , 2,  1)  \
    TIMER_PIN_MAP( 3, PC9     , 2,  3)  \
    TIMER_PIN_MAP( 4, PA15    , 1,  11) \
    TIMER_PIN_MAP( 5, PA8     , 1,  10) \
    TIMER_PIN_MAP( 6, PB10    , 1,  8)  \
    TIMER_PIN_MAP( 7, PB11    , 1,  9)  \
    TIMER_PIN_MAP( 8, PB1     , 2,  7)  \
    TIMER_PIN_MAP( 9, PA3     , 2,  6)

/**********************************************************************
 * Gyro MPU6500                                                       *
 **********************************************************************/

#define USE_SPI_GYRO
#define GYRO_1_SPI_INSTANCE  SPI1

#define GYRO_1_EXTI_PIN      PC4
#define GYRO_1_CS_PIN        SPI1_NSS_PIN
#define GYRO_1_ALIGN         CW0_DEG

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500
#define USE_ACC
#define USE_ACC_SPI_MPU6500

/**********************************************************************
 * OSD MAX7456                                                        *
 **********************************************************************/

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI2
#define MAX7456_SPI_CS_PIN   SPI2_NSS_PIN

/**********************************************************************
 * Flash 16MB W25Q128                                                 *
 **********************************************************************/

#define USE_FLASH
#define FLASH_SPI_INSTANCE   SPI3
#define FLASH_CS_PIN         SPI3_NSS_PIN

//#define USE_FLASH_M25P16
#define USE_FLASH_W25Q128FV
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH

/**********************************************************************
 * Baro BMP280/DPS310 & Mag HMC5883/QMC5883                           *
 **********************************************************************/

#define USE_BARO
#define BARO_I2C_INSTANCE   (I2CDEV_1)
#define USE_BARO_BMP280
#define USE_BARO_DPS310

#define USE_MAG
#define MAG_I2C_INSTANCE    (I2CDEV_1)
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883

/**********************************************************************
 * Default settings                                                   *
 **********************************************************************/

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_OFF
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 500

/**********************************************************************
 * Debug                                                              *
 **********************************************************************/

//#define USE_TIMER_MAP_PRINT 

