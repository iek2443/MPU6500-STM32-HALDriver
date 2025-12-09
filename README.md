# MPU6500-STM32-HALDriver

A lightweight, modular and well-documented driver for the **MPU-6500** 6-axis accelerometer and gyroscope sensor using the **STM32 HAL** I2C interface.

---

## ✨ Features

- ✅ Reads **accelerometer** data (X, Y, Z axes)
- ✅ Reads **gyroscope** data (X, Y, Z axes)
- ✅ Reads **on-chip temperature sensor**
- ✅ Power mode control (sleep, normal, low-power accelerometer-only)
- ✅ Sample rate and Digital Low Pass Filter (DLPF) configuration
- ✅ Full-scale range configuration for both accelerometer and gyroscope
- ✅ Axis standby (per-axis enable/disable) support
- ✅ Device ID verification via `WHO_AM_I`
- ✅ Compatible with all STM32 series via HAL
- ✅ Clean, modular C code with Doxygen-style documentation

---

## 🔧 Requirements

- STM32 HAL library  
  (e.g. `stm32f4xx_hal.h`, `stm32g0xx_hal.h`, `stm32f1xx_hal.h`, etc.)
- I2C peripheral enabled and configured
- MPU6500 connected to an I2C bus  
  - Default 7-bit address: `0x68` (AD0 = GND) or `0x69` (AD0 = VCC)
---

## ⚙️ Configuration

You must configure the I2C port and the MPU6500 I2C address inside the `.h` file.

### 1️⃣ Include your MCU-specific HAL header

```c
#include "stm32f4xx_hal.h"   // Change this according to your MCU family
```

### 2️⃣ Declare the I2C handle used by the MPU6500

```c
extern I2C_HandleTypeDef hi2c1;
```
If you use another I2C peripheral (e.g., hi2c2), simply change it.

### 3️⃣ Select which I2C port the driver should use

```c
#define mpu_port   &hi2c1
```
If you use another I2C peripheral (e.g., hi2c2), simply change it.

### 4️⃣ Configure the I2C address using the AD0 pin

If the AD0 pin is connected to VCC, the AD0_ON macro must be defined; if it is connected to GND, no configuration is required. 
By default, this driver does not define AD0_ON.
```c
#define AD0_ON //If the AD0 pin is connected to VCC, the AD0_ON macro must be defined
//#define AD0_ON //If the AD0 pin is connected to GND, the AD0_ON macro must not be defined.
/**
 * @brief MPU6500 I2C address selection.
 *
 * If AD0 is tied to GND → address = 0x68
 * If AD0 is tied to VCC → define AD0_ON → address = 0x69
 */
#ifdef AD0_ON
#define MPU6500_ADDR   (0x69 << 1)
#else
#define MPU6500_ADDR   (0x68 << 1)
#endif

```

## 🚀 Usage Example
```c
#include "Mpu_6500.h"

MPU6500_RawData_t data;

int main(void) {
    HAL_Init();
    MX_I2C1_Init(); // Initialize your I2C peripheral

    MPU6500_Init();

    while (1) {

        MPU6500_Read_Raw(&data, MPU_READ_ALL_SENSORS);   // Read and store all data
        HAL_Delay(100);
    }
}
```
