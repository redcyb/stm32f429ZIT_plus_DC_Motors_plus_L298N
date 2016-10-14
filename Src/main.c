#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050.h"

I2C_HandleTypeDef hi2c3;

int main(void)
{
    SystemInit();
    HAL_Init();
    
    GPIO_InitTypeDef GPIO_InitStruct;

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */

    __GPIOA_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __I2C3_CLK_ENABLE();

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0x10;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c3.Init.OwnAddress2 = 0x11;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_I2C_Init(&hi2c3);

    I2Cdev_init(&hi2c3); // init of i2cdevlib.  
    // You can select other i2c device anytime and 
    // call the same driver functions on other sensors

    printf("Before circle \n");
    
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    int16_t c_ax, c_ay, c_az;
    int16_t c_gx, c_gy, c_gz;

    MPU6050_initialize();
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    MPU6050_getMotion6(&c_ax, &c_ay, &c_az, &c_gx, &c_gy, &c_gz);
    
    while (1)
    {
        MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        printf("Accel: %d    %d    %d\n", ax - c_ax, ay - c_ay, az - c_az);
        printf("Gyro: %d    %d    %d\n\n", gx - c_gx, gy - c_gy, gz - c_gz);
        
        HAL_Delay(1000);
    }
}

void SysTick_Handler()
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}