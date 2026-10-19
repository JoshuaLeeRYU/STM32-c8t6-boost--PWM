#include "INA219.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>



#define INA219_ADDRESS                            0x45 /* INA219 传感器的设备地址 */
 
#define INA219_REG_CONFIG                         0x00 /* 配置寄存器的地址 */
#define INA219_REG_SHUNTVOLTAGE                   0x01 /* 分流电压寄存器的地址 */
#define INA219_REG_BUSVOLTAGE                     0x02 /* 总线电压寄存器的地址 */
#define INA219_REG_POWER                          0x03 /* 功率寄存器的地址 */
#define INA219_REG_CURRENT                        0x04 /* 电流寄存器的地址 */
#define INA219_REG_CALIBRATION                    0x05 /* 校准寄存器的地址 */
 
#define INA219_CONFIG_RESET                       0x8000 /* 复位位的配置值 */
 
#define INA219_CONFIG_BVOLTAGERANGE_16V           0x0000 /* 电压测量范围的配置值 0 - 16 V */
#define INA219_CONFIG_BVOLTAGERANGE_32V           0x2000 /* 电压测量范围的配置值 0 - 32 V  */
 
#define INA219_CONFIG_GAIN_1_40MV                 0x0000 /* 增益和分辨率的配置值 增益为1，电流测量范围为40mV*/
#define INA219_CONFIG_GAIN_2_80MV                 0x0800 /* 增益和分辨率的配置值 增益为2，电流测量范围为80mV*/
#define INA219_CONFIG_GAIN_4_160MV                0x1000 /* 增益和分辨率的配置值 增益为4，电流测量范围为160mV*/
#define INA219_CONFIG_GAIN_8_320MV                0x1800 /* 增益和分辨率的配置值 增益为8，电流测量范围为320mV*/
 
#define INA219_CONFIG_BADCRES_9BIT                0x0000 /* 总线电压ADC分辨率的配置值 范围为0到511 */
#define INA219_CONFIG_BADCRES_10BIT               0x0080 /* 总线电压ADC分辨率的配置值 范围为0到1023 */
#define INA219_CONFIG_BADCRES_11BIT               0x0100 /* 总线电压ADC分辨率的配置值 范围为0到2047 */
#define INA219_CONFIG_BADCRES_12BIT               0x0180 /* 总线电压ADC分辨率的配置值 范围为0到4097 */
#define INA219_CONFIG_BADCRES_12BIT_2S_1060US     0x0480 /* 总线电压ADC采样和平均化的配置值 2次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为1060微秒*/
#define INA219_CONFIG_BADCRES_12BIT_4S_2130US     0x0500 /* 总线电压ADC采样和平均化的配置值 4次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为2130微秒*/
#define INA219_CONFIG_BADCRES_12BIT_8S_4260US     0x0580 /* 总线电压ADC采样和平均化的配置值 8次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为4260微秒*/
#define INA219_CONFIG_BADCRES_12BIT_16S_8510US    0x0600 /* 总线电压ADC采样和平均化的配置值 16次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为8510微秒*/
#define INA219_CONFIG_BADCRES_12BIT_32S_17MS      0x0680 /* 总线电压ADC采样和平均化的配置值 32次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为17毫秒*/
#define INA219_CONFIG_BADCRES_12BIT_64S_34MS      0x0700 /* 总线电压ADC采样和平均化的配置值 64次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为34微秒*/
#define INA219_CONFIG_BADCRES_12BIT_128S_69MS     0x0780 /* 总线电压ADC采样和平均化的配置值 128次12位总线电压采样，然后对它们进行平均化，整个过程耗时约为69微秒*/
 
#define INA219_CONFIG_SADCRES_9BIT_1S_84US        0x0000 /* 电流采样分辨率的配置值 1次9位电流采样，整个过程耗时约为84微秒*/
#define INA219_CONFIG_SADCRES_10BIT_1S_148US      0x0008 /* 电流采样分辨率的配置值 1次10位电流采样，整个过程耗时约为148微秒*/
#define INA219_CONFIG_SADCRES_11BIT_1S_276US      0x0010 /* 电流采样分辨率的配置值 1次11位电流采样，整个过程耗时约为276微秒*/
#define INA219_CONFIG_SADCRES_12BIT_1S_532US      0x0018 /* 电流采样分辨率的配置值 1次12位电流采样，整个过程耗时约为532微秒*/
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US     0x0048 /* 电流采样和平均化的配置值 2次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为1060微秒*/
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US     0x0050 /* 电流采样和平均化的配置值 4次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为2130微秒*/
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US     0x0058 /* 电流采样和平均化的配置值 8次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为4260微秒*/
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US    0x0060 /* 电流采样和平均化的配置值 16次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为8510微秒*/
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS      0x0068 /* 电流采样和平均化的配置值 32次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为17微秒*/
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS      0x0070 /* 电流采样和平均化的配置值 54次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为34微秒*/
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS     0x0078 /* 电流采样和平均化的配置值 128次12位总线电流采样，然后对它们进行平均化，整个过程耗时约为69微秒*/
 
#define INA219_CONFIG_MODE_MASK                   0x07 /* 掩码，用于提取配置寄存器中的模式位*/
#define INA219_CONFIG_MODE_POWERDOWN              0x00 /* INA219 传感器处于电源关闭模式*/
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED        0x01 /* INA219 传感器在检测到电流变化时进行一次瞬态的电压测量*/
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED        0x02 /* INA219 传感器在检测到总线电压变化时进行一次瞬态的电压测量*/
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED    0x03 /* INA219 传感器在检测到电流或总线电压变化时进行一次瞬态的电压测量*/
#define INA219_CONFIG_MODE_ADCOFF                 0x04 /* INA219 传感器的ADC关闭，不进行任何电流或电压测量*/
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS       0x05 /* INA219 传感器以连续模式进行电流测量，并持续地测量瞬态的电压*/
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS       0x06 /* INA219 传感器以连续模式进行总线电压测量，并持续地测量瞬态的电压*/
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS   0x07 /* INA219 传感器以连续模式进行电流和总线电压测量，并持续地测量瞬态的电压*/

void INA219_Write16(uint8_t Register, uint16_t Value)
{
    uint8_t value[2];
    value[0] = (Value >> 8) & 0xff;  // 高字节在前
    value[1] = (Value >> 0) & 0xff;  // 低字节在后
    HAL_I2C_Mem_Write(&hi2c1, (INA219_ADDRESS << 1), Register, I2C_MEMADD_SIZE_8BIT, (uint8_t *)value, 2,50);
}

uint16_t INA219_Read16(uint8_t Register)
{
    uint8_t Value[2];
 
    HAL_StatusTypeDef Status=HAL_I2C_Mem_Read(&hi2c1, (INA219_ADDRESS << 1), Register, I2C_MEMADD_SIZE_8BIT, Value, 2, 50);
    if (Status != HAL_OK) {
        return 65535;
    }
    return ((Value[0] << 8) | Value[1]);
}

void INA219_Reset(void)
{
    INA219_Write16(INA219_REG_CONFIG, INA219_CONFIG_RESET);
    HAL_Delay(5);
}

void INA219_setCalibration(uint16_t CalibrationData)
{
    INA219_Write16(INA219_REG_CALIBRATION, CalibrationData);
}

void INA219_setConfig(uint16_t Config)
{
    HAL_Delay(20);
    INA219_Write16(INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_16V_400mA(void)
{
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
 
    uint16_t ina219_calibrationValue = 33553;
 
    INA219_setCalibration(ina219_calibrationValue);
    INA219_setConfig(config);
}

uint8_t INA219_Init(void)
{
    HAL_Delay(20);
    uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(&hi2c1, (INA219_ADDRESS << 1), 3, 2);
 
    if(ina219_isReady == HAL_OK)
    {
        INA219_Reset();
        INA219_setCalibration_16V_400mA();
 
        return 0;
    }
 
    else
    {
        return 1;
    }
}

uint32_t INA219_ReadBusVoltage(void)
{
    uint16_t BusVoltage = INA219_Read16(INA219_REG_BUSVOLTAGE);
    if (BusVoltage == 65535) {
        return 65535; // 返回65535表示读取失败
    }
    uint32_t v = ((BusVoltage >> 3) * 4);
    return v;
}

uint16_t INA219_ReadCurrent(void)
{
    uint16_t Current = INA219_Read16(INA219_REG_CURRENT);
    if (Current == 65535) {
        return 65535; // 返回65535表示读取失败
    }
 
    return Current; 
}
