/*
 * This program turns on the 4 leds of the stm32f4 discovery board
 * one after another.
 * It defines shortcut definitions for the led pins and
 * stores the order of the leds in an array which is being
 * iterated in a loop.
 *
 * This program is free human culture like poetry, mathematics
 * and science. You may use it as such.
 */

/* stm32f4_discovery.h is located in Utilities/STM32F4-Discovery
 * and defines the GPIO Pins where the leds are connected.
 * Including this header also includes stm32f4xx.h and
 * stm32f4xx_conf.h, which includes stm32f4xx_gpio.h
 */
#include <stm32f4_discovery.h>

#include "mpu6050.h"


/*
  The 7-bit address of the I2C slave.
  For the MPU6050, this is 0x68 if the AD0 pin is held low, and 0x69 if the
  AD0 pin is held high.
*/
#define MPU6050_I2C_ADDR 0x68


static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART2 TX on PA2 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART2 pins to AF2 */
  // TX = PA2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE); // enable USART2
}


static void
serial_putchar(USART_TypeDef* USARTx, uint32_t c)
{
  while(!(USARTx->SR & USART_FLAG_TC));
  USART_SendData(USARTx, c);
}


static void
serial_puts(USART_TypeDef *usart, const char *s)
{
  while (*s)
    serial_putchar(usart, (uint8_t)*s++);
}


static void
serial_output_hexdig(USART_TypeDef* USARTx, uint32_t dig)
{
  serial_putchar(USARTx, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


static void
serial_output_hexbyte(USART_TypeDef* USARTx, uint8_t byte)
{
  serial_output_hexdig(USARTx, byte >> 4);
  serial_output_hexdig(USARTx, byte & 0xf);
}


static void
setup_i2c_for_mpu6050()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;

  /* Use I2C1, with SCL on PB8 and SDA on PB9. */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

  I2C_InitStruct.I2C_ClockSpeed = 400000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStruct);

  I2C_Cmd(I2C1, ENABLE);
}


static uint8_t
read_mpu6050_reg(uint8_t reg)
{
  uint8_t val;

  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  I2C_SendData(I2C1, reg);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Receiver);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  I2C_AcknowledgeConfig(I2C1, DISABLE);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  val = I2C_ReceiveData(I2C1);

  I2C_GenerateSTOP(I2C1, ENABLE);

  return val;
}


static void
write_mpu6050_reg(uint8_t reg, uint8_t val)
{
  I2C_GenerateSTART(I2C1, ENABLE);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    ;

  I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    ;

  I2C_SendData(I2C1, reg);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  I2C_SendData(I2C1, val);
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    ;

  I2C_GenerateSTOP(I2C1, ENABLE);
}


static void
setup_mpu6050(void)
{
  for (;;)
  {
    uint8_t res;

    /*
      First take it out of sleep mode (writes seem to not stick until we take it
      out of sleep mode). Then issue a reset to get a well-defined starting state
      (and go out of sleep mode again).
    */
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x02);
    delay(300000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    if (res != 0x02)
      continue;
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x82);
    delay(300000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    if (res != 0x40)
      continue;
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x02);
    delay(300000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    if (res != 0x02)
      continue;

    /* Disable digital low-pass filter (DLPF) */
    write_mpu6050_reg(MPU6050_REG_CONFIG, 0);
    /* 1000 Hz sample rate. */
    write_mpu6050_reg(MPU6050_REG_SMPRT_DIV, 7);
    /* Lowest resolution, +-2000 degrees / second and +-16g. */
    write_mpu6050_reg(MPU6050_REG_GYRO_CONFIG, 3 << 3);
    write_mpu6050_reg(MPU6050_REG_ACCEL_CONFIG, 3 << 3);
    /* Disable the Fifo (write 0xf8 to enable temp+gyros_accel). */
    write_mpu6050_reg(MPU6050_REG_FIFO_EN, 0x00);
    /*
      Interrupt. Active high, push-pull, hold until cleared, cleared only on
      read of status.
    */
    write_mpu6050_reg(MPU6050_REG_INT_PIN_CFG, 0x20);
    /* Enable FIFO overflow and data ready interrupts. */
    write_mpu6050_reg(MPU6050_REG_INT_ENABLE, 0x11);
    /* Disable the FIFO and external I2C master mode. */
    write_mpu6050_reg(MPU6050_REG_USER_CTRL, 0x00);

    break;
  }
}


int main(void)
{
  uint8_t res;

  delay(2000000);
  setup_serial();
  setup_i2c_for_mpu6050();
  delay(2000000);
  setup_mpu6050();

  serial_puts(USART2, "Hello world, ready to blink!\r\n");

  res = read_mpu6050_reg(MPU6050_REG_WHOAMI);
  serial_puts(USART2, "\r\nMPU6050: whoami=0x");
  serial_output_hexbyte(USART2, res);
  res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
  serial_puts(USART2, "\r\nMPU6050: pwr_mgmt_1=0x");
  serial_output_hexbyte(USART2, res);

  while (1)
  {
    int16_t temp_data;
    uint8_t high, low;

    serial_puts(USART2, "\r\nRead sensors ...");
    temp_data = (int16_t)(
      ((uint16_t)read_mpu6050_reg(MPU6050_REG_TEMP_OUT_H) << 8) |
      read_mpu6050_reg(MPU6050_REG_TEMP_OUT_L));
    serial_puts(USART2, " temp_h=0x");
    serial_output_hexbyte(USART2, (temp_data >> 8));
    serial_puts(USART2, " temp_l=0x");
    serial_output_hexbyte(USART2, (temp_data & 0xff));
    serial_puts(USART2, "\r\n");

    high = read_mpu6050_reg(MPU6050_REG_ACCEL_XOUT_H);
    low = read_mpu6050_reg(MPU6050_REG_ACCEL_XOUT_L);
    serial_puts(USART2, "Accel_x=0x");
    serial_output_hexbyte(USART2, high);
    serial_output_hexbyte(USART2, low);
    serial_puts(USART2, "\r\n");

    high = read_mpu6050_reg(MPU6050_REG_ACCEL_YOUT_H);
    low = read_mpu6050_reg(MPU6050_REG_ACCEL_YOUT_L);
    serial_puts(USART2, "Accel_y=0x");
    serial_output_hexbyte(USART2, high);
    serial_output_hexbyte(USART2, low);
    serial_puts(USART2, "\r\n");

    high = read_mpu6050_reg(MPU6050_REG_ACCEL_ZOUT_H);
    low = read_mpu6050_reg(MPU6050_REG_ACCEL_ZOUT_L);
    serial_puts(USART2, "Accel_z=0x");
    serial_output_hexbyte(USART2, high);
    serial_output_hexbyte(USART2, low);
    serial_puts(USART2, "\r\n");

    delay(30000000);
  }

  return 0;
}
