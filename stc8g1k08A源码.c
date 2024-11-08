/**
 * @file 文件复制到 天问block 中编译, 编译成功后使用 stc isp 下载到STC8G1K08A芯片中
 * @author b站-仁泉之子
 * @brief
 * @version 1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 */
#include <lint.h>
#include <stdio.h>
#include <string.h>
#include <STC8HX.h>
uint32 sys_clk = 24000000; // 设置PWM、定时器、串口、EEPROM频率参数
#include "lib/ADC.h"
#include "lib/UART.h"
#include "lib/delay.h"
#define boolean unsigned char
#define true 1
#define false 0

#define ADC_PIN ADC_P33
#define SHUT_PIN P3_2
#define N_SHUT_PIN P5_4
#define SHUT_IN_PIN P5_5

// 4.7k 4.7k 电压分压, 电源3.3v 测量范围0-6.6v
// 基准3.0v, 触发电压3.8v
uint16 adcValue = 0;
// V/6.6 = adcValue/1024
uint16 trigAdcValue = 590; // 限制为3.8V
boolean isShut = false;
uint8 cmdData[8] = {0};
uint8 tempData = 0;

// ---------------------- 串口发送接收 ----------------------
// FIFO环形缓冲队列
typedef struct FIFOBuffer
{
  unsigned char headPos;    // 缓冲区头部位置
  unsigned char tailPos;    // 缓冲区尾部位置
  unsigned char bufferSize; // 缓冲区长度
  unsigned char *buffer;    // 缓冲区数组
};
#define UartReceiveSize 16
#define UartSendSize 16
unsigned char uartReceiveBuffer[UartReceiveSize] = {0};
unsigned char uartSendBuffer[UartSendSize] = {0};
struct FIFOBuffer uartReceiveFIFO; // 接收缓冲区
struct FIFOBuffer uartSendFIFO;    // 发送缓冲区
unsigned char isSend = 0;

unsigned char FIFOBuffer_available(struct FIFOBuffer *fifo_buffer)
{
  return fifo_buffer->headPos != fifo_buffer->tailPos;
}

void FIFOBuffer_flush(struct FIFOBuffer *fifo_buffer)
{
  fifo_buffer->headPos = 0;
  fifo_buffer->tailPos = 0;
}
unsigned char FIFOBuffer_read(struct FIFOBuffer *fifo_buffer)
{
  unsigned char buf = 0;
  // 如果头尾接触表示缓冲区为空
  if (fifo_buffer->headPos != fifo_buffer->tailPos)
  {
    buf = fifo_buffer->buffer[fifo_buffer->headPos];
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize)
    {
      fifo_buffer->headPos = 0;
    }
  }
  return buf;
}

void FIFOBuffer_push(struct FIFOBuffer *fifo_buffer, unsigned char buf)
{
  fifo_buffer->buffer[fifo_buffer->tailPos] = buf; // 从尾部追加
  if (++fifo_buffer->tailPos >= fifo_buffer->bufferSize)
  { // 尾节点偏移
    fifo_buffer->tailPos = 0;
  }
  if (fifo_buffer->tailPos == fifo_buffer->headPos)
  {
    if (++fifo_buffer->headPos >= fifo_buffer->bufferSize)
    {
      fifo_buffer->headPos = 0;
    }
  }
}
// 串口1发送接收中断
void UART1_Isr(void) interrupt 4
{
  if (TI)
  {
    TI = 0;
    // 队列中还有数据, 继续发送
    if (uartSendFIFO.headPos != uartSendFIFO.tailPos)
    {
      SBUF = FIFOBuffer_read(&uartSendFIFO);
    }
    else
    {
      isSend = 0;
    }
  }
  if (RI)
  {
    RI = 0;
    // 接收到数据, 放入队列
    FIFOBuffer_push(&uartReceiveFIFO, SBUF);
  }
}
void uart1WriteBuf(unsigned char *buffer, unsigned char length)
{
  unsigned char i = 0;
  for (i = 0; i < length; i++)
  {
    FIFOBuffer_push(&uartSendFIFO, buffer[i]);
  }
  if (isSend == 0)
  {
    isSend = 1;
    SBUF = FIFOBuffer_read(&uartSendFIFO);
  }
}
void initUartCommand()
{
  uartReceiveFIFO.headPos = 0;
  uartReceiveFIFO.tailPos = 0;
  uartReceiveFIFO.bufferSize = UartReceiveSize;
  uartReceiveFIFO.buffer = uartReceiveBuffer;

  uartSendFIFO.headPos = 0;
  uartSendFIFO.tailPos = 0;
  uartSendFIFO.bufferSize = UartSendSize;
  uartSendFIFO.buffer = uartSendBuffer;
}
// ---------------------- 串口发送接收 结束 ----------------------

//---------------------- 定时器 ----------------------
void T_IRQ0(void) interrupt 1 using 1
{
}
void Timer0_Init(void) // 32768微秒@24.000MHz
{
  AUXR &= 0x7F; // 定时器时钟12T模式
  TMOD &= 0xF0; // 设置定时器模式
  TL0 = 0x00;   // 设置定时初始值
  TH0 = 0x00;   // 设置定时初始值
  TF0 = 0;      // 清除TF0标志
  TR0 = 1;      // 定时器0开始计时
  ET0 = 1;      // 打开定时器0中断
}
// ---------------------- 定时器 结束 ----------------------

void adcReadValue()
{
  ADC_CONTR |= 0x40; // 启动 AD 转换
  while (!(ADC_CONTR & 0x20))
    ;                 // 查询 ADC 完成标志
  ADC_CONTR &= ~0x20; // 清完成标志

  adcValue = ADC_RES; // 存储 ADC 的 10 位结果的高 2 位
  adcValue <<= 8;
  adcValue |= ADC_RESL; // 存储 ADC 的 10 位结果的低 8 位

  ADC_RES = 0;
  ADC_RESL = 0;
}

void shutdownTask()
{
  adcReadValue();
  if (isShut == true)
  {
    return;
  }
  // 主动触发关机
  if (SHUT_IN_PIN == 1)
  {
    delay10us();
    if (SHUT_IN_PIN == 1)
    {
      isShut = true;
      // 关闭电源
      SHUT_PIN = 0;
      return;
    }
  }
  // 电压大于触发电压
  if (adcValue > trigAdcValue)
  {
    delay10us();
    adcReadValue();
    if (adcValue > trigAdcValue)
    {
      isShut = true;
      // 关闭电源
      SHUT_PIN = 0;
      delay10us();
      // 泄放电容中的电压
      N_SHUT_PIN = 1;
      delay(3000);
      N_SHUT_PIN = 0;
    }
  }
}

void uartLoopRead()
{
  while (uartReceiveFIFO.headPos != uartReceiveFIFO.tailPos)
  {
    tempData = FIFOBuffer_read(&uartReceiveFIFO);
    // 读取ADC值
    if (tempData == 0x01)
    {
      cmdData[0] = 0xf0;
      cmdData[1] = 0x0f;
      cmdData[2] = adcValue >> 8;
      cmdData[3] = adcValue & 0xff;
      uart1WriteBuf(cmdData, 4);
    }
  }
}

void setup()
{
  P3M1 &= ~0x04;
  P3M0 |= 0x04; // 推挽输出
  P5M1 &= ~0x10;
  P5M0 |= 0x10; // 推挽输出
  P5M1 |= 0x20;
  P5M0 &= ~0x20;
  P3M1 |= 0x08;
  P3M0 &= ~0x08; // 高阻输入
  SHUT_PIN = 0;
  N_SHUT_PIN = 0;                                               // 高阻输入
  uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_1); // 初始化串口
  Timer0_Init();
  // adc_init(ADC_PIN, ADC_SYSclk_DIV_2, ADC_10BIT); // ADC初始化，三个参数ADC引脚，时钟分频双数2-32，输出值位数12BIT最大分率-12位的ADC输出12位，10位的输出10位
  initUartCommand();
  // ADC 控制器  电源  启动  完成  --   15通道模拟选择位  单片机STC8G1K08A-8PIN   P30 0000 P31  0001 P32  0010  P33  0011  P54  0100   P55   0101
  // ADC_CONTR    B7    B6    B5   B4   B3   B2   B1   B0
  ADC_CONTR = 0b10000011;
  ADCCFG = 0b00100010; // 转换结果右对齐。 ADC_RES 保存结果的高 2 位， ADC_RESL 保存结果的低 8 位。
  ES = 1;              // 允许串行口中断
  EA = 1;              // 允许总中断
  delay(100);
  SHUT_PIN = 1; // 开机
}

void loop()
{
  shutdownTask();
  uartLoopRead();
}

void main(void)
{
  setup();
  while (1)
  {
    loop();
  }
}
