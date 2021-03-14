/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buttons.h"
#include "flash.h"
#include "lcd.h"
#include "bmi270.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

LTDC_HandleTypeDef hltdc;

OSPI_HandleTypeDef hospi1;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SPI_HandleTypeDef hspi2;

I2C_HandleTypeDef  hi2c1;

#define PI 22/(float)7

const int zOff = 258 ;
const int xOff = 36;
const int yOff = -36;
const int cSize =32;
const int view_plane = 258;
const float angle = PI/60;


/* USER CODE BEGIN Includes */
#define DEV_ADD          (0x68<<1)
#define UNIT_SELECT_ADD  0x3B
#define UNIT_SELECT_DATA 0x68
#define OPR_MODE_ADD     0x3D
#define OPR_MODE_DATA    0xFC
#define EUL_X_ADD        0x1A
#define EUL_Y_ADD        0x1C
#define EUL_Z_ADD        0x1E
#define LIA_X_ADD        0x28
#define LIA_Y_ADD        0x2A
#define LIA_Z_ADD        0x2C
/* USER CODE END Includes */

#define BUS_TIMEOUT             1000


/* USER CODE BEGIN PV */

uint16_t audiobuffer[48000] __attribute__((section (".audio")));


char logbuf[1024 * 4] __attribute__((section (".persistent"))) __attribute__((aligned(4)));
uint32_t log_idx __attribute__((section (".persistent")));
__attribute__((used)) __attribute__((section (".persistent"))) volatile uint32_t boot_magic;

uint32_t boot_buttons;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI1_Init(void);
static void MX_NVIC_Init(void);
// static void MX_I2C1_Init(void);
static void draw_line(uint16_t, uint16_t, uint16_t, uint16_t, uint32_t);
static void draw_pixel(uint16_t, uint16_t, uint32_t);
static void raster_circle(uint16_t, uint16_t, uint16_t, uint32_t);
static void ellipse(uint16_t , uint16_t , uint16_t , uint16_t , uint32_t);
static void draw_cube(uint32_t color);
static void zrotate(float q);
static void yrotate(float q);
static void xrotate(float q);
static void print_cube(uint32_t color);

/* Callback function prototypes for the BMI270 Sensor API */
int8_t bmi2_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi2_i2c_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len);
void bmi2_delay_us(uint32_t period);

/* Static variables */
static struct bmi2_dev bmi2;
static struct bmi2_dev s_bmi270;
static uint8_t dev_addr;
static volatile bool bmi2_intr_recvd = false;
static volatile uint32_t last_time_us = 0;
#define READ_WRITE_LEN     UINT8_C(46)
uint8_t GTXBuffer[2048], GRXBuffer[2048];




unsigned char eul_x_msb = 0, eul_x_lsb = 0, eul_y_msb = 0, eul_y_lsb = 0, eul_z_msb = 0, eul_z_lsb = 0;
unsigned char lia_x_msb = 0, lia_x_lsb = 0, lia_y_msb = 0, lia_y_lsb = 0, lia_z_msb = 0, lia_z_lsb = 0;

void HAL_Delay_us(uint32_t Delay_us)
{
   uint32_t delay_time;
   delay_time = (uint32_t)(Delay_us/1000+1);
   HAL_Delay(delay_time);
}

static int8_t set_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(&config, 1, dev);

    /* Map data ready interrupt to interrupt pin. */
    // rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);

    if (rslt == BMI2_OK)
    {
        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the gyro configurations. */
        rslt = bmi270_set_sensor_config(&config, 1, dev);
    }
    else{
      lcd_backlight_off();
      while(1){};
    }

    return rslt;
}


void BMI270_MODULE_INIT(void)
{
    int8_t rslt;
    /* Create an instance of sensor data structure. */
    struct bmi2_sensor_data sensor_data = { 0 };

    /* Assign gyro sensor to variable. */
    uint8_t sens_list = BMI2_GYRO;

    /* Initialize the interrupt status of gyro. */
    uint16_t int_status = 0;
    bmi2_interface_init(&s_bmi270);
    rslt = bmi270_init(&s_bmi270);
    if(rslt != BMI2_OK){
      //lcd_backlight_off();
      //while(1){};
    }
    bmi270_sensor_enable(&sens_list, 1, &s_bmi270);
    set_gyro_config(&s_bmi270);

}


int8_t SensorAPI_I2Cx_Read(uint8_t slave_address7, uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr)
{
    uint16_t DevAddress = slave_address7;// << 1;
#if 0//def SUPPORT_STM32_L475VGTX
   subaddress = subaddress + 2;
#endif
//   PDEBUG("DevAddress0 =0x%x reg_addr=0x%x\n",DevAddress,subaddress);
             
   // send register address
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &subaddress, 0, BUS_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c1, DevAddress, pBuffer, ReadNumbr, BUS_TIMEOUT);
    return 0;
}

int8_t SensorAPI_I2Cx_Write(uint8_t slave_address7, uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr)
{
    uint16_t DevAddress = slave_address7;// << 1;
#if 0//def SUPPORT_STM32_L475VGTX
   subaddress = subaddress + 2;
#endif
//   PDEBUG("DevAddress1 =0x%x reg_addr=0x%x\n",DevAddress,subaddress);

    GTXBuffer[0] = subaddress;
    memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

    // send register address
    HAL_I2C_Master_Transmit(&hi2c1, DevAddress, GTXBuffer, WriteNumbr+1, BUS_TIMEOUT);
    return 0;
}

void bmi2_interface_init(struct bmi2_dev *bmi)
{

    if (bmi != NULL)
    {
        /* To initialize the user I2C function */
        dev_addr = BMI2_I2C_PRIM_ADDR;
        //dev_addr = 0x68<<1;
        bmi->intf = BMI2_I2C_INTF;
        bmi->read = SensorAPI_I2Cx_Read;
        bmi->write = SensorAPI_I2Cx_Write;
        


        /* Assign device address to interface pointer */
        bmi->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bmi->delay_us = HAL_Delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi->read_write_len = READ_WRITE_LEN;

        /* Assign to NULL to load the default config file. */
        bmi->config_file_ptr = NULL;
    } else {
      lcd_backlight_off();
    }


}

void eul_x_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, EUL_X_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}

void eul_y_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, EUL_Y_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}

void eul_z_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, EUL_Z_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}

void lia_x_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, LIA_X_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}

void lia_y_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, LIA_Y_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}

void lia_z_read(unsigned char * tab){
    HAL_I2C_Mem_Read(&hi2c1, DEV_ADD, LIA_Z_ADD, I2C_MEMADD_SIZE_8BIT, tab, 2, 100);
}


float cube3d[8][3] = {
      {xOff - cSize,yOff + cSize,zOff - cSize},
      {xOff + cSize,yOff + cSize,zOff - cSize},
      {xOff - cSize,yOff - cSize,zOff - cSize},
      {xOff + cSize,yOff - cSize,zOff - cSize},
      {xOff - cSize,yOff + cSize,zOff + cSize},
      {xOff + cSize,yOff + cSize,zOff + cSize},
      {xOff - cSize,yOff - cSize,zOff + cSize},
      {xOff + cSize,yOff - cSize,zOff + cSize}
    };
uint8_t cube2d[8][2];







/*
static void matrixmult(int A[4][8], float R[4][4]);
static void rotMatAboutX(float Rx[4][4], float);
static void rotMatAboutY(float Ry[4][4], float);
static void rotMatAboutZ(float Rz[4][4], float);
static void sideView(int A[4][8], uint32_t color);*/


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}


int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Power pin as Input
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1_LOW);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LTDC_Init();
  MX_SPI2_Init();
  MX_OCTOSPI1_Init();
  MX_SAI1_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  lcd_init(&hspi2, &hltdc);
  memset(framebuffer, 0xff, 320*240*2);

  /* USER CODE END 2 */

  BMI270_MODULE_INIT();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  flash_memory_map(&hospi1);

  // Sanity check, sometimes this is triggered
  uint32_t add = 0x90000000;
  uint32_t* ptr = (uint32_t*)add;
  if(*ptr == 0x88888888) {
    Error_Handler();
  }

  uint32_t color = 0x0000;
  uint32_t i = 0;

  // Create a continuous square wave and loop it using DMA in circular mode
  /*
  for (uint32_t i = 0; i < sizeof(audiobuffer) / sizeof(audiobuffer[0]); i++) {
    audiobuffer[i] = (i % (48000 / 500)) > 48 ? 200 : -200;
  }
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, audiobuffer, sizeof(audiobuffer) / sizeof(audiobuffer[0]));
  */

   //bq24072_init();

  uint16_t x = 160,y = 120;
  float yaw = 0, pitch = 0, roll = 0;

  struct bmi2_sensor_data sensor_data = { 0 };


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // draw_line(0, 100, 0, 100, 1);

    bmi270_get_sensor_data(&sensor_data, 1, &s_bmi270);
    float x = 0;

    x = lsb_to_dps(sensor_data.sens_data.gyr.x, 2000, s_bmi270.resolution);
   // pitch=sensor_data.sens_data.gyr.x;
   // roll=sensor_data.sens_data.gyr.y;
    yaw=sensor_data.sens_data.gyr.z;

    uint32_t buttons = buttons_get();
    if(buttons & B_Left) {
      roll+=0.02;
      if(roll>2*PI) roll-=2*PI;
    }
    if(buttons & B_Right) {
      roll-=0.02;
      if(roll<0) roll+=2*PI;
    }
    if(buttons & B_Up) {
      pitch+=0.02;
      if(pitch>2*PI) pitch-=2*PI;
    }
    if(buttons & B_Down) {
      pitch-=0.02;
      if(pitch<0) pitch+=2*PI;
    }
    if(buttons & B_B) {
       yaw+=0.02;
      if(yaw>2*PI) yaw-=2*PI;
    }
    if(buttons & B_A) {
       yaw-=0.02;
      if(yaw<0) yaw+=2*PI;
    }
    if(buttons & B_GAME) {
      
      color = 0x333333;
    }
    if(buttons & B_TIME) {
      
      color = 0x555555;
      lcd_backlight_on();
    }
    if(buttons & B_PAUSE) {
      
      color = 0x777777;
      lcd_backlight_off();
    }


//read from IMU
    /*
    unsigned char buff[3];

      eul_x_read(buff);
  //    HAL_Delay(25);
      eul_x_lsb = buff[0];
      eul_x_msb = buff[1];

      roll=((eul_x_msb*256.0)+eul_x_lsb)/16.0;

      eul_y_read(buff);
  //    HAL_Delay(25);
      eul_y_lsb = buff[0];
      eul_y_msb = buff[1];

      pitch=((eul_y_msb*256.0)+eul_y_lsb)/16.0;

      eul_z_read(buff);
  //    HAL_Delay(25);
      eul_z_lsb = buff[0];
      eul_z_msb = buff[1];

      yaw=((eul_z_msb*256.0)+eul_z_lsb)/16.0;
 //end of read from IMU     
    */
    /*
    for(int x=0; x < 320; x++) {
      for(int y=0; y < 240; y++) {
        // framebuffer[(y*320)+x] = i;
        if(((x + i)/10 % 2 == 0) && (((y + i)/10 % 2 == 0))){
          framebuffer[(y*320)+x] = color;
        } else {
          framebuffer[(y*320)+x] = 0xffff;
        }
        
        // i++;
      }
      // i++;
    }
    */

    //framebuffer[(120*320)+160] = color;

   //draw_line(100+x,100+y,120+x,120+y, color);
   //raster_circle(x, y, 10, color);
   // ellipse(x, y, 10, 10, color);

    /*

    uint16_t ax=160 + 70 * sin(yaw), ay=70 + 20 * cos(yaw);
    uint16_t bx=160 + 70 * sin(yaw+1.57), by=70 + 20 * cos(yaw+1.57);
    uint16_t cx=160 + 70 * sin(yaw+3.14), cy=70 + 20 * cos(yaw+3.14);
    uint16_t dx=160 + 70 * sin(yaw+4.71), dy=70 + 20 * cos(yaw+4.71);

    uint16_t ex=160 + 70 * sin(yaw), ey=170 + 20 * cos(yaw);
    uint16_t fx=160 + 70 * sin(yaw+1.57), fy=170 + 20 * cos(yaw+1.57);
    uint16_t gx=160 + 70 * sin(yaw+3.14), gy=170 + 20 * cos(yaw+3.14);
    uint16_t hx=160 + 70 * sin(yaw+4.71), hy=170 + 20 * cos(yaw+4.71);

    draw_line(ax, ay, bx, by, color);
    draw_line(bx, by, cx, cy, color);
    draw_line(cx, cy, dx, dy, color);
    draw_line(dx, dy, ax, ay, color);

    draw_line(ex, ey, fx, fy, color);
    draw_line(fx, fy, gx, gy, color);
    draw_line(gx, gy, hx, hy, color);
    draw_line(hx, hy, ex, ey, color);

    draw_line(ex, ey, ax, ay, color);
    draw_line(fx, fy, bx, by, color);
    draw_line(gx, gy, cx, cy, color);
    draw_line(hx, hy, dx, dy, color);

    */
/*

int A[4][8]=
    {{0, 50, 50, 0, 0, 50, 50, 0},
    {0, 0, 50, 50, 0, 0, 50, 50},
    {0, 0, 0, 0, 50, 50, 50, 50},
    {1, 1, 1 ,1 ,1 ,1, 1, 1}};


    rotMatAboutX(A, roll);
    rotMatAboutY(A, pitch);
    rotMatAboutZ(A, yaw);

    sideView(A, 0x000000);*/

    cube3d[0][0]=xOff - cSize; cube3d[0][1]=yOff + cSize; cube3d[0][2]=zOff - cSize;
    cube3d[1][0]=xOff + cSize; cube3d[1][1]=yOff + cSize; cube3d[1][2]=zOff - cSize;
    cube3d[2][0]=xOff - cSize; cube3d[2][1]=yOff - cSize; cube3d[2][2]=zOff - cSize;
    cube3d[3][0]=xOff + cSize; cube3d[3][1]=yOff - cSize; cube3d[3][2]=zOff - cSize;
    cube3d[4][0]=xOff - cSize; cube3d[4][1]=yOff + cSize; cube3d[4][2]=zOff + cSize;
    cube3d[5][0]=xOff + cSize; cube3d[5][1]=yOff + cSize; cube3d[5][2]=zOff + cSize;
    cube3d[6][0]=xOff - cSize; cube3d[6][1]=yOff - cSize; cube3d[6][2]=zOff + cSize;
    cube3d[7][0]=xOff + cSize; cube3d[7][1]=yOff - cSize; cube3d[7][2]=zOff + cSize;
  

    xrotate(pitch);
    yrotate(roll);
    zrotate(yaw);

    print_cube(0x000000);

    
    HAL_Delay(20);

    print_cube(0xffffff);

/*
    draw_line(ax, ay, bx, by, 0xffffff);
    draw_line(bx, by, cx, cy, 0xffffff);
    draw_line(cx, cy, dx, dy, 0xffffff);
    draw_line(dx, dy, ax, ay, 0xffffff);

    draw_line(ex, ey, fx, fy, 0xffffff);
    draw_line(fx, fy, gx, gy, 0xffffff);
    draw_line(gx, gy, hx, hy, 0xffffff);
    draw_line(hx, hy, ex, ey, 0xffffff);

    draw_line(ex, ey, ax, ay, 0xffffff);
    draw_line(fx, fy, bx, by, 0xffffff);
    draw_line(gx, gy, cx, cy, 0xffffff);
    draw_line(hx, hy, dx, dy, 0xffffff);

   */

    //ellipse(x, y, 10, 10, 0xffffff);
    i++;
    // if(i % 30 == 0) {
    //   if(color == 0xf800) {
    //     color = 0x7e0;
    //   } else {
    //     color = 0xf800;
    //   }
      
    // }
// HAL_Delay(500);
// for(int x=0; x < 320; x++) {
// for(int y=0; y < 240; y++) {
// framebuffer[(y*320)+x] = 0x7e0;
// }
// }
// HAL_Delay(500);
// for(int x=0; x < 320; x++) {
// for(int y=0; y < 240; y++) {
// framebuffer[(y*320)+x] = 0x1f;
// }
// }
// HAL_Delay(500);
  }
  /* USER CODE END 3 */
}
/*
void sideView(int A[4][8], uint32_t color){
    int xdisp = 106, ydisp = 120;
    draw_line(xdisp+A[1][0], ydisp+A[2][0], xdisp+A[1][1], ydisp+A[2][1], color);
    draw_line(xdisp+A[1][1], ydisp+A[2][1], xdisp+A[1][2], ydisp+A[2][2], color);
    draw_line(xdisp+A[1][2], ydisp+A[2][2], xdisp+A[1][3], ydisp+A[2][3], color);
    draw_line(xdisp+A[1][3], ydisp+A[2][3], xdisp+A[1][0], ydisp+A[2][0], color);
    draw_line(xdisp+A[1][4], ydisp+A[2][4], xdisp+A[1][5], ydisp+A[2][5], color);
    draw_line(xdisp+A[1][5], ydisp+A[2][5], xdisp+A[1][6], ydisp+A[2][6], color);
    draw_line(xdisp+A[1][6], ydisp+A[2][6], xdisp+A[1][7], ydisp+A[2][7], color);
    draw_line(xdisp+A[1][7], ydisp+A[2][7], xdisp+A[1][4], ydisp+A[2][4], color);
    draw_line(xdisp+A[1][0], ydisp+A[2][0], xdisp+A[1][4], ydisp+A[2][4], color);
    draw_line(xdisp+A[1][1], ydisp+A[2][1], xdisp+A[1][5], ydisp+A[2][5], color);
    draw_line(xdisp+A[1][6], ydisp+A[2][6], xdisp+A[1][2], ydisp+A[2][2], color);
    draw_line(xdisp+A[1][7], ydisp+A[2][7], xdisp+A[1][3], ydisp+A[2][3], color);
    draw_line(xdisp+A[1][7], ydisp+A[2][7], xdisp+A[1][3], ydisp+A[2][3], color);
}

void rotMatAboutX(float Rx[4][4], float x){
   float  xrad = PI*x/(float)360;
   float cosx = cos(xrad), sinx = sin(xrad);
    Rx[1][1]=cosx;
    Rx[1][2]=-sinx;
    Rx[2][1]=sinx;
    Rx[2][2]=cosx;
}

void rotMatAboutY(float Ry[4][4], float y){
    float  yrad = PI*y/(float)360;
    float cosy = cos(yrad), siny = sin(yrad);
    Ry[0][0]=cosy;
    Ry[0][2]=siny;
    Ry[2][0]=-siny;
    Ry[2][2]=cosy;
}

void rotMatAboutZ(float Rz[4][4], float z){
    float  zrad = PI*z/(float)360;
    float cosz = cos(zrad), sinz = sin(zrad);
    Rz[0][0]=cosz;
    Rz[0][1]=-sinz;
    Rz[1][0]=sinz;
    Rz[1][1]=cosz;
}

void matrixmult(int A[4][8], float R[4][4]){
    float res[4][8];
    for(int i = 0; i<4; i++){
        for(int j = 0; j<8; j++){
            res[i][j] = 0;
            for(int k = 0; k<4; k++){
                res[i][j]+=R[i][k]*A[k][j];
            }
        }
    }
    for(int i = 0; i<4; i++){
        for(int j = 0; j<8; j++){
            A[i][j] = res[i][j];
        }
    }
}

*/

void print_cube(uint32_t color) {
  //calculate 2d points
  for(uint8_t i = 0; i < 8; i++) {
    cube2d[i][0] = (unsigned char)((cube3d[i][0] * view_plane / cube3d[i][2]) + (240/2));
    cube2d[i][1] = (unsigned char)((cube3d[i][1] * view_plane / cube3d[i][2]) + (320/2));
  }

  draw_cube(color);
}

void draw_cube(uint32_t color) {
  draw_line(cube2d[0][0],cube2d[0][1],cube2d[1][0],cube2d[1][1],color);
  draw_line(cube2d[0][0],cube2d[0][1],cube2d[2][0],cube2d[2][1],color);
  draw_line(cube2d[0][0],cube2d[0][1],cube2d[4][0],cube2d[4][1],color);
  draw_line(cube2d[1][0],cube2d[1][1],cube2d[5][0],cube2d[5][1],color);
  draw_line(cube2d[1][0],cube2d[1][1],cube2d[3][0],cube2d[3][1],color);
  draw_line(cube2d[2][0],cube2d[2][1],cube2d[6][0],cube2d[6][1],color);
  draw_line(cube2d[2][0],cube2d[2][1],cube2d[3][0],cube2d[3][1],color);
  draw_line(cube2d[4][0],cube2d[4][1],cube2d[6][0],cube2d[6][1],color);
  draw_line(cube2d[4][0],cube2d[4][1],cube2d[5][0],cube2d[5][1],color);
  draw_line(cube2d[7][0],cube2d[7][1],cube2d[6][0],cube2d[6][1],color);
  draw_line(cube2d[7][0],cube2d[7][1],cube2d[3][0],cube2d[3][1],color);
  draw_line(cube2d[7][0],cube2d[7][1],cube2d[5][0],cube2d[5][1],color);
  //delay(500);
}

void zrotate(float q) {
  float tx,ty,temp;
  for(uint8_t i = 0; i < 8; i++) {
    tx = cube3d[i][0] - xOff;
    ty = cube3d[i][1] - yOff;
    temp = tx * cos(q) - ty * sin(q);
    ty = tx * sin(q) + ty * cos(q);
    tx = temp;
    cube3d[i][0] = tx + xOff;
    cube3d[i][1] = ty + yOff;
  }
}

void yrotate(float q) {
  float tx,tz,temp;
  for(uint8_t i = 0; i < 8; i++) {
    tx = cube3d[i][0] - xOff;
    tz = cube3d[i][2] - zOff;
    temp = tz * cos(q) - tx * sin(q);
    tx = tz * sin(q) + tx * cos(q);
    tz = temp;
    cube3d[i][0] = tx + xOff;
    cube3d[i][2] = tz + zOff;
  }
}

void xrotate(float q) {
  float ty,tz,temp;
  for(uint8_t i = 0; i < 8; i++) {
    ty = cube3d[i][1] - yOff;
    tz = cube3d[i][2] - zOff;
    temp = ty * cos(q) - tz * sin(q);
    tz = ty * sin(q) + tz * cos(q);
    ty = temp;
    cube3d[i][1] = ty + yOff;
    cube3d[i][2] = tz + zOff;
  }
}

void draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color)
{
 
  int16_t dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int16_t dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  int16_t err = (dx>dy ? dx : -dy)/2, e2;
 
  for(;;){
    framebuffer[(y0*320)+x0] = color;
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
 
}

void draw_pixel(uint16_t x0, uint16_t y0, uint32_t color)
{
  framebuffer[(y0*320)+x0] = color;
}

void ellipse(uint16_t x0, uint16_t y0, uint16_t r1, uint16_t r2, uint32_t color)
{
  float t;
  for(t=0; t+=0.1; t<6.28)
    draw_pixel(x0 + (r1 * sin(t)), y0 + (r2 * cos(t)), color);
}

void raster_circle(uint16_t x0, uint16_t y0, uint16_t radius, uint32_t color)
{
    int16_t f = 1 - radius;
    int16_t ddF_x = 0;
    int16_t ddF_y = -2 * radius;
    int16_t x = 0;
    int16_t y = radius;
 
    draw_pixel(x0, y0 + radius, color);
    draw_pixel(x0, y0 - radius, color);
    draw_pixel(x0 + radius, y0, color);
    draw_pixel(x0 - radius, y0, color);
 
    while(x < y) 
    {
        if(f >= 0) 
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;    
        draw_pixel(x0 + x, y0 + y, color);
        draw_pixel(x0 - x, y0 + y, color);
        draw_pixel(x0 + x, y0 - y, color);
        draw_pixel(x0 - x, y0 - y, color);
        draw_pixel(x0 + y, y0 + x, color);
        draw_pixel(x0 - y, y0 + x, color);
        draw_pixel(x0 + y, y0 - x, color);
        draw_pixel(x0 - y, y0 - x, color);
    }
}

void boot_magic_set(uint32_t magic)
{
  boot_magic = magic;
}

uint32_t boot_magic_get(void)
{
  return boot_magic;
}

void GW_EnterDeepSleep(void)
{
  // Stop SAI DMA (audio)
  HAL_SAI_DMAStop(&hsai_BlockA1);

  // Enable wakup by PIN1, the power button
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_LOW);

  lcd_backlight_off();

  // Leave a trace in RAM that we entered standby mode
  boot_magic = BOOT_MAGIC_STANDBY;

  // Delay 500ms to give us a chance to attach a debugger in case
  // we end up in a suspend-loop.
  for (int i = 0; i < 10; i++) {
      wdog_refresh();
      HAL_Delay(50);
  }
  HAL_PWR_EnterSTANDBYMode();

  // Execution stops here, this function will not return
  while(1) {
    // If we for some reason survive until here, let's just reboot
    HAL_NVIC_SystemReset();
  }

}

// Returns buttons that were pressed at boot
uint32_t GW_GetBootButtons(void)
{
  return boot_buttons;
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();

/*
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  */

  /**Configure Analog filter
  */
  //HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**I2C1 GPIO Configuration    
  PB6     ------> I2C1_SCL
  PB7     ------> I2C1_SDA 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
/*
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 140;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_OSPI
                              |RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 192;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 5;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 4;
  PeriphClkInitStruct.PLL3.PLL3N = 9;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 24;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_CLKP;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* OCTOSPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IIPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 60;
  hltdc.Init.AccumulatedVBP = 7;
  hltdc.Init.AccumulatedActiveW = 380;
  hltdc.Init.AccumulatedActiveH = 247;
  hltdc.Init.TotalWidth = 392;
  hltdc.Init.TotalHeigh = 255;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 320;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 240;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 255;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0x24000000;
  pLayerCfg.ImageWidth = 320;
  pLayerCfg.ImageHeight = 240;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = GFXMMU_VIRTUAL_BUFFER0_BASE;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 20;
  hospi1.Init.ChipSelectHighTime = 2;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.ClkChipSelectHighTime = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //Speaker
 // HAL_GPIO_WritePin(GPIO_Speaker_enable_GPIO_Port, GPIO_Speaker_enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_Speaker_enable_Pin */
  GPIO_InitStruct.Pin = GPIO_Speaker_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_Speaker_enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_PAUSE_Pin BTN_GAME_Pin BTN_TIME_Pin */
  GPIO_InitStruct.Pin = BTN_PAUSE_Pin|BTN_GAME_Pin|BTN_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pin : BTN_PWR_Pin */
  GPIO_InitStruct.Pin = BTN_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD1 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_A_Pin BTN_Left_Pin BTN_Down_Pin BTN_Right_Pin
                           BTN_Up_Pin BTN_B_Pin */
  GPIO_InitStruct.Pin = BTN_A_Pin|BTN_Left_Pin|BTN_Down_Pin|BTN_Right_Pin
                          |BTN_Up_Pin|BTN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) {
    // Blink display to indicate failure
    lcd_backlight_off();
    HAL_Delay(500);
    lcd_backlight_on();
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
