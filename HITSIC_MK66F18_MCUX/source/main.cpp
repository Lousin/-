/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright 2018 - 2020 HITSIC
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hitsic_common.h"

/** HITSIC_Module_DRV */
#include "drv_ftfx_flash.hpp"
#include "drv_disp_ssd1306.hpp"
#include "drv_imu_invensense.hpp"
#include "drv_dmadvp.hpp"
#include "drv_cam_zf9v034.hpp"

/** HITSIC_Module_SYS */
#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "sys_uartmgr.hpp"
#include "cm_backtrace.h"
//#include "easyflash.h"

/** HITSIC_Module_LIB */
#include "lib_graphic.hpp"

/** HITSIC_Module_APP */
#include "app_menu.hpp"
#include "app_svbmp.hpp"

/** FATFS */
#include "ff.h"
#include "sdmmc_config.h"
FATFS fatfs;                                   //逻辑驱动器的工作区

#include "sc_adc.h"
#include "sc_ftm.h"

/** HITSIC_Module_TEST */
#include "drv_cam_zf9v034_test.hpp"
#include "app_menu_test.hpp"
#include "drv_imu_invensense_test.hpp"
#include "sys_fatfs_test.hpp"
#include "sys_fatfs_diskioTest.hpp"

/** SCLIB_TEST */
#include "sc_test.hpp"
#include "image.h"


void BEEP_test(void);//外部中断函数声明
void pit_ledtest(void);//定时器中断函数声明
void moto_l(void);// 定时器中断，电机转动函数
void servo();
void servo_pid();

static float KP_M = 0.0;
static float KI_M = 0.0;
static float KP_S = 0.016;
static float KD_S = 0.01;
static float LIMIT_S_High = 8.8;
static float LIMIT_S_Low = 6.5;
static float pwm_servo;
static float pwm_servo_l;
static float pwm_moto_l = 27;//左电机速度（用于菜单调节）
static float pwm_moto_r = 27;//右电机速度（用于菜单调节）
float error_now_s=0;
float error_last_s=0;
const float servo_mid=7.45;
static menu_list_t *testList_1;//菜单变量（电机）
static menu_list_t *testList_2;//菜单变量（舵机）
static menu_list_t *testList_3;//菜单变量（摄像头图像处理）（内含前瞻和阈值）

void MENU_DataSetUp(void);
void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);




cam_zf9v034_configPacket_t cameraCfg;
dmadvp_config_t dmadvpCfg;
dmadvp_handle_t dmadvpHandle;

inv::i2cInterface_t imu_i2c(nullptr, IMU_INV_I2cRxBlocking, IMU_INV_I2cTxBlocking);
inv::mpu6050_t imu_6050(imu_i2c);

disp_ssd1306_frameBuffer_t dispBuffer;
graphic::bufPrint0608_t<disp_ssd1306_frameBuffer_t> bufPrinter(dispBuffer);

void main(void)
{
    /** 初始化阶段，关闭总中断 */
    HAL_EnterCritical();
    /** 初始化时钟 */
    RTECLK_HsRun_180MHz();
    /** 初始化引脚路由 */
    RTEPIN_Basic();
    RTEPIN_Digital();
    RTEPIN_Analog();
    RTEPIN_LPUART0_DBG();
    RTEPIN_UART0_WLAN();
    /** 初始化外设 */
    RTEPIP_Basic();
    RTEPIP_Device();
    /** 初始化调试串口 */
    DbgConsole_Init(0U, 921600U, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("Welcome to HITSIC !\n");
    PRINTF("GCC %d.%d.%d\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    /** 初始化CMBackTrace */
    cm_backtrace_init("HITSIC_MK66F18", "2020-v3.0", "v4.1.1");
    /** 初始化ftfx_Flash */
    FLASH_SimpleInit();
    /** 初始化EasyFlash */
    //easyflash_init();
    /** 初始化PIT中断管理器 */
    pitMgr_t::init();
    /** 初始化I/O中断管理器 */
    extInt_t::init();
    /** 初始化OLED屏幕 */
    DISP_SSD1306_Init();
    extern const uint8_t DISP_image_100thAnniversary[8][128];
    DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    DISP_SSD1306_delay_ms(1000);
    //SDK_DelayAtleastUs(1000*1000,CLOCK_GetFreq(kCLOCK_CoreSysClk));

    /** 初始化菜单 */
    MENU_Init();
    MENU_Data_NvmReadRegionConfig();
    MENU_Data_NvmRead(menu_currRegionNum);
    /** 菜单挂起 */
    MENU_Suspend();

    //初始化部分：

    cam_zf9v034_configPacket_t cameraCfg;
    CAM_ZF9V034_GetDefaultConfig(&cameraCfg);
    /** 初始化摄像头 */

    //设置摄像头配置
    CAM_ZF9V034_CfgWrite(&cameraCfg);                                   //写入配置
    dmadvp_config_t dmadvpCfg;
    CAM_ZF9V034_GetReceiverConfig(&dmadvpCfg, &cameraCfg);    //生成对应接收器的配置数据，使用此数据初始化接受器并接收图像数据。
    DMADVP_Init(DMADVP0, &dmadvpCfg);
    dmadvp_handle_t dmadvpHandle;
    DMADVP_TransferCreateHandle(&dmadvpHandle, DMADVP0, CAM_ZF9V034_UnitTestDmaCallback);
    uint8_t *imageBuffer0 = new uint8_t[DMADVP0->imgSize];
    uint8_t *imageBuffer1 = new uint8_t[DMADVP0->imgSize];
    uint8_t *fullBuffer = NULL;
    disp_ssd1306_frameBuffer_t *dispBuffer = new disp_ssd1306_frameBuffer_t;
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer0);
    DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, imageBuffer1);
    DMADVP_TransferStart(DMADVP0, &dmadvpHandle);
    //TODO: 在这里初始化摄像头
    /** 初始化IMU */
    //TODO: 在这里初始化IMU（MPU6050）
    /** 菜单就绪 */

    /** 控制环初始化 */
    //TODO: 在这里初始化控制环
    /*
     * pitMgr定时中断，第一参数的单位是ms,第二个参数是取余数的值，第三个参数是中断函数
     */
    pitMgr_t::insert(6U, 3, moto_l, pitMgr_t::enable);
    pitMgr_t::insert(20U, 5, servo, pitMgr_t::enable);
    //PORT_SetPinInterruptConfig(PORTE, 10U, kPORT_InterruptFallingEdge);
    //extInt_t::insert(PORTE, 10U, BEEP_test);
    //pitMgr_t::insert(5000U, 23U, pit_ledtest, pitMgr_t::enable);
    /** 初始化结束，开启总中断 */
    MENU_Resume();
    HAL_ExitCritical();
    /*
     * 菜单挂起函数，显示图像之前要挂起菜单
     */
    MENU_Suspend();
    float f = arm_sin_f32(0.6f);


    while (true)
    {

        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        THRE();
        head_clear();
        image_main();
        dispBuffer->Clear();
        const uint8_t imageTH = 160;
        for (int i = 0; i < cameraCfg.imageRow; i += 2)
        {
            int16_t imageRow = i >> 1;//除以2 为了加速;
            int16_t dispRow = (imageRow / 8) + 1, dispShift = (imageRow % 8);
            for (int j = 0; j < cameraCfg.imageCol; j += 2)
            {
                int16_t dispCol = j >> 1;
                if (fullBuffer[i * cameraCfg.imageCol + j] > imageTH)
                {
                    dispBuffer->SetPixelColor(dispCol, imageRow, 1);
                }
            }
        }
        DISP_SSD1306_BufferUpload((uint8_t*) dispBuffer);
        DMADVP_TransferSubmitEmptyBuffer(DMADVP0, &dmadvpHandle, fullBuffer);
        servo_pid();
        //此处可以添加车模保护措施
        //DISP_SSD1306_Print_F8x16(7,64,"pwm_servo_l");
    }
}



void MENU_DataSetUp(void)
{
//    DISP_SSD1306_Print_F8x16(7,64,"pwm_servo_l");
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
    testList_1 = MENU_ListConstruct("Motor_1", 20, menu_menuRoot);
    assert(testList_1);
    testList_2 = MENU_ListConstruct("Steer_1", 20, menu_menuRoot);
    assert(testList_2);
    testList_3 = MENU_ListConstruct("Picture_1", 20, menu_menuRoot);
    assert(testList_3);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, testList_1, "Motor_1", 0, 0));
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, testList_2, "Steer_1", 0, 0));
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, testList_3, "Picture_1", 0, 0));
    {
        MENU_ListInsert(testList_1, MENU_ItemConstruct(varfType, &KP_M, "KP_M", 10, menuItem_data_global));
        MENU_ListInsert(testList_1, MENU_ItemConstruct(varfType, &KI_M, "KI_M", 11, menuItem_data_global));
        MENU_ListInsert(testList_1, MENU_ItemConstruct(varfType, &pwm_moto_l, "Pwm_moto_l", 12, menuItem_data_global));//左电机pwm占空比
        MENU_ListInsert(testList_1, MENU_ItemConstruct(varfType, &pwm_moto_r, "Pwm_moto_r", 13, menuItem_data_global));//右电机pwm占空比
        MENU_ListInsert(testList_2, MENU_ItemConstruct(varfType, &KP_S, "KP_S", 14, menuItem_data_global));
        MENU_ListInsert(testList_2, MENU_ItemConstruct(varfType, &KD_S, "KD_S", 15, menuItem_data_global));
        MENU_ListInsert(testList_2, MENU_ItemConstruct(varfType, &LIMIT_S_High, "LIMIT_S_High", 16, menuItem_data_global));
        MENU_ListInsert(testList_2, MENU_ItemConstruct(varfType, &LIMIT_S_Low, "LIMIT_S_Low", 17, menuItem_data_global));
        MENU_ListInsert(testList_3, MENU_ItemConstruct(variType, &threshold, "Threshold", 18, menuItem_data_global));//阈值
        MENU_ListInsert(testList_3, MENU_ItemConstruct(variType, &prospect, "Prospect", 19, menuItem_data_global));//前瞻
           }

    //TODO: 在这里添加子菜单和菜单项
}

/*
 * 电机是两个通道控制一个电机（两个通道里的占空比怎么给？（一般是一个通道给0，另外一个通道给占空比，为什么？（可参阅今天ppt里的链接）））
 * 电机驱动
 */
void moto_l()
{
    SCFTM_PWM_Change(FTM0, kFTM_Chnl_0 ,20000U,20);
    SCFTM_PWM_Change(FTM0, kFTM_Chnl_1 ,20000U,0);
    SCFTM_PWM_Change(FTM0, kFTM_Chnl_2 ,20000U,20);
    SCFTM_PWM_Change(FTM0, kFTM_Chnl_3 ,20000U,0);
}
void pit_ledtest(void)
{
    GPIO_PortToggle(GPIOC,1U<<0);
}
void BEEP_test(void)
{
    GPIO_PortToggle(GPIOC,1U<<0);
   // GPIO_PinWrite(GPIOC,0,0U);
}
void servo_pid()
{
    error_now_s = 94-(int)mid_line[prospect];;
    pwm_servo = KP_S *error_now_s  + KD_S*(error_now_s - error_last_s );
    pwm_servo_l = pwm_servo+7.85;
    if(pwm_servo_l>LIMIT_S_High)
    {
       pwm_servo_l = LIMIT_S_High;
    }
    else if(pwm_servo_l<LIMIT_S_Low)
    {
        pwm_servo_l = LIMIT_S_Low;
    }
    error_last_s = error_now_s;
}

void servo()
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50U,pwm_servo_l);
}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数

    //TODO: 添加图像处理（转向控制也可以写在这里）
}

#if (defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET)

#endif
