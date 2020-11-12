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

float error_t1=0;
float error_t2=0;
const float servo_mid=7.45;
float servo_pwm=7.45;
void BEEP_test(void);//11.07添加  外部中断函数声明
void pit_ledtest(void);//11.07添加 定时器中断函数声明
void moto_l(void);//11.10 定时器中断，电机转动函数
void servo();
void servo_pid();

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

    HAL_ExitCritical();
    /*
     * 摄像头测试函数，使用时应该注释
    */
    //CAM_ZF9V034_UnitTest();
    //SDK_DelayAtleastUs(1000*1000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    inv::IMU_UnitTest_AutoRefresh();
    sc::SC_UnitTest_AutoRefresh();

    MENU_Resume();
    /*
     * 菜单挂起函数，显示图像之前要挂起菜单
     */
    MENU_Suspend();

    /*
     * 哈工大100周年校庆图片展示
     */

    //DISP_SSD1306_delay_ms(100);
    //DISP_SSD1306_BufferUpload((uint8_t*) DISP_image_100thAnniversary);
    //DISP_SSD1306_delay_ms(100);
    //DISP_SSD1306_BufferUploadDMA((uint8_t*) DISP_image_100thAnniversary);
    //CAM_ZF9V034_UnitTest();
    //DISP_SSD1306_BufferUpload((uint8_t*) &dispBuffer);

    /*
     * 电机恒定速度输出
     */
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,0);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,30);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,30);
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,0);
    //SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,7.45);

    float f = arm_sin_f32(0.6f);


    while (true)
    {

        while (kStatus_Success != DMADVP_TransferGetFullBuffer(DMADVP0, &dmadvpHandle, &fullBuffer));
        //THRE();
        //head_clear();
        //image_main();
        dispBuffer->Clear();
        const uint8_t imageTH = 100;
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
        //TODO: 在这里添加车模保护代码
    }
}



void MENU_DataSetUp(void)
{
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(nullType, NULL, "EXAMPLE", 0, 0));
    static float P = 10.9, I = 3.14, D = 12.14;
    static menu_list_t *PID;
    PID = MENU_ListConstruct("PID", 20, menu_menuRoot);
    assert(PID);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, PID, "PID", 0, 0));
    MENU_ListInsert(PID, MENU_ItemConstruct(varfType, &P, "P", 10, menuItem_data_global));
    MENU_ListInsert(PID, MENU_ItemConstruct(varfType, &I, "I", 11, menuItem_data_global));
    MENU_ListInsert(PID, MENU_ItemConstruct(varfType, &D, "D", 1, menuItem_data_global));

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
    float pwm_error=0;
    error_t1=get_error();
    pwm_error=0.015*error_t1+0.01*(error_t1-error_t2);////只用PD
    servo_pwm=servo_mid+pwm_error;
    if( servo_pwm<6.8)
        servo_pwm=6.8;
    else if(servo_pwm>8.2)
        servo_pwm=8.2;

    error_t2=error_t1;
};
void servo()
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,servo_pwm);
}

void CAM_ZF9V034_DmaCallback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds)
{
    //TODO: 补完本回调函数

    //TODO: 添加图像处理（转向控制也可以写在这里）
}

#if (defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET)

#endif
