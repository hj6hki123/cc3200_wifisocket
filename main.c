//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     -   HTTP Server
// Application Overview -   This is a sample application demonstrating
//                          interaction between HTTP Client(Browser) and 
//                          SimpleLink Device.The SimpleLink device runs an 
//                          HTTP Server and user can interact using web browser.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_HTTP_Server
// or
// doc\examples\CC32xx_HTTP_Server.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup httpserver
//! @{
//
//****************************************************************************

// Standard includes
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "simplelink.h"
#include "netcfg.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pinmux.h"
#include "gpio_if.h"
#include "uart_if.h"
#include "osi.h"
#include "smartconfig.h"

#include "hw_uart.h"
#include "udma.h"
#include "udma_if.h"

#include "gpio_if.h"
#include "timer.h"
#include "timer_if.h" 

#include "sdhost.h"


#include "gpio.h"

#include "wdt_if.h"
#include "wdt.h"


#define APP_NAME		        "HTTP Server"
#define AP_SSID_LEN_MAX         (33)
#define UART_PRINT              Report
#define ROLE_INVALID            (-5)

#define LED_STRING              "LED"
#define LED1_STRING             "LED1_zfrghrey43543534egdgdghdyht4sdsd sfs sdfw565$#%"
#define LED2_STRING             ",LED2_"
#define LED_ON_STRING          "ON"
#define LED_OFF_STRING         "OFF"

#define OOB_TASK_PRIORITY				(1)
#define SPAWN_TASK_PRIORITY				(9)
#define OSI_STACK_SIZE					(4096)//(1024)

#define SH_GPIO_3       				(3)  /* P58 - Device Mode */

#define ROLE_INVALID                                    (-5)

#define AUTO_CONNECTION_TIMEOUT_COUNT   (50)   /* 5 Sec */
#define SL_STOP_TIMEOUT                 (30)

#define UART_RX_LENGTH 1200
#define DEBUG_PUTTY 0


#define USERFILE        "userfile.txt"
#define SYSFILE         "sysfile.txt"
#define KWHFILEA        "kwhfilea.txt"
#define KWHFILEB        "kwhfileb.txt"
#define PROTECT         "protect.txt"
#define NAMEA           "name_a.txt"
#define NAMEB           "name_b.txt"
#define SYSTEXT         "The quick brown fox jumps over the lazy dog"
#define KWH_DIR         "kWh"

/////////////// SPI FLASH FILE OPERATION ///////////////////////
#define USER_FILE_NAME          "/sys/ad.txt"
#define SPI_FLASH_NAMEA           "/sys/name_a.txt"
#define SPI_FLASH_NAMEB           "/sys/name_b.txt"
#define SPI_FLASH_KWHFILEA        "/sys/kwhfilea.txt"
#define SPI_FLASH_KWHFILEB        "/sys/kwhfileb.txt"



// FOR WPS  ===> Open AP name to connect to. E.g: "TP-LINK_2D0D68"
#define SSID_NAME               "cc3200demo"
#define WPS_PIN_CODE            "88664422"


#define IP_ADDR            0x787243ed
#define PORT_NUM            9998
#define BUF_SIZE            10 /* 1400 */
#define TCP_PACKET_COUNT     1 /* 1000 */
#define UDP_PACKET_COUNT   1000
     
#define WD_PERIOD_MS 				100000
#define MAP_SysCtlClockGet 			800000

//#define MILLISECONDS_TO_TICKS(ms) 	((MAP_SysCtlClockGet / 1000) * (ms))

#define FUSE_AMP_VALUE 10              //10 AMP FUSE

#include <string.h>
#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

//driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "utils.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"


//Free_rtos/ti-rtos includes
#include "osi.h"

// common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "common.h"

#include "smartconfig.h"
#include "pinmux.h"
#include "udma.h"
#include "udma_if.h"
#include  "netapp.h"
#include "gpio_if.h"
#include "timer.h"
#include "timer_if.h"

#include "sdhost.h"

#include "hw_uart.h"
#include "udma.h"
#include "udma_if.h"

#include "gpio.h"

#include "wdt_if.h"
#include "wdt.h"

#define APPLICATION_NAME        "HTTP Server"
#define APPLICATION_VERSION     "1.1.1"
#define AP_SSID_LEN_MAX         (33)
#define ROLE_INVALID            (-5)

#define LED_STRING              "LED"
//#define LED1_STRING             "LED1_"
#define LED2_STRING             ",LED2_"
#define LED_ON_STRING           "ON"
#define LED_OFF_STRING          "OFF"

#define OOB_TASK_PRIORITY               (1)
//#define OSI_STACK_SIZE                  (2048)
#define SH_GPIO_3                       (3)  /* P58 - Device Mode */
#define ROLE_INVALID                    (-5)
#define AUTO_CONNECTION_TIMEOUT_COUNT   (50)   /* 5 Sec */

//#define WD_PERIOD_MS 	

#define UART_RX_LENGTH 1200
#define BUF_SIZE           1400
#define TCP_PACKET_COUNT     1 /* 1000 */
#define  FRESULT int
// Application specific status/error codes
#define  UINT8 unsigned char
/////////////// SPI FLASH FILE OPERATION ///////////////////////
#define USER_FILE_NAME          "/sys/ad.txt"
#define SPI_FLASH_NAMEA           "/sys/name_a.txt"
#define SPI_FLASH_NAMEB           "/sys/name_b.txt"
#define SPI_FLASH_KWHFILEA        "/sys/kwhfilea.txt"
#define SPI_FLASH_KWHFILEB        "/sys/kwhfileb.txt"

#define SL_NETAPP_IPV4_ACQUIRED    1
#define SL_NETAPP_IPV6_ACQUIRED    2
#define SL_NETAPP_SOCKET_TX_FAILED 3
#define SL_NETAPP_IP_LEASED        4
#define SL_NETAPP_IP_RELEASED      5
#define UDP_PACKET_COUNT   1000

#define WD_PERIOD_MS 				100000
#define MAP_SysCtlClockGet 			800000


#define PERIODIC_TEST_CYCLES    80000000
#define PERIODIC_TEST_LOOPS     5
#define SL_NETAPP_HTTPGETTOKENVALUE        1
#define SL_NETAPP_HTTPPOSTTOKENVALUE    2
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received 
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned int   g_uiPortNum = PORT_NUM;
int g_iSimplelinkRole = ROLE_INVALID;
signed int g_uiIpAddress = 0;
unsigned char g_ucSSID[AP_SSID_LEN_MAX];
int BsdTcpClient(unsigned short usPort);
enum MCUNetworkStateEnum
{
    MCU_SLHost_UNINIT       = 0x0001, // CC3200 NW Driver Uninitialized
    MCU_SLHost_INIT         = 0x0002, // CC3200 NW Driver Initialized
    MCU_AP_ASSOC            = 0x0004, // CC3200 Associated to AP
    MCU_IP_ALLOC            = 0x0008, // CC3200 has IP Address
    MCU_IP_LEASED           = 0x0010, // CC3200 Server Initialized
    MCU_SERVER_INIT         = 0x0020, // CC3200 Server Initialized
    MCU_CLIENT_CONNECTED    = 0x0040, // CC3200 Client Connected to Server
    MCU_PING_COMPLETE       = 0x0080, // Ping to AP or server completed
    MCU_CONNECTION_FAIL     = 0x0100  // Wlan connection failed
};

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

unsigned char POST_token[] = "__SL_P_ULD";
unsigned char GET_token[]  = "__SL_G_ULD";
unsigned char GET_token2[]  = "__SL_G_UFK";//for read 2 meters
unsigned char GET_token3[]  = "__SL_G_UFJ";//for read 2 meters
unsigned char GET_KWH_token[]  = "__SL_G_KWH";
unsigned char GET_KWH_ALL_token[] = "__SL_G_KLL";
unsigned char POST_token_kwh_query[] =  "__SL_P_KWH";
unsigned char GET_VERSION_token[] =  "__SL_G_VER";
//*****************************************************************************
// Variable related to Connection status
//*****************************************************************************
volatile unsigned short g_usMCNetworkUstate = 0;

int g_uiSimplelinkRole = ROLE_INVALID;
unsigned int g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
volatile unsigned char g_ucConnectTimeout =0;





_u16 g_macaddress[6];


unsigned char rx_buf[40],tmp_array_ptr[4];
char rx_kwh_buf[40];
unsigned char meter_packet[40] =
{
  // complete name
  0x00, 0x00, 0x00, 0x10, 0x2d, // length of first data structure (11 bytes excluding length byte)
  0xe6, 0x00, 0x01, 0x17, 0x71,
  0x02, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0x79, 0x6c, 0xc1
  
};

unsigned char BannerText[30];
unsigned char BannerText2[UART_RX_LENGTH];//static const
char big_page[1500]; //for save all kWh from sdcard
unsigned int big_page_ptr = 0;

unsigned int Array_Ptr=0, Read_Array_Ptr=0;
unsigned int Array_Ptr2=0, Read_Array_Ptr2=0;
unsigned char relay0 = 0x00;
unsigned char relay1 = 0x00;
static tBoolean bRxDone;

//////////FOR GPIO SW BUTTON////////
unsigned int uiGPIOPort;
unsigned char pucGPIOPin;
unsigned char ucPinValue;
unsigned char going_action_sw2, going_action_sw3;
//////////FOR GPIO SW BUTTON////////        


unsigned int ADE7953_REAL_DATA=0, ADE7953_REAL_DATA_tmp;
double f;
unsigned long l;
unsigned int const IRMS_REF[]={0x028e89, 0x0515b1, 0x0a0d8b,0x1cfde0}; //167574 333233 658827 1900000
double const IRMS_FAC[]={722.12e-6, 720.216e-6, 717.34355e-6, 697.66788e-6 }; //697.66788e-6 
//double const IRMS_FAC[]={722.12e-6, 720.216e-6, 717.3436e-6  }; new
//double const IRMS_FAC[]={722.12e-6, 720.216e-6, 720.9783e-6 };
unsigned char cChar, PingPong=0xff;
double AVA, AWATT, BVA, BWATT;
double  kwh=0.0, ws, double_acc_ws=0.0, double_acc_ws_B=0.0, double_acc_wh, dst_double_acc_ws;
unsigned int debbb[10];
unsigned int ddi=0;
unsigned short crc_value;
unsigned char acc_ws[22], *p_acc_ws, NAME_A_REQUIRED, NAME_B_REQUIRED, WRITE_NAME_A_2_SDCARD = 0, WRITE_NAME_B_2_SDCARD = 0;
OsiTaskHandle* pTaskHandle_sdcard;
unsigned char ss, mm ,hh, day, month, date;
unsigned char mm_backup; 
unsigned short year;

unsigned char TIME_A_REQUIRED=0, TIME_B_REQUIRED=0,   WRITE_KWH_TO_FILE = 0;
unsigned char tt, sdcard_working=0, query_kwh_working=0;
unsigned int big_page_index=0;
char wps_going=0,  device_get_ip = 0;
unsigned int switch_buf[2]={0x00,0x00};
unsigned char meter_name_A[20] = {  
               0x00,
               0x00,
               0x08,
               0x0D,
               
               0xE8,
               0xAB,
               0x8B,
               0xE6,
               
               0x94,
               0xB9,
               0xE5,
               0x90,
               
               0x8D,
               0xE5,
               0xAD,
               0x97,
               
               0x00,
               0x00,
               0x00,
               0x00
               };

unsigned char meter_name_B[20] = {  
               0x01,
               0x00,
               0x08,
               0x0D,
               
               0xE8,
               0xAB,
               0x8B,
               0xE6,
               
               0x94,
               0xB9,
               0xE5,
               0x90,
               
               0x8D,
               0xE5,
               0xAD,
               0x97,
               
               0x00,
               0x00,
               0x00,
               0x00
               };




#ifdef USE_FREERTOS
char year_month_date_str[20],write_kwh_file[20], kWh_log_buf[50];
unsigned char still_query=false;
volatile unsigned short g_usStatusInfo = 0;  //FOR WPS
char g_cBsdBuf[BUF_SIZE];

char g_cBsdBuf2[]="d0:31:10:12:43:22,x,192.168.100.200";
unsigned long  g_ulDestinationIp = IP_ADDR;        // Client IP address
   
int send_udp_state = 4;
      //4: all
      //3: z
      //2: y
      //1: x
      //0: don't send

//////////////// 20201205 prepare METER_INFO send to TCP Server////////////////
#define PACKET_METER_INFO_LENGTH 28
char g_csend[PACKET_METER_INFO_LENGTH];
void PACKET_METER_INFO_PREPARE_TO_TCP_SERVER();
//////////////// 20201205 prepare METER_INFO send to TCP Server////////////////


//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************

//*****************************************************************************
//
//! \brief Application defined hook (or callback) function - assert
//!
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//!
//! \return none
//!
//*****************************************************************************
volatile tBoolean g_bFeedWatchdog = true;
volatile unsigned long g_ulWatchdogCycles;


void DS1302_wrbyte(unsigned char wrbyte);
unsigned char DS1302_rdbyte();
unsigned char DS1302_read(unsigned char addr);
void setup_time_date_ds1302(unsigned char *p);
void get_time_date(void);
#ifdef SDCARD_WROK
static void ListKWHFiles(DIR *dir, char *rr_ptr);
#endif
void SwitchToStaMode(int iMode);
void WpsConnectPushButton();
void ResetNWP();
int BsdUdpServer(unsigned short usPort);
int BsdUdpClient(unsigned short usPort);

void ade7953_init(void);
void uart_timer_go(void);
unsigned int no_connected_sta_counter=0;
long lRetVal = -1;
char g_cBsdBuf[BUF_SIZE];
char strlumresp[BUF_SIZE];

char const fw_ver_buf[]="2015-10-08 v3";

static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* Wait */
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    //while (g_ulStatus!=2) 
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
        
#endif
    }

    return SUCCESS;

}
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_ulDestinationIp = IP_ADDR;
    g_uiPortNum = PORT_NUM;
    g_ulPacketCount = TCP_PACKET_COUNT;
}
static void RebootMCU()
{

    sl_Stop(30); 

    MAP_PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);

    MAP_UtilsDelay(8000000);

    MAP_PRCMHibernateIntervalSet(330);

    MAP_PRCMHibernateEnter();

    while(1);

}


void WatchdogIntHandler(void)
{
    //
    // If we have been told to stop feeding the watchdog, return immediately
    // without clearing the interrupt.  This will cause the system to reset
    // next time the watchdog interrupt fires.
    //
    if(!g_bFeedWatchdog)
    {
        return;
    }
    //
    // After 10 interrupts, switch On LED6 to indicate system reset
    // and don't clear watchdog interrupt which causes system reset
    //
    

    //
    // Clear the watchdog interrupt.
    //
    
    //MAP_WatchdogIntClear(WDT_BASE);  //交給 HTTPServerTask 來清
    
 

    
    //
    // Increment our interrupt counter.
    //
    g_ulWatchdogCycles++;

}


long ReadFileFromDevice( unsigned char *filename,unsigned char *pData, unsigned int Len)
{
    //long lRetVal = -1;
   
    unsigned long ulToken;
    long lFileHandle;
    //
    // open a user file for reading
    //
    lRetVal = sl_FsOpen( filename,
                        FS_MODE_OPEN_READ ,
                        &ulToken,
                        &lFileHandle);
    
     //CREATE FILE IF FILE NOT EXIST
 /*    lRetVal = sl_FsOpen(filename,
                FS_MODE_OPEN_CREATE(65536, \
                          _FS_FILE_OPEN_FLAG_COMMIT|FS_MODE_OPEN_READ),
                        &ulToken,
                        &lFileHandle);
  */   
     
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        
        return -1;
    }

    //
    // read the data and compare with the stored buffer
    //

    
     lRetVal = sl_FsRead(lFileHandle,
                   0, //OFFSET
                     pData, Len );
     
     
    if ((lRetVal < 0) )
    {
            lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            //ASSERT_ON_ERROR(FILE_READ_FAILED);
            return -1;
       
    }
         
        
    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        //ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
    
    }

    return 0;
}


long WriteFileToDevice(unsigned char *filename,unsigned char *pData, unsigned int Len, unsigned char w_type)
{
   // long lRetVal = -1;

    unsigned long ulToken;
    long lFileHandle;
    //
    // open a user file for WRITE
    //
 /*
     if(w_type ==1)*/
     lRetVal = sl_FsOpen(filename,  //filename
                FS_MODE_OPEN_CREATE(65536, \
                          _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                        &ulToken,
                        &lFileHandle);
  //   else
    
  /*  lRetVal = sl_FsOpen(SPI_FLASH_NAMEA, //USER_FILE_NAME
                        FS_MODE_OPEN_WRITE,
                        &ulToken,
                        &lFileHandle);
    */
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        //ASSERT_ON_ERROR(FILE_OPEN_READ_FAILED);
         return -1;
   
    }
 
    
        
         lRetVal = sl_FsWrite(lFileHandle,
                    0, 
                    pData, Len);
         
         
        
        
        if (lRetVal < 0)
        {
            lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            //ASSERT_ON_ERROR(FILE_READ_FAILED);
            return -1;
        }
    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        //ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
        asm("NOP");
    }

    return 0;
}


void get_time_date(void)
{  
        unsigned char i, j;
        // 讀秒 
        i = DS1302_read(0x81);        
        j = (i&0x70)>>4;
        i &= 0x0F;
        ss = j*10+i;

        // 讀分
        i = DS1302_read(0x83);
        j = (i&0x70)>>4;
        i &= 0x0F;
        mm = j*10+i;

        // 讀時      
        i = DS1302_read(0x85);
        j = (i&0x30)>>4;
        i &= 0x0F;
        hh = j*10+i;        
        
        //date
        i = DS1302_read(0x87);
        j = (i&0x30)>>4;
        i &= 0x0F;
        date = j*10+i;  
        
        //month
        i = DS1302_read(0x89);
        j = (i&0x10)>>4;
        i &= 0x0F;
        month = j*10+i;        

        //day
        day = DS1302_read(0x8B);
                
        // year 
        i = DS1302_read(0x8D);
        j = (i&0xF0)>>4;
        i &= 0x0F;
        year = 2000+j*10+i;
         
        sprintf(year_month_date_str,"KWH/%d%02d%02d",year, month, date);          
}

void get_write_kwh_file_by_date_time(void)
{
       sprintf(write_kwh_file,"%s/%02d.TXT",year_month_date_str, hh);
}




#ifdef SDCARD_WROK

DIR last_visit_dir;
static void
ListDirectory(DIR *dir)
{
    FILINFO fno;
    FRESULT res;
    //unsigned long ulSize;
    //tBoolean bIsInKB;
    DIR dir2;
    const char r[]="/KWH/";
    char rr[20]="";

    
   // still_query = false; 20150112
    for(;;)
    {
      
      memcpy (&last_visit_dir, dir, sizeof(DIR) );  // Backup current DIR
      res = f_readdir(dir, &fno);           // Read a directory item
      if (res != FR_OK || fno.fname[0] == 0 || big_page_ptr >1450 )
      {
        //if(big_page_ptr >1450)  20150112
          //    still_query = true; 20150112
       // return dir;  //try for next time search again to avoid from head search again OK??
        break;                                // Break on error or end of dir
      }
      
      if(fno.fattrib == 0x10)
      {                  //0x10 is dir type
          strcpy(rr,r);
          strcat(rr,fno.fname);  // rr is like ==> "/KWH/20141117"
          res = f_opendir(&dir2,rr);
          ListKWHFiles(&dir2, rr);
          
          
          if(big_page_ptr >1450)
               break;
          
      }
         /*
      ulSize = fno.fsize;
      bIsInKB = false;
      if(ulSize > 1024)
      {
          ulSize = ulSize/1024;
          bIsInKB = true;
      }
      */
      //Report("->%-15s%5d %-2s    %-5s\n\r",fno.fname,ulSize,\
        (bIsInKB == true)?"KB":"B",(fno.fattrib&AM_DIR)?"Dir":"File");
    }

}


char rr3_last_ok[22]="";  //20150112
static void
ListKWHFiles(DIR *dir, char *rr_ptr)
{
    FILINFO fno;
    FRESULT res;
   
    FIL fp;
    WORD Size;
    char rr2[22]="";
    char rr3[22]="";
    

    strcpy(rr2,rr_ptr);
    strcat(rr2,"/");
    for(;;)
    {
      res = f_readdir(dir, &fno);           // Read a directory item
      
     
        
      if (res != FR_OK || fno.fname[0] == 0 || big_page_ptr >1450)
      {
        ////////////// backup last OK rr3 //////////////  20150112
        if( big_page_ptr >1450 )
        {
               strcpy( rr3_last_ok, rr3 );
               still_query = true;
        }       
        break;                                // Break on error or end of dir
      }
      
      if(fno.fattrib == 0x20)
      {                 //0x20 is file type
          strcpy(rr3,rr2);
          strcat(rr3,fno.fname);    //rr3 is like ==> "/KWH/20141117/07.TXT"
          
            if( still_query == true)
            {
                 if( strcmp(rr3_last_ok, rr3) == 0 )
                     still_query = false;
                 //else
                 continue;             
            }
       
              
          res = f_open(&fp,rr3,FA_READ);
          if(res == FR_OK)
          {
               memset( rx_buf, 0, sizeof(rx_buf) );
               res = f_read(&fp,rx_buf,30,&Size);
                
               strcpy(&big_page[big_page_ptr], rr3);  //填入檔案路徑
               big_page_ptr += strlen(rr3);                
               strcpy(&big_page[big_page_ptr], (char *)rx_buf);  //填入KWH內容
               big_page_ptr += strlen((char *)rx_buf); 
               
               
               
          }
          else
          {
              asm("NOP");
          }
      }
       asm("NOP");     
      //Report("->%-15s%5d %-2s    %-5s\n\r",fno.fname,ulSize,\
        (bIsInKB == true)?"KB":"B",(fno.fattrib&AM_DIR)?"Dir":"File");
    }

}
#endif

int BsdUdpClient(unsigned short usPort)
{
    int             iCounter;
    //short           sTestBufLen;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    
int             iStatus;
int             iSockID;
    
    long            lLoopCount = 0;




    //filling the UDP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);
    //sAddr.sin_addr.s_addr = g_ulDestinationIp;

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a UDP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( iSockID < 0 )
    {
        // error
       // ASSERT_ON_ERROR(UCP_CLIENT_FAILED);
    }

    // for a UDP connection connect is not required
    // sending 1000 packets to the UDP server
    //while (lLoopCount < g_ulPacketCount)
    {
        // sending packet
        /*iStatus = sl_SendTo(iSockID, g_cBsdBuf, sTestBufLen, 0,
                                (SlSockAddr_t *)&sAddr, iAddrSize);*/
        
        iStatus = sl_SendTo(iSockID, g_cBsdBuf2, strlen(g_cBsdBuf2), 0,
                                (SlSockAddr_t *)&sAddr, iAddrSize);//0919179306
        
        
          
          
        if( iStatus <= 0 )
        {
            // error
            sl_Close(iSockID);
           // ASSERT_ON_ERROR(UCP_CLIENT_FAILED);
        }
        lLoopCount++;
    }

    //UART_PRINT("Sent %u packets successfully\n\r",g_ulPacketCount);
    UART_PRINT("Sent UDP packets successfully\n\r");

    //closing the socket after sending 1000 packets
    sl_Close(iSockID);

    return 0;//SUCCESS;
}
//****************************************************************************
//
//! \brief Opening a UDP server side socket and receiving data
//!
//! This function opens a UDP socket in Listen mode and waits for an incoming
//! UDP connection.
//!    If a socket connection is established then the function will try to
//!    read 1000 UDP packets from the connected client.
//!
//! \param[in]          port number on which the server will be listening on
//!
//! \return             0 on success, Negative value on Error.
//
//****************************************************************************
int BsdUdpServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    long            lLoopCount = 0;
    short           sTestBufLen;



    sTestBufLen  = BUF_SIZE;
    //filling the UDP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a UDP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( iSockID < 0 )
    {
        // error
       // ASSERT_ON_ERROR(UCP_SERVER_FAILED);
    }

    // binding the UDP socket to the UDP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        //ASSERT_ON_ERROR(UCP_SERVER_FAILED);
    }

    // no listen or accept is required as UDP is connectionless protocol
    /// waits for 1000 packets from a UDP client
    //while (lLoopCount < g_ulPacketCount)
    while ( 1 )
    {
        iStatus = sl_RecvFrom(iSockID, g_cBsdBuf, sTestBufLen, 0,
                     ( SlSockAddr_t *)&sAddr, (SlSocklen_t*)&iAddrSize );

        ////////// printing received packet content /////////
        g_cBsdBuf[iStatus] = '\0';
        UART_PRINT("%s\n\r",g_cBsdBuf );
         
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        //ASSERT_ON_ERROR(UCP_SERVER_FAILED);
    }
    lLoopCount++;
    
    if( 'i' == g_cBsdBuf[0]  && 'p' == g_cBsdBuf[1]  && '=' == g_cBsdBuf[2]  )      
    {
           g_ulDestinationIp = sAddr.sin_addr.s_addr;
        //g_ulDestinationIp = sAddr.sin_addr.s_addr | 0xff000000; //弄成廣播封包 ex: x.x.x.255, 成功讓爻域server收到
        //g_ulDestinationIp = sAddr.sin_addr.s_addr | 0xffffffff; //弄成廣播封包 ex: 255.255.255.255, 爻域server收不到

        asm("NOP");
            
        BsdUdpClient(31999);
    }
    
    
    if( 's' == g_cBsdBuf[0]  && 'e' == g_cBsdBuf[1]  && 'n' == g_cBsdBuf[2] 
      && 'd' == g_cBsdBuf[3]  && '=' == g_cBsdBuf[4]  && 'x' == g_cBsdBuf[5]  ) 
  
                           {
                              send_udp_state = 1;
	                           //4: all
	                           //3: z
	                           //2: y
	                           //1: x
	                           //0: don't send
                           }
     if( 's' == g_cBsdBuf[0]  && 'e' == g_cBsdBuf[1]  && 'n' == g_cBsdBuf[2] 
      && 'd' == g_cBsdBuf[3]  && '=' == g_cBsdBuf[4]  && 'y' == g_cBsdBuf[5]  ) 
                           {
                              send_udp_state = 2;
	                           //4: all
	                           //3: z
	                           //2: y
	                           //1: x
	                           //0: don't send
                           }
     if( 's' == g_cBsdBuf[0]  && 'e' == g_cBsdBuf[1]  && 'n' == g_cBsdBuf[2] 
      && 'd' == g_cBsdBuf[3]  && '=' == g_cBsdBuf[4]  && 'z' == g_cBsdBuf[5]  ) 
                           {
                              send_udp_state = 3;
	                           //4: all
	                           //3: z
	                           //2: y
	                           //1: x
	                           //0: don't send
                           }
    
     if( 's' == g_cBsdBuf[0]  && 'e' == g_cBsdBuf[1]  && 'n' == g_cBsdBuf[2] 
      && 'd' == g_cBsdBuf[3]  && '=' == g_cBsdBuf[4]  && 'a' == g_cBsdBuf[5] && 'l' == g_cBsdBuf[6]  && 'l' == g_cBsdBuf[7] )  
                           {
                              send_udp_state = 4;
	                           //4: all
	                           //3: z
	                           //2: y
	                           //1: x
	                           //0: don't send
                           }
    
     if( 'p' == g_cBsdBuf[0]  && 'a' == g_cBsdBuf[1]  && 'u' == g_cBsdBuf[2] 
      && 's' == g_cBsdBuf[3]  && 'e' == g_cBsdBuf[4]  )   
                        
                           {
                              send_udp_state = 0;
	                           //4: all
	                           //3: z
	                           //2: y
	                           //1: x
	                           //0: don't send
                           }

    
    
            //////// blink red led to indicate received packet ////////
            GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED on
            MAP_UtilsDelay(8000000);
            GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 1);  //  (GPIO17)  D10 RED off
    
    }
    
    
     
    
    

   // UART_PRINT("Recieved %u packets successfully\n\r",g_ulPacketCount);

    //closing the socket after receiving 1000 packets
//    sl_Close(iSockID);

//    return 0;//SUCCESS;
}


void UDPSERVERTask( void *pvParameters )
{
   //  while (1);
       if(g_uiDeviceModeConfig == ROLE_STA)
            while (!(g_usMCNetworkUstate & MCU_AP_ASSOC) || !(g_usMCNetworkUstate & MCU_IP_ALLOC));
       
       if(g_uiDeviceModeConfig == ROLE_AP)
            while ( g_usMCNetworkUstate !=24 ); 
            
        
       MAP_UtilsDelay(500);
     asm("NOP");
     
     BsdUdpServer(32000);
  
}
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}

void tcp_client( void *pvParameters )
{
 
       ///////////// AP MODE  ///////////////////
       if(g_uiDeviceModeConfig == ROLE_AP)
            while ( g_usMCNetworkUstate !=24 );

                   
       ////////////// STA MODE  /////////////////
       if(g_uiDeviceModeConfig == ROLE_STA)
         while (!(g_usMCNetworkUstate & MCU_AP_ASSOC) || !(g_usMCNetworkUstate & MCU_IP_ALLOC));

       //////// CONNECT TO SERVER  ////////
       while(1)
       {
          BsdTcpClient(9998);    
      
          MAP_UtilsDelay(500);
          asm("NOP");
       }
     //BsdUdpServer(32000);
      while(1);
}






FRESULT res;
long long_res;

void SDCARDTask( void *pvParameters )
{
#ifdef SDCARD_WROK  
    FIL fp;
    FATFS fs;
    
    DIR dir;
    static DIR q_dir;
    WORD Size;
#endif    
    static unsigned int counter=0;
    unsigned char ucLEDStatus;
  
  
    GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  //PIN_61 = 1  (GPIO06) Relay_1 (通電)
    GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 1  (GPIO04) Relay_0 (通電)
    
     
      
      
    
     while( device_get_ip == 0) 
    { 
      
         asm("NOP");
      
        if(no_connected_sta_counter >500)
              break;
    }
    
    
#ifdef SDCARD_WROK    
    
    f_mount(0,&fs);

    /////////////// try to open kWh Directory ///////////////////
  
    res = f_opendir(&dir,KWH_DIR);
    if(res == FR_OK)
    {
         // Already Exists Directory
         asm("NOP");
    }
    else
    {    // Non Exists Directory, We need create it !!
         res = f_mkdir(KWH_DIR);  //create directory API
         asm("NOP");   
         if( res != FR_OK)
         {
              asm("NOP");
         }
     
    }
    
    ////////////////////// READ KWH A FILE /////////////////////////////
     res = f_open(&fp,KWHFILEA,FA_READ);
     if(res == FR_OK)
     {
               memset(acc_ws, 0, sizeof(acc_ws));
               f_read(&fp,acc_ws,22,&Size);
               
               
               res = f_close(&fp);
           
               //設定回 kWh A的值 還原成 "瓦秒"  到 double_acc_ws 變數中
               double_acc_ws =  atof( (char *)acc_ws) * 3600000;//3.6 * 3600000;//atof( (char *) &acc_ws[10]);
               //還原成 "瓦秒"  就是將kWh值  * 3600000
               
               asm("NOP");
       
      }
      else
      {
               
               asm("NOP");
      }
#endif     
     ////////////////////// READ KWH A FILE from SPI-FLASH /////////////////////////////
     memset(acc_ws, 0, sizeof(acc_ws));
     long_res = ReadFileFromDevice( SPI_FLASH_KWHFILEA, acc_ws, 22);
     if(long_res == -1)
     {
        long_res =  WriteFileToDevice( SPI_FLASH_KWHFILEA, acc_ws, 20, 0);//1
       asm("NOP");
     }
     else
     {
         //設定回 kWh A的值 還原成 "瓦秒"  到 double_acc_ws 變數中
               double_acc_ws =  atof( (char *)acc_ws) * 3600000;//3.6 * 3600000;//atof( (char *) &acc_ws[10]);
               //還原成 "瓦秒"  就是將kWh值  * 3600000
        asm("NOP");
           
     }
      
     
#ifdef SDCARD_WROK
    ////////////////////// READ KWH B FILE /////////////////////////////
     res = f_open(&fp,KWHFILEB,FA_READ);
     if(res == FR_OK)
     {
               memset(acc_ws, 0, sizeof(acc_ws));
               f_read(&fp,acc_ws,22,&Size);
               
               
               res = f_close(&fp);
           
               //設定回 kWh B的值 還原成 "瓦秒" 到 double_acc_ws_B 變數中              
               double_acc_ws_B = atof( (char *)acc_ws) * 3600000; // 21.5 * 3600000;
               //還原成 "瓦秒"  就是將kWh值  * 3600000
               asm("NOP");
       
      }
      else
      {
                
               asm("NOP");
      }         
#endif     
     ////////////////////// READ KWH B FILE from SPI-FLASH /////////////////////////////
     memset(acc_ws, 0, sizeof(acc_ws));
     long_res = ReadFileFromDevice( SPI_FLASH_KWHFILEB, acc_ws, 22);
     if(long_res == -1)
     {
       asm("NOP");
          
          long_res =  WriteFileToDevice( SPI_FLASH_KWHFILEB, acc_ws, 20, 0);//1
     
     }
     else
     {
        //設定回 kWh B的值 還原成 "瓦秒" 到 double_acc_ws_B 變數中              
        double_acc_ws_B = atof( (char *)acc_ws) * 3600000; // 21.5 * 3600000;
        //還原成 "瓦秒"  就是將kWh值  * 3600000
        asm("NOP");
           
     }

#ifdef SDCARD_WROK
     ////////////////////// READ NAME A FILE /////////////////////////////
     /*
     res = f_open(&fp,NAMEA,FA_READ);
     if(res == FR_OK)
     {
                f_read(&fp,meter_name_A,20,&Size);
 
                res = f_close(&fp);
   
               asm("NOP");
       
      }
      else
      {
                
                asm("NOP");
      }       
     */
#endif     
     //////////  SPI_FLASH_NAMEA READ /////////////
     long_res = ReadFileFromDevice( SPI_FLASH_NAMEA, meter_name_A, 20);
     meter_name_A[0] = 0x00;  
     meter_name_A[1] = 0x00; 
     meter_name_A[2] = 0x08; 
     if(long_res == -1)  
          long_res =  WriteFileToDevice( SPI_FLASH_NAMEA, meter_name_A, 20, 0);//1
     
#ifdef SDCARD_WROK       
     ////////////////////// READ NAME B FILE /////////////////////////////
     /*res = f_open(&fp,NAMEB,FA_READ);
     if(res == FR_OK)
     {
                f_read(&fp,meter_name_B,20,&Size);
 
                res = f_close(&fp);
   
               asm("NOP");
       
      }
      else
      {
                
                asm("NOP");
      }    
     */
#endif
     
     //////////  SPI_FLASH_NAMEB READ /////////////
     long_res = ReadFileFromDevice( SPI_FLASH_NAMEB, meter_name_B, 20);
     meter_name_B[0] = 0x01;  
     meter_name_B[1] = 0x00; 
     meter_name_B[2] = 0x08; 
     if(long_res == -1)  
          long_res =  WriteFileToDevice( SPI_FLASH_NAMEB, meter_name_B, 20, 0);//1
     
 
     
    debbb[0] = 0;
              
    while(1)
    {
      
  
           if(wps_going==1)
                 continue;
             
           counter++;
           
           if( counter %30000 == 0)
           {
                ////////// toggle RED LED D10 ///////
                ucLEDStatus = GPIO_IF_Get(17, GPIOA2_BASE, 0x02);

                if(ucLEDStatus == 1)
                {
                      
                        GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED
                }
                else
                {
                        GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 1);  //  (GPIO17)  D10 RED
                }

           }
          
        
           if (sdcard_working == 1)
           {
                   
                     
              counter = 0;
                
           
              get_time_date();

              
              if(mm != mm_backup)
              {
                 ////////////////////// FOR UPDATE TIME TO APP EVERY MINIUTE !!!
                 TIME_A_REQUIRED = 1;
                 TIME_B_REQUIRED = 1;
                 mm_backup = mm;
                 WRITE_KWH_TO_FILE = 1;
                 
              }
     
             //////////////////////// check_create_YMD_dir Directory ////////////////////////////
#ifdef SDCARD_WROK              
             res = f_opendir(&dir,year_month_date_str);
             if(res == FR_OK)
             {
                  // Already Exists Directory
                  asm("NOP");
             }
             else if( res == FR_NO_FILE )
             {    // Non Exists Directory, We need create it !!
                 res = f_mkdir(year_month_date_str);  //create directory API
                 asm("NOP");   
                 if( res != FR_OK)
                 {
                     asm("NOP");
                 }     
             }  
             
             
             
             
             /////////////////////////// Write kWh Log File//////////////////////////////
             get_write_kwh_file_by_date_time();
             res = f_open(&fp, write_kwh_file, FA_OPEN_EXISTING); //check file exist ?
             if(res == FR_OK)
             {                         // file exist! , fclose 
                f_close(&fp);
                asm("NOP");
             }
             else
             {                         // file non-exist , create it and wirth Kwh 
                res = f_open(&fp,write_kwh_file,FA_CREATE_ALWAYS|FA_WRITE);
                sprintf(kWh_log_buf, "A=%fkWh\nB=%fkWh",double_acc_ws/3600000, double_acc_ws_B/3600000);
                res = f_write(&fp,kWh_log_buf,sizeof(kWh_log_buf),&Size);
                f_close(&fp);
                asm("NOP");
             }
               
             
             //////////////////////// protect.txt write /////////////////////////////////
             /////////////////////為了保護  kwhfilea.txt 掛掉,導致下一次開機時watt-second從 0.0開始
             
             
             res = f_open(&fp,PROTECT,FA_CREATE_ALWAYS|FA_WRITE);
             if(res == FR_OK)
             {
                res = f_write(&fp,"hello",5,&Size);  //20141117 for test apped kwh to file                
                
                f_close(&fp);
             }
             else
             {
             
                asm("NOP");
                //PRCMSOCReset();
                RebootMCU();
                ddi++;
                

  
             }  
             
             
             //////////////////////// Write KWH A  to SDCARD ///////////////////////////
             p_acc_ws = (unsigned char *)&double_acc_ws;
               
             acc_ws[0] = *p_acc_ws;
             acc_ws[1] = *(p_acc_ws+1);
             acc_ws[2] = *(p_acc_ws+2);
             acc_ws[3] = *(p_acc_ws+3); 
             acc_ws[4] = *(p_acc_ws+4); 
             acc_ws[5] = *(p_acc_ws+5); 
             acc_ws[6] = *(p_acc_ws+6); 
             acc_ws[7] = *(p_acc_ws+7);                      
                     
             res = f_open(&fp,KWHFILEA,FA_CREATE_ALWAYS|FA_WRITE);
             if(res == FR_OK)
             {
                //res = f_write(&fp,acc_ws,8,&Size);  //20141117 for test apped kwh to file
                memset(kWh_log_buf,0,sizeof(kWh_log_buf));
                //sprintf(kWh_log_buf, "A=%.6fkWh",double_acc_ws/3600000);     //20141117 for test apped kwh to file
                sprintf(kWh_log_buf, "%.6f",double_acc_ws/3600000);    //Unit: kWh
                res = f_write(&fp,kWh_log_buf,strlen(kWh_log_buf),&Size);  //20141117 for test apped kwh to file
                
                
                f_close(&fp);
               
              
                asm("NOP");
                if( res == FR_RW_ERROR)
                {
                    debbb[1]++;
                    
                  
                }
                else
                    debbb[0]++;
       
             }
             else
             {
             
                asm("NOP");
                //PRCMSOCReset();
                RebootMCU();
                ddi++;
                

  
             }   
             
#endif             
            if( WRITE_KWH_TO_FILE == 1)
            {
             ///////////// write kWh to SPI-FLASH   SPI_FLASH_KWHFILEA //////////
            memset(kWh_log_buf,0,sizeof(kWh_log_buf));
             //sprintf(kWh_log_buf, "A=%.6fkWh",double_acc_ws/3600000);     //20141117 for test apped kwh to file
            sprintf(kWh_log_buf, "%.6f",double_acc_ws/3600000);    //Unit: kWh
            long_res =  WriteFileToDevice( SPI_FLASH_KWHFILEA, (unsigned char *)kWh_log_buf, strlen(kWh_log_buf), 1);//0
            
            }
                   
            
             
#ifdef SDCARD_WROK
             //////////////////////// Write KWH B to SDCARD ///////////////////////////
             p_acc_ws = (unsigned char *)&double_acc_ws_B;
               
             acc_ws[0] = *p_acc_ws;
             acc_ws[1] = *(p_acc_ws+1);
             acc_ws[2] = *(p_acc_ws+2);
             acc_ws[3] = *(p_acc_ws+3); 
             acc_ws[4] = *(p_acc_ws+4); 
             acc_ws[5] = *(p_acc_ws+5); 
             acc_ws[6] = *(p_acc_ws+6); 
             acc_ws[7] = *(p_acc_ws+7);                      
                     
             res = f_open(&fp,KWHFILEB,FA_OPEN_ALWAYS|FA_WRITE);
             if(res == FR_OK)
             {
                //res = f_write(&fp,acc_ws,8,&Size);
                
                //sprintf(kWh_log_buf, "B=%.6fkWh",double_acc_ws_B/3600000);     //20141117 for test apped kwh to file
               memset(kWh_log_buf,0,sizeof(kWh_log_buf));
               sprintf(kWh_log_buf, "%.6f",double_acc_ws_B/3600000);  //Unit: kWh
                res = f_write(&fp,kWh_log_buf,strlen(kWh_log_buf),&Size);  //20141117 for test apped kwh to file
                
                
                f_close(&fp);
                
              
                asm("NOP");
                if( res == FR_RW_ERROR)
                {
                    debbb[1]++;
                    
                
                    
                }
                else
                    debbb[0]++;
       
               
       
             }
             else
             {
               
              
                //PRCMSOCReset();
                RebootMCU();
               
                 
                  ddi++;
                  asm("NOP");
                 
  
             }                
#endif            
             ///////////// write kWh to SPI-FLASH   SPI_FLASH_KWHFILEB //////////
              if( WRITE_KWH_TO_FILE == 1)
            {
            memset(kWh_log_buf,0,sizeof(kWh_log_buf));
            sprintf(kWh_log_buf, "%.6f",double_acc_ws_B/3600000);  //Unit: kWh
            long_res =  WriteFileToDevice( SPI_FLASH_KWHFILEB, (unsigned char *)kWh_log_buf, strlen(kWh_log_buf), 1);//0
                   WRITE_KWH_TO_FILE = 0;
            }
            
             

      
             //////////////////////// Write Name A to SDCARD ///////////////////////////
               if( WRITE_NAME_A_2_SDCARD == 1)
             {
                   WRITE_NAME_A_2_SDCARD = 0;
                   
                   //MAP_PRCMPeripheralClkDisable(PRCM_WDT, PRCM_RUN_MODE_CLK);
                   ////////////// SPI FLASH NAMEA WRITE //////////////////
                   
                   meter_name_A[0] = 0x00;  
                    meter_name_A[1] = 0x00; 
                  meter_name_A[2] = 0x08; 
                   long_res =  WriteFileToDevice( SPI_FLASH_NAMEA, meter_name_A, 20, 1);//0
                   
#ifdef SDCARD_WROK                   
                   //MAP_PRCMPeripheralClkEnable(PRCM_WDT, PRCM_RUN_MODE_CLK);
              
#endif               
               
             }

             //////////////////////// Write Name B to SDCARD ///////////////////////////
             if( WRITE_NAME_B_2_SDCARD == 1)
             {
                   WRITE_NAME_B_2_SDCARD = 0;
                   
                   
                   //MAP_PRCMPeripheralClkDisable(PRCM_WDT, PRCM_RUN_MODE_CLK);
                    ////////////// SPI FLASH NAMEB WRITE //////////////////
                   meter_name_B[0] = 0x01;  
                   meter_name_B[1] = 0x00; 
                   meter_name_B[2] = 0x08; 
                   long_res =  WriteFileToDevice( SPI_FLASH_NAMEB, meter_name_B, 20, 1);//0
                                            
                   //MAP_PRCMPeripheralClkEnable(PRCM_WDT, PRCM_RUN_MODE_CLK);
                   
#ifdef SDCARD_WROK                   
                   
                    /*
                   res = f_open(&fp,NAMEB,FA_CREATE_ALWAYS|FA_WRITE);
                   if(res == FR_OK)
                   {
                      f_write(&fp,meter_name_B,sizeof(meter_name_B),&Size);
                      f_sync(&fp);
                      res = f_close(&fp);
                 
                      //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  //GREEN LED ON
                     asm("NOP");
             
                   }
                   else
                   {
                     
                    
                      //PRCMSOCReset();
                     
        
                   }  
                       */       
#endif               
             }
         
             sdcard_working =0;
             
           }
           
#ifdef SDCARD_WROK      
           
           if (query_kwh_working == 1)
           {
                
                 res = f_open(&fp, (char *)rx_buf, FA_READ);
             
                 if(res ==FR_OK)
                 {
                      for (int i=0;i<40;i++)
                           rx_kwh_buf[i] = '\0';
                      f_read(&fp,rx_kwh_buf,30,&Size);
                 }
               
                 res = f_close(&fp);
                 query_kwh_working = 0;
                 
                 
           
    
           }
           
           if (query_kwh_working == 2)
           {
             
                   /////////////// qruey all kwh files /////////////
                  big_page_ptr = 0;
                  memset( big_page, 0, sizeof(big_page) );
                  
                  if(still_query == false)
                      res = f_opendir(&q_dir,"/KWH");
                  else 
                  {
                      memcpy (&q_dir, &last_visit_dir, sizeof(DIR) );  // Backup current DIR
                      res = FR_OK;
                      
                  }
                  
                  if( res == FR_OK)
                  {
                      //Message("Opening root directory.................... [ok]\n\n\r");
                     // Message("/\n\r");
                      ListDirectory(&q_dir);
                  }
                  else
                  {
                      //Message("Opening root directory.................... [Failed]\n\n\r");
                      asm("NOP");
                  }
                  
                  query_kwh_working = 0;
                  
                  big_page_index=0;  //for http get token ( __SL_G_KLL ) use
                  
           }
#endif           
    }
}


//*****************************************************************************
//
//! Application defined hook (or callback) function - the tick hook.
//! The tick interrupt can optionally call this
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationTickHook( void )
{
}

//*****************************************************************************
//
//! Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
	while(1)
    {

    }
}

//*****************************************************************************
//
//! Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void )
{

}

//*****************************************************************************
//
//! Application provided stack overflow hook function.
//!
//! \param  handle of the offending task
//! \param  name  of the offending task
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationStackOverflowHook( OsiTaskHandle *pxTask, signed char *pcTaskName)
{
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

void vApplicationMallocFailedHook()
{
    while(1)
  {
    // Infinite loop;
  }
}
#endif

    int             iSockID;
    int             iStatus;

int BsdTcpClient(unsigned short usPort)
{
    //creating a loop 2020/12/3
    int             iCounter;
    short           sTestBufLen;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;

    long            lLoopCount = 0;
 
 
  
    
    sTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);
    //sAddr.sin_addr.s_addr = (unsigned int)IP_ADDR;  //g_ulDestinationIp;
    iAddrSize = sizeof(SlSockAddrIn_t);
    

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    
    if( iSockID < 0 )
    {
        sl_Close(iSockID);          
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
        
    }
   
    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);          
        ASSERT_ON_ERROR(CONNECT_ERROR);
      
    }

    //iStatus =sl_Send(iSockID, g_cBsdBuf, 5, 0 );
    
    while (1)
    {
               //將讀取的值給strlumresp ，並返回接收成功或失敗
               iStatus = sl_Recv(iSockID, strlumresp, sizeof(strlumresp), 0);
     
               if(iStatus<=0)
               {
                   sl_Close(iSockID); 
                   return FALSE;
                 
               }
               else{
                    
               switch(strlumresp[0])
               {
                    case '0':
                      GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  
                      GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  
                         break;
                    case '1':
                      GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  
                      GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  
                         break;
                    case '2':
                      GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0); 
                      GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  
                         break;
                    case '3':
                      GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1); //PIN_61 = 1  (GPIO06) Relay_1 (通電)
                      GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 1  (GPIO04) Relay_0 (通電)
                         break;
                    default:
                      break;
                 
               }}
               
      

                        
   
    }


}

void sl_WlanEvtHdlr(SlWlanEvent_t *pSlWlanEvent)
{
    switch((pSlWlanEvent)->Event)
    {
      case SL_WLAN_CONNECT_EVENT:
    	  g_usMCNetworkUstate |= MCU_AP_ASSOC;
          memcpy(g_ucSSID,pSlWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_name,
                  pSlWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);          
         break;
      case SL_WLAN_DISCONNECT_EVENT:
    	  g_usMCNetworkUstate &= ~MCU_AP_ASSOC;
    	  g_usMCNetworkUstate &= ~MCU_IP_ALLOC;
          
    
          GPIO_IF_Set(16, GPIOA2_BASE, 0X01, 1); //  (GPIO16)  D7 BLUE off
          device_get_ip = 0;
          
         break;
      default:
        break;
    }
}

void sl_NetAppEvtHdlr(SlNetAppEvent_t *pNetAppEvent)
{
    switch((pNetAppEvent)->Event)
    {
      case SL_NETAPP_IPV4_ACQUIRED:
      case SL_NETAPP_IPV6_ACQUIRED:
    	  g_usMCNetworkUstate |= MCU_IP_ALLOC;
          g_uiIpAddress = pNetAppEvent->EventData.ipAcquiredV4.ip;
          
          //////// 20150811 update g_cBsdBuf2 when change ip address ////////// 
          sprintf(&g_cBsdBuf2[20],"%d.%d.%d.%d",SL_IPV4_BYTE(g_uiIpAddress,3),SL_IPV4_BYTE(g_uiIpAddress,2),
          SL_IPV4_BYTE(g_uiIpAddress,1),SL_IPV4_BYTE(g_uiIpAddress,0) );
          
//          g_ulDestinationIp = 0xff<<24 |
//                      SL_IPV4_BYTE(g_uiIpAddress,1) <<16 |
//                      SL_IPV4_BYTE(g_uiIpAddress,2) <<8 |
//                      SL_IPV4_BYTE(g_uiIpAddress,3);
//          
          
          GPIO_IF_Set(16, GPIOA2_BASE, 0X01, 0); // (GPIO16)  D7 BLUE on
          device_get_ip = 1;


           
          
          break;
      case SL_NETAPP_IP_LEASED:
    	  g_usMCNetworkUstate |= MCU_IP_LEASED;
    	  break;
      default:
        break;
    }
}

char byte2char(unsigned char u)
{
       char  hex[] = "0123456789ABCDEF";
       
       
       
       return hex[u];
}
unsigned char find_hex(unsigned char u)
{
       char  hex[] = "0123456789ABCDEF";
       char  hex2[] = "0123456789abcdef";
       
       for(int i=0;i<16;i++)
            if ( (hex[i] == u) || ( hex2[i] == u) )
                return i;
       
       
       return 0;
}
int find_packet(void)
{
    
    int i;
    for(i=Array_Ptr2-1; i>=0 ;i--)
    {
        if(BannerText2[i]== 0x55 && BannerText2[i-1]== 0xaa && BannerText2[i-22]== 0xaa && BannerText2[i-23]== 0x55)
        {
          
            asm("NOP");
            return i-21;
        }
      
        
    }
    return 0;
}

int find_packet2(unsigned char k)
{
    
    int i;
    for(i=Array_Ptr2-1; i>=0 ;i--)
    {
         //找 BannerText2[i-21]== 0x01
        if(BannerText2[i]== 0x55 && BannerText2[i-1]== 0xaa && BannerText2[i-21]== k && BannerText2[i-22]== 0xaa && BannerText2[i-23]== 0x55)
        {
          
            asm("NOP");
            return i-21;
        }
      
    }
    return 0;
}




void my_wps_pair(void)
{  
    //
    // Switch to STA mode if device is not in this mode
    //
    SwitchToStaMode(0);

    //Delete All the Stored Profile
    sl_WlanProfileDel(0xFF);  //20150210

    //Reset Connection Policy
    sl_WlanPolicySet(SL_POLICY_CONNECTION,SL_CONNECTION_POLICY(0,0,0,0,0),NULL,0);
    ResetNWP();
    //
    // Connecting to WLAN AP with WPS security, using Push Button method.
    // The AP parameters are set with static values defined at the top.
    // After this function call we will be connected and have IP address.
    //
    
    g_usMCNetworkUstate &= ~MCU_AP_ASSOC;
    g_usMCNetworkUstate &= ~MCU_IP_ALLOC;
        
    WpsConnectPushButton();
       
    sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);   
    asm("NOP");
    
}
//*****************************************************************************
//
//! This function gets triggered when HTTP Server receives Application
//! defined GET and POST HTTP Tokens.
//!
//! \param pHttpServerEvent Pointer indicating http server event
//! \param pHttpServerResponse Pointer indicating http server response
//!
//! \return None
//!
//*****************************************************************************
SlHttpServerEvent_t *ijjhnk;
void sl_HttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, SlHttpServerResponse_t *pSlHttpServerResponse)
{
	UINT8 strLenVal = 0;
        
        ijjhnk = pSlHttpServerEvent;
  
    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE:
        {
          UINT8 status, *ptr;
          int i;

          ptr = pSlHttpServerResponse->ResponseData.token_value.data;
          pSlHttpServerResponse->ResponseData.token_value.len = 0;
          
          
         if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_KWH_ALL_token,
                    strlen((const char *)GET_KWH_ALL_token)) == 0)
          {
              
              
              debbb[2] = strlen(rx_kwh_buf);
              
              for ( i=0;i<64;i++)  //15, 30, 60, 64 --> OK   // 65, 90, 150-->failed
              {
                  if(big_page[big_page_index + i] == '\0')
                       break;
                  ptr[i] = big_page[big_page_index + i];  
              }
             
              big_page_index += i;//64;
              ptr += i;//64;
              pSlHttpServerResponse->ResponseData.token_value.len += i;//64;
            
              *ptr = '\0';
              
              break;
              
          }
            
             if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_VERSION_token,
                    strlen((const char *)GET_VERSION_token)) == 0)
          {
              
              
            
              debbb[2] = strlen(fw_ver_buf);
              
              for (int i=0;i<strlen(fw_ver_buf);i++)
              {
                 
                  ptr[i] = fw_ver_buf[i];  
              }
             
              ptr += strlen(fw_ver_buf);
              pSlHttpServerResponse->ResponseData.token_value.len += strlen(fw_ver_buf);
            
              *ptr = '\0';
              
              break;
              
          }
         
         
         
          if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_KWH_token,
                    strlen((const char *)GET_KWH_token)) == 0)
          {
              
              
              debbb[2] = strlen(rx_kwh_buf);
              
              for (int i=0;i<strlen(rx_kwh_buf);i++)
              {
                 
                  ptr[i] = rx_kwh_buf[i];  
              }
             
              ptr += strlen(rx_kwh_buf);
              pSlHttpServerResponse->ResponseData.token_value.len += strlen(rx_kwh_buf);
            
              *ptr = '\0';
              
              break;
              
          }
          
          if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_token2,
                    strlen((const char *)GET_token2)) == 0)
          {
            
                asm("NOP");
                
                 Read_Array_Ptr = find_packet2(1);
                 for (int i=0;i<20;i++)//44
                 {
                  ptr[2*i]   = byte2char ( (BannerText2[Read_Array_Ptr+i]>>4) & 0x0f );
                  ptr[2*i+1] = byte2char ( BannerText2[Read_Array_Ptr+i] & 0x0f );
                    
                 }
                 
                 ptr += 40;//88
                 pSlHttpServerResponse->ResponseData.token_value.len += 40;
                 
                 *ptr = '\0';
                
                break;

          }

          if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_token3,
                    strlen((const char *)GET_token3)) == 0)
          {
            
                asm("NOP");
                
                 Read_Array_Ptr = find_packet2(0);
                 for (int i=0;i<20;i++)//44
                 {
                  ptr[2*i]   = byte2char ( (BannerText2[Read_Array_Ptr+i]>>4) & 0x0f );
                  ptr[2*i+1] = byte2char ( BannerText2[Read_Array_Ptr+i] & 0x0f );
                    
                 }
                 
                 ptr += 40;//88
                 pSlHttpServerResponse->ResponseData.token_value.len += 40;
                 
                 *ptr = '\0';
                
                break;

          }
          if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_token,
                    strlen((const char *)GET_token)) == 0)
          {
            status = GPIO_IF_Get(17, GPIOA2_BASE, 0x02);
            strLenVal = strlen(LED1_STRING);
            

            
            
            if(NAME_A_REQUIRED !=0 )
            {
                  NAME_A_REQUIRED++;
                  for (int i=0;i<20;i++)
                  {
                  ptr[2*i]   = byte2char ( (meter_name_A[i]>>4) & 0x0f );
                  ptr[2*i+1] = byte2char ( meter_name_A[i] & 0x0f );
                    
                  }
                  if(NAME_A_REQUIRED>=3)//3  5  10
                  {
                       NAME_A_REQUIRED = 0;
                       TIME_A_REQUIRED = 1;
                  }
                  
            }
            else if(NAME_B_REQUIRED !=0 )
            {
                  NAME_B_REQUIRED++;
                  for (int i=0;i<20;i++)
                  {
                  ptr[2*i]   = byte2char ( (meter_name_B[i]>>4) & 0x0f );
                  ptr[2*i+1] = byte2char ( meter_name_B[i] & 0x0f );
                    
                  }
                  if(NAME_B_REQUIRED>=3)//3  5  10
                  {
                       NAME_B_REQUIRED = 0;
                       TIME_B_REQUIRED = 1;
                  }
                  
            }       
            else if(TIME_A_REQUIRED !=0 )
            {
                  TIME_A_REQUIRED++;
                  get_time_date();
                  
                  ptr[0] = byte2char ( (0x00>>4) & 0x0f );
                  ptr[1] = byte2char (  0x00 & 0x0f );
                  
                  ptr[2] = byte2char ( (0x00>>4) & 0x0f );
                  ptr[3] = byte2char (  0x00 & 0x0f );
                  
                  ptr[4] = byte2char ( (0x02>>4) & 0x0f );
                  ptr[5] = byte2char (  0x02 & 0x0f );
                  
                  ptr[6] = byte2char ( (0x08>>4) & 0x0f );
                  ptr[7] = byte2char (  0x08 & 0x0f );
                  
                  ptr[8] = byte2char ( ss / 10 );
                  ptr[9] = byte2char ( ss % 10 );
                  
                  ptr[10] = byte2char ( mm / 10 );
                  ptr[11] = byte2char ( mm % 10 );                 
                  
                  ptr[12] = byte2char ( hh / 10 );
                  ptr[13] = byte2char ( hh % 10 );
                  
                  ptr[14] = byte2char (  date / 10 );
                  ptr[15] = byte2char (  date % 10 ); 
                  
                  ptr[16] = byte2char (  month / 10 );
                  ptr[17] = byte2char (  month % 10 );
                  
                  ptr[18] = byte2char ( 0 );
                  ptr[19] = byte2char ( day ); 

                  tt = year % 2000;
                  
                  
                  ptr[20] = byte2char ( tt/10 );  //2014的14
                  ptr[21] = byte2char (  tt%10 );      //2014的14
                  
                  ptr[22] = byte2char ( (0x20>>4) & 0x0f );  //2014的20
                  ptr[23] = byte2char (  0x20 & 0x0f );      //2014的20        
                  
                  for (int i=12;i<20;i++)
                  {
                  ptr[2*i]   = 0;//byte2char ( (meter_name_B[i]>>4) & 0x0f );
                  ptr[2*i+1] = 0;//byte2char ( meter_name_B[i] & 0x0f );
                    
                  }
                  if(TIME_A_REQUIRED>=3)//3  5  10
                       TIME_A_REQUIRED = 0;
                  
            }            
            
            else if(TIME_B_REQUIRED !=0 )
            {
                  TIME_B_REQUIRED++;
                  get_time_date();
                  
                  ptr[0] = byte2char ( (0x01>>4) & 0x0f );
                  ptr[1] = byte2char (  0x01 & 0x0f );
                  
                  ptr[2] = byte2char ( (0x00>>4) & 0x0f );
                  ptr[3] = byte2char (  0x00 & 0x0f );
                  
                  ptr[4] = byte2char ( (0x02>>4) & 0x0f );
                  ptr[5] = byte2char (  0x02 & 0x0f );
                  
                  ptr[6] = byte2char ( (0x08>>4) & 0x0f );
                  ptr[7] = byte2char (  0x08 & 0x0f );
                  
                  ptr[8] = byte2char ( ss / 10 );
                  ptr[9] = byte2char ( ss % 10 );
                  
                  ptr[10] = byte2char ( mm / 10 );
                  ptr[11] = byte2char ( mm % 10 );                 
                  
                  ptr[12] = byte2char ( hh / 10 );
                  ptr[13] = byte2char ( hh % 10 );
                  
                  ptr[14] = byte2char ( date / 10 );
                  ptr[15] = byte2char ( date % 10 ); 
                  
                  ptr[16] = byte2char ( month / 10 );
                  ptr[17] = byte2char ( month % 10 );
                  
                  ptr[18] = byte2char ( 0 );
                  ptr[19] = byte2char ( day ); 

                  tt = year % 2000;
                  
                  
                  ptr[20] = byte2char ( tt/10 );  //2014的14
                  ptr[21] = byte2char (  tt%10 );      //2014的14
                  
                  ptr[22] = byte2char ( (0x20>>4) & 0x0f );  //2014的20
                  ptr[23] = byte2char (  0x20 & 0x0f );      //2014的20        
                  
                  for (int i=12;i<20;i++)
                  {
                  ptr[2*i]   = 0;//byte2char ( (meter_name_B[i]>>4) & 0x0f );
                  ptr[2*i+1] = 0;//byte2char ( meter_name_B[i] & 0x0f );
                    
                  }
                  if(TIME_B_REQUIRED>=3)// 3  5  10
                       TIME_B_REQUIRED = 0;
                  
            }                       
        
                
                
                
            else
            {
              
                  Read_Array_Ptr = find_packet();
                  for (int i=0;i<20;i++)
                  {
                  ptr[2*i]   = byte2char ( (BannerText2[Read_Array_Ptr+i]>>4) & 0x0f );
                  ptr[2*i+1] = byte2char ( BannerText2[Read_Array_Ptr+i] & 0x0f );
                    
                  }
            
            }
            
         
          
            ptr += 40;
            pSlHttpServerResponse->ResponseData.token_value.len += 40;
            
           
            if(status & 0x01)
            {
              strLenVal = strlen(LED_ON_STRING);
              memcpy(ptr, LED_ON_STRING, strLenVal);
              ptr += strLenVal;
              pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            }
            else
            {
              strLenVal = strlen(LED_OFF_STRING);
              memcpy(ptr, LED_OFF_STRING, strLenVal);
              ptr += strLenVal;
              pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            }
            status = GPIO_IF_Get(16, GPIOA2_BASE, 0X01);  //D7 BLUE
            strLenVal = strlen(LED2_STRING);
            memcpy(ptr, LED2_STRING, strLenVal);
            ptr += strLenVal;
            pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            if(status & 0x01)
            {
              strLenVal = strlen(LED_ON_STRING);
              memcpy(ptr, LED_ON_STRING, strLenVal);
              ptr += strLenVal;
              pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            }
            else
            {
              strLenVal = strlen(LED_OFF_STRING);
              memcpy(ptr, LED_OFF_STRING, strLenVal);
              ptr += strLenVal;
              pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            }
       
            ////////////////////////Array_Ptr///////////////////////
            strLenVal = 4;//sizeof(tmp_array_ptr);
            sprintf(ptr ,"%04d",Array_Ptr2);
            //memcpy(ptr, tmp_array_ptr, strLenVal);
            ptr += strLenVal;
            pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            
            ////////////////////////Read_Array_Ptr///////////////////////           
            sprintf(ptr ,"%04d",Read_Array_Ptr);
            //memcpy(ptr, tmp_array_ptr, strLenVal);
            ptr += strLenVal;
            pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            
            *ptr = '\0';
          }

        }
        break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE:
        {
          UINT8 led, jj;
          UINT8 *ptr = pSlHttpServerEvent->EventData.httpPostData.token_name.data;

          if(memcmp(ptr, POST_token_kwh_query, strlen((const char *)POST_token_kwh_query)) == 0)
          {
              ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;
              for(jj=0;jj<strlen((char *)ptr)-3;jj++)
              {
              
                rx_buf[jj] = ptr[jj];
                           
              }
              rx_buf[jj]='\0';
              asm("NOP");
              
              if(rx_buf[0] == 'A' && rx_buf[1] == 'L' && rx_buf[2] == 'L')
                  query_kwh_working = 2;   //SMART PHONE WANT TO READ ALL KWH, BUT ONE TIME PASS < 1500BYTES
              else                  
                  query_kwh_working = 1;
              
          }
          
          
          if(memcmp(ptr, POST_token, strlen((const char *)POST_token)) == 0)
          {
            ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;
            
            for(int jj=0;jj<40;jj+=2)
            {
              
                rx_buf[jj/2] = find_hex(ptr[jj])<<4 | find_hex(ptr[jj+1])   ;
            
                
            }

            strLenVal = strlen(LED_STRING);
            if(memcmp(ptr, LED_STRING, strLenVal) != 0)
            {
              
              if( rx_buf[0]==0x01  &&   rx_buf[1]==0x00 )
              {    ///////////////////////////// device B ///////////////////////////
                   ///////////////check  on
                  if(rx_buf[2]==0x00)
                  {
                        //GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  //PIN_61 = 0  (GPIO06)
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 0  (GPIO04)
                        relay1 = 0x00;
                  }
                  ///////////////check  off
                  else if(rx_buf[2]==0x80)
                  {
                        //GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  //PIN_61 = 1  (GPIO06)
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 1  (GPIO04)
                        relay1 = 0x80;
                    
                  }  
                  else if(rx_buf[2]==0x08 && rx_buf[3]== 0)
                  {
                    ////// GET NAME B REQUIRED
                    asm("NOP");
                    NAME_B_REQUIRED = 1;
                    
                  }
                  else if(rx_buf[2]==0x08 && rx_buf[3]!= 0)
                  {
                    ////// SET NAME B REQUIRED
                    asm("NOP");
                    for (int i=3;i<20;i++)
                    {
                        meter_name_B[i] = rx_buf[i];
                    } 
                    NAME_B_REQUIRED = 1;
                    WRITE_NAME_B_2_SDCARD = 1;
                  }         
                  else if (rx_buf[2]==0x02 && rx_buf[3]!= 0)
                  { 
                        ////// SET Time B REQUIRED
                        setup_time_date_ds1302(&rx_buf[4]);
                        TIME_B_REQUIRED = 1;
                    
                  }
                  else if (rx_buf[2]==0x02 && rx_buf[3]== 0)
                  { 
                        ////// GET Time B REQUIRED
                        
                        TIME_B_REQUIRED = 1;
                    
                  }
                  else if (rx_buf[2]==0x10 && rx_buf[3]== 0)
                  { 
                        ////// Clear kWh B REQUIRED
                        
                        double_acc_ws_B = 0.0;
                    
                  }
              }
              else if( rx_buf[0]==0x00  &&   rx_buf[1]==0x00 )
              {   ////////////////////////////// devie A ////////////////////////////////
                  ///////////////check away on
                  if(rx_buf[2]==0x00)
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  //PIN_61 = 0  (GPIO06)
                        //GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 0  (GPIO04)
                        relay0 = 0x00;
                  }
                  ///////////////check away off
                  else if(rx_buf[2]==0x80)
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  //PIN_61 = 1  (GPIO06)
                        //GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 1  (GPIO04)
                        relay0 = 0x80;
                    
                  }
                  else if(rx_buf[2]==0x08 && rx_buf[3]== 0)
                  {
                    ////// GET NAME A REQUIRED
                    asm("NOP");
                    NAME_A_REQUIRED = 1;
                    
                  }
                  else if(rx_buf[2]==0x08 && rx_buf[3]!= 0)
                  {
                    ////// SET NAME A REQUIRED
                    asm("NOP");
                    for (int i=3;i<20;i++)
                    {
                        meter_name_A[i] = rx_buf[i];
                    } 
                    NAME_A_REQUIRED = 1;
                    
                    WRITE_NAME_A_2_SDCARD = 1;
                  }   
                  else if (rx_buf[2]==0x02 && rx_buf[3]!= 0)
                  { 
                        ////// SET Time A REQUIRED
                        setup_time_date_ds1302(&rx_buf[4]);
                        TIME_A_REQUIRED = 1;
                    
                  }
                  else if (rx_buf[2]==0x02 && rx_buf[3]== 0)
                  { 
                        ////// GET Time A REQUIRED
                        
                        TIME_A_REQUIRED = 1;
                    
                  }
                  
                  else if (rx_buf[2]==0x10 && rx_buf[3]== 0)
                  { 
                        ////// Clear kWh A REQUIRED
                        
                        double_acc_ws = 0.0;
                    
                  }
                  
              }
              else if( rx_buf[0]==0xff  &&   rx_buf[1]==0xff )
              {
                  ///////////////check away off
                  if(rx_buf[2]==0x80)
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  //PIN_61 = 0  (GPIO06)
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 0  (GPIO04)
                        relay0 = 0x80;
                        relay1 = 0x80;
                  }
                  ///////////////check away on
                  else if(rx_buf[2]==0x00)
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  //PIN_61 = 1  (GPIO06)
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 1  (GPIO04)
                        relay0 = 0x00;
                        relay1 = 0x00;                        
                    
                  }
                  else if( rx_buf[2]==0x04 && rx_buf[3]==0x00)
                  {
                                           
                        /*
                        NAME_A_REQUIRED = 1;
                        NAME_B_REQUIRED = 1;
                        TIME_A_REQUIRED = 1;
                        TIME_B_REQUIRED = 1;
                        */
                        asm("NOP");
                  }
           
    

    
              }
              
              
              asm("NOP");
              break;
              
            }
            ptr += strLenVal;
            led = *ptr;
            strLenVal = strlen(LED_ON_STRING);
            ptr += strLenVal;
            if(led == '1')
            {
              if(memcmp(ptr, LED_ON_STRING, strLenVal) == 0)
              {
                      GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED on
                    
                      #if DEBUG_PUTTY
                      Report("LedOn\n\r");
                      #endif
                     // Report("%c%c%c%c",0xff, 0xff, 0x04, 0x00);
                      MAP_UARTCharPut(CONSOLE,0xff);
                      MAP_UARTCharPut(CONSOLE,0xff);
                      MAP_UARTCharPut(CONSOLE,0x04);//get meter data
                      MAP_UARTCharPut(CONSOLE,0x00);
                      //Report("%c%c%c%c",0x31, 0x32, 0x38, 0x39);

              }
              else
              {
                      GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 1);  //  (GPIO17)  D10 RED off
                      #if DEBUG_PUTTY
                      Report("LedOff\n\r");
                      #endif
                     
              }
            }
            else if(led == '2')
            {

            }

          }
        }
          break;
        default:
          break;
    }
}
//****************************************************************************
//
//!	\brief Connects to the Network in AP or STA Mode - If ForceAP Jumper is
//!                                             Placed, Force it to AP mode
//!
//! \return	                	None
//
//****************************************************************************
//char pcSsidName[]="good ade7953";
char pcSsidName[]="KsMeter(      )";

void ConnectToNetwork()
{
    char ucAPSSID[32];
    unsigned short len, config_opt;

    
        
    // staring simplelink
    g_uiSimplelinkRole =  sl_Start(NULL,NULL,NULL);
    

    
    
//long slResult;
unsigned char macAddr[SL_MAC_ADDR_LEN + 2];     /* crazy bug */
unsigned char macAddrLen = SL_MAC_ADDR_LEN + 2;

sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL, &macAddrLen, macAddr);

for(int i=0;i<=5;i++)
{
  g_macaddress[i]=(macAddr[i]);
  
}

pcSsidName[8]  = byte2char ( (macAddr[3]>>4) & 0x0f );
pcSsidName[9]  = byte2char ( macAddr[3] & 0x0f );
pcSsidName[10]  = byte2char ( (macAddr[4]>>4) & 0x0f );
pcSsidName[11]  = byte2char ( macAddr[4] & 0x0f );
pcSsidName[12]  = byte2char ( (macAddr[5]>>4) & 0x0f );
pcSsidName[13]  = byte2char ( macAddr[5] & 0x0f );


g_cBsdBuf2[0] = byte2char ( (macAddr[0]>>4) & 0x0f ); //"d0:31:10:12:43:22,x,";
g_cBsdBuf2[1] = byte2char ( (macAddr[0]) & 0x0f );

g_cBsdBuf2[3] = byte2char ( (macAddr[1]>>4) & 0x0f );
g_cBsdBuf2[4] = byte2char ( (macAddr[1]) & 0x0f );

g_cBsdBuf2[6] = byte2char ( (macAddr[2]>>4) & 0x0f );
g_cBsdBuf2[7] = byte2char ( (macAddr[2]) & 0x0f );

g_cBsdBuf2[9] = byte2char ( (macAddr[3]>>4) & 0x0f );
g_cBsdBuf2[10]= byte2char ( (macAddr[3]) & 0x0f );

g_cBsdBuf2[12]= byte2char ( (macAddr[4]>>4) & 0x0f );
g_cBsdBuf2[13]= byte2char ( (macAddr[4]) & 0x0f );

g_cBsdBuf2[15]= byte2char ( (macAddr[5]>>4) & 0x0f );
g_cBsdBuf2[16]= byte2char ( (macAddr[5]) & 0x0f );


    // Device is not in STA mode and Force AP Jumper is not Connected - Switch to STA mode
    if(g_uiSimplelinkRole != ROLE_STA && g_uiDeviceModeConfig == ROLE_STA )
    {
        //Switch to STA Mode
    	sl_WlanSetMode(ROLE_STA);
        sl_Stop(SL_STOP_TIMEOUT);
        g_usMCNetworkUstate = 0;
        g_uiSimplelinkRole =  sl_Start(NULL,NULL,NULL);
    }

    //Device is not in AP mode and Force AP Jumper is Connected - Switch to AP mode
    if(g_uiSimplelinkRole != ROLE_AP && g_uiDeviceModeConfig == ROLE_AP )
    {
         //Switch to AP Mode
    	sl_WlanSetMode(ROLE_AP);
        sl_Stop(SL_STOP_TIMEOUT);
        g_usMCNetworkUstate = 0;
        g_uiSimplelinkRole =  sl_Start(NULL,NULL,NULL);
    }

    //No Mode Change Required
    if(g_uiSimplelinkRole == ROLE_AP)
    {
       //waiting for the AP to acquire IP address from Internal DHCP Server
       while (!(g_usMCNetworkUstate & MCU_IP_ALLOC))
       {

       }
       char iCount=0;
       //Read the AP SSID
       memset(ucAPSSID,'\0',AP_SSID_LEN_MAX);
       len = AP_SSID_LEN_MAX;
       config_opt = WLAN_AP_OPT_SSID;
       
       sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char*) ucAPSSID);
        
       sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName), (unsigned char*)pcSsidName);

       sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char*) ucAPSSID);
       
#if DEBUG_PUTTY
       Report("\n\rDevice is in AP Mode, Please Connect to AP [%s] and"
          "type [mysimplelink.net] in the browser \n\r",ucAPSSID);
#endif       
       //Blink LED 3 times to Indicate AP Mode
       for(iCount=0;iCount<3;iCount++)
       {
           //Turn RED LED On
           GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED on
           osi_Sleep(400);
           
           //Turn RED LED Off
           GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 1);  //  (GPIO17)  D10 RED off
           osi_Sleep(400);
       }

    }
    else
    {
    //waiting for the device to Auto Connect
    while (((!(g_usMCNetworkUstate & MCU_AP_ASSOC)) || !(g_usMCNetworkUstate & MCU_IP_ALLOC))&&
           g_ucConnectTimeout < AUTO_CONNECTION_TIMEOUT_COUNT)
    {
   
        osi_Sleep(50);
        
       
        osi_Sleep(50);
        
        g_ucConnectTimeout++;
    }
    //Couldn't connect Using Auto Profile
    if(g_ucConnectTimeout == AUTO_CONNECTION_TIMEOUT_COUNT)
    {
        //Blink Red LED to Indicate Connection Error
        GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED on
        g_usMCNetworkUstate &= ~MCU_AP_ASSOC;
        g_usMCNetworkUstate &= ~MCU_IP_ALLOC;

        Report("Use Smart Config Application to configure the device.\n\r");
        //Connect Using Smart Config
        //SmartConfigConnect();

        
        no_connected_sta_counter = 0;
        //Waiting for the device to Auto Connect
        while (!(g_usMCNetworkUstate & MCU_AP_ASSOC) || !(g_usMCNetworkUstate & MCU_IP_ALLOC))
        {
            MAP_UtilsDelay(500);
            
            GPIO_IF_GetPortNPin     (22,&uiGPIOPort,&pucGPIOPin);
	    ucPinValue = GPIO_IF_Get(22,uiGPIOPort,pucGPIOPin);
              
	    if(ucPinValue == 1)
	    {
                                    
               going_action_sw2 = 1;
	    }
	    else
	    {
                if( going_action_sw2 == 1)
                {                                                     
                    my_wps_pair();
                    going_action_sw2 = 0;
                  
                }
            }  
            
            no_connected_sta_counter++;
            
            if(no_connected_sta_counter > 1000)
                break;
           
        }

    }

    Report("\n\rDevice is in STA Mode, Connect to the AP[%s] and type"
          "IP address [%d.%d.%d.%d] in the browser \n\r",g_ucSSID,
          SL_IPV4_BYTE(g_uiIpAddress,3),SL_IPV4_BYTE(g_uiIpAddress,2),
          SL_IPV4_BYTE(g_uiIpAddress,1),SL_IPV4_BYTE(g_uiIpAddress,0));
    
    /*
    sprintf(&g_cBsdBuf2[20],"%d.%d.%d.%d",SL_IPV4_BYTE(g_uiIpAddress,3),SL_IPV4_BYTE(g_uiIpAddress,2),
          SL_IPV4_BYTE(g_uiIpAddress,1),SL_IPV4_BYTE(g_uiIpAddress,0) );
    */
  
    }
}


//****************************************************************************
//
//!	\brief Read Force AP GPIO and Configure Mode - 1(Access Point Mode)
//!                                                  - 0 (Station Mode)
//!
//! \return	                	None
//
//****************************************************************************
static void ReadDeviceConfiguration()
{
	unsigned int uiGPIOPort;
	unsigned char pucGPIOPin;
	unsigned char ucPinValue;
        
        //Read GPIO
	GPIO_IF_GetPortNPin     (SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);
        
        //If Connected to VCC, Mode is AP
	if(ucPinValue == 1)
	{
            //AP Mode
            g_uiDeviceModeConfig = ROLE_AP;
	}
	else
	{
            //STA Mode
            g_uiDeviceModeConfig = ROLE_STA;
	}

}


unsigned short ModRTU_CRC(unsigned char *buf, int len)
{
  unsigned short crc = 0xFFFF;
  int pos, i;

  for (pos = 0; pos < len; pos++) {
    crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    for ( i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}


//*****************************************************************************
//
//!    \brief Connecting to a WLAN Accesspoint
//!    This function connects to the required AP (SSID_NAME).
//!    This code example assumes the AP doesn't use WIFI security.
//!    The function will return only once we are connected 
//!    and have acquired IP address
//!
//!    \param[in] None
//!
//!    \return None
//!
//!    \note
//!
//!    \warning    If the WLAN connection fails or we don't aquire an IP address,
//!                We will be stuck in this function forever.
//
//*****************************************************************************
void 
WpsConnectPushButton()
{
    SlSecParams_t secParams;

    secParams.Key = "";
    secParams.KeyLen = 0;
    secParams.Type = SL_SEC_TYPE_WPS_PBC;

    sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams,0);
//g_usMCNetworkUstate
    wps_going=1;

    while(!(g_usMCNetworkUstate & MCU_AP_ASSOC) || !(g_usMCNetworkUstate & MCU_IP_ALLOC))
    {
        //_SlNonOsMainLoopTask();
        asm("NOP");
    }
    wps_going=0;

}

//*****************************************************************************
//
//! Check the device mode and switch to STATION(STA) mode
//! restart the NWP to activate STATION mode
//!
//! \param  iMode (device mode)
//!
//! \return None
//
//*****************************************************************************
void SwitchToStaMode(int iMode)
{
	//if(iMode != ROLE_STA)
	{
	    sl_WlanSetMode(ROLE_STA);
	    MAP_UtilsDelay(80000);
	    sl_Stop(10);
	    MAP_UtilsDelay(80000);
	    sl_Start(0,0,0);
	}

}

// Reset the NWP
void ResetNWP()
{
  sl_Stop(10);
  asm("NOP");
  sl_Start(NULL, NULL, NULL);
  asm("NOP");
}


//****************************************************************************
//
//!	\brief OOB Application Main Task - Initializes SimpleLink Driver and
//!                                              Handles HTTP Requests
//! \param[in]              	pvParameters is the data passed to the Task
//!
//! \return	                	None
//
//****************************************************************************
//long lRetVal = -1;
static void HTTPServerTask(void *pvParameters)
{
  
      tBoolean bRetcode;
      unsigned char ucLEDStatus;
      
    extern volatile unsigned long g_ulRefTimerInts;
  
//    unsigned int uiMode;
     
    memset(g_ucSSID,'\0',AP_SSID_LEN_MAX);
    
    //Read Device Mode Configuration
    ReadDeviceConfiguration();

    //Connect to Network
    ConnectToNetwork();




/////////////////////// fill ip address ///////////////////////    
    sprintf(&g_cBsdBuf2[20],"%d.%d.%d.%d",SL_IPV4_BYTE(g_uiIpAddress,3),SL_IPV4_BYTE(g_uiIpAddress,2),
          SL_IPV4_BYTE(g_uiIpAddress,1),SL_IPV4_BYTE(g_uiIpAddress,0) );
    
   g_ulDestinationIp = 0xff<<24 |
                      SL_IPV4_BYTE(g_uiIpAddress,1) <<16 |
                      SL_IPV4_BYTE(g_uiIpAddress,2) <<8 |
                      SL_IPV4_BYTE(g_uiIpAddress,3);


    BsdUdpClient(31999);
    Array_Ptr2 = 0;
    
    
    
    
        

    //////////////////// add WDT ///////////////////////
  
    //
    // Enable the peripherals used by this example.
    //
    MAP_PRCMPeripheralClkEnable(PRCM_WDT, PRCM_RUN_MODE_CLK);
    

    //
    // Set up the watchdog interrupt handler.
    //
    WDT_IF_Init(WatchdogIntHandler, MILLISECONDS_TO_TICKS(WD_PERIOD_MS));
    
    //
    // Start the timer. Once the timer is started, it cannot be disable.
    //
    MAP_WatchdogEnable(WDT_BASE);
    bRetcode = MAP_WatchdogRunning(WDT_BASE);
    if(!bRetcode)
    {
       WDT_IF_DeInit();
    }
    //////////////////// END WDT /////////////////////////
    
   
  
    
    ///////////////  uart dma start ///////////////

    uart_timer_go();
  
    
  
  
    //Handle Async Events
    while(1)
    {
      
      WatchdogIntClear(WDT_BASE);
         
         
      if ( bRxDone )
      {
        
     
     
         bRxDone = false;
    //
    // Setup DMA transfer for UART A0
    //
    UDMASetupTransfer(UDMA_CH8_UARTA0_RX,
                  UDMA_MODE_BASIC,
                  8,//8
                  UDMA_SIZE_8,
                  UDMA_ARB_2,
                  (void *)(UARTA0_BASE+UART_O_DR),
                  UDMA_SRC_INC_NONE,
                  //(void *)BannerText[Array_Ptr], 會 dma error !! 收不到data
                  &BannerText[Array_Ptr],
                  UDMA_DST_INC_8);
          MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_RX);
          
#if DEBUG_PUTTY
           Report("\t\tArray_Ptr=%d \n\r",Array_Ptr);
#endif           
      }
                 
     
      if(g_ulRefTimerInts >=11)  //11->0.387S   9->0.317S
      {
 
        
        if(Array_Ptr2 >= 1120 )
             Array_Ptr2 = 0;
        
        ///////////// Decode ADE7953 Packet ///////////////
        BannerText2[Array_Ptr2++] = 0x55;
        BannerText2[Array_Ptr2++] = 0xAA;
        if(PingPong)
        {
            BannerText2[Array_Ptr2++] = 0x00;  //MAC ADDRESS LSB
            BannerText2[Array_Ptr2++] = 0x00;  //MAC ADDRESS MSB
            
            if(relay0 & 0x80)
                 BannerText2[Array_Ptr2++] |= 0x80;  //relay0 off
            else
                 BannerText2[Array_Ptr2++] &= ~0x80;  //relay0 on

        }
        else
        {
            BannerText2[Array_Ptr2++] = 0x01;  //MAC ADDRESS LSB
            BannerText2[Array_Ptr2++] = 0x00;  //MAC ADDRESS MSB
            
             if(relay1 & 0x80)
                 BannerText2[Array_Ptr2++] |= 0x80;  //relay1 off
            else
                 BannerText2[Array_Ptr2++] &= ~0x80;  //relay1 on
        }
        //BannerText2[Array_Ptr2++] = 0x00;  //PAYLOAD LENGTH LSB
        BannerText2[Array_Ptr2++] = 0x10;  //PAYLOAD LENGTH MSB
                
        ///////////// VRMS //////////////
        
        ADE7953_REAL_DATA = BannerText[0] |  BannerText[1] <<8   |  BannerText[2] << 16 ;
        f = 3.76008599e-5 * (float)(ADE7953_REAL_DATA);
        //printf("VRMS=%ld.%03uV\n\r", (long)f, (f- (long)f )*1000);
        f *= 100;
        l = (long)f;
           
        BannerText2[Array_Ptr2++] = (l>>8) & 0xff;
        BannerText2[Array_Ptr2++] = l & 0xff;
        
        if(PingPong)
        {
                ///////////// IRMSA //////////////
                
                ADE7953_REAL_DATA = BannerText[3] |  BannerText[4] <<8   |  BannerText[5] << 16 ;
            
                if(ADE7953_REAL_DATA <= IRMS_REF[0])
                     f = IRMS_FAC[0] * (double)ADE7953_REAL_DATA;
                else if ( ADE7953_REAL_DATA <= IRMS_REF[1]) 
                     f = IRMS_FAC[1] *  (double)ADE7953_REAL_DATA;
                else
                     f = IRMS_FAC[2] *  (double)ADE7953_REAL_DATA;
              
                //f*=2.38; //20151228
                
                l = (long)f;
                
                //  printf("IRMSA = %ld.%ldA\n\r",  l/100,   l-(l/100)*100  );
               
                BannerText2[Array_Ptr2++] = (l>>8) & 0xff;
                BannerText2[Array_Ptr2++] = l & 0xff;
                asm("NOP");
                      /* 
        
                //  Electronic fuse A
                if(l/100 > FUSE_AMP_VALUE)        
                {
                      GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  //PIN_61 = 1  (GPIO06)
                      relay0 = 0x80;
                      
                      
GPIO_IF_Set(9, GPIOA1_BASE, 0x02, 1);  // (GPIO09) buzzer on
MAP_UtilsDelay(80000);
MAP_UtilsDelay(80000);
MAP_UtilsDelay(80000);
GPIO_IF_Set(9, GPIOA1_BASE, 0x02, 0);  // (GPIO09) buzzer off

                  
                }*/
        
        }
        else
        {
                ///////////// IRMSB //////////////  
                
                ADE7953_REAL_DATA = BannerText[6] |  BannerText[7] <<8   |  BannerText[8] << 16 ;
            
                if(ADE7953_REAL_DATA <= IRMS_REF[0])
                     f = IRMS_FAC[0] * (double)ADE7953_REAL_DATA;
                else if ( ADE7953_REAL_DATA <= IRMS_REF[1]) 
                     f = IRMS_FAC[1] *  (double)ADE7953_REAL_DATA-(double)8;
                else if ( ADE7953_REAL_DATA >= IRMS_REF[3] ) 
                     f = IRMS_FAC[3] *  (double)ADE7953_REAL_DATA-(double)20;  //15A
                else
                     f = IRMS_FAC[2] *  (double)ADE7953_REAL_DATA-(double)12;

                 //f*=2.38; //20151228
                 
                //printf(" f to int = %ld\n\r",(long)f);
                l = (long)f;
              
                //  printf("IRMSB = %ld.%ldA\n\r",  l/100,   l-(l/100)*100  );
                BannerText2[Array_Ptr2++] = (l>>8) & 0xff;
                BannerText2[Array_Ptr2++] = l & 0xff;      
                
                 asm("NOP");
                
                /*
                //  Electronic fuse B
                if(l/100 > FUSE_AMP_VALUE)        
                {
                      GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 1  (GPIO04)
                      relay1 = 0x80;
                      
                      
GPIO_IF_Set(9, GPIOA1_BASE, 0x02, 1);  // (GPIO09) buzzer on
MAP_UtilsDelay(80000);
MAP_UtilsDelay(80000);
MAP_UtilsDelay(80000);
GPIO_IF_Set(9, GPIOA1_BASE, 0x02, 0);  // (GPIO09) buzzer off

                  
                } */
        }
        
        
        ///////////// AVA //////////////
        
        ADE7953_REAL_DATA = BannerText[9] |  BannerText[10] <<8   |  BannerText[11] << 16 ;    
        AVA = f = 0.468 * ADE7953_REAL_DATA;
        //f *= 100;
        
        
        
        ///////////// BVA //////////////
        
        ADE7953_REAL_DATA = BannerText[12] |  BannerText[13] <<8   |  BannerText[14] << 16 ;    
        BVA = f = 0.468 * ADE7953_REAL_DATA;
        //f *= 100;
        
        
        if(PingPong)
        {
             //////////////  AWATT ///////////////     
              
             ADE7953_REAL_DATA = BannerText[15] |  BannerText[16] <<8   |  BannerText[17] << 16 ;  
             
             ADE7953_REAL_DATA_tmp =  ADE7953_REAL_DATA & 0x00ff0000;
             if(  ADE7953_REAL_DATA_tmp == 0x00ff0000)               
             //if(  ADE7953_REAL_DATA & 0x00ff0000)
             {
                       l = 0;
             }
             else
             {
                       AWATT = f = 0.468 * ADE7953_REAL_DATA;
                       //f *= 100;
                       l = (long)f;
             }
             asm("nop");
             BannerText2[Array_Ptr2+3] = (l>>16) & 0xff;
             BannerText2[Array_Ptr2+4] = (l>>8) & 0xff;
             BannerText2[Array_Ptr2+5] = l & 0xff;
             
             
             ////////////////  PFA //////////////     
             f = AWATT / AVA;
             f *= 100;
             l = (long)f;
             BannerText2[Array_Ptr2+2] = l;
             
             
             /////////////// kWhA ///////////////
             
             ws = AWATT * 0.01 * 0.387;  // 0.317 ==> 0.3s (sample period)

             double_acc_ws += ws;  // 277.778e-6 = 1/360
   
             double_acc_wh = double_acc_ws / 3600;          


             l = (unsigned long)double_acc_wh;
             BannerText2[Array_Ptr2+6] = (l & 0xff000000)>>24;
             BannerText2[Array_Ptr2+7] = (l & 0x00ff0000)>>16;
             BannerText2[Array_Ptr2+8] = (l & 0x0000ff00)>>8;
             BannerText2[Array_Ptr2+9] = (l & 0x000000ff) ;
             
             asm("NOP");
             

        }
        else
        {
             //////////////  BWATT ///////////////     
              
             ADE7953_REAL_DATA = BannerText[18] |  BannerText[19] <<8   |  BannerText[20] << 16 ;  
                 
             ADE7953_REAL_DATA_tmp =  ADE7953_REAL_DATA & 0x00ff0000;
             if(  ADE7953_REAL_DATA_tmp == 0x00ff0000)             
            // if(  ADE7953_REAL_DATA & 0x00ff0000)
             {
                       l = 0;
             }
             else
             {
                       BWATT = f = 0.468 * ADE7953_REAL_DATA;
                       //f *= 100;
                       l = (long)f;
             }
             asm("NOP");
             BannerText2[Array_Ptr2+3] = (l>>16) & 0xff;
             BannerText2[Array_Ptr2+4] = (l>>8) & 0xff;
             BannerText2[Array_Ptr2+5] = l & 0xff;
             
             
             ////////////////  PFB //////////////    
             f = BWATT / BVA;
             f *= 100;
             l = (long)f;
             BannerText2[Array_Ptr2+2]  = l;
             
             
             /////////////// kWhB ///////////////
             
             ws = BWATT * 0.01 * 0.387;  // 0.317 ==> 0.3s (sample period)

             double_acc_ws_B += ws;  // 277.778e-6 = 1/360
   
             double_acc_wh = double_acc_ws_B / 3600;          

             l = (unsigned long)double_acc_wh;
             BannerText2[Array_Ptr2+6] = (l & 0xff000000)>>24;
             BannerText2[Array_Ptr2+7] = (l & 0x00ff0000)>>16;
             BannerText2[Array_Ptr2+8] = (l & 0x0000ff00)>>8;
             BannerText2[Array_Ptr2+9] = (l & 0x000000ff) ;
             
             
        }

        ////////////////  PERIOD //////////////  
        ADE7953_REAL_DATA = BannerText[21] |  BannerText[22] <<8 ;  
        f = 223800.0 / (float)(ADE7953_REAL_DATA+1);
             //printf("PERIOD=%fHz\n\r", 233000.0 / (float)(ADE7953_REAL_DATA+1) );         
             //printf("%ld.%03uHz\n\r",  (long)f, (f- (long)f )*1000);                      
        f *= 100;
        l = (long)f;
        BannerText2[Array_Ptr2++] = (l>>8) & 0xff;
        BannerText2[Array_Ptr2++] = l & 0xff;     
           
        
        /////////////// CRC //////////////
        crc_value = ModRTU_CRC(&BannerText2[Array_Ptr2-6],14);
        BannerText2[Array_Ptr2+9] = crc_value >>8;
        BannerText2[Array_Ptr2+8] = crc_value &0xff;
        
        
        Array_Ptr2+=10;
        
        
             
        BannerText2[Array_Ptr2++] = 0xAA;  
        BannerText2[Array_Ptr2++] = 0x55;
        asm("NOP");
        Array_Ptr = 0; // RESET Array_Ptr FOR DATA STORE FROM ADE7953

        
        
        
               
        //////////////////////SW2 GPIO_22////////////////////////////// WPS PAIR
	GPIO_IF_GetPortNPin     (22,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(22,uiGPIOPort,pucGPIOPin);
              
	if(ucPinValue == 1)
	{
            
            asm("NOP");
            
            going_action_sw2 = 1;
	}
	else
	{
            if( going_action_sw2 == 1)
            {
                going_action_sw2 = 0;
                
                
                
                /*
                 ///////////////check away on
                  if(relay0==0x00)
                  {
                        
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 1  (GPIO04)
                        relay0 = 0x80;
                  }
                  ///////////////check away off
                  else if(relay0==0x80)
                  {
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 0  (GPIO04)                        
                        relay0 = 0x00;
                    
                  */
                MAP_PRCMPeripheralClkDisable(PRCM_WDT, PRCM_RUN_MODE_CLK);
                //WDT_IF_DeInit();
                my_wps_pair();
                asm("NOP");
                //PRCMSOCReset();
                RebootMCU();
                 
            }
           
          
	}
        
     
        //////////////////////SW3 GPIO_13//////////////////////////////
        GPIO_IF_GetPortNPin     (13,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(13,uiGPIOPort,pucGPIOPin);
              
	if(ucPinValue == 1)
	{
            going_action_sw3 = 1;
            asm("NOP");
	}
	else
	{
            if( going_action_sw3 == 1)
            {
                going_action_sw3 = 0;
                
                BsdUdpClient(31999);
                
                
                
                
                 
                 ///////////////check away on
                  if(relay1==0x00 || relay0==0x00 )
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 0);  //PIN_61 = 1  (GPIO06)
                        relay1 = 0x80;
                        relay0 = 0x80;
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 0);  //PIN_59 = 0  (GPIO04)
                       
                  }
                  ///////////////check away off
                  else //if(relay1==0x80)
                  {
                        GPIO_IF_Set(6, GPIOA0_BASE, 0x40, 1);  //PIN_61 = 0  (GPIO06)                
                        relay1 = 0x00;
                        relay0 = 0x00;
                        GPIO_IF_Set(4, GPIOA0_BASE, 0x10, 1);  //PIN_59 = 0  (GPIO04)
                    
                  }  
                  
                  
            }
            asm("NOP");
	}
         
 
        
     //
    // Disable the FIFO.
    //
        

           
      HWREG(UARTA0_BASE + UART_O_LCRH) &= ~(UART_LCRH_FEN);//成功清掉uart fifo裏多餘的資料

      asm("NOP");  

      HWREG(UARTA0_BASE + UART_O_LCRH) |= UART_LCRH_FEN;//將將uart fifo  enable起來
        

       
        
          
         bRxDone = false;
         //
         // Setup DMA transfer for UART A0
         //
         MAP_UARTDMADisable(UARTA0_BASE,UART_DMA_RX);
         
        sdcard_working = 1; 
   //     while (sdcard_working);   20141124解除, test OK
        //g_ulRefTimerInts = 0;    
         
         
         
         UDMASetupTransfer(UDMA_CH8_UARTA0_RX,
                  UDMA_MODE_BASIC,
                  8,//8
                  UDMA_SIZE_8,
                  UDMA_ARB_2,
                  (void *)(UARTA0_BASE+UART_O_DR),
                  UDMA_SRC_INC_NONE,
                  //(void *)BannerText[Array_Ptr], 會 dma error !! 收不到data
                  &BannerText[Array_Ptr],
                  UDMA_DST_INC_8);
          MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_RX);

          PingPong = ~PingPong;
          
            ucLEDStatus = GPIO_IF_Get(15, GPIOA1_BASE, 0x80);

            if(ucLEDStatus == 1)
            {
                  
                    GPIO_IF_Set(15, GPIOA1_BASE, 0x80, 0);  //  (GPIO15)  D6 GREEN on
            }
            else
            {
                    GPIO_IF_Set(15, GPIOA1_BASE, 0x80, 1);  // (GPIO15) D6 GREEN off
            }

            
          g_ulRefTimerInts = 0;   
  
      }
       
      
    
    }
}
//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
/*static void DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t	  CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

*/



//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void
TimerBaseIntHandler(void)
{
  //
  // Clear the timer interrupt.
  //
  Timer_IF_InterruptClear(g_ulBase);

  g_ulTimerInts ++;

  //MAP_UARTCharPut(CONSOLE,0X39);
              
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int tcpsendcount=0;
int firstset=0;
void
TimerRefIntHandler(void)
{

  //
  // Clear the timer interrupt.
  //
  Timer_IF_InterruptClear(g_ulRefBase);

  
  if(g_ulRefTimerInts ==0) 
  {  //READ VRMS
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x1c);
  }
  else if(g_ulRefTimerInts ==1) 
  {  //READ IRMSA
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x1a);
    
  }
  else if(g_ulRefTimerInts ==2) 
  {  //READ IRMSB
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x1b);
    
  }  
  else if(g_ulRefTimerInts ==3) 
  {  //READ AVA
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x10);
    
  }    
  else if(g_ulRefTimerInts ==4) 
  {  //READ BVA
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x11);
    
  }    
  else if(g_ulRefTimerInts ==5) 
  {  //READ ADE7953_REG_AWATT
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x12);
    
  }      
  else if(g_ulRefTimerInts ==6) 
  {  //READ ADE7953_REG_BWATT
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x13);
    
  }    /*
  else if(g_ulRefTimerInts ==7) 
  {  //READ ADE7953_REG_PFA
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x01);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x0A);
    
  }  */    
  else if(g_ulRefTimerInts ==7) 
  {  //READ ADE7953_REG_PERIOD
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x01);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x0E);
    
  }      
  else if(g_ulRefTimerInts ==8) 
  {  //READ ADE7953_REG_PERIOD
     MAP_UARTCharPut(CONSOLE, 0x35);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x03);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x7F);
    
  }  
  
  
  g_ulRefTimerInts ++;
  
  if(g_ulRefTimerInts > 95)//100 OK
        g_ulRefTimerInts = 100;   //for debug , adjust to 0 after debug
    
 
  
  
  
  ////////// for TCP send Use////////////////////

  
  /*
  tcpsendcount+=1;
  if(tcpsendcount>=100 && iStatus>0)
    {
      
      PACKET_METER_INFO_PREPARE_TO_TCP_SERVER();

      sl_Send(iSockID, g_csend, 28, 0 );

      tcpsendcount=0;
    }
  */

  
}





//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
	MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//							uart isr FUNCTION
//****************************************************************************


static void UARTIntHandler()
{
  if(!bRxDone)
  {
    MAP_UARTDMADisable(UARTA0_BASE,UART_DMA_RX);
    bRxDone = true;
    
    Array_Ptr +=8;
    if(Array_Ptr >= UART_RX_LENGTH)
         Array_Ptr = 0;
    
   
  }
  else
  {
    MAP_UARTDMADisable(UARTA0_BASE,UART_DMA_TX);
  }

 // MAP_UARTIntClear(UARTA0_BASE,UART_INT_DMATX|UART_INT_DMARX);
  MAP_UARTIntClear(UARTA0_BASE,UART_INT_DMARX);
  
 
  
 
}


void ade7953_init(void)
{
  
    MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);   // 52358 --> 1.882ms   40000 --> 946.9us    43500  --> 1.149ms   42000 --> 1.042ms
                                          // 41800 --> 1.04ms   41000 --> 935.3us      41500-->1.038ms
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); //60358  --> 2.348ms    55358-->2.074ms
     MAP_UARTCharPut(CONSOLE, 0xFE);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0xAD);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
    
    
    MAP_UtilsDelay(454300);  //454300 --> 31.982ms
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x01);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x20);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x30);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
    
    
    MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x07);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);

    
    MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x81);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x40);
    
        MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x80);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x40);
    
    
    MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x8C);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x40);    
    
    
    MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x08);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x03);
     // MAP_UARTCharPut(CONSOLE, 0x04);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);    
    
    
    
   MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x09);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x03); //
     // MAP_UARTCharPut(CONSOLE, 0x04);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);    
     
    
    
    MAP_UtilsDelay(454300);
    
     MAP_UARTCharPut(CONSOLE, 0xCA);
     MAP_UtilsDelay(41300);  
     MAP_UARTCharPut(CONSOLE, 0x02);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x01);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x05);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);
     MAP_UtilsDelay(41300); 
     MAP_UARTCharPut(CONSOLE, 0x00);    
       
    
    
     MAP_UtilsDelay(564300);
    
    
    
}

//****************************************************************************
//							DS1302 FUNCTION
//****************************************************************************

void DS1302_write(unsigned char addr,unsigned char mydata)

{

    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 0); //CE=0;
    asm("NOP");
    GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);  //SCLK=0;
    asm("NOP");

    GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 1);  //IO=1;
    asm("NOP");
    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 1); //CE=1;
    asm("NOP");

    DS1302_wrbyte(addr);

    DS1302_wrbyte(mydata);

    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 1);  //CE=1;
    asm("NOP");
    GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);  //SCLK=0;
    asm("NOP");

     GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 1);  //IO=1;
    asm("NOP");
    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 0);  //CE=0;
    asm("NOP");

}

unsigned char DS1302_read(unsigned char addr)

{

    unsigned char rdbyte;

    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 0); //CE=0;
    asm("NOP");
    GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);//SCLK=0;
    asm("NOP");

    GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 1);  //IO=1;
    asm("NOP");
    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 1); //CE=1;
    asm("NOP");

    DS1302_wrbyte(addr);

    rdbyte = DS1302_rdbyte();

    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 1);  //CE=1;
    asm("NOP");
    GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);//SCLK=0;
    asm("NOP");

    GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 1);  //IO=1;
    asm("NOP");
    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 0);  //CE=0;
    asm("NOP");

    return rdbyte;

}

void DS1302_wrbyte(unsigned char wrbyte)

{

    unsigned char i=8;
    
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);  //SET IO OUT
    
    while(i-->0)

    {

        if((wrbyte & 0x01)!=0)  GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 1);//IO=1;

        else                    GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 0);//IO=0;

       GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 1);  //SCK=1;

        wrbyte>>=1;

       GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);  //SCK=0;

    }

}

unsigned char DS1302_rdbyte()
{

    unsigned char i=8,rdbyte=0;
    
    //ucPinValue = HWREG(0X40004400); // 可檢查GPIO DIRECTION

    MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_IN);  //SET IO IN
    
    //ucPinValue = HWREG(0X40004400); // 可檢查GPIO DIRECTION
         
    while(i-->0)

    {
      
        rdbyte>>=1;

        GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);  //SCK=0;
        
        ucPinValue = GPIO_IF_Get(7,GPIOA0_BASE,0x80);
        if( ucPinValue ) 
        {
          rdbyte|=0x80;
        
        }
        else
        {
            asm("NOP");
        }
            
        GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 1);  //SCK=1;

    }

    MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);  //SET IO OUT
    
    return rdbyte;

}

void setup_time_date_ds1302(unsigned char *p)
{
  
    DS1302_write(0x8E,0x00);    //unlock WP

    DS1302_write(0x80,p[0]);    //秒

    DS1302_write(0x82,p[1]);    //分
   
    DS1302_write(0x84,p[2]);    //時

    
    
    DS1302_write(0x86,p[3]);    //DATE
    
    DS1302_write(0x88,p[4]);    //MONTH
    
    DS1302_write(0x8A,p[5]);    //day
        
    DS1302_write(0x8C,p[6]);    //YEAR
    
   
    
    DS1302_write(0x8E,0x80);    //LOCK WP
  
}




void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
 
}










void uart_timer_go(void)
{
  
   
    //
    // Setup DMA transfer for UART A0
    //
    UDMASetupTransfer(UDMA_CH8_UARTA0_RX,
                  UDMA_MODE_BASIC,
                  8,//8
                  UDMA_SIZE_8,
                  UDMA_ARB_2,
                  (void *)(UARTA0_BASE+UART_O_DR),
                  UDMA_SRC_INC_NONE,
                  //(void *)BannerText,
                  &BannerText[Array_Ptr],
                  UDMA_DST_INC_8);

    //
    // Enable DMA request from UART
    //
    MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_RX);
    

#ifdef SDCARD_WROK
    
    //
    // Enable MMCHS
    //
    MAP_PRCMPeripheralClkEnable(PRCM_SDHOST,PRCM_RUN_MODE_CLK);

    //
    // Reset MMCHS
    //
    MAP_PRCMPeripheralReset(PRCM_SDHOST);

    //
    // Configure MMCHS
    //
    MAP_SDHostInit(SDHOST_BASE);

    //
    // Configure card clock
    //
    MAP_SDHostSetExpClk(SDHOST_BASE,MAP_PRCMPeripheralClockGet(PRCM_SDHOST),10000000);  

#endif    
    
    ade7953_init();//20180922 
    
    
    g_ulRefTimerInts = 0;

 
  //
  // Base address for first timer
  //
  g_ulBase = TIMERA0_BASE;
  //
  // Base address for second timer
  //
  g_ulRefBase = TIMERA1_BASE;
  //
  // Configuring the timers
  //
  //Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
  Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

  //
  // Setup the interrupts for the timer timeouts.
  //
  //Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
  Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);
  //////////////  

  //
  // Turn on the timers
  //

//  Timer_IF_Start(g_ulRefBase, TIMER_A,
//                  //PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 20);//250ms
//                   // PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 166);//30ms
//                    PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 142);//35ms
    Timer_IF_Start(g_ulRefBase, TIMER_A,
                  35);//35ms
  
  

    //bRxDone = 0;
    bRxDone = true;
    
  
}
void tcp_client_sender(void *pvParameters)
{
  
  while(1)
  {
       if(iStatus>0)
        {
           PACKET_METER_INFO_PREPARE_TO_TCP_SERVER();

           sl_Send(iSockID, g_csend, 28, 0 );
        }
       MAP_UtilsDelay(830000 * 5); //-->0.2s x 5

    
  }
 
  
  
}


void main()
{

    //Board Initialization
    BoardInit();
    
    
    bRxDone = false;

    //
    // Initialize uDMA
    //
    UDMAInit();
    
    

    PinMuxConfig();
    
    //
    // Set the SD card clock as output pin
    //
    MAP_PinDirModeSet(PIN_07,PIN_DIR_MODE_OUT);  
    
    
    
    
    //
    // Register interrupt handler for UART
    //
    MAP_UARTIntRegister(UARTA0_BASE,UARTIntHandler);
    //
    // Enable DMA done interrupts for uart
    //
    MAP_UARTIntEnable(UARTA0_BASE,UART_INT_DMARX);

    //
    // Initialising the Terminal.
    //
    MAP_UARTConfigSetExpClk(CONSOLE,MAP_PRCMPeripheralClockGet(CONSOLE_PERIPH),
                            4800,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
 
    
    

    
    //Change Pin 58 Configuration from Default to Pull Down
    PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);
 
    
   
    //
    // Initialize GREEN and ORANGE LED
    //
    GPIO_IF_LedConfigure(LED1|LED2|LED3);
    
  
    //Turn Off the LEDs
    GPIO_IF_LedOff(MCU_ALL_LED_IND);
   
    

      GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 0);  //  (GPIO17)  D10 RED on
      GPIO_IF_Set(15, GPIOA1_BASE, 0x80, 0);  //  (GPIO15)  D6 GREEN on
      
    
      GPIO_IF_Set(16, GPIOA2_BASE, 0X01, 0); // (GPIO16)  D7 BLUE on
   
     
      asm("NOP");
    
      GPIO_IF_Set(17, GPIOA2_BASE, 0x02, 1);  //(GPIO17) D10 RED off
      GPIO_IF_Set(15, GPIOA1_BASE, 0x80, 1);  // (GPIO15) D6 GREEN off
      
    
      GPIO_IF_Set(16, GPIOA2_BASE, 0X01, 1); //  (GPIO16)  D7 BLUE off
   
   

      
      
    /*  
    //
    // Configure PIN_63 for ds1302  SCLK
    //  
    GPIO_IF_Set(8, GPIOA1_BASE, 0x01, 0);  //PIN_63 = 0  (GPIO07)
       
    
    //
    // Configure PIN_50 (GPIO_00) for ds1302  CE
    //  

    GPIO_IF_Set(0, GPIOA0_BASE, 0x01, 0);  //PIN_50 = 0  (GPIO00)
            
    //
    // Configure PIN_62 (GPIO_07) for ds1302  IO 
    //  
  
    GPIO_IF_Set(7, GPIOA0_BASE, 0x80, 0);  //PIN_62 = 0  (GPIO07)
   
  */
      
    // 初始化係數

    DS1302_write(0x8E,0x00);     //unlock WP
    
   DS1302_write(0x90,0xA9);    //super cap   //a9--> 2k , 2 diodes
/*
    DS1302_write(0x8E,0x00);     //unlock WP

    DS1302_write(0x80,0x00);    //00秒

    DS1302_write(0x82,0x59);    //35分
   
    DS1302_write(0x84,0x23);    //23時

    
    
    DS1302_write(0x86,0x22);    //22 DATE
    
    DS1302_write(0x88,0x11);    //11 MONTH
    
    DS1302_write(0x8A,0x03);    //WEDNESDAY
        
    DS1302_write(0x8C,0x14);    //2014 YEAR
    
    
    
    
    DS1302_write(0x8E,0x80);    //LOCK WP
 */   
    
    
 
    
    
    ////////////////////// BUTTON SW TEST /////////////////////////////////
     
        //////////////////////SW2 GPIO_22//////////////////////////////
	GPIO_IF_GetPortNPin     (22,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(22,uiGPIOPort,pucGPIOPin);
        
       
	if(ucPinValue == 1)
	{
            
            asm("NOP");
	}
	else
	{
            
            asm("NOP");
	}
   
        
        
        //////////////////////SW3 GPIO_13//////////////////////////////
        GPIO_IF_GetPortNPin     (13,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(13,uiGPIOPort,pucGPIOPin);
        
       
	if(ucPinValue == 1)
	{
            
            asm("NOP");
	}
	else
	{
            
            asm("NOP");
	}
        
        
        
        // Configure PIN_3 (GPIO_12)  for IR  INPUT
        GPIO_IF_GetPortNPin     (12,&uiGPIOPort,&pucGPIOPin);
	ucPinValue = GPIO_IF_Get(12,uiGPIOPort,pucGPIOPin);
        
	if(ucPinValue == 1)
	{
            
            asm("NOP");
	}
	else
	{
            
            asm("NOP");
	}
    
        

    //
    // Configure the UART Tx and Rx FIFO level to 1/8 i.e 2 characters
    //
    UARTFIFOLevelSet(UARTA0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
    

    //GPIO_IF_LedOn( MCU_ORANGE_LED_GPIO);
    bRxDone = false;
    Array_Ptr = 0;
 /*  
    //
    // Setup DMA transfer for UART A0
    //
    UDMASetupTransfer(UDMA_CH8_UARTA0_RX,
                  UDMA_MODE_BASIC,
                  8,//8
                  UDMA_SIZE_8,
                  UDMA_ARB_2,
                  (void *)(UARTA0_BASE+UART_O_DR),
                  UDMA_SRC_INC_NONE,
                  //(void *)BannerText,
                  &BannerText[Array_Ptr],
                  UDMA_DST_INC_8);

    //
    // Enable DMA request from UART
    //
    MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_RX);
    

#ifdef SDCARD_WROK
    
    //
    // Enable MMCHS
    //
    MAP_PRCMPeripheralClkEnable(PRCM_SDHOST,PRCM_RUN_MODE_CLK);

    //
    // Reset MMCHS
    //
    MAP_PRCMPeripheralReset(PRCM_SDHOST);

    //
    // Configure MMCHS
    //
    MAP_SDHostInit(SDHOST_BASE);

    //
    // Configure card clock
    //
    MAP_SDHostSetExpClk(SDHOST_BASE,MAP_PRCMPeripheralClockGet(PRCM_SDHOST),10000000);  

#endif    
    
    ade7953_init();
    
    
    g_ulRefTimerInts = 0;

 
  //
  // Base address for first timer
  //
  g_ulBase = TIMERA0_BASE;
  //
  // Base address for second timer
  //
  g_ulRefBase = TIMERA1_BASE;
  //
  // Configuring the timers
  //
  //Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
  Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

  //
  // Setup the interrupts for the timer timeouts.
  //
  //Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
  Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);
  //////////////  

  //
  // Turn on the timers
  //

//  Timer_IF_Start(g_ulRefBase, TIMER_A,
//                  //PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 20);//250ms
//                   // PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 166);//30ms
//                    PERIODIC_TEST_CYCLES * PERIODIC_TEST_LOOPS / 142);//35ms
    Timer_IF_Start(g_ulRefBase, TIMER_A,
                  35);//35ms
  
  

    //bRxDone = 0;
    bRxDone = true;
    */     
    //
    // Simplelinkspawntask
    //
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    
    //
    // Create HTTP Server Task
    //
    osi_TaskCreate(HTTPServerTask, (signed char*)"HTTPServerTask",OSI_STACK_SIZE, NULL, OOB_TASK_PRIORITY, NULL );//OOB_TASK_PRIORITY
   
    
    //
    // Create the SDCARD  task
    //
    osi_TaskCreate( SDCARDTask, ( signed char * ) "TASK3", OSI_STACK_SIZE,NULL,  OOB_TASK_PRIORITY, pTaskHandle_sdcard );
    //
    // Start OS Scheduler
    //
    
     
    //
    // Create the UDP SERVER  task
    //
    osi_TaskCreate( UDPSERVERTask, ( signed char * ) "UDPSERVERTask", OSI_STACK_SIZE,NULL,  OOB_TASK_PRIORITY, NULL );
  
    //
    // Create the tcp_client  task
    // 
    osi_TaskCreate( tcp_client, ( signed char * ) "tcp_client", OSI_STACK_SIZE,NULL,  OOB_TASK_PRIORITY, NULL );
    
    
    // Create the tcp_client_sender task
    osi_TaskCreate( tcp_client_sender, ( signed char * ) "tcp_client_sender", OSI_STACK_SIZE,NULL,  OOB_TASK_PRIORITY, NULL );
    
    //
    // Start OS Scheduler
    //
    osi_start();
    
  

 }



void PACKET_METER_INFO_PREPARE_TO_TCP_SERVER()//收集20Btyes到g_csend[],準備每5秒送給TCP SEVER
{
  
     Read_Array_Ptr = find_packet();
     
     for (int i=0;i<20;i++)
     {
         g_csend[i]=BannerText2[Read_Array_Ptr+i];

     }
     switch_buf[0]=GPIO_IF_Get(6,GPIOA0_BASE,0x40);//讀取switch狀態
     switch_buf[1]=GPIO_IF_Get(4,GPIOA0_BASE,0x10);//讀取switch狀態
     
        
     g_csend[20]=switch_buf[0];
     g_csend[21]=switch_buf[1];
     for(int i=0;i<=5;i++)
     {
     
          g_csend[22+i]=g_macaddress[i];
     }

     
     
     
     
     asm("NOP");
}


