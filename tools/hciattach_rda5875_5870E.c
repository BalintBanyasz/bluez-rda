/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010-2011  RDA Micro <anli@rdamicro.com>
 *  This file belong to RDA micro
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>

typedef unsigned int __u32;
typedef unsigned short __u16;
typedef unsigned char __u8;

#define TCC_BT_DEVICE_PATH			"/dev/tcc_bt_dev"
#define BT_DEV_MAJOR_NUM 			234
#define IOCTL_BT_DEV_POWER   	_IO(BT_DEV_MAJOR_NUM, 100)
#define IOCTL_BT_DEV_CTRL	_IO(BT_DEV_MAJOR_NUM, 102)

//#define __RDA_CHIP_R10_5870E__   //使用RDA5870E
//#define __RDA_CHIP_R11_5875__    //使用RDA5875
#define __RDA_CHIP_R11_5876__      //使用RDA5875Y,RDA5876
//#define EXTERNAL_32K               //若外部输入32K时钟到PIN#10，开此宏.


/*此函数实现拉高LDO_DON,并延时50ms*/
void RDA_pin_to_high(int fd)
{
#if 0
   RDA_BT_POWER_ON;//open LDO_ON
   usleep(5000); 
   GPIO_SetBtReset(SCI_TRUE);//open reset  
   usleep(20000);
#endif
	int bt_on_off = 1;
	ioctl(fd, IOCTL_BT_DEV_CTRL, &bt_on_off);		
}
/*此函数实现拉低LDO_DON,并延时50ms*/
void RDA_pin_to_low(int fd)
{
#if 0
   RDA_BT_POWER_OFF;//shut down LDO_ON 
   usleep(5000);
   GPIO_SetBtReset(SCI_FALSE);//shut down reset  
   usleep(20000);
#endif
	int bt_on_off = 0;
	ioctl(fd, IOCTL_BT_DEV_CTRL, &bt_on_off);		
}

void rdabt_write_memory(int fd,__u32 addr,__u32 *data,__u8 len,__u8 memory_type)
{
   __u16 num_to_send;	
   __u16 i,j;
   __u8 data_to_send[256]={0};
   __u32 address_convert;
   
   data_to_send[0] = 0x01;
   data_to_send[1] = 0x02;
   data_to_send[2] = 0xfd;
   data_to_send[3] = (__u8)(len*4+6);
   data_to_send[4] = (memory_type/*+0x80*/);  // add the event display
   data_to_send[5] = len;
   if(memory_type == 0x01)
   {
      address_convert = addr*4+0x200;
      data_to_send[6] = (__u8)address_convert;
      data_to_send[7] = (__u8)(address_convert>>8);
      data_to_send[8] = (__u8)(address_convert>>16);
      data_to_send[9] = (__u8)(address_convert>>24);	  
   }
   else
   {
      data_to_send[6] = (__u8)addr;
      data_to_send[7] = (__u8)(addr>>8);
      data_to_send[8] = (__u8)(addr>>16);
      data_to_send[9] = (__u8)(addr>>24);
   }
   for(i=0;i<len;i++,data++)
   {
       j=10+i*4;
       data_to_send[j] =  (__u8)(*data);
       data_to_send[j+1] = (__u8)((*data)>>8);
       data_to_send[j+2] = (__u8)((*data)>>16);
       data_to_send[j+3] = (__u8)((*data)>>24);
   }
   num_to_send = 4+data_to_send[3];

   write(fd,&(data_to_send[0]),num_to_send);
   	
}



void RDA_uart_write_simple(int fd,__u8* buf,__u16 len)
{
    __u16 num_send;	
	write(fd,buf,len);
	usleep(10000);//10ms?
}

void RDA_uart_write_array(int fd,__u32 buf[][2],__u16 len,__u8 type)
{
   __u32 i;
   for(i=0;i<len;i++)
   {
      rdabt_write_memory(fd,buf[i][0],&buf[i][1],1,type);
      usleep(12000);//12ms?
   }	
}

#ifdef __RDA_CHIP_R11_5876__
__u32 rdabt_rf_init_12[][2] = 
{   
{0x0000003f,0x00000000},//
{0x00000001,0x00001FFF},//                                                               
{0x00000006,0x000007F7},//padrv_set,increase the power.                                  
{0x00000008,0x000001E7},//                                                               
{0x00000009,0x00000520},//                                                               
{0x0000000B,0x000003DF},//filter_cap_tuning<3:0>1101                                     
{0x0000000C,0x000085E8},//                                                               
{0x0000000F,0x00000DBC},// 0FH,16'h1D8C; 0FH,16'h1DBC;adc_clk_sel=1 20110314 ;adc_digi_pw
{0x00000012,0x000007F7},//padrv_set,increase the power.                                  
{0x00000013,0x00000327},//agpio down pullen .                                            
{0x00000014,0x00000CCC},//h0CFE; bbdac_cm 00=vdd/2.                                      
{0x00000015,0x00000526},//Pll_bypass_ontch:1,improve ACPR.                               
{0x00000016,0x00008918},//add div24 20101126                                             
{0x00000018,0x00008800},//add div24 20101231                                             
{0x00000019,0x000010C8},//pll_adcclk_en=1 20101126                                       
{0x0000001A,0x00009128},//Mdll_adcclk_out_en=0                                           
{0x0000001B,0x000080C0},//1BH,16'h80C2                                                   
{0x0000001C,0x00003613},//                                                               
{0x0000001D,0x000013E3},//Pll_cp_bit_tx<3:0>1110;13D3                                    
{0x0000001E,0x0000300C},//Pll_lpf_gain_tx<1:0> 00;304C                                   
{0x00000023,0x00002222},//                                                               
{0x00000024,0x0000359F},//                                                               
{0x00000027,0x00000011},//                                                               
{0x00000028,0x0000124F},//                                                               
{0x00000039,0x0000A5FC},//      
{0x0000003f,0x00000001},//                                                        
{0x00000000,0x0000043F},//agc                                                            
{0x00000001,0x0000467F},//agc                                                            
{0x00000002,0x000028FF},//agc//2011032382H,16'h68FF;agc                                  
{0x00000003,0x000067FF},//agc                                                            
{0x00000004,0x000057FF},//agc                                                            
{0x00000005,0x00007BFF},//agc                                                            
{0x00000006,0x00003FFF},//agc                                                            
{0x00000007,0x00007FFF},//agc                                                            
{0x00000018,0x0000F3F5},//                                                               
{0x00000019,0x0000F3F5},//                                                               
{0x0000001A,0x0000E7F3},//                                                               
{0x0000001B,0x0000F1FF},//                                                               
{0x0000001C,0x0000FFFF},//                                                               
{0x0000001D,0x0000FFFF},//                                                               
{0x0000001E,0x0000FFFF},//                                                               
{0x0000001F,0x0000FFFF},//padrv_gain;9FH,16'hFFEC;padrv_gain20101103;improve ACPR;       
#ifdef EXTERNAL_32K
        {0x00000023,0x00004224},//;ext32k
#endif
{0x00000025,0x000043E1},//ldo_vbit:110,1.96v                                             
{0x00000026,0x00004BB5},//reg_ibit:101,reg_vbit:110,1.12v,reg_vbit_deepsleep:110,750mV   
{0x00000032,0x00000079},//TM mod                                                         
{0x0000003f,0x00000000},//
};

__u32 RDA5876_ENABLE_SPI[][2] =
{
    {0x40240000,0x0004f39c},                               
};

__u32 RDA5876_DISABLE_SPI[][2] = 
{
	{0x40240000,0x0000f29c},
};

__u32 RDA5876_PSKEY_RF[][2] =
{
{0x40240000,0x0004f39c},
{0x800000C0,0x00000020},
{0x800000C4,0x003F0000},
{0x800000C8,0x00414003},
{0x800000CC,0x004225BD},
{0x800000D0,0x004908E4},
{0x800000D4,0x0043B074},
{0x800000D8,0x0044D01A},
{0x800000DC,0x004A0800},
{0x800000E0,0x0054A020},
{0x800000E4,0x0055A020},
{0x800000E8,0x0056A542},
{0x800000EC,0x00574C18},
{0x800000F0,0x003F0001},
{0x800000F4,0x00410900},
{0x800000F8,0x0046033F},
{0x800000FC,0x004C0000},
{0x80000100,0x004D0015},
{0x80000104,0x004E002B},
{0x80000108,0x004F0042},
{0x8000010C,0x0050005A},
{0x80000110,0x00510073},
{0x80000114,0x0052008D},
{0x80000118,0x005300A7},
{0x8000011C,0x005400C4},
{0x80000120,0x005500E3},
{0x80000124,0x00560103},
{0x80000128,0x00570127},
{0x8000012C,0x0058014E},
{0x80000130,0x00590178},
{0x80000134,0x005A01A1},
{0x80000138,0x005B01CE},
{0x8000013C,0x005C01FF},
{0x80000140,0x003F0000},
{0x80000040,0x10000000},
};

__u32 RDA5876_DCCAL[][2]=
{
	{0x0000003f,0x00000000},
{0x00000030,0x00000129},
{0x00000030,0x0000012B},
};

__u32 RDA5876_PSKEY_MISC[][2] =
{
	{0x800004ec,0x788dffff}, ///disable edr	
	{0x800004f0,0x83713f98}, ///disable 3m esco ev4 ev5
	
  {0x80000070,0x00006058},  //6050
#ifdef EXTERNAL_32K
	{0x80000074,0xa5025010},
#else
	{0x80000074,0x05025010},//0x05025010 for no sleep ; 0xa5025010 for sleep
#endif
//	{0x80000084,0x9098C007},
	{0x80000084,0xb4b8C007},
	{0x800000a0,0x00000000},
#ifdef EXTERNAL_32K
	{0x800000a4,0x08a0280a},
#else
	{0x800000a4,0x00000000},

#endif
	{0x800000a8,0x0bbfbf30},//min power level
	{0x80000040,0x07023000},//PSKEY: modify flag
};

__u32 RDA5876_TRAP[][2] = 
{
  {0x40180100,0x000068b8},//inc power
	{0x40180120,0x000069f4},
	
	{0x40180104,0x000066b8},//dec power
	{0x40180124,0x000069f4},

	
	{0x40180108,0x0001544c},//esco w
	{0x40180128,0x0001568c},
	
	{0x80000100,0xe3a0704f}, ///2ev3 ev3 hv3
	{0x4018010c,0x0000bae8},//esco packet
	{0x4018012c,0x80000100},


	{0x40180110,0x000247a0},//sleep patch
	{0x40180130,0x0002475c},
  {0x40180114,0x0000f8c4},///all rxon
  {0x40180134,0x00026948},

  {0x40180118,0x000130b8},///qos PRH_CHN_QUALITY_MIN_NUM_PACKETS
  {0x40180138,0x0001cbb4},
  
  
  {0x4018011c,0x0000bac8},
  {0x4018013c,0x0000bae4},

	{0x40180000,0x0000ff00},  
};


void RDA5876_RfInit(int fd)
{
	RDA_uart_write_array(fd,RDA5876_ENABLE_SPI,sizeof(RDA5876_ENABLE_SPI)/sizeof(RDA5876_ENABLE_SPI[0]),0);
	RDA_uart_write_array(fd,rdabt_rf_init_12,sizeof(rdabt_rf_init_12)/sizeof(rdabt_rf_init_12[0]),1);
	usleep(50000);//50ms?
}

void RDA5876_Pskey_RfInit(int fd)
{
	RDA_uart_write_array(fd,RDA5876_PSKEY_RF,sizeof(RDA5876_PSKEY_RF)/sizeof(RDA5876_PSKEY_RF[0]),0);
}

void RDA5876_Dccal(int fd)
{
	RDA_uart_write_array(fd,RDA5876_DCCAL,sizeof(RDA5876_DCCAL)/sizeof(RDA5876_DCCAL[0]),1);
	RDA_uart_write_array(fd,RDA5876_DISABLE_SPI,sizeof(RDA5876_DISABLE_SPI)/sizeof(RDA5876_DISABLE_SPI[0]),0);
}

void RDA5876_Pskey_Misc(int fd)
{
	RDA_uart_write_array(fd,RDA5876_PSKEY_MISC,sizeof(RDA5876_PSKEY_MISC)/sizeof(RDA5876_PSKEY_MISC[0]),0);
}

void RDA5876_Trap(int fd)
{
	RDA_uart_write_array(fd,RDA5876_TRAP,sizeof(RDA5876_TRAP)/sizeof(RDA5876_TRAP[0]),0);
}


#if 0
void RDA5876_DUT_Test(int fd)
{
    RDA_uart_write_array(fd,RDA5876_DUT,sizeof(RDA5876_DUT)/sizeof(RDA5876_DUT[0]),0);
}
#endif

#endif

#ifdef __RDA_CHIP_R11_5875__


__u32 RDA5872P_5400_PSKEY[][2] = 
{   
    {0x0000003f,0x00000000},
	{0x00000001,0x00001FFF},//;
	{0x00000006,0x000007F4},//;padrv_set,increase the power.
	{0x00000008,0x000001E7},//;
	{0x00000009,0x00000520},//;
	{0x0000000B,0x000003DF},//;filter_cap_tuning<3:0>1101
	{0x0000000C,0x000085e8},//;
	{0x0000000F,0x00001dCC},//;
	{0x00000012,0x000007F4},//;padrv_set,increase the power.
	{0x00000013,0x00000327},//;padrv_set,increase the power.
	{0x00000014,0x00000CFE},//;
	{0x00000015,0x00000526},//;Pll_bypass_ontch:1,improve ACPR.
	{0x00000016,0x00008918},//;Pll_bypass_ontch:1,improve ACPR.
	{0x00000018,0x00008800},//;Pll_bypass_ontch:1,improve ACPR.
	{0x00000019,0x000010c8},//;Pll_bypass_ontch:1,improve ACPR.
	{0x0000001B,0x000080c2},//;
	{0x0000001C,0x00003613},//;
	{0x0000001D,0x000013E3},//;Pll_cp_bit_tx<3:0>1110;13D3
	{0x0000001E,0x0000300C},//;Pll_lpf_gain_tx<1:0> 00;304C
	{0x00000023,0x00003335},//;Pll_bypass_ontch:1,improve ACPR.
	{0x00000024,0x00008aaf},//;
	{0x00000027,0x00001112},//;Pll_bypass_ontch:1,improve ACPR.
	{0x00000028,0x0000345f},//;Pll_bypass_ontch:1,improve ACPR.
	{0x0000003f,0x00000001},
	{0x00000000,0x0000043F},//;agc
	{0x00000001,0x0000467F},//;agc
	{0x00000002,0x000068FF},//;agc
	{0x00000003,0x000067FF},//;agc
	{0x00000004,0x000057FF},//;agc
	{0x00000005,0x00007BFF},//;agc
	{0x00000006,0x00003FFF},//;agc
	{0x00000007,0x00007FFF},//;agc
	{0x00000018,0x0000b1B1}, //; 98H,16'hB1B1;                                                             
	{0x00000019,0x0000b1b1}, //; 99H,16'hB1B1;                                                             
	{0x0000001a,0x0000b1b1}, //; 9AH,16'hB1B1;                                                             
	{0x0000001b,0x0000b0b0}, //; 9BH,16'hB0B0;                                                             
	{0x0000001c,0x0000d7b0}, //; 9CH,16'hD7B0;                                                             
	{0x0000001d,0x0000e4e0}, //; 9DH,16'hE4E0;                                                             
	{0x0000001e,0x0000e7e9}, //; 9EH,16'hE7E9;                                                             
	{0x0000001F,0x0000e9E9},//;padrv_gain
#ifdef EXTERNAL_32K
	{0x00000023,0x00004224},//;ext32k
#endif
#ifdef RDA5875_USE_DCDC
	{0x00000024,0x00009b10},
#else
	{0x00000024,0x00000110},
#endif
	{0x00000025,0x000043A0},//;ldo_vbit:110,2.04v
	{0x00000026,0x000049B5},//;reg_ibit:100,reg_vbit:110,1.1v,reg_vbit_deepsleep:110,750mV
	{0x00000032,0x00000059},//;TM mod
	{0x0000003f,0x00000000}
};

__u32 RDA5872P_ENABLE_SPI[][2] =
{
    {0x40240000,0x0004f39c},                               
};

__u32 RDA5872P_DISABLE_SPI[][2] = 
{
	{0x40240000,0x0000f29c},
};

__u32 RDA5872P_PSKEY_RF[][2] =
{
    {0x40240000,0x0004f39c}, //; SPI2_CLK_EN PCLK_SPI2_EN
	{0x800000C0,0x00000020}, //; CHIP_PS PSKEY: Total number -----------------
	{0x800000C4,0x003F0000}, //;         PSKEY: Page 0
	{0x800000C8,0x00410003}, //;         PSKEY: Swch_clk_ADC
	{0x800000CC,0x004225BD}, //;         PSKEY: dig IF 625K IF  by lihua20101231
	{0x800000D0,0x004908E4}, //;         PSKEY: freq_offset_for rateconv  by lihua20101231(replace 47H)
	{0x800000D4,0x0043B074}, //;         PSKEY: AM dac gain, 20100605
	{0x800000D8,0x0044D01A}, //;         PSKEY: gfsk dac gain, 20100605//22
	{0x800000DC,0x004A0800}, //;         PSKEY: 4AH=0800
	{0x800000E0,0x0054A020}, //;         PSKEY: 54H=A020;agc_th_max=A0;agc_th_min=20
	{0x800000E4,0x0055A020}, //;         PSKEY: 55H=A020;agc_th_max_lg=A0;agc_th_min_lg=20
	{0x800000E8,0x0056A542}, //;         PSKEY: 56H=A542
	{0x800000EC,0x00574C18}, //;         PSKEY: 57H=4C18
	{0x800000F0,0x003F0001}, //;         PSKEY: Page 1               
	{0x800000F4,0x00410900}, //;         PSKEY: C1=0900;Phase Delay, 20101029 
	{0x800000F8,0x0046033F}, //;         PSKEY: C6=033F;modulation Index;delta f2=160KHZ,delta f1=164khz
	{0x800000FC,0x004C0000}, //;         PSKEY: CC=0000;20101108   
	{0x80000100,0x004D0015}, //;         PSKEY: CD=0015;           
	{0x80000104,0x004E002B}, //;         PSKEY: CE=002B;           
	{0x80000108,0x004F0042}, //;         PSKEY: CF=0042            
	{0x8000010C,0x0050005A}, //;         PSKEY: D0=005A            
	{0x80000110,0x00510073}, //;         PSKEY: D1=0073            
	{0x80000114,0x0052008D}, //;         PSKEY: D2=008D            
	{0x80000118,0x005300A7}, //;         PSKEY: D3=00A7            
	{0x8000011C,0x005400C4}, //;         PSKEY: D4=00C4            
	{0x80000120,0x005500E3}, //;         PSKEY: D5=00E3            
	{0x80000124,0x00560103}, //;         PSKEY: D6=0103            
	{0x80000128,0x00570127}, //;         PSKEY: D7=0127            
	{0x8000012C,0x0058014E}, //;         PSKEY: D8=014E            
	{0x80000130,0x00590178}, //;         PSKEY: D9=0178            
	{0x80000134,0x005A01A1}, //;         PSKEY: DA=01A1            
	{0x80000138,0x005B01CE}, //;         PSKEY: DB=01CE            
	{0x8000013C,0x005C01FF}, //;         PSKEY: DC=01FF            
	{0x80000140,0x003F0000}, //;         PSKEY: Page 0
	{0x80000040,0x10000000}, //;         PSKEY: Flage
	//{0x40240000,0x0000f29c}, //; SPI2_CLK_EN PCLK_SPI2_EN 
};

__u32 RDA5872P_DCCAL[][2]=
{
	{0x00000030,0x00000129},
	{0x00000030,0x0000012b}
};

__u32 RDA5872P_PSKEY_MISC[][2] =
{
	{0x80000070,0x00006058},
#ifdef EXTERNAL_32K
	{0x80000074,0xa5025010},
#else
	{0x80000074,0x05025010},//0x05025010 for no sleep ; 0xa5025010 for sleep
#endif
	{0x80000084,0x9098C007},
	{0x800000a0,0x00000000},
	{0x800000a4,0x00000000},
	{0x800000a8,0x0bbfbf30},//min power level
	{0x80000040,0x07023000},//PSKEY: modify flag
	{0x800004ec,0x788dffff}, ///disable edr
};

__u32 RDA5872P_TRAP[][2] = 
{
    {0x800004ec,0x788dFFFF},//shut down edr	

	{0x40180100,0x00013158},//:    e3a00064    d...    MOV      r0,#0x64
	{0x40180120,0x00027e68},
	{0x40180104,0x00013240},//:    e3a00064    d...    MOV      r0,#0x64
	{0x40180124,0x00027e68},

	{0x40180108,0x0002ED2C},
	{0x40180128,0x000309E4},
	{0x80008ba8,0xe3a01007},
	{0x4018010c,0x0001cec8},
	{0x4018012c,0x80008ba8},
	{0x40180110,0x00002A98},
	{0x40180130,0x00003074},
	{0x40180114,0x00012040},
	{0x40180134,0x00021D40},

#ifndef EXTERNAL_32K
       {0x40180118,0x0002475c},// for no 32K
	{0x40180138,0x00024718},// for no 32K
#endif

	{0X40180000,0x00003f00},
};



void RDA5872P_RfInit(int fd)
{
	RDA_uart_write_array(fd,RDA5872P_ENABLE_SPI,sizeof(RDA5872P_ENABLE_SPI)/sizeof(RDA5872P_ENABLE_SPI[0]),0);
	RDA_uart_write_array(fd,RDA5872P_5400_PSKEY,sizeof(RDA5872P_5400_PSKEY)/sizeof(RDA5872P_5400_PSKEY[0]),1);
	usleep(50000);//50ms?
}

void RDA5872P_Pskey_RfInit(int fd)
{
	RDA_uart_write_array(fd,RDA5872P_PSKEY_RF,sizeof(RDA5872P_PSKEY_RF)/sizeof(RDA5872P_PSKEY_RF[0]),0);
}

void RDA5872P_Dccal(int fd)
{
	RDA_uart_write_array(fd,RDA5872P_DCCAL,sizeof(RDA5872P_DCCAL)/sizeof(RDA5872P_DCCAL[0]),1);
	RDA_uart_write_array(fd,RDA5872P_DISABLE_SPI,sizeof(RDA5872P_DISABLE_SPI)/sizeof(RDA5872P_DISABLE_SPI[0]),0);
}

void RDA5872P_Pskey_Misc(int fd)
{
	RDA_uart_write_array(fd,RDA5872P_PSKEY_MISC,sizeof(RDA5872P_PSKEY_MISC)/sizeof(RDA5872P_PSKEY_MISC[0]),0);
}

void RDA5872P_Trap(int fd)
{
	RDA_uart_write_array(fd,RDA5872P_TRAP,sizeof(RDA5872P_TRAP)/sizeof(RDA5872P_TRAP[0]),0);
}
#endif

#ifdef __RDA_CHIP_R10_5870E__
__u32 rda5870e_set_tm[][2] = 
{       
	{0x0000003f,0x00000001},//page 1    
	{0x00000032,0x0000000b},//;TM mod B2H [2:0]   
	{0x0000003f,0x00000000},//page 0
};

__u32 rdabt_rf_init_10_e[][2] = 
{   
    {0x0000003f,0x00000000},//page 0
    {0x00000001,0x00000FFF},//;Padrv_gain_tb_en
    {0x00000006,0x000003FF},//;PSK 
    {0x00000008,0x000000FF},//;
    {0x00000009,0x0000046C},//;
    {0x0000000B,0x0000021F},//;
    {0x0000000C,0x000085D8},//;
    {0x0000000F,0x00001CC8},//;adc_refi_cal_reg<2:0> Set to 000,20100820.
    {0x00000012,0x00000107},//;GFSK
    {0x0000001B,0x0000E224},//;
    {0x0000001C,0x0000F5F3},//;Xtal_capbank,20100820
    {0x0000001D,0x000021BB},//;5870E EVM,20100827.
    {0x00000021,0x00000000},//;Gain_psk_hi4_PSK_4>5>6>7
    {0x00000022,0x00000000},//;Gain_psk_hi4_PSK_4>5>6>7
    {0x00000023,0x00002458},//;Gain_psk_hi4_PSK_8>9>A>B
    {0x00000024,0x00008ddd},//;Gain_psk_hi4_PSK_C>D>E>F
    {0x00000025,0x00000000},//;Gain_fsk_hi4 GFSK 0>1>2>3
    {0x00000026,0x00000000},//;Gain_fsk_hi4 GFSK 4>5>6>7
    {0x00000027,0x00001235},//;Gain_fsk_hi4 GFSK 8>9>A>B
    {0x00000028,0x00005888},//;Gain_fsk_hi4 GFSK C>D>E>F
    {0x0000003f,0x00000001},//page 1
    {0x00000000,0x00004005},//;;agc0
    {0x00000001,0x00004025},//;;agc1
    {0x00000002,0x00005025},//;;agc2
    {0x00000003,0x0000506D},//;;agc3
    {0x00000004,0x000050bD},//;;agc4
    {0x00000005,0x0000713D},//;;agc5
    {0x00000006,0x00007A3D},//;;agc6
    {0x00000007,0x00007E3E},//;;agc7
    {0x0000000A,0x0000001F},//;
    {0x0000000D,0x00000017},//;APC
    {0x00000011,0x00000000},//;;padrv_gain_1
    {0x00000012,0x00000000},//;;padrv_gain_2
    {0x00000013,0x00000000},//;;padrv_gain_3
    {0x00000014,0x00000000},//;;padrv_gain_4
    {0x00000015,0x00000000},//;;padrv_gain_5
    {0x00000016,0x00000000},//;;padrv_gain_6
    {0x00000017,0x00000000},//;;padrv_gain_7
    {0x00000018,0x00000000},//;;padrv_gain_8
    {0x00000019,0x00000000},//;;padrv_gain_9
    {0x0000001A,0x00001818},//;;padrv_gain_A
    {0x0000001B,0x00001818},//;;padrv_gain_B
    {0x0000001C,0x00006e6e},//;;padrv_gain_C
    {0x0000001D,0x00006e6e},//;;padrv_gain_D
    {0x0000001E,0x0000a7a7},//;;padrv_gain_E
    //{0x1F,0xb5b5},//;;padrv_gain_F
    {0x0000001F,0x0000d8d8},//;;padrv_gain_F gfsk 4dBm,psk 2.4dBm
    //{0x1F,0xdddd},//;;padrv_gain_F gfsk 5dBm,psk 3dBm
    //{0x1F,0xe4e4},//;;padrv_gain_F gfsk 6dBm,psk 4.1dBm
    //{0x1F,0xecec},//;;padrv_gain_F gfsk 7dBm,psk 5.2dBm (max power)
    {0x00000023,0x00004221},//;;use EXT32k
#ifdef __587x_USE_DCDC__
    {0x00000024,0x000048d1},//dcdc_enable set to 1
    {0x00000026,0x000045f5},//lower regulator voltage, current goes from DCDC
#else
    {0x00000024,0x00000090},//dcdc_enable set to 1
    {0x00000026,0x00004535},//;;1,reg_vbit_normal<2:0>Set to 010；2，reg_vbit_deepsleep<2:0> Set to 111。
#endif
    {0x0000002F,0x0000114E},//;;
    {0x0000003f,0x00000000},//page 0
};

__u32 rdabt_pskey_rf_10_e[][2] =
{
#if 1
//rf para setting
{0x40240000,0x0004F39c},//enable spi2
{0x800000C0,0x0000000f},//PSKEY: Total number
{0x800000C4,0x003F0000},//PSKEY: page0
{0x800000C8,0x00410003},//PSKEY: Swch_clk_adc
{0x800000CC,0x004224EC},//PSKEY: 625k if
{0x800000D0,0x0047C939},//PSKEY: mod_adc_clk
{0x800000D4,0x00431a74},//PSKEY: AM dac gain
{0x800000D8,0x0044d01A},//PSKEY: gfsk dac gain
{0x800000DC,0x004a0800},//PSKEY: 4A=0800
{0x800000E0,0x0054a020},//PSKEY: 54=A020 AGCMAX=A0 AGCMIN=20
{0x800000E4,0x0055a020},//PSKEY: 55=A020 AGC_TH_max_lg=a0 agc_th_min_lg =0x20
{0x800000E8,0x0056a542},//PSKEY: 56=a542
{0x800000EC,0x00574c18},//PSKEY: 57=4c18
{0x800000F0,0x003f0001},//PSKEY: page=1
{0x800000F4,0x00410c80},//PSKEY: phase delay
{0x800000F8,0x003f0000},//PSKEY: page =0
{0x800000Fc,0x00000000},//PSKEY: page =0
{0x80000040,0x10000000},//PSKEY: flag
{0x40240000,0x0000F29c},//enable spi2

#else
//rf para setting
{0x40240000,0x0004F39c},//enable spi2
{0x800000C0,0x00000014},//PSKEY: Total number
{0x800000C4,0x003F0000},//PSKEY: page0
{0x800000C8,0x00410003},//PSKEY: Swch_clk_adc
{0x800000CC,0x004224EC},//PSKEY: 625k if
{0x800000D0,0x0047C939},//PSKEY: mod_adc_clk
{0x800000D4,0x00431a74},//PSKEY: AM dac gain
{0x800000D8,0x0044d01A},//PSKEY: gfsk dac gain
{0x800000DC,0x004a0800},//PSKEY: 4A=0800
{0x800000E0,0x004d008a},
{0x800000E4,0x004e1f1f},
{0x800000E8,0x00694094},
{0x800000EC,0x006a1b1b},
{0x800000F0,0x0054a020},//PSKEY: 54=A020 AGCMAX=A0 AGCMIN=20                    
{0x800000F4,0x0055a020},//PSKEY: 55=A020 AGC_TH_max_lg=a0 agc_th_min_lg =0x20   
{0x800000F8,0x0056a542},//PSKEY: 56=a542                                        
{0x800000Fc,0x00574c18},//PSKEY: 57=4c18                                        
{0x80000100,0x003f0001},//PSKEY: page=1                                         
{0x80000104,0x00410c80},//PSKEY: phase delay                                    
{0x80000108,0x004603f1},
{0x8000010c,0x003f0000},//PSKEY: page =0                                        
{0x80000110,0x00000000},//PSKEY: page =0                                        
{0x80000040,0x10000000},//PSKEY: flag                                           
{0x40240000,0x0000F29c},//enable spi2

#endif
};
__u32 rdabt_dccal_10_e[][2]=
{
{0x0000003f,0x00000000},
{0x00000030,0x00000169},
{0x0000003C,0x00000002},
{0x00000030,0x0000016B},
{0x00000030,0x0000012B}
};
__u32 rda_pskey_10[][2] =
{
//sleep
{0x80000070,0x00002008},//disalbe esco
{0x80000074,0xa5025010},//sleep
{0x800000a8,0x0Bbfbf30},//min power level
{0x80000040,0x04003000},//PSKEY: modify flag

{0x800000a4,0x08a0280a},
{0x80000040,0x02000000},
                              
//{0x80003ff8,,0x788dffff},//disable edr
#ifndef __RDA_SHARE_CRYSTAL__

#endif
};
__u32 rdabt_unsniff_prerxon_10[2] = {0x800067f4,0x00000864};

__u32 rda_filter_10[2] = {0x800004c4,0x00000000};//{0x01,0x72,0xfc,0x05,0x01,0x00,0x00,0x00,0x00};

__u32 rda_trap_10[][2] = 
{
{0x800000C4,0xFC00FC1B},//tc map table
{0x800000C8,0xFC00FC66},
{0x800000CC,0xFD42FC42},
{0x800000D0,0xFC00FC34},
{0x800000D4,0xFC00FC76},
{0x800000D8,0xFC1DFC56},
{0x800000DC,0xFC0BFC5B},
{0x800000E0,0xFC00FC1E},
{0x800000E4,0x00000000},
{0x800078A8,0x800000C4},

{0x80000000,0xea00001e},//sleep rom
{0x80000080,0xe1a00000},
{0x80000084,0xe1a00000},
{0x80000088,0xe1a00000},
{0x8000008c,0xe1a00000},
{0x80000090,0xe2800001},
{0x80000094,0xe3500020},
{0x80000098,0xbafffff8},
{0x8000009c,0xe3a00b8a},
{0x800000a0,0xe280fffe},
{0x800000a4,0xe3a0f480},
{0x40180010,0x00022bc8},
{0x40180030,0x800000a4},

{0x40180014,0x00003e20},//name length patch
{0x40180034,0x00022df8},
                      
{0x40180018,0x0002bbac},//diable simple pair
{0x40180038,0x0002dc14},
                      
                      
{0x80000004,0xea00003A},//master poll interval to 0
{0x800000F4,0xe59f5014},
{0x800000F8,0xe5d55001},
{0x800000Fc,0x03550000},
{0x80000100,0xe3a05cfd},
{0x80000104,0x1285f010},
{0x80000108,0x03a02f7e},
{0x8000010c,0xe285f00c},
{0x80000110,0x800057Dc},
{0x4018001c,0x0000fd08},
{0x4018003c,0x00032e7c},

#if 0
{0x80000008,0xea000041},//orign opcode =0
{0x80000114,0xe3500000},
{0x80000118,0xe3a00b95},
{0x8000011c,0x1280f0e4},
{0x80000120,0xe280f0d8},
{0x40180020,0x000254d4},
{0x40180040,0x00032e80},
#else
{0x80000008,0xea000069},
{0x800001b4,0xe3a0eb95},
{0x800001b8,0xe28ee0e4},
{0x800001bc,0xea000003},
{0x800001c0,0xe51ff004},
{0x800001c4,0x00001974},
{0x800001c8,0xe51ff004},
{0x800001cc,0x00001688},
{0x800001d0,0xe92d4008},
{0x800001d4,0xe3a02004},
{0x800001d8,0xe3a01000},
{0x800001dc,0xe3a00000},
{0x800001e0,0xebfffff6},
{0x800001e4,0xe1a02000},
{0x800001e8,0xe5921000},
{0x800001ec,0xe59f0018},
{0x800001f0,0xe5810000},
{0x800001f4,0xe3a00001},
{0x800001f8,0xe1c201b2},
{0x800001fc,0xe8bd4008},
{0x80000200,0xe3a00000},
{0x80000204,0xe3a01000},
{0x80000208,0xeaffffee},
{0x8000020c,0x0000fcc0},
{0x40180020,0x000254d4},
{0x40180040,0x00032e80},
#endif

{0x8000000c,0xea000044},//disable switch twice
{0x80000124,0xe1a05001},
{0x80000128,0xe5d11000},
{0x8000012C,0xe3510001},
{0x80000130,0xe3a01b66},
{0x80000134,0x0281f014},
{0x80000138,0x1281f044},
{0x40180100,0x00019810},
{0x40180120,0x00032e84},

{0x80000010,0xea00004a},///queues init 
{0x80000140,0xe3a03f56},
{0x80000144,0xe3a01006},
{0x80000148,0xe3a0ebab},
{0x8000014c,0xe28eff53},
{0x40180104,0x0002ad44},
{0x40180124,0x00032e88},

#if defined( _RDA_PATCH_UART3_) || defined(_RDA_PATCH_UART4_)
{0x80000014,0xea0014b4},//buff init start address
#else
{0x80000014,0xEA00004d},
{0x80000150,0xe59f0048}, 
{0x80000154,0xe3a01004},        
{0x80000158,0xe1c010b6},  
{0x8000015c,0xe1c010b4},
{0x80000160,0xe59f103c},
{0x80000164,0xe2812048},
{0x80000168,0xe580200c},
{0x8000016c,0xe3a00000},
{0x80000170,0xe581005c},
{0x80000174,0xe59f202c},
{0x80000178,0xe3a00008},
{0x8000017c,0xe1c200b6},
{0x80000180,0xe1c200b4},
{0x80000184,0xe2813060},
{0x80000188,0xe5823008},
{0x8000018c,0xe2812094},
{0x80000190,0xe581208c},
{0x80000194,0xe59f1010},
{0x80000198,0xe1c108b8},
{0x8000019c,0xe49df004},
{0x800001a0,0x80002c58},
{0x800001a4,0x80002c94},
{0x800001a8,0x80002c6c},
{0x800001ac,0x80003fd4},
#endif

{0x40180108,0x00008468},
{0x40180128,0x00032e8c},
                      
                      
{0x4018010c,0x00011e2c},//sniff slave accept auto
{0x4018012c,0x00012020},
                      
{0x40180110,0x00003448},//uart len>240
{0x40180130,0x0001d548},

#if defined(_RDA_PATCH_UART4_)
{0x80000018,0xea000080},///delay reset
{0x80000220,0xe3a00e32},
{0x80000224,0xe59fe010},
{0x80000228,0xe3a01c55},
{0x8000022c,0xe281f084},
{0x80000230,0xe59f0008},
{0x80000234,0xe3a01c0f},
{0x80000238,0xe281f014},
{0x8000023c,0x80000230},
{0x80000240,0x00000c03},
{0x40180114,0x00000f10},
{0x40180134,0x00032e90},


{0x8000001c,0xea00008b},////acl buff
{0x80000250,0xe3a02004},
{0x80000254,0xe3a03000},
{0x80000258,0xe3a0ec3a},
{0x8000025c,0xe28ef0a0},
{0x40180118,0x00003a98},
{0x40180138,0x00032e94},


{0x80000020,0xea000098},////fcd1
{0x80000288,0xe5d00000},
{0x8000028c,0xe3500000},
{0x80000290,0x059f0020},
{0x80000294,0x0a000005},
{0x80000298,0xe3500004},
{0x8000029c,0x059f0018},
{0x800002a0,0x0a000002},
{0x800002a4,0xe3500008},
{0x800002a8,0x159f0010},
{0x800002ac,0x059f0010},
{0x800002b0,0xe5810010},
{0x800002b4,0xe1a0f00e},
{0x800002b8,0x00008c03},
{0x800002bc,0x00008c01},
{0x800002c0,0x00008a01},
{0x800002c4,0x00006612},
{0x4018011c,0x00023254},
{0x4018013c,0x00032e98},
#endif

#if defined( _RDA_PATCH_UART3_) || defined(_RDA_PATCH_UART4_)
{0x50000044,0x00000001},
#else
{0x50000044,0x00220003},
#endif
{0x8000005c,0x03300000},//
{0x80000040,0x00000080},//flag


#if defined(_RDA_PATCH_UART2_) || defined(_RDA_PATCH_UART3_)  || defined(_RDA_PATCH_UART4_)

#if 1
#if defined( _RDA_PATCH_UART3_)  || defined(_RDA_PATCH_UART4_)
{0x80000064,0x0030d400},//new baudrate 3200000
#else
{0x80000064,0x000e1000},//new baudrate 921600
#endif

{0x8000005c,0x03300000},//convert baud delay 16slot
{0x80000040,0x00000280},//flag
#endif
#if defined (_RDA_PATCH_UART3_)  || defined(_RDA_PATCH_UART4_)
{0x80004c38,0x00000001},//flag chip
#else
{0x80004c38,0x00000000},
#endif
{0x80004c3c,0x00000000},


{0x40180004,0x00003208},
{0x40180024,0x800051B4},//patch uart rx
                      
{0x800000C0,0xEA001454},
{0x40180008,0x000034E4},
{0x40180028,0x80005214},//patch uart tx
                      
{0x4018000C,0x00031C58},
{0x4018002C,0x800052A4},//patch uart convert baud
#if defined(_RDA_PATCH_UART4_)
{0x40180000,0x0000ffff},//patch flag
#else
{0x40180000,0x00001fff},//patch flag
#endif
#else
{0x40180000,0x00001ff8},//patch flag
#endif

};
void RDA5870E_SET_TM(int fd)
{	
	RDA_uart_write_array(fd,rda5870e_set_tm,sizeof(rda5870e_set_tm)/sizeof(rda5870e_set_tm[0]),1);
	usleep(50000);//50ms?
}

void RDABT_rf_Intialization_r10_e(int fd)
{
	RDA_uart_write_array(fd,rdabt_rf_init_10_e,sizeof(rdabt_rf_init_10_e)/sizeof(rdabt_rf_init_10_e[0]),1);
	usleep(50000);//50ms?
}
void Rdabt_Pskey_Write_rf_r10_e(int fd)
{
	RDA_uart_write_array(fd,rdabt_pskey_rf_10_e,sizeof(rdabt_pskey_rf_10_e)/sizeof(rdabt_pskey_rf_10_e[0]),0);
}
void rdabt_DC_write_r10_e(int fd)
{
	//RDA_uart_write_array(fd,rdabt_dccal_10_e,sizeof(rdabt_dccal_10_e)/sizeof(rdabt_dccal_10_e[0]),1);
   __u32 i;
   for(i=0;i<((sizeof(rdabt_dccal_10_e)/sizeof(rdabt_dccal_10_e[0])) -1);i++)
   {
      rdabt_write_memory(fd,rdabt_dccal_10_e[i][0],&rdabt_dccal_10_e[i][1],1,1);
      usleep(12000);//12ms?
   }	
      usleep(20000);
  
      rdabt_write_memory(fd,rdabt_dccal_10_e[i][0],&rdabt_dccal_10_e[i][1],1,1);
      usleep(12000);//12ms?

}
void Rdabt_Pskey_Write_r10(int fd)
{
	RDA_uart_write_array(fd,rda_pskey_10,sizeof(rda_pskey_10)/sizeof(rda_pskey_10[0]),0);
}

void Rdabt_unsniff_prerxon_write_r10(int fd)
{
	RDA_uart_write_array(fd,rdabt_unsniff_prerxon_10,sizeof(rdabt_unsniff_prerxon_10)/sizeof(rdabt_unsniff_prerxon_10[0]),0);
}
void Rdabt_setfilter_r10(int fd)
{
	RDA_uart_write_array(fd,rda_filter_10,sizeof(rda_filter_10)/sizeof(rda_filter_10[0]),0);
}
void Rdabt_trap_write_r10(int fd)
{
	RDA_uart_write_array(fd,rda_trap_10,sizeof(rda_trap_10)/sizeof(rda_trap_10[0]),0);
}

#endif
int RDABT_core_Intialization(int fd)
{
	int bt_fd = -1;
	bt_fd = open(TCC_BT_DEVICE_PATH, O_RDWR);
	if( bt_fd < 0 ) 
	{
		perror("[###### TCC BT #######] open error");
		return -1;
	}

#ifdef __RDA_CHIP_R11_5875__
	RDA_pin_to_high(bt_fd);
	RDA5872P_RfInit(fd);
	RDA5872P_Pskey_RfInit(fd);
	RDA_pin_to_low(bt_fd);
	RDA_pin_to_high(bt_fd); 
	usleep(50000);
	RDA5872P_RfInit(fd);   
	RDA5872P_Pskey_RfInit(fd);
	RDA5872P_Dccal(fd);	
	RDA5872P_Pskey_Misc(fd);
	RDA5872P_Trap(fd);
#elif defined(__RDA_CHIP_R10_5870E__)

	RDA_pin_to_high(bt_fd);
	RDABT_rf_Intialization_r10_e(fd);
	RDA_pin_to_low(bt_fd);
	RDA_pin_to_high(bt_fd);   
	usleep(50000); 
	//RDA5870E_SET_TM(fd);
	RDABT_rf_Intialization_r10_e(fd);	
	Rdabt_Pskey_Write_rf_r10_e(fd);
	usleep(5000); 
	rdabt_DC_write_r10_e(fd);
	Rdabt_Pskey_Write_r10(fd);
	Rdabt_unsniff_prerxon_write_r10(fd);
	Rdabt_setfilter_r10(fd);
	Rdabt_trap_write_r10(fd);
#elif defined(__RDA_CHIP_R11_5876__)

	int bt_on_off = 0;
	ioctl(bt_fd, IOCTL_BT_DEV_POWER, &bt_on_off);
	usleep(5000);
	bt_on_off = 1;
	ioctl(bt_fd, IOCTL_BT_DEV_POWER, &bt_on_off);
	usleep(5000);

	RDA_pin_to_high(bt_fd);
	RDA5876_RfInit(fd);
	RDA5876_Pskey_RfInit(fd);

	RDA_pin_to_low(bt_fd);
	RDA_pin_to_high(bt_fd); 
	usleep(50000);

	RDA5876_RfInit(fd);   
	RDA5876_Pskey_RfInit(fd);

	RDA5876_Dccal(fd);	
	RDA5876_Trap(fd);               
  RDA5876_Pskey_Misc(fd);           
#endif

	
	return 0;
}

