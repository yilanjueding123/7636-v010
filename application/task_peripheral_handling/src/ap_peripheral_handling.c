#include "ap_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_state_handling.h"
//#include "drv_l1_system.h"
#include "driver_l1.h"
#include "drv_l1_cdsp.h"

#define LED_STATUS_FLASH	1
#define LED_STATUS_BLINK	2

#define CRAZY_KEY_TEST		0		// Send key events faster than human finger can do
#define LED_ON              1
#define LED_OFF             0

#define C_VIDEO   	0  //录像模式
#define C_PHOTO   	1  //拍照模式
#define C_AUDIO   	2  //录音模式	
#define C_MOTION    3  //移动侦测

#define STA_VIDEO	0 
#define STA_MOTION	1 

static INT8U 	g_led_count;
static INT8U 	g_led_r_state;	//0 = OFF;	1=ON;	2=Flicker
static INT8U	g_led_g_state;
static INT8U	g_led_b_state;
static INT8U	g_led_flicker_state;	//0=同时闪烁	1=交替闪烁
static INT8U    led_red_flag;
static INT8U    led_green_flag;
static INT8U    led_blue_flag;
static INT8U    led_ir_flag;
static INT8U    savefile_led_flag = 0; //保存录像文件时闪灯的标志, =1,闪灯

extern INT8S video_record_sts;
extern volatile  INT8U pic_down_flag;
extern volatile  INT8U video_down_flag;
#if AUD_RECORD_EN
extern volatile  INT8U audio_down_flag;
extern INT8U audio_record_sts;
extern INT8U ap_audio_record_sts_get(void);
#endif
extern INT8U usb_state_get(void);
extern void usb_state_set(INT8U flag);

#if TV_DET_ENABLE
INT8U tv_plug_in_flag;
INT8U tv_debounce_cnt = 0;
#endif

#if 1 //C_SCREEN_SAVER == CUSTOM_ON
	INT8U auto_off_force_disable = 0;
	void ap_peripheral_auto_off_force_disable_set(INT8U);
#endif

static INT8U led_flash_timerid;
static INT16U config_cnt;
static INT16U power_off_cnt=0;
//----------------------------
    typedef struct {
        INT8U byRealVal ;
        INT8U byCalVal;
    } AD_MAP_t;


//----------------------------
	extern void avi_adc_gsensor_data_register(void **msgq_id, INT32U *msg_id);
	INT8U gsensor_data[2][32] = {0};
	#define C_BATTERY_STABLE_THRESHOLD		4  // Defines threshold number that AD value is deemed stable

#if C_BATTERY_DETECT == CUSTOM_ON
	static INT16U low_voltage_cnt = 0;
    static INT32U battery_value_sum=0;
	static INT8U bat_ck_cnt=0;
#endif



#if USE_ADKEY_NO
	static INT8U ad_detect_timerid;
	static INT16U ad_value;
	static KEYSTATUS ad_key_map[USE_ADKEY_NO+1];
	static INT16U adc_key_release_value_stable;

	#define C_RESISTOR_ACCURACY				5//josephhsieh@140418 3			// 2% accuracy
	#define C_KEY_PRESS_WATERSHED			600//josephhsieh@140418 175
	#define C_KEY_STABLE_THRESHOLD			4//josephhsieh@140418 3			// Defines threshold number that AD value of key is deemed stable
	#define C_KEY_FAST_JUDGE_THRESHOLD 		40			// Defines threshold number that key is should be judge before it is release. 0=Disable
	#define C_KEY_RELEASE_STABLE_THRESHOLD	4  // Defines threshold number that AD value is deemed stable

	INT16U adc_key_value;

	INT32U key_pressed_cnt;
	INT8U fast_key_exec_flag;
	INT8U normal_key_exec_flag;
	INT8U long_key_exec_flag;
#endif

static INT32U key_active_cnt;
static INT8U power_off_timerid;
static INT8U usbd_detect_io_timerid;
static KEYSTATUS key_map[USE_IOKEY_NO];
static INT8U key_detect_timerid;
static INT16U adp_out_cnt;
static INT16U usbd_cnt;
#if USB_PHY_SUSPEND == 1
static INT16U phy_cnt = 0;
#endif
static INT16U adp_cnt;
 INT8U  adp_status;
INT8U  usbd_exit;
volatile INT8U s_usbd_pin;
//	prototypes
void ap_peripheral_key_init(void);
void ap_peripheral_rec_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_usbd_plug_out_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_mode_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_capture_key_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_power_on_exe(INT16U *tick_cnt_ptr);
void ap_peripheral_null_key_exe(INT16U *tick_cnt_ptr);
#if USE_ADKEY_NO
	void ap_peripheral_ad_detect_init(INT8U adc_channel, void (*bat_detect_isr)(INT16U data));
	void ap_peripheral_ad_check_isr(INT16U value);
#endif	

static INT8U UsbMode_Tf_flag = 0;
static INT8U Low_Voltage_Flag = 0;
static INT8U work_mode = 0xFF;

//闪灯模式
//= RED_FLIKER: 红灯闪；  
//= BLUE_FLIKER：蓝灯闪；
//= GREEN_FLIKER：绿灯闪
static INT32U LedFilkerStep = 0; 
static INT8U LedFilkerCnt = 0; //闪灯次数
static INT32U LastLedMode = 0xaa; //上一次的闪灯状态，用于恢复原来的LED状态
static INT32U NowLedMode = 0x55;  //本次闪灯状态
INT32U LedFilkerTime = 0;  //LED亮或灭的时间，=128为1秒，闪灯时亮与灭时间相同

#define RED_FLIKER		0X10 //红灯闪
#define BLUE_FLIKER		0X20 //蓝灯闪
#define GREEN_FLIKER	0X30 //绿灯闪


INT8U work_mode_get(void)
{
	return work_mode;
}

void work_mode_set(INT8U mode)
{
	 if (mode != work_mode)
	 	work_mode = mode;
}

INT8U ap_peripheral_low_vol_get(void)
{
	return Low_Voltage_Flag;
}

INT8U ap_peripheral_UsbMode_tf_get(void)
{
	return UsbMode_Tf_flag;
}

void LDO12_Switch(INT8U OnOff)
{
	INT32U ret;
	
	ret = R_SYSTEM_POWER_CTRL0;
	
	if (OnOff) ret |= 0x01; //enable LDO12 and LDO33
	else ret &= (~0x01);
	
	R_SYSTEM_POWER_CTRL0 = ret;
}

INT8U SaveFile_Led_Flag_Get(void)
{
	return savefile_led_flag;
}

void SaveFile_Led_Flag_Set(INT8U flag)
{
	savefile_led_flag = flag;
}

void power_pin_set(INT8U onoff)
{
 
    gpio_init_io(POWER_ENABLE_PIN, GPIO_OUTPUT);
  	gpio_set_port_attribute(POWER_ENABLE_PIN, ATTRIBUTE_HIGH);
  	gpio_write_io(POWER_ENABLE_PIN, onoff);
}

INT8U ap_peripheral_power_key_read(int pin)
{
	int status;


	#if  (KEY_TYPE == KEY_TYPE5)
		status = gpio_read_io(pin);
	#else
	switch(pin)
	{
		case PWR_KEY0:
			status = sys_pwr_key0_read();
			break;
		case PWR_KEY1:
			status = sys_pwr_key1_read();
			break;
	}
	#endif

	if (status!=0)
		 return 1;
	else return 0;
}

static void init_usbstate(void)
{
	static INT8U usb_dete_cnt=0;
	static INT8U err_cnt=0;

	while(++err_cnt<100)
	{
		if(sys_pwr_key1_read())
			usb_dete_cnt++;
		else
		{
			usb_dete_cnt=0;
			break;
		}
		if(usb_dete_cnt > 3) break;
		OSTimeDly(2);
	}
	if(usb_dete_cnt > 3)
	usb_state_set(3);
	err_cnt=0;
}

INT8U key_sta_detect(INT32U io)
{
	INT32U count, save, t;
	count = 0;
	save = 0;
	while(1)
	{
		t = gpio_read_io(io);
		if (save != t)
		{
			save = t;
			count = 0;
		}
		else
		{
			if (++count > 50) break;
		}
	}
	if (save) return 1;
	else return 0;
}

static void charge_flash_pro(void)
{
 	if(gpio_read_io(CHARGE_DETECTION_PIN)== 0)
	 {
		led_red_off();
	    g_led_b_state=2;
	 }
	else
	 {
		//led_red_off();
		//led_green_off();
		 led_red_on();
		 led_blue_off();
		 g_led_b_state=0;
	 }
	
}
void ap_peripheral_init(void)
{
	power_off_timerid = usbd_detect_io_timerid = led_flash_timerid = 0xFF;
	key_detect_timerid = 0xFF;
	power_off_cnt=C_IDLE_SLEEP;

	power_pin_set(1); //enable external power
	LDO12_Switch(0); //disable internal power
  	LED_pin_init();

	init_usbstate();

	gpio_init_io(CHARGE_DETECTION_PIN,GPIO_INPUT);
  	gpio_set_port_attribute(CHARGE_DETECTION_PIN, ATTRIBUTE_LOW);
  	gpio_write_io(CHARGE_DETECTION_PIN, DATA_HIGH);	//pull high

	ap_peripheral_key_init();

#if USE_ADKEY_NO
	ad_detect_timerid = 0xFF;
	ap_peripheral_ad_detect_init(AD_BAT_DETECT_PIN, ap_peripheral_ad_check_isr);
#else
	adc_init();
#endif
	config_cnt = 0;
}



void LED_pin_init(void)
{
	INT32U type;
	//led init as ouput pull-low
  	gpio_init_io(LED1, GPIO_OUTPUT);
  	gpio_set_port_attribute(LED1, ATTRIBUTE_HIGH);
  	gpio_write_io(LED1, LED1_ACTIVE^1);
  	
  	gpio_init_io(LED2, GPIO_OUTPUT);
  	gpio_set_port_attribute(LED2, ATTRIBUTE_HIGH);
  	gpio_write_io(LED2, LED2_ACTIVE^1);

  	gpio_init_io(LED3, GPIO_OUTPUT);
  	gpio_set_port_attribute(LED3, ATTRIBUTE_HIGH);
  	gpio_write_io(LED3, LED3_ACTIVE^1);


  	gpio_init_io(IR_LED1, GPIO_OUTPUT);
  	gpio_set_port_attribute(IR_LED1, ATTRIBUTE_HIGH);
  	gpio_write_io(IR_LED1, IR_LED_ACTIVE^1);

  	led_red_flag=LED_OFF;
	led_green_flag=LED_OFF;
	led_blue_flag = LED_OFF;
	led_ir_flag = LED_OFF;
	//LED init放到其他地方去
	#if 1
	type = LED_INIT;
	msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
	#endif
	
  	sys_registe_timer_isr(LED_blanking_isr);	//timer base c to start adc convert
}
/*
void led_lvd_power_off(void)
{
	INT32U t;
	led_all_off();
	for(t=0; t<5; t++)
	{
		OSTimeDly(20);
		led_blue_on();
		OSTimeDly(20);
		led_blue_off();
	}
}
*/
void led_power_off(void)
{	
	INT32U type;

	type = DISABLE_KEY;
	msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);
	led_all_off();
	/*
	if (Low_Voltage_Flag)
	{
		sys_release_timer_isr(LED_blanking_isr);
		led_lvd_power_off();
		sys_registe_timer_isr(LED_blanking_isr);
	}
	else
	*/
	{	sys_release_timer_isr(LED_blanking_isr);
		OSTimeDly(20);
		led_red_on();
		led_blue_on();
		OSTimeDly(200);        		
		led_all_off();
		sys_registe_timer_isr(LED_blanking_isr);
	}
}
/*
void led_motion_reday(void)
{
	sys_release_timer_isr(LED_blanking_isr);
	led_green_off();
	led_blue_on();
	OSTimeDly(30);
	led_blue_off();
	sys_registe_timer_isr(LED_blanking_isr);
}

void led_video_reday(void)
{
	INT32U t;
	
	//led_red_off();
	//sys_release_timer_isr(LED_blanking_isr);
	for(t=0; t<3; t++)
	{
		led_green_off();
		OSTimeDly(20);
		led_green_on();
		OSTimeDly(20);
	}
	//sys_registe_timer_isr(LED_blanking_isr);
	led_green_off();
}
*/
extern INT8U card_space_less_flag;
extern volatile  INT8U pic_down_flag;
static INT8U ir_power_flag = 0;
void set_led_mode(LED_MODE_ENUM mode)
{
	INT8U i;
	//static INT8U prev_mode=0xaa;
    //INT32U type;
    //static INT8U led_r_b_flag = 0;
    INT8U led_red_cnt_set;
	
	g_led_g_state = 0;	//3oE??÷oiAAA
	g_led_r_state = 0;
	g_led_b_state = 0; 
	g_led_flicker_state = 0;
	//目前做法是容量小于设置的最低容量后，灯不在给出任何响应
	//if(card_space_less_flag)
		//return;
	LastLedMode = NowLedMode;
	switch((INT32U)mode)
	{
		case LED_INIT://初始化
			led_all_off();
			led_red_on();
			DBG_PRINT("led_type = LED_INIT\r\n");
			break;
		case LED_UPDATE_PROGRAM://开始升级
			led_all_off();
			g_led_r_state=5;
			DBG_PRINT("led_type = LED_UPDATE_PROGRAM\r\n");
			break;
		case LED_UPDATE_FINISH://升级完成
			led_all_off();
			g_led_r_state = 1;
			DBG_PRINT("led_type = LED_UPDATE_FINISH\r\n");
			break;
		case LED_UPDATE_FAIL://升级失败
			sys_release_timer_isr(LED_blanking_isr);
			led_all_off();
			for(i=0;i<2;i++)
			{
				OSTimeDly(15);
				led_red_on();
				OSTimeDly(15);
				led_red_off();
			}
			DBG_PRINT("led_type = LED_UPDATE_FAIL\r\n");
			sys_registe_timer_isr(LED_blanking_isr);
			ap_peripheral_handling_power_off();  //升级失败关机
			break;
		case LED_CAPTURE_FAIL://拍照失败
			for(i=0;i<2;i++)
			{
				led_red_off();
				led_blue_on();
				OSTimeDly(50);
				led_blue_off();
			}
		case LED_WAITING_CAPTURE://拍照待机
			if(usb_state_get())
		   	{
		   		g_led_r_state = 0;
		    	charge_flash_pro();
		   	}
			else
			{
				led_red_off();	
				led_blue_on();
				DBG_PRINT("led_type = LED_WAITING_CAPTURE\r\n");
			}
			break;
		case LED_CAPTURE://拍照
			//led_green_on();
			led_blue_off();
			DBG_PRINT("led_type = LED_CAPTURE\r\n");
			break;			
		case LED_WAITING_RECORD://录像待机
			if(usb_state_get())
		   	{
		    	charge_flash_pro();
		   	}
			else
			{
				led_blue_off();
				led_red_on();		
				DBG_PRINT("led_type = LED_WAITING_RECORD\r\n");
			}
			break;
		case LED_RECORD_READY:
			led_all_off();
			LedFilkerTime = 26; //125ms
			LedFilkerCnt = 1;
			LedFilkerStep = RED_FLIKER;
			DBG_PRINT("led_type = LED_RECORD_READY\r\n");
			break;	
		case LED_RECORD://录像
			led_red_off();
			led_blue_off();
			//g_led_r_state = 3;
			//g_led_flicker_state = 0;
			DBG_PRINT("led_type = LED_RECORD\r\n");
			break;
		case LED_AUDIO_READY:
			DBG_PRINT("led_type = LED_AUDIO_READY\r\n");
			led_all_off();
			LedFilkerTime = 26; //320ms
			LedFilkerCnt = 2;
			LedFilkerStep = 0x40;	
			break;	
		case LED_WAITING_AUDIO_RECORD://录音待机		
			//led_all_off();
			led_red_off();
			led_blue_on();
			DBG_PRINT("led_type = LED_WAITING_AUDIO_RECORD\r\n");
			break;
		case LED_AUDIO_RECORD://录音
			led_red_off();
			led_blue_off();
			DBG_PRINT("led_type = LED_AUDIO_RECORD\r\n");
			break;
		case LED_MOTION_READY:	
			DBG_PRINT("led_type = LED_MOTION_READY\r\n");
			led_all_off();
			LedFilkerTime = 26; //320ms
			LedFilkerCnt = 3;
			LedFilkerStep = RED_FLIKER;	
			break;
		case LED_MOTION_WAITING://移动侦测待机		
			led_red_off();
			led_blue_off();
			//g_led_b_state = 3;
			//g_led_flicker_state = 0;
			//led_blue_off();
			DBG_PRINT("led_type = LED_MOTION_WAITING\r\n");
			break;
		case LED_MOTION_DETECTION://移动侦测	
			led_blue_off();
			led_red_off();
			//g_led_b_state = 2;
			//g_led_g_state = 3;
			//g_led_flicker_state = 0;
			DBG_PRINT("led_type = LED_MOTION_DETECTION\r\n");
			break;
		case LED_CARD_DETE_SUC://检测到卡
			if(storage_sd_upgrade_file_flag_get() == 2) break;
			if(usb_state_get())
		   	{
		    	charge_flash_pro();
		   	}
			else
			{
				LedFilkerTime = 26; //320ms
				LedFilkerCnt = 1;
				LedFilkerStep = RED_FLIKER|0x1;	
				DBG_PRINT("led_type = LED_CARD_DETE_SUC\r\n");
			}
			break;
		case LED_USB_CONNECT://连接USB	
			if(LastLedMode != mode)
			{
				g_led_count=0;
			}
			led_all_off();
			led_blue_on();
 			//g_led_r_state = 2;	//红灯同时慢闪
			//g_led_flicker_state = 0;
			DBG_PRINT("led_type = LED_USB_CONNECT\r\n");
			break;
		case LED_NO_SDC://无卡
			if (ap_peripheral_low_vol_get()) break; 
			if(usb_state_get())
		   	{
		    	charge_flash_pro();
		   	}
			else
			{
				DBG_PRINT("led_type = LED_NO_SDC\r\n");
				led_all_off();
				LedFilkerTime = 16; //125ms
				LedFilkerCnt = 5;
				LedFilkerStep = BLUE_FLIKER;	
				/*
				g_led_r_state = 5;	//红灯同时慢闪
				g_led_flicker_state = 0;
				*/
			}
			break;
		case LED_SDC_FULL://卡满
		case LED_CARD_NO_SPACE://卡内剩余空间不够
			if(storage_sd_upgrade_file_flag_get() == 2) break;
			
			if(usb_state_get())
		   	{
		    	charge_flash_pro();
		   	}
			else
			{
				DBG_PRINT("led_type = LED_SDC_FULL\r\n");
				led_red_on();
				led_blue_off();
				LedFilkerTime = 16; //125ms
				LedFilkerCnt = 5;
				LedFilkerStep = BLUE_FLIKER;	
				/*
				g_led_r_state = 5;	//红灯同时慢闪
				g_led_flicker_state = 0;
				*/
			}
			break;      
		case LED_LVD_POWER_OFF://低电压关机
			led_red_off();
			led_blue_on();
			LedFilkerTime = 26; //200ms
			LedFilkerCnt = 5;
			LedFilkerStep = RED_FLIKER;	
			break;
		case LED_CHARGE_FULL://电已充满
			led_red_on();
			led_blue_off();
			g_led_b_state = 0;
			DBG_PRINT("led_type = LED_CHARGE_FULL\r\n");
			break;
		case LED_CHARGEING://正在充电
			//g_led_b_state = 2;
			if((LastLedMode == LED_CHARGE_FULL)||(LastLedMode ==LED_CHARGEING))
		   	{
		   		if(usb_state_get())
		   	    {
		        	charge_flash_pro();
		   	    }
		   		break;
		   	}
		   	led_red_off();
			g_led_b_state = 2;
			g_led_flicker_state = 0;
			DBG_PRINT("led_type = LED_CHARGEING\r\n");
			break;
		case LED_STATUS_INDICATORS: //状态提示
			DBG_PRINT("led_type = LED_STATUS_INDICATORS\r\n");
			break;
		case LED_IR_STATUS:
			DBG_PRINT("led_type = LED_IR_STATUS\r\n");
			if (ir_power_flag) led_red_cnt_set = 3;
			else led_red_cnt_set = 2;

			LedFilkerTime = 26;//200ms
			LedFilkerCnt = led_red_cnt_set;
			LedFilkerStep = RED_FLIKER;
			
			ir_power_flag ^= 1;
			if (ir_power_flag) led_ir_on(); //红外灯开
			else led_ir_off();//红外灯关
			break;
	}
	NowLedMode = mode;
}

void led_red_on(void)
{
  if(led_red_flag != LED_ON)
    {
	  gpio_write_io(LED2, LED2_ACTIVE^0);
	  led_red_flag=LED_ON;
    }
}

void led_ir_on(void)
{
    if(led_ir_flag != LED_ON)
    {
	  gpio_write_io(IR_LED1, IR_LED_ACTIVE^0);
	  led_ir_flag=LED_ON;
    }
}

void led_blue_on(void)
{
    if(led_blue_flag != LED_ON)
    {
	  gpio_write_io(LED1, LED1_ACTIVE^0);
	  led_blue_flag=LED_ON;
    }
}

void led_green_on(void)
{
    if(led_green_flag != LED_ON)
    {
	  gpio_write_io(LED3, LED3_ACTIVE^0);
	  led_green_flag=LED_ON;
    }
}
void led_all_off(void)
{   if(led_blue_flag != LED_OFF)
    {
	   gpio_write_io(LED1, LED1_ACTIVE^1);
	   led_blue_flag=LED_OFF;
    }
    if(led_red_flag != LED_OFF)
    {
	   gpio_write_io(LED2, LED2_ACTIVE^1);
	   led_red_flag=LED_OFF;
    }
    if(led_green_flag != LED_OFF)
    {
	   gpio_write_io(LED3, LED3_ACTIVE^1);
	   led_green_flag=LED_OFF;
    }
    if(led_ir_flag != LED_OFF)
	{
	  gpio_write_io(IR_LED1,IR_LED_ACTIVE^1);
	  led_ir_flag=LED_OFF;
	}
}

void led_ir_off(void)
{
	if(led_ir_flag != LED_OFF)
	{
	  gpio_write_io(IR_LED1,IR_LED_ACTIVE^1);
	  led_ir_flag=LED_OFF;
	}
}

void led_blue_off(void)
{
	if(led_blue_flag != LED_OFF)
	{
	  gpio_write_io(LED1,LED1_ACTIVE^1);
	  led_blue_flag=LED_OFF;
	}
}

void led_red_off(void)
{
	if(led_red_flag != LED_OFF)
	{
	 gpio_write_io(LED2, LED2_ACTIVE^1);
	 led_red_flag=LED_OFF;
	}
}

void led_green_off(void)
{
	if(led_green_flag != LED_OFF)
	{
	 gpio_write_io(LED3, LED3_ACTIVE^1);
	 led_green_flag=LED_OFF;
	}
}

INT8U run_led_fliker(void)
{
	static INT8U LedCnt = 0;
    static INT8U LedTime = 0;
    INT32U type;

	if(LedFilkerStep)
	{	
		
		switch(LedFilkerStep)
		{
			case 0x10:
				led_red_off();
				LedFilkerStep = 0x12;
				break;
			case 0x11:
				led_red_on();
				LedFilkerStep = 0x13;
				break;
			case 0x20:
				led_blue_off();
				LedFilkerStep = 0x22;
				break;
			case 0x21:
				led_blue_on();
				LedFilkerStep = 0x23;
				break;
			case 0x30:
				led_green_off();
				LedFilkerStep = 0x32;
				break;
			case 0x31:
				led_green_on();
				LedFilkerStep = 0x33;
				break;
			case 0x40:
				led_red_off();
				led_blue_off();
				LedFilkerStep = 0x42;
				break;
			case 0x41:
				led_red_on();
				led_blue_on();
				LedFilkerStep = 0x43;
				break;
			case 0x12:
			case 0x22:
			case 0x32:
			case 0x13:
			case 0x23:
			case 0x33:
			case 0x42:
			case 0x43:
				if (++LedTime > LedFilkerTime) //200ms
				{
					LedTime = 0;
					switch(LedFilkerStep)
					{
						case 0x12:
							LedFilkerStep = 0x11;
							break;
						case 0x13:
							LedFilkerStep = 0x10;
							break;
						case 0x22:
							LedFilkerStep = 0x21;
							break;
						case 0x23:
							LedFilkerStep = 0x20;
							break;
						case 0x32:
							LedFilkerStep = 0x31;
							break;
						case 0x33:
							LedFilkerStep = 0x30;
							break;
						case 0x42:
							LedFilkerStep = 0x41;
							break;
						case 0x43:
							LedFilkerStep = 0x40;
							break;
					}
					if (++LedCnt > (LedFilkerCnt<<1))
					{
						LedCnt = 0;
						LedFilkerStep = 0xFAA;
					}
				}
				break;
			case 0xFAA:	
				switch(NowLedMode)
				{
					case LED_NO_SDC:
						//break;
					case LED_SDC_FULL:
						//break;
					case LED_CARD_NO_SPACE:
						type = 0;
						msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;
					case LED_CARD_DETE_SUC:
						if(usb_state_get())
						{
							charge_flash_pro();
						}
						led_blue_off();
						led_red_on();
						//type = LED_WAITING_RECORD;
			    		//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;	
					case LED_POWER_OFF:
						//type = 1;
						//msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;
					case LED_LVD_POWER_OFF:
						type = 0;
						msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;
					case LED_IR_STATUS:
						led_red_off();
						break;
					case LED_RECORD_READY://此时LED_RECORD_READY已经被LED_RECORD消息冲掉
					case LED_RECORD:
						led_blue_off();
						led_red_off();
						if(usb_state_get())
						{
							charge_flash_pro();
						}
						break;
					case LED_MOTION_READY:
					case LED_MOTION_DETECTION:
						led_blue_off();
						led_red_off();
						if(usb_state_get())
						{
							charge_flash_pro();
						}
						//type = LED_MOTION_WAITING;
			    		//msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;	
					case LED_AUDIO_READY:
					case LED_AUDIO_RECORD:
						led_red_off();
						led_blue_off();
						if(usb_state_get())
						{
							charge_flash_pro();
						}
						break;	
					default:
						type = LastLedMode;
			    		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
						break;		
				}
				LedFilkerStep = 0;
				break;	
			default:
				break;	
		}
	}
	if (LedFilkerStep) return 1;
	else return 0;
}

extern INT8U video_stop_flag;
extern INT8U KeyScan_flag_get(void);
void LED_blanking_isr(void)
{	
   INT8U type=NULL;
   //static INT8U red_cnt = 0;
  // static INT32U red_time = 0;
  // static INT8U red_flag = 0;
   static INT8U red_flash_flag=0;
   static INT8U flash_flag=0;
   static INT8U charge_cnt = 0;

	//if(card_space_less_flag)
	//	return;

	if(++g_led_count >= 128){
		g_led_count = 0;
	}
	
	//+++++++++++++++++++++++++++++++++++++++
	if (run_led_fliker()) return; 
	//++++++++++++++++++++++++++++++++++++++++
	/*
	if (savefile_led_flag)
	{//存贮录像文件时LED快闪:
		if ((video_record_sts&0x2) == 0) 
		{
			savefile_led_flag = 0;
		}
		else
		{
			savefile_led_flag++;
			if (savefile_led_flag < 10)
			{
				led_blue_on();
			}
			else
			{
				led_blue_off();
				if (savefile_led_flag > 17) savefile_led_flag = 1;
			}
		}
		return;
	}
	*/
	if(g_led_r_state ==1)
	{ //=0;默认RB, =1;RG, =2,BG
		led_red_on();
	}
	else if(g_led_r_state == 2)
	{
		if(g_led_count == 0) 
		{
			if (flash_flag)
			{
				flash_flag = 0;
				led_red_on();
			}	
			else
			{
				flash_flag = 1;
				led_red_off();
			}
		}
	}
	else if(g_led_r_state == 3)
	{
		if(g_led_count < 64)
			led_red_on();
		else
			led_red_off();
	}
	else if(g_led_r_state == 4)
	{
		if (g_led_count%32 == 0)
		{
			if (red_flash_flag)
			{
				red_flash_flag = 0;
				led_red_on();
			}
			else
			{
				red_flash_flag = 1;
				led_red_off();
			}
		}
	}
	else if(g_led_r_state == 5)
	{
		if (g_led_count%16 == 0)
		{
			if (red_flash_flag)
			{
				red_flash_flag = 0;
				led_red_on();
			}
			else
			{
				red_flash_flag = 1;
				led_red_off();
			}
		}
	}
	else if(g_led_r_state == 6)
	{
		if (g_led_count%8 == 0)
		{
			if (red_flash_flag)
			{
				red_flash_flag = 0;
				led_red_on();
			}
			else
			{
				red_flash_flag = 1;
				led_red_off();
			}
		}
	}

	if(g_led_b_state ==1)
	{ //=0;默认RB, =1;RG, =2,BG
		led_blue_on();
	}
	else if(g_led_b_state == 2)
	{
		if(g_led_count == 0)
		{
			if (usb_state_get())
			{
				if (++charge_cnt > 2)
				{
					charge_cnt = 0;
					if (flash_flag)
					{
						led_blue_on();
						flash_flag = 0;
					}
					else
					{
				    	led_blue_off();
						flash_flag = 1;
					}
				}			
			}
			else
			{		
				if (flash_flag)
				{
					if (g_led_flicker_state == 0) 
						led_blue_on();
					else
						led_blue_off();
					flash_flag = 0;
				}
				else
				{
					if (g_led_flicker_state == 0)
				    	led_blue_off();
					else
						led_blue_on();
					flash_flag = 1;
				}
			}
		}
	}
	else if(g_led_b_state == 3)
	{
		if(g_led_count/64 == g_led_flicker_state)
			led_blue_on();
		else
			led_blue_off();
	}
	else if(g_led_b_state == 4)
	{
		if (g_led_count%32 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				if (g_led_flicker_state == 0)
					led_red_on();
				else
					led_red_off();
			}
			else
			{
				flash_flag = 1;
				if (g_led_flicker_state == 0)
					led_red_off();
				else
					led_red_on();
			}
		}
	}
	else if(g_led_b_state == 5)
	{
		if (g_led_count%16 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				if (g_led_flicker_state == 0)
					led_red_on();
				else
					led_red_off();
			}
			else
			{
				flash_flag = 1;
				if (g_led_flicker_state == 0)
					led_red_off();
				else
					led_red_on();
			}
		}
	}
	else if(g_led_b_state == 6)
	{
		if (g_led_count%8 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				if (g_led_flicker_state == 0)
					led_red_on();
				else
					led_red_off();
			}
			else
			{
				flash_flag = 1;
				if (g_led_flicker_state == 0)
					led_red_off();
				else
					led_red_on();
			}
		}
	}
	
		
    if(g_led_g_state ==1)
	{ //=0;默认RB, =1;RG, =2,BG
		led_green_on();
	}
	else if(g_led_g_state == 2)
	{
		if(g_led_count  == 0)
		{
			if (flash_flag)
			{ 
				led_green_on();
				flash_flag = 0;
			}
			else
			{
				led_green_off();
				flash_flag = 1;
			}
		}
	}
	else if(g_led_g_state == 3)
	{
		if(g_led_count < 64)
			led_green_on();
		else
			led_green_off();
	}
	else if(g_led_g_state == 4)
	{
		if (g_led_count%32 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				led_green_on();
			}
			else
			{
				flash_flag = 1;
				led_green_off();
			}
		}
	}
	else if(g_led_g_state == 5)
	{
		if (g_led_count%16 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				led_green_on();
			}
			else
			{
				flash_flag = 1;
				led_green_off();
			}
		}
	}
	else if(g_led_g_state == 6)
	{
		if (g_led_count%8 == 0)
		{
			if (flash_flag)
			{
				flash_flag = 0;
				led_green_on();
			}
			else
			{
				flash_flag = 1;
				led_green_off();
			}
		}
	}
 //  ap_peripheral_key_judge();
   // msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SINGLE_JUGE, &type, sizeof(INT8U), MSG_PRI_NORMAL);
    if (KeyScan_flag_get() == 0)
	{
		msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_SINGLE_JUGE, &type, sizeof(INT8U), MSG_PRI_NORMAL);
	}


}




#if C_MOTION_DETECTION == CUSTOM_ON

void ap_peripheral_motion_detect_judge(void)
{
	INT32U result;
	//DBG_PRINT("-\r\n");//fan
	if(video_down_flag)
		return;
	result = hwCdsp_MD_get_result();
	DBG_PRINT("MD_result = 0x%x\r\n",result);
	//if(result>0x40){MD_SENSITIVE
	if(result>MD_SENSITIVE){ //lx 2015-09-14
		msgQSend(ApQ, MSG_APQ_MOTION_DETECT_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
	}
}

void ap_peripheral_motion_detect_start(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_START);
}

void ap_peripheral_motion_detect_stop(void)
{
	motion_detect_status_set(MOTION_DETECT_STATUS_STOP);
}
#endif

#if USE_ADKEY_NO
void ap_peripheral_ad_detect_init(INT8U adc_channel, void (*ad_detect_isr)(INT16U data))
{
	#if C_BATTERY_DETECT == CUSTOM_ON
   	battery_value_sum=0;
	bat_ck_cnt=0;
	#endif
//	ad_value_cnt = 0;
	
	adc_init();
	adc_vref_enable_set(TRUE);
	//adc_conv_time_sel(1);//lx
	adc_conv_time_sel(4);
	adc_manual_ch_set(adc_channel);
	adc_manual_callback_set(ad_detect_isr);
	if (ad_detect_timerid == 0xFF) {
		ad_detect_timerid = AD_DETECT_TIMER_ID;
		sys_set_timer((void*)msgQSend, (void*) PeripheralTaskQ, MSG_PERIPHERAL_TASK_AD_DETECT_CHECK, ad_detect_timerid, PERI_TIME_INTERVAL_AD_DETECT);
	}
}

void ap_peripheral_ad_check_isr(INT16U value)
{
	ad_value = value;
}

INT16U adc_key_release_calibration(INT16U value)
{
	return value;
}

void ap_peripheral_clr_screen_saver_timer(void)
{
	key_active_cnt = 0;
}

#if  0//(KEY_TYPE == KEY_TYPE1)||(KEY_TYPE==KEY_TYPE2)||(KEY_TYPE==KEY_TYPE3)||(KEY_TYPE==KEY_TYPE4)||(KEY_TYPE==KEY_TYPE5)



#else

/*
	0.41v => 495
	0.39v =>
	0.38v => 460
	0.37v => 
	0.36v => 440
	0.35v =>
	0.34v =>
*/
/*
enum {
	BATTERY_CNT = 8,
	BATTERY_Lv3 = 495*BATTERY_CNT,
	BATTERY_Lv2 = 460*BATTERY_CNT,
	BATTERY_Lv1 = 440*BATTERY_CNT
};
*/
enum {
	BATTERY_CNT = 8,
	BATTERY_Lv3 = 2515*BATTERY_CNT,	 //4V	
	BATTERY_Lv2 = 2319*BATTERY_CNT,  //3.75V
	BATTERY_Lv1 = 2175*BATTERY_CNT   //3.55V 
};

#if USE_ADKEY_NO == 6
static INT32U adc_key_factor_table[USE_ADKEY_NO] = {	// x1000
	// 6 AD-keys
	//680K, 300K, 150K, 68K, 39K, 22K
	1969, 2933, 4182, 5924, 7104, 8102
};
#else
static INT32U adc_key_factor_table[USE_ADKEY_NO] = {	// x1000
	// 1 AD-keys
	//680K
	1969
};
#endif
//static INT32U ad_time_stamp;

INT32U adc_key_judge(INT32U adc_value)
{
	INT32U candidate_key;
	INT32U candidate_diff;
	INT32U i, temp1, temp2, temp3, diff;

	candidate_key = USE_ADKEY_NO;
	candidate_diff = 0xFFFFFFFF;
	temp1 = 1000 * adc_value;   // to avoid "decimal point"

	temp2 = adc_key_release_calibration(adc_key_release_value_stable); // adc_battery_value_stable = stable adc value got when no key press
	                                                           // temp2: adc theoretical value 
	for (i=0; i<USE_ADKEY_NO; i++) {
		temp3 = temp2 * adc_key_factor_table[i];              // temp3: the calculated delimiter   
		if (temp1 >= temp3) {
			diff = temp1 - temp3;
		} else {
			diff = temp3 - temp1;
		}
		// DBG_PRINT("adc:[%d], bat:[%d], diff:[%d]\r\n", temp1, temp3, diff);

		if (diff > candidate_diff) {
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			ASM(NOP);
			
			break;
		}

		candidate_key = i;
		candidate_diff = diff;
	}

	if (candidate_key < USE_ADKEY_NO) {
		//DBG_PRINT("\r\nKey %d", candidate_key+1);
//		power_off_time_beep_1 = 0;
//		power_off_time_beep_2 = 0;
//		power_off_time_beep_3 = 0;
#if C_SCREEN_SAVER == CUSTOM_ON
		key_active_cnt = 0;
		ap_peripheral_lcd_backlight_set(BL_ON);
#endif
	}
	
	return candidate_key;
}

//#define SA_TIME	50	//seconds, for screen saver time. Temporary use "define" before set in "STATE_SETTING".
#define SPI_SAVE_FILE_THRESHOLD 		0x0888	//0x0863	//低电压自动保存文件
#define SPI_LVD_THRESHOLD 				0x0860	//0x080C	//0x081F	//自动关机门槛电压

void ap_peripheral_ad_key_judge(void)
{
	static INT32U adc_val_cnt = 0;
	static INT32U adc_val_sum = 0;
	static INT8U  Start_LowVol_PowOff_Flag = 0;
	INT32U type;
	
	adc_val_cnt++;
	adc_val_sum += (ad_value >> 4);
	if (adc_val_cnt >= 64)
	{
		adc_val_sum = (adc_val_sum >> 6); //
		 #if C_BATTERY_DETECT == CUSTOM_ON
		 if (adc_val_sum >= 0x10000)
		 {
		 	adc_val_sum = 0xFF00;
		 }
		  if (Start_LowVol_PowOff_Flag) return;

		 if (adc_val_sum <= SPI_LVD_THRESHOLD)
		 {
		    if (++low_voltage_cnt >= 3)
		    {
				//检测是否在录像, 录像保存或拍照保存状态
				Low_Voltage_Flag = 1;
				DBG_PRINT("LOW VOLTAGE POWER OFF!\r\n");
				type = LED_LVD_POWER_OFF;
			    msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &type, sizeof(INT32U), MSG_PRI_NORMAL);
				Start_LowVol_PowOff_Flag = 1;
		    }
		 }
		 else 
		 {
		 	//EncordVideo_Unlock_Flag = 1;
		 	if (Low_Voltage_Flag == 0)
		 	{
		 		low_voltage_cnt = 0;
         	}
         }
        	//ap_peripheral_battery_check_calculate(adc_val_sum);
		 #endif
		adc_val_cnt = 0;
		adc_val_sum = 0;
	}
	adc_manual_sample_start();
}

#endif // AD-Key
#endif

void ap_peripheral_night_mode_set(INT8U type)
{
	if(type) {
	  	gpio_write_io(IR_CTRL, 1);
	} else {
	  	gpio_write_io(IR_CTRL, 0);
	}
}

void ap_peripheral_key_init(void)
{
	INT32U i;

	gp_memset((INT8S *) &key_map, NULL, sizeof(KEYSTATUS));
	ap_peripheral_key_register(GENERAL_KEY);
	
	for (i=0 ; i<USE_IOKEY_NO ; i++)
	{
		 if (key_map[i].key_io == PW_KEY)
		 {
		 	;
		 } 
		 else if (key_map[i].key_io) 
		{
  		  key_map[i].key_cnt = 0;
		  gpio_init_io(key_map[i].key_io, GPIO_INPUT);
		  gpio_set_port_attribute(key_map[i].key_io, ATTRIBUTE_LOW);
  		 // gpio_write_io(key_map[i].key_io, KEY_ACTIVE^1);
  		 gpio_write_io(key_map[i].key_io,  key_map[i].key_active^1);  //Liuxi modified in 2015-01-15
		 DBG_PRINT("INIT\r\n");
  	   }
   }
}

void ap_peripheral_key_register(INT8U type)
{
INT32U i;
	
if (type == GENERAL_KEY) 
	{
         key_map[0].key_io = PW_KEY;
		 key_map[0].key_function = (KEYFUNC) ap_peripheral_power_on_exe;
		 key_map[0].key_active= 1;
		 key_map[0].long_key_flag=0;
		 //key_map[0].double_key_flag = 0;
		 key_map[0].step = 0xFF;
		 key_map[0].keypress = 0;
		 
/*
		 key_map[1].key_io = VIDEO_KEY;
		 key_map[1].key_function = (KEYFUNC) ap_peripheral_mode_key_exe;
		 key_map[1].key_active= VIDEO_KEY_ACTIVE;//1;
		 key_map[1].long_key_flag=0;
		 key_map[1].double_key_flag = 0;
*/		 
		 ad_key_map[0].key_io = FUNCTION_KEY;
		 ad_key_map[0].key_function = (KEYFUNC) ap_peripheral_null_key_exe;
	}
else if (type == USBD_DETECT) 
	{
        #if USE_IOKEY_NO
		for (i=0 ; i<USE_IOKEY_NO ; i++) {
			//if(key_map[i].key_io != POWER_KEY)
			if(key_map[i].key_io != PW_KEY)
			key_map[i].key_io = NULL;
		}
        #endif
        #if USE_ADKEY_NO		
		for (i=0 ; i<USE_ADKEY_NO ; i++) {
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}
        #endif
	} 
else if (type == DISABLE_KEY)
	{
        #if USE_IOKEY_NO
		for (i=0 ; i<USE_IOKEY_NO ; i++) {
			key_map[i].key_io = NULL;
		}
        #endif
        #if USE_ADKEY_NO		
		for (i=0 ; i<USE_ADKEY_NO ; i++) {
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}
        #endif
	}
else if (type == BETTERY_LOW_STATUS_KEY)
	{
		for (i=0 ; i<USE_IOKEY_NO ; i++) {
			key_map[i].key_io = NULL;
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}
		//key_map[0].key_io = PW_KEY;
		//key_map[0].key_function = (KEYFUNC) ap_peripheral_pw_key_exe;
        #if USE_ADKEY_NO		
		for (i=0 ; i<USE_ADKEY_NO ; i++) {
			ad_key_map[i].key_function = ap_peripheral_null_key_exe;
		}
        #endif
	}

}


extern INT8U ap_state_config_auto_off_get(void);

INT8U long_pw_key_pressed = 0;
#if CRAZY_KEY_TEST == 1
INT8U crazy_key_enable=0;
INT32U crazy_key_cnt=0;
#endif
void ap_peripheral_charge_det(void)
{
	INT16U pin_state=0;
	static INT16U prev_pin_state=0;
	INT16U led_type;
	static INT8U loop_cnt=0;
	static INT16U prev_ledtype=0;
	pin_state=gpio_read_io(CHARGE_DETECTION_PIN);
	//DBG_PRINT("pin_state=%d",pin_state);
	if(pin_state == prev_pin_state)
	  {
	   loop_cnt++;
	  }
	else
	   loop_cnt=0;
	if(loop_cnt >=3)
	  {
	  loop_cnt=3;
	   if(pin_state)
		  led_type=LED_CHARGE_FULL;
	   else
		  led_type=LED_CHARGEING;  
	   if(prev_ledtype != led_type)
		  {
		   prev_ledtype=led_type;
			//if(((video_record_sts & 0x02) == 0))
			  {
				if(storage_sd_upgrade_file_flag_get() != 2)
				 msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
			  }
		   //msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
		  }
	  }
	
	prev_pin_state=pin_state;

}

INT8U read_pw_key0(void)
{
	if (sys_pwr_key0_read() != 0) return 1;
	else return 0;
}


#define KEY_PRESS_OVER				0xFFFF //按键超时
#define KEY_STEP_RESET				0xFF   //检测复位
#define KEY_RELEASE_OVER			32+16  //0.75秒, 释放按超时

#define KEY_PRESS_VAL_LONG		0x02 //长按键值
#define KEY_PRESS_VAL_SHORT		0x01 //短按键值

#define COUNT_PERSS_KEY_NUM		2 //连接次数
void ap_peripheral_key_judge(void)
{
	INT32U i;
	INT16U key_down=0;
	INT32U type = NULL;
	static INT8U print_cnt = 0;
	//static INT8U  old_sta = 0xFE;
	//static INT32U Err_Correct_cnt = 0;

	if(++print_cnt >64)
	{
		DBG_PRINT("Low_Voltage_Flag = %d\n", Low_Voltage_Flag);
		print_cnt = 0;
	}
	if (ap_peripheral_low_vol_get()) return;
	/*
	for (i=0 ; i<USE_IOKEY_NO ; i++) 
	{
		if(key_map[i].key_io == PW_KEY) 
		{
			key_down = read_pw_key0();	//gpio_read_io(key_map[i].key_io);
		}
		else 
		{
			if(key_map[i].key_active) key_down = gpio_read_io(key_map[i].key_io);
			else key_down = !gpio_read_io(key_map[i].key_io);
		}
		if(key_down)
		{
			key_map[i].key_cnt += 1;  
			if(!(key_map[i].long_key_flag))
			{
			//#if SUPPORT_LONGKEY == CUSTOM_ON
				if(key_map[0].double_key_flag > 1)
				{
					do_click = 1;
				} 
				else
				{
					if (key_map[i].key_cnt >= Long_Single_width)
					{
						key_map[i].long_key_flag=1;
						key_map[i].key_function(&(key_map[i].key_cnt));
						key_map[0].double_key_flag = 0;
					}
				}					
			} 
			else
			//#endif
			{
				key_map[i].key_cnt = 0;
			}
			if (key_map[i].key_cnt == 65535)
			{
				key_map[i].key_cnt = Long_Single_width+4;
			}
		} 
		else 
		{
			if (key_map[i].long_key_flag == 1)
			{
				key_map[i].long_key_flag=0;
			}
			else if (do_click)
			{
				key_map[i].key_cnt = Double_Single_width;
				key_map[i].key_function(&(key_map[i].key_cnt));
				key_map[i].key_cnt = 0; //未达到短按时间时, 计数清零
				key_map[0].double_key_flag = 0;
				do_click = 0;
			}
			else if(key_map[i].key_cnt > 1 )//Short_Single_width)
			{	
				if (key_map[i].key_cnt >= Short_Single_width)
				{
					if (key_map[0].double_key_flag > Long_Single_width)
					{
						key_map[i].key_function(&(key_map[i].key_cnt));
						key_map[i].key_cnt = 0; //未达到短按时间时, 计数清零
						key_map[0].double_key_flag = 0;
					}
				}
				key_map[0].double_key_flag++;
			}
		}
	}
	*/
	for (i=0; i<USE_IOKEY_NO; i++)
	{
		if (key_map[i].key_io == PW_KEY)	
		{//Power key (PWR_ON0)
			key_down = read_pw_key0();	
		}
		else 
		{//Other
			if(key_map[i].key_active) key_down = gpio_read_io(key_map[i].key_io);
			else key_down = !gpio_read_io(key_map[i].key_io);
		}
		
		switch(key_map[i].step)
		{
			case 0://第一次按下后进入
				if (key_down)
				{//按下
					if (!key_map[i].long_key_flag)
					{//长按标志为0时
						key_map[i].key_cnt++;
						if (key_map[i].key_cnt >= Long_Single_width)
						{//长按键	
							key_map[i].keypress = 0;
							key_map[i].long_key_flag = 1;
							key_map[i].key_cnt = KEY_PRESS_VAL_LONG; //一次长按键键值
							key_map[i].key_function(&key_map[i].key_cnt);
							key_map[i].step = KEY_STEP_RESET;
						}
						else 
						{//短按键
							if (key_map[i].key_cnt >= 3) key_map[i].keypress = 1;			
						}	
					}
					else 
					{//长按标志为1时
						key_map[i].key_cnt = 0;
					}	
				}
				else 
				{//松开
					if((key_map[i].keypress)&&(key_map[i].long_key_flag == 0)) 
					{//有效按键
						key_map[i].step = 1; 
					}
					else 
					{//无效触发
						key_map[i].step = KEY_STEP_RESET; //复位
					}
					key_map[i].key_cnt = 1;
				}				
				break;
			case 1: //一次短按处理
				if (!key_down)
				{//松开
						key_map[i].key_cnt++;
						if (key_map[i].key_cnt >= KEY_RELEASE_OVER)	
						{//松开超时处理
							if (key_map[i].keypress)
							{//有效按键时, 为一次短按键处理
								key_map[i].keypress = 0;
								key_map[i].key_cnt = KEY_PRESS_VAL_SHORT; //一次短按键键值
								key_map[i].key_function(&key_map[i].key_cnt);
								key_map[i].step = KEY_STEP_RESET; //复位
							}	
							else 
							{//无效按键时复位
								key_map[i].long_key_flag = 0;
								key_map[i].step = KEY_STEP_RESET;//复位
							}
						}
				}
				else 
				{//按下
					if (key_map[i].keypress)
					{
						key_map[i].key_cnt = 1;
						key_map[i].step = 2;
					}
				}
				break;
			case 2://N次短按键处理
				key_map[i].key_cnt++;
				if (key_down)
				{//按下
					if (key_map[i].key_cnt >= KEY_PRESS_OVER) key_map[i].key_cnt = KEY_PRESS_OVER;
				}
				else 
				{//松开
					if (key_map[i].key_cnt >= Short_Single_width)
					{
							if (key_map[i].keypress) 
							{	
								key_map[i].keypress++;
								key_map[i].step = 3;
							}
							key_map[i].key_cnt = 1;
					}
				}
				break;
			case 3://N次短按键处理
				key_map[i].key_cnt++;
				if (!key_down)
				{
					if (key_map[i].key_cnt >= KEY_RELEASE_OVER)
					{
						key_map[i].key_cnt = KEY_PRESS_VAL_SHORT|(key_map[i].keypress<<8);
						key_map[i].step = KEY_STEP_RESET;
						key_map[i].keypress = 0;
						key_map[i].key_function(&key_map[i].key_cnt);
					}
				}
				else 
				{
					if (key_map[i].keypress >= COUNT_PERSS_KEY_NUM) //重新多次按键次数判断
					{
						key_map[i].keypress = 0xFF;
						key_map[i].key_cnt  = 0;
						key_map[i].step = KEY_STEP_RESET; //复位
					}
					else	
					{
							key_map[i].key_cnt = 1;
							key_map[i].step = 2;
					}
				}
				break;
			default: //清除各标志的状态
				if (key_down)
				{
					if ((key_map[i].long_key_flag == 0)&&(key_map[i].keypress == 0)) 
					{//检测到按键
						key_map[i].step = 0;
						key_map[i].keypress = 0;
						key_map[i].key_cnt = 1;
					}
				}
				else
				{
					if (key_map[i].keypress == 0xFF)
					{
						if (++key_map[i].key_cnt > KEY_RELEASE_OVER) 
						{
							key_map[i].key_cnt = 0;
							key_map[i].keypress = 0;
						}
					}
					key_map[i].long_key_flag = 0;
				}
				break;		
		}
	}
	
	
	#if 1
	if ( !auto_off_force_disable && !s_usbd_pin && !(usb_state_get())) 
	{
		if(power_off_cnt)
		{
			power_off_cnt--;
			if(power_off_cnt < 1)
			{
				DBG_PRINT("[Idle waiting to turn off...]");
				type = 0;
				msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
			}
		}
	}
	else
	{
		power_off_cnt=C_IDLE_SLEEP;	
	}
	#endif
}

void ap_peripheral_handling_power_off(void)
{
	//if (s_usbd_pin) return;
	if (usb_state_get()) return;
	
	#if AUD_RECORD_EN
	while(video_down_flag||audio_down_flag)
	#else
	while(video_down_flag)
	#endif
	{
		OSTimeDly(1);
	}
	while(read_pw_key0());
	{//等待松开关机键
		OSTimeDly(1);
	}
    DBG_PRINT("[POWER OFF]\r\n");
	led_all_off();
    LDO12_Switch(0); //disable internal power
    OSTimeDly(1);
    power_pin_set(0);
    while(1);
    OSTimeDly(100);
}

void ap_peripheral_adaptor_out_judge(void)
{
	INT32U type;
	static INT32U Ext_usb_cnt = 0;

	adp_out_cnt++;
	switch(adp_status) {
		case 0: //unkown state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN)) {
				adp_cnt++;
				if (adp_cnt > 16) {
					adp_out_cnt = 0;
					adp_cnt = 0;
					adp_status = 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				  #if C_BATTERY_DETECT == CUSTOM_ON && USE_ADKEY_NO
					//battery_lvl = 1;
				  #endif
				}
			} else {
				adp_cnt = 0;
			}

			if (adp_out_cnt > 24) {
				adp_out_cnt = 0;
				adp_status = 3;
			  #if C_BATTERY_DETECT == CUSTOM_ON && USE_ADKEY_NO
				//battery_lvl = 2;
				low_voltage_cnt = 0;
			  #endif
			}
			break;

		case 1: //adaptor in state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN)) {
				if (adp_out_cnt > 8) {
					adp_status = 2;
					#if C_BATTERY_DETECT == CUSTOM_ON
					low_voltage_cnt = 0;
					#endif
					// 璝棵辊玂臔秨璶翴獹璉
					if(screen_saver_enable) {
						screen_saver_enable = 0;
						#if C_SCREEN_SAVER == CUSTOM_ON
  						//ap_state_handling_lcd_backlight_switch(1);
  						#endif
					}					
				} 
			} else {
				adp_out_cnt = 0;
			}
			break;

		case 2: //adaptor out state
			if (!ap_peripheral_power_key_read(ADP_OUT_PIN)) {
				if ((adp_out_cnt > PERI_ADP_OUT_PWR_OFF_TIME)) {
					//ap_peripheral_handling_power_off();
					DBG_PRINT("[Exit USB And Power Off...]");
					adp_out_cnt=0;
					usb_state_set(0);
					type = 1;
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					adp_status = 0xFF;
				}
				adp_cnt = 0;
			} else {
				adp_cnt++;
				if (adp_cnt > 3) {
					adp_out_cnt = 0;
					adp_status = 1;
					usbd_exit = 0;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				}
			}
			break;

		case 3://adaptor initial out state
			if (ap_peripheral_power_key_read(ADP_OUT_PIN)) {
				if (adp_out_cnt > 3) {
					adp_out_cnt = 0;
					adp_status = 1;
					OSQPost(USBTaskQ, (void *) MSG_USBD_INITIAL);
				} 
			} else {
			if(usb_state_get())
				usb_state_set(0);
				adp_out_cnt = 0;
			}
			break;
		default:
			break;
	}
	///DBG_PRINT("USB_PIN=%d\r\n",s_usbd_pin);
	if (s_usbd_pin == 1) {
		usbd_cnt++;
		if (!ap_peripheral_power_key_read(C_USBDEVICE_PIN)) {
			if (usbd_cnt > 3) {
				ap_peripheral_usbd_plug_out_exe(&usbd_cnt);	
			} 
		} else {
			usbd_cnt = 0;
		}
	}

#if USB_PHY_SUSPEND == 1
	if (s_usbd_pin == 0)
	{
		if (ap_peripheral_power_key_read(C_USBDEVICE_PIN)) {
			if (phy_cnt == PERI_USB_PHY_SUSPEND_TIME) {
				// disable USB PHY CLK for saving power
				DBG_PRINT("Turn Off USB PHY clk (TODO)\r\n");				
				phy_cnt++;	// ヘ琌 Turn Off 暗Ω
			}
			else if (phy_cnt<PERI_USB_PHY_SUSPEND_TIME) {
				phy_cnt++;
			}
			Ext_usb_cnt = 0;
		}
		else {
			phy_cnt = 0;
			/*
			if (Ext_usb_cnt < 256) Ext_usb_cnt++;
			else if (Ext_usb_cnt == 256) 
			{
				Ext_usb_cnt++;
				DBG_PRINT("Pull out the USB plug\r\n");	
				type = 1;
				msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);			
			}
			*/
		}
	}
#endif

}

static INT8U pc_camera_flag = 0;
#if C_MOTION_DETECTION == CUSTOM_ON	
	extern void ap_video_record_md_disable(void);
#endif	
void ap_peripheral_power_on_exe(INT16U *tick_cnt_ptr)
{
    INT32U type = NULL;
    //INT32U i;
    
	if (!s_usbd_pin) 
	{
		DBG_PRINT("KyeCode = 0x%X\r\n", *tick_cnt_ptr);
		if (usb_state_get() == 0)
		{//USB供电时无效
			switch(*tick_cnt_ptr)
			{
				case KEY_PRESS_VAL_LONG: //一次长按键
					type = 1;
					msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_NORMAL);
					DBG_PRINT("[POWER_KEY ACTIVE...]");
					break;
				case KEY_PRESS_VAL_SHORT://一次短按键
					if ((ap_state_handling_storage_id_get() == NO_STORAGE)&&card_space_less_flag)
						break;
					if (video_down_flag) 
						break;
					DBG_PRINT("[APQ_VIDEO_RECORD_ACTIVE...]\r\n");
					ap_state_config_record_time_set(AUTO_SAVE_VIDEO_TIME);
					msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					work_mode_set(STA_VIDEO);
					break;
				case (KEY_PRESS_VAL_SHORT|(2<<8))://二次短按键
					if ((ap_state_handling_storage_id_get() == NO_STORAGE)&&card_space_less_flag)
						break;
					if (video_down_flag)
						break;
					DBG_PRINT("[MSG_APQ_MD_SET...]\r\n");
					ap_state_config_record_time_set(5);
					msgQSend(ApQ, MSG_APQ_MD_SET, NULL, NULL, MSG_PRI_NORMAL);
					work_mode_set(STA_MOTION);
					break;
			}
		}		
	}
	else
	{
		if (usb_state_get() == 1)
		{
			if (*tick_cnt_ptr == KEY_PRESS_VAL_LONG)
			{
				pc_camera_flag ^= 1;
				if (pc_camera_flag) led_ir_on();
				else led_ir_off();
			}
			else if (*tick_cnt_ptr == KEY_PRESS_VAL_SHORT)
			{
				if (ap_state_handling_storage_id_get() != NO_STORAGE)
				{
					OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
				}
			}
		}
	}
	*tick_cnt_ptr = 0;
}

void ap_peripheral_rec_key_exe(INT16U *tick_cnt_ptr)
{
	//INT32U led_type;
	 
	if (!s_usbd_pin)
	{
	/*
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
			if((!pic_down_flag)&&(!video_down_flag)&&(!card_space_less_flag))
			{
				#if SUPPORT_LONGKEY == CUSTOM_ON
				if(*tick_cnt_ptr >= Long_Single_width)
				{ 
					if (((video_record_sts&0x6)||(audio_record_sts&0x2)) == 0)
					{
							led_type = LED_MOTION_READY;
			    			msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
					else
					{
						led_type = LED_STATUS_INDICATORS; 
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
				}
				else 
				#endif
				{
					if (((video_record_sts&0x6)||(audio_record_sts&0x2)) == 0)
					{
						DBG_PRINT("AUDIIO_RECORD\r\n");
						msgQSend(ApQ, MSG_APQ_AUDO_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
					else
					{
						led_type = LED_STATUS_INDICATORS; 
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
				}
			}
		}
		*/
	}
	else
	{
		#if SUPPORT_LONGKEY == CUSTOM_ON
		if(*tick_cnt_ptr >= 64)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);

		}
		else
		#endif
		{
			//if(!pic_down_flag)
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}
	*tick_cnt_ptr = 0;
}

void ap_peripheral_capture_key_exe(INT16U *tick_cnt_ptr)
{
	//INT32U led_type;//Sence_Mode
	
	if (!s_usbd_pin)
	{
	/*
		if (ap_state_handling_storage_id_get() != NO_STORAGE)
		{
			if((!pic_down_flag)&&(!card_space_less_flag)&&(!video_down_flag))
			{
				#if SUPPORT_LONGKEY == CUSTOM_ON
				if(*tick_cnt_ptr >= Long_Single_width)
				{ 
					led_type = LED_IR_STATUS; //打开或关闭红外灯
	    			msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
				}
				else 
				#endif
				{
					if (((video_record_sts&0x6)||(audio_record_sts&0x2)) == 0)
					{
						DBG_PRINT("[CAPTUER_ACTIVE...]\r\n");
						msgQSend(ApQ, MSG_APQ_CAPTUER_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
					else
					{
						led_type = LED_STATUS_INDICATORS; 
						msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_LED_SET, &led_type, sizeof(INT32U), MSG_PRI_NORMAL);
					}
				}
			} 
		}
		*/
	} 
	else
	{ 
		#if SUPPORT_LONGKEY == CUSTOM_ON
		if(*tick_cnt_ptr >= 64)
		{
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);

		}
		else
		#endif
		{
			//if(!pic_down_flag)
			//OSQPost(USBTaskQ, (void *) MSG_USBD_SWITCH);
		}
	}
	*tick_cnt_ptr = 0;
}

void ap_peripheral_mode_key_exe(INT16U *tick_cnt_ptr)
{
	//INT32U led_type;//Sence_Mode
	if (!s_usbd_pin)
	{
		if ((ap_state_handling_storage_id_get() != NO_STORAGE)&&(!card_space_less_flag))
		{
			if (*tick_cnt_ptr >= Short_Single_width)
			{
				if ((!pic_down_flag)&&(!video_down_flag))
				{
					if (work_mode_get() == STA_VIDEO)
					{
						//work_mode_set(STA_VIDEO);
						DBG_PRINT("[APQ_VIDEO_RECORD_ACTIVE...]\r\n");
						msgQSend(ApQ, MSG_APQ_VIDEO_RECORD_ACTIVE, NULL, NULL, MSG_PRI_NORMAL);
					}
					else if (work_mode_get() == STA_MOTION)
					{
						//work_mode_set(STA_MOTION);
						DBG_PRINT("[MSG_APQ_MD_SET...]\r\n");
						msgQSend(ApQ, MSG_APQ_MD_SET, NULL, NULL, MSG_PRI_NORMAL);
					}
				}
			}
		}
	} 
	*tick_cnt_ptr = 0;
}


INT16U ap_peripheral_uvc_flag(void)
{
	return pc_camera_flag;
}
void ap_peripheral_uvc_flag_set(INT8U flag)
{
	if (pc_camera_flag != flag)
	pc_camera_flag = flag;
}

void ap_peripheral_usbd_plug_out_exe(INT16U *tick_cnt_ptr)
{
	msgQSend(ApQ, MSG_APQ_DISCONNECT_TO_PC, NULL, NULL, MSG_PRI_NORMAL);
	*tick_cnt_ptr = 0;
}

void ap_peripheral_null_key_exe(INT16U *tick_cnt_ptr)
{
	
}

void ap_peripheral_auto_off_force_disable_set(INT8U auto_off_disable)
{
	auto_off_force_disable = auto_off_disable;
}
