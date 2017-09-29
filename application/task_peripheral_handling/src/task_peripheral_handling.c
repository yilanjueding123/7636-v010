#include "task_peripheral_handling.h"
#include "ap_state_config.h"
#include "ap_browse.h"
#include "ap_display.h"

//+++
#include "LDWs.h"
#if (Enable_Lane_Departure_Warning_System)

extern LDWsParameter LDWPar;
extern INT32U LDWS_buf_addrs;

INT32U LDWS_Play_Voice_Start_Time = 0;
INT8U  LDWS_Show_Dbg_Msg = 0;
INT8U  LDWS_Key_On_flag = 0;
#endif
//---


MSG_Q_ID PeripheralTaskQ;
void *peripheral_task_q_stack[PERIPHERAL_TASK_QUEUE_MAX];
static INT8U peripheral_para[PERIPHERAL_TASK_QUEUE_MAX_MSG_LEN];
INT8U screen_saver_enable = 0;
INT32U battery_charge_icon_blink_cnt = 0;
INT32U battery_low_blink_cnt = 0;
INT32U display_insert_sdc_cnt = 0;
INT32U motion_detect_peripheral_cnt = 0;
INT32U G_sensor_power_on_time = 0;
INT32U sensor_error_power_off_timer = 0;
INT8U  usb_charge_cnt=0;
INT32U idle_count = 0;

#if TV_DET_ENABLE
extern INT8U tv_debounce_cnt;
#endif

extern INT8U PreviewIsStopped;
extern volatile INT8U cdsp_hangup_flag;
extern INT8U adp_status;
extern INT32U gp_ae_awb_process(void);
extern void gp_isp_iso_set(INT32U iso);
extern void set_led_mode(LED_MODE_ENUM mode);
static INT8U watch_dog_cnt=0;
extern INT8U  gc_ad_init_done;
//extern INT8S video_record_sts;
extern INT8U ap_video_record_sts_get(void);
#if AUD_RECORD_EN
extern INT8U audio_record_sts;
extern INT8U ap_audio_record_sts_get(void);
#endif

static INT8U  KeyScan_flag = 0;
INT8U KeyScan_flag_get(void)
{
	return KeyScan_flag;
}

void task_peripheral_handling_init(void)
{
	PeripheralTaskQ = msgQCreate(PERIPHERAL_TASK_QUEUE_MAX, PERIPHERAL_TASK_QUEUE_MAX, PERIPHERAL_TASK_QUEUE_MAX_MSG_LEN);
	ap_peripheral_init();
}
INT8U card_space_less_flag=0;
extern void power_pin_set(INT8U onoff);
extern INT8U ap_peripheral_handling_usb_state_get(void);
extern INT8U read_pw_key0(void);
extern INT8S video_record_sts;
extern INT8U usb_state_get(void);
void task_peripheral_handling_entry(void *para)
{
	INT32U msg_id, power_on_auto_rec_delay_cnt;
	INT8U usb_detect_start; //tick = 0 ,tv_polling_start = 0;
	INT8U aeawb_start;
	INT8S ret = 0;
	INT8U volume_show_time;
	INT32U KeyScan_delay = 128*2; //��ʱ����ɨ�谴��
	INT32U dec_power_key = 0;
	INT32U type;

	aeawb_start = 1;
	volume_show_time = 0;
	usb_detect_start = 0;
	while(1) {
		if(msgQReceive(PeripheralTaskQ, &msg_id, peripheral_para, PERIPHERAL_TASK_QUEUE_MAX_MSG_LEN) == STATUS_FAIL) {
			continue;
		}
		if(++watch_dog_cnt >= 128)
			{
			 watch_dog_cnt=0;
			 //DBG_PRINT("-----CLEAR WATCH_DOG-----\r\n");
		     watchdog_clear();
			}
	//	DBG_PRINT("Handling_message_receive\r\n");
        switch (msg_id) {
        	case MSG_PERIPHERAL_TASK_KEY_REGISTER:
        		ap_peripheral_key_register(peripheral_para[0]);
        		break;

        	case MSG_PERIPHERAL_TASK_USBD_DETECT_INIT:
        		usb_detect_start = 1;
				power_on_auto_rec_delay_cnt = 2*128/PERI_TIME_INTERVAL_AD_DETECT;	//2s				
        		break;

        	case MSG_PERIPHERAL_TASK_LED_SET:
        		//ap_peripheral_led_set(peripheral_para[0]);
        		set_led_mode(peripheral_para[0]);
				if(peripheral_para[0] ==LED_CARD_NO_SPACE)
					card_space_less_flag =1;
        		break;
        	case MSG_PERIPHERAL_TASK_LED_FLASH_SET:
				//ap_peripheral_led_flash_set();
        		break;
			case MSG_PERIPHERAL_TASK_LED_BLINK_SET:
				//ap_peripheral_led_blink_set();
				break;
			
			case MSG_PERIPHERAL_TASK_BATTERY_CHARGE_ICON_BLINK_START:
				battery_charge_icon_blink_cnt = 0x002f;
				break;
			case MSG_PERIPHERAL_TASK_BATTERY_CHARGE_ICON_BLINK_STOP:
				battery_charge_icon_blink_cnt = 0;
				break;
				
			case MSG_PERIPHERAL_TASK_BATTERY_LOW_BLINK_START:
				battery_low_blink_cnt = 0x001f;
				battery_low_blink_cnt |= 0x0a00;
				break;
			case MSG_PERIPHERAL_TASK_BATTERY_LOW_BLINK_STOP:
				if(battery_low_blink_cnt){
					battery_low_blink_cnt=0;
				}
				break;
			case MSG_PERIPHERAL_TASK_DISPLAY_PLEASE_INSERT_SDC:
				display_insert_sdc_cnt = 0x006f;
				break;
			case MSG_PERIPHERAL_TASK_TIMER_VOLUME_ICON_SHOW:
				volume_show_time = peripheral_para[0]*20;
				break;
			
			case MSG_PERIPHERAL_TASK_G_SENSOR_POWER_ON_START:
				G_sensor_power_on_time = 0x0300;
				break;
			case MSG_PERIPHERAL_TASK_SINGLE_JUGE:
				if (KeyScan_delay != 0)
				{//��ʱɨ�谴��
					KeyScan_delay--;
				}
				else
				{//��⵽��Դ���ɿ��Ϳ���ɨ�谴��
					dec_power_key = (!read_pw_key0());
					if (dec_power_key) KeyScan_flag = 1;
				}
				break;
#if USE_ADKEY_NO
        	case MSG_PERIPHERAL_TASK_AD_DETECT_CHECK:
				// ��������������ж�Ҫ�ȶ�
        		if(KeyScan_flag) ap_peripheral_key_judge();
				//-----------------------------------------------------------------------
				//if((s_usbd_pin)||(ap_peripheral_usb_poweron_get() != 0))
				if((s_usbd_pin)||usb_state_get())
				{
					if(++usb_charge_cnt >31)
				 	{
			          	ap_peripheral_charge_det();
					 	usb_charge_cnt=0;
				 	}
				}
				//-----------------------------------------------------------------------
				#if 1
        		ap_peripheral_ad_key_judge();
        		#endif
	
        		if(gc_ad_init_done==0)
        		{
        			gc_ad_init_done = 1;
        		}
        		#if GPDV_BOARD_VERSION != GPCV1237A_Aerial_Photo
        		ap_peripheral_config_store();
        		#endif

        		if(usb_detect_start == 1) {
    				ap_peripheral_adaptor_out_judge();
        			if(power_on_auto_rec_delay_cnt) {
        				power_on_auto_rec_delay_cnt -= 1;
        				if(!power_on_auto_rec_delay_cnt) {
  							//if ((usb_state_get() == 2)&&(ap_state_handling_storage_id_get() != NO_STORAGE))  					
  							if (usb_state_get() == 2)  					
							{//������⵽USB��ţ��ִ��
								//msgQSend(ApQ, MSG_APQ_POWER_ON_AUTO_RECORD, NULL, NULL, MSG_PRI_NORMAL); //�����Զ�¼��
								//KeyScan_flag = 1;
								
								msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_TIMER_STOP, NULL, NULL, MSG_PRI_NORMAL);
								type = FALSE;
								msgQSend(StorageServiceQ, MSG_STORAGE_SERVICE_FREESIZE_CHECK_SWITCH, &type, sizeof(INT8U), MSG_PRI_NORMAL);
								type = USBD_DETECT;
								msgQSend(PeripheralTaskQ, MSG_PERIPHERAL_TASK_KEY_REGISTER, &type, sizeof(INT8U), MSG_PRI_NORMAL);	
        					}
        				}
        			}

					if(cdsp_hangup_flag) { //10s
						PreviewIsStopped = 1;
						DBG_PRINT("too many cdsp overflow!!! Auto Power Off...\r\n");
						type = 0;
						msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_URGENT);
					} else if(PreviewIsStopped == 0) {
						sensor_error_power_off_timer++;
						if(sensor_error_power_off_timer == 0) {
							sensor_error_power_off_timer--;
						}
						if(sensor_error_power_off_timer > 10*128/PERI_TIME_INTERVAL_AD_DETECT) { //10s auto power off
							PreviewIsStopped = 1;
							DBG_PRINT("no scaler interrup occurs!!! Auto Power Off...\r\n");
							type = 0;
							msgQSend(ApQ, MSG_APQ_POWER_KEY_ACTIVE, &type, sizeof(INT32U), MSG_PRI_URGENT);
						}
					}
        		}
			
			  #if C_MOTION_DETECTION == CUSTOM_ON
        		if(motion_detect_peripheral_cnt){
        			motion_detect_peripheral_cnt--;
        			if(!motion_detect_peripheral_cnt){
        				ap_peripheral_motion_detect_judge();
        				motion_detect_peripheral_cnt= 2;
        			}
        		}
			  #endif
        		#if GPDV_BOARD_VERSION != GPCV1237A_Aerial_Photo
				if(volume_show_time){
					volume_show_time -- ;
					if(!volume_show_time){
	 					msgQSend(ApQ, MSG_APQ_VOLUME_SHOW_END, NULL, NULL, MSG_PRI_NORMAL);
	 				}
 				}
 				#endif
				if(aeawb_start) {			
					ret = gp_ae_awb_process();
					if(ret != 0) {
						//DBG_PRINT(".");
					}
					else {
						//DBG_PRINT("\r\nae, awb process done!! \r\n");
						//DBG_PRINT("*");
					}
				}	
        		
				break;
#endif

#if GPDV_BOARD_VERSION != GPCV1237A_Aerial_Photo
            case MSG_PERIPHERAL_TV_POLLING_START:
                tv_polling_start = 1;
			#if TV_DET_ENABLE
               	tv_debounce_cnt = 0;
            #endif
                break;

            case MSG_PERIPHERAL_TV_POLLING_STOP:
                tv_polling_start = 0;
			#if TV_DET_ENABLE
               	tv_debounce_cnt = 0;
            #endif
                break;

			case MSG_PERIPHERAL_TASK_TV_DETECT:
				ap_peripheral_tv_detect();
				break;

			case MSG_PERIPHERAL_TASK_READ_GSENSOR:
				ap_peripheral_read_gsensor();
				break;
#endif

#if C_MOTION_DETECTION == CUSTOM_ON
//        	case MSG_PERIPHERAL_TASK_MOTION_DETECT_JUDGE:
//        		ap_peripheral_motion_detect_judge();
//        		break;
        	case MSG_PERIPHERAL_TASK_MOTION_DETECT_START:
        		ap_peripheral_motion_detect_start();
        		motion_detect_peripheral_cnt= 32;
        		break;
        	case MSG_PERIPHERAL_TASK_MOTION_DETECT_STOP:
        		ap_peripheral_motion_detect_stop();
        		motion_detect_peripheral_cnt= 0;
        		break;
        	case MSG_PERIPHERAL_TASK_MOTION_DETECT_DELAY:
        		motion_detect_peripheral_cnt= 0xff;
        		break;
#endif
#if C_SCREEN_SAVER == CUSTOM_ON
			case MSG_PERIPHERAL_TASK_LCD_BACKLIGHT_SET:
			ap_peripheral_clr_screen_saver_timer();
			ap_peripheral_lcd_backlight_set(peripheral_para[0]);
        		break;
			case MSG_PERIPHERAL_TASK_SCREEN_SAVER_ENABLE:
        		screen_saver_enable = 1;
        		break;
#endif
			case MSG_PERIPHERAL_TASK_NIGHT_MODE_SET:
        		ap_peripheral_night_mode_set(peripheral_para[0]);
				break;

        	case MSG_PERIPHERAL_USBD_EXIT:
        		usbd_exit = 1;
        		break;

		case MSG_PERIPHERAL_TASK_ISP_ISO_SET:
			gp_isp_iso_set(peripheral_para[0]);
			break;

		#if (Enable_Lane_Departure_Warning_System)
		case MSG_LDWS_DO:
		
//		  	gpio_write_io(IO_B15, 1);	

			LDWSmoothSobel_720P_GP420( (unsigned char*)LDWS_buf_addrs, LDWPar.sobelImg_ptr, LDWPar.workingImg_W, LDWPar.workingImg_H, &LDWPar); //get input image buffer
			houghTransform( LDWPar.sobelImg_ptr, LDWPar.workingImg_W, LDWPar.workingImg_H, LDWPar.countTable_ptr, &LDWPar);
			LDWs( LDWPar.countTable_ptr, LDWPar.ROI, &LDWPar);
			LDWTurnKey( &LDWPar, LDWPar.sobelImg_ptr, LDWPar.workingImg_W, LDWPar.workingImg_H,  LDWPar.ROI );				
			dynamicROI(&LDWPar);
			
//		  	gpio_write_io(IO_B15, 0);

		  	LDWS_Show_Dbg_Msg++;
		  	if(LDWS_Show_Dbg_Msg >= 5)
		  	{
	
				LDWS_Show_Dbg_Msg = 0;
			}


			if(LDWPar.LDW_keyFlg && !LDWS_Key_On_flag)
			{

				ap_state_handling_icon_show_cmd(ICON_VIDEO_LDW_SART, NULL, NULL);

				LDWS_Key_On_flag = 1;
				
				if((OSTimeGet()- LDWS_Play_Voice_Start_Time) >= 50)
				{
					if(ap_LDW_get_from_config(LDW_ONOFF_SOUND))
						audio_effect_play(EFFECT_LDW_TurnOn);
					LDWS_Play_Voice_Start_Time = OSTimeGet();
				}
			}
			else if(!LDWPar.LDW_keyFlg && LDWS_Key_On_flag)
			{

				ap_state_handling_icon_clear_cmd(ICON_VIDEO_LDW_SART, NULL, NULL);

				LDWS_Key_On_flag = 0;

				if((OSTimeGet()- LDWS_Play_Voice_Start_Time) >= 50)
				{
					if(ap_LDW_get_from_config(LDW_ONOFF_SOUND))
						audio_effect_play(EFFECT_LDW_TurnOff);
					LDWS_Play_Voice_Start_Time = OSTimeGet();
				}
			}

			if(LDWPar.LDW_keyFlg && LDWPar.LDWsAlarmFlg)
			{

				if((OSTimeGet()- LDWS_Play_Voice_Start_Time) >= 50)
				{
					audio_effect_play(EFFECT_LDW_Alarm);
					LDWS_Play_Voice_Start_Time = OSTimeGet();
				}
			}

			#endif
		break;
		
        	default:
        		break;
        }
    }
}
