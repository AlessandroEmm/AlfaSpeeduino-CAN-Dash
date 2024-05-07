#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <ESP32CAN.h>
#include <XPT2046_Touchscreen.h>

/* RTOS priorities, higher number is more important */
#define CAN_TX_PRIORITY     1
#define CAN_RX_PRIORITY     4

#define CAN_TX_RATE_ms      1500
#define CAN_RX_RATE_ms      500

void canReceive(void *pvParameters);

static TaskHandle_t canRxTask = NULL;
twai_message_t tx_frame;

#define MYFONT 1
#define ILI9341_2_DRIVER 
#define TFT_WIDTH  240
#define TFT_HEIGHT 320 
#define TFT_BL   21            // LED back-light control pin
#define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)
#define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" and so on.
#define TFT_SCLK 14
#define TFT_CS   15  // Chip select control pin
#define TFT_DC   2  // Data Command control pin
#define TFT_RST  12  // Reset pin (could connect to Arduino RESET pin)
#define TFT_BL   21  // LED back-light

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);



#define SPI_FREQUENCY  55000000 // STM32 SPI1 only (SPI2 maximum is 27MHz)
//#define TOUCH_DRIVER 0x2046
#define SPI_READ_FREQUENCY  20000000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
 #define SPI_TOUCH_FREQUENCY  2500000

static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

uint16_t rpm, tps, lambda, mapData, iat, clt, fuelTrim, fuelPw, lambdaTarget, errors, syncerrors, baro, advance, correction, fuelPressure, oilPressure, oilTemp, voltage; 
uint8_t revLimiter;
unsigned int rpmBefore;

SPIClass mySpi = SPIClass(VSPI);

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif



 static lv_style_t miniCaptionStyle;
 
  static lv_style_t medCaptionStyle;

  static lv_style_t warningText;

static lv_obj_t *loading_screen;
static lv_obj_t * bar;
static lv_obj_t *date_time_label;
static lv_obj_t *battery_label;
static lv_obj_t *battery_caption;
static lv_obj_t *tps_label;
static lv_obj_t *tps_caption;
static lv_obj_t *lambda_label;
static lv_obj_t *lambda_caption;
static lv_obj_t *lambdaTarget_label;
static lv_obj_t *lambdaTarget_caption;
static lv_obj_t *baro_label;
static lv_obj_t *baro_caption;
static lv_obj_t *pw_label;
static lv_obj_t *pw_caption;
static lv_obj_t *iat_caption;
static lv_obj_t *clt_caption;
static lv_obj_t *iat_label;
static lv_obj_t *clt_label;
static lv_obj_t *syncl_label;
static lv_obj_t *advance_label;
static lv_obj_t *advance_caption;
static lv_obj_t *gammaCorr_label;
static lv_obj_t *gammaCorr_caption;
static lv_obj_t *ego_label;
static lv_obj_t *ego_caption;
static lv_obj_t *rpm_caption;
static lv_obj_t *rpm_label;
static lv_obj_t *fp_caption;
static lv_obj_t *fp_label;
static lv_obj_t *oilp_caption;
static lv_obj_t *oilp_label;
static lv_obj_t *oilt_caption;
static lv_obj_t *oilt_label;
static lv_obj_t *syncLoss_caption;
static lv_obj_t *syncLoss_label;
static lv_obj_t *fuelPw_caption;
static lv_obj_t *fuelPw_label;
static lv_meter_indicator_t *indic;
static lv_obj_t *process_list;
static lv_anim_t a;
static lv_obj_t * wueIndicator;
static lv_obj_t * wueIndicatorLabel;
static lv_obj_t * syncIndicator;
static lv_obj_t * syncIndicatorLabel;
static lv_obj_t * bar_caption;

static void set_rpm(void * bars, int32_t temp)
{
    lv_bar_set_value(bar, temp, LV_ANIM_ON);
}
/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}


void canReceive(void *pvParameters) {
	const TickType_t xDelay = CAN_RX_RATE_ms / portTICK_PERIOD_MS;
	twai_message_t rx_frame;

	for (;;) {
    if (ESP32CAN_OK == ESP32Can.CANReadFrame(&rx_frame)) {  /* only print when CAN message is received*/
      switch (rx_frame.identifier) {
      case 0x520:
      {
        rpm = ((rx_frame.data[1] << 8) | (rx_frame.data[0]));
        tps = (rx_frame.data[3] << 8) | (rx_frame.data[2]);
        mapData = (rx_frame.data[5] << 8) | (rx_frame.data[4]);
        lambda = ((rx_frame.data[7] << 8) | (rx_frame.data[6]));
        

        }
      break;
      case 0x521:
      {   
             advance = ((rx_frame.data[5] << 8) | (rx_frame.data[4]));

        }
      break;
      case 0x522:
      {   
             fuelPw = ((rx_frame.data[1] << 8) | (rx_frame.data[0]));
             
        }
      break;
      case 0x524:
      {   
             correction = ((rx_frame.data[3] << 8) | (rx_frame.data[2]));
             
        }
      break;
      case 0x526:
      {   
             revLimiter = ((rx_frame.data[1]));
             
        }
      break;
      case 0x527:
      {   
             lambdaTarget = ((rx_frame.data[7] << 8) | (rx_frame.data[6]));
             
          

        }
      break;
      case 0x530:
      {      voltage = (rx_frame.data[1] << 8) | (rx_frame.data[0]);
             baro = ((rx_frame.data[3] << 8) | (rx_frame.data[2]));
             iat = (rx_frame.data[5] << 8) | (rx_frame.data[4]);
             clt = (rx_frame.data[7] << 8) | (rx_frame.data[6]);
        }
      break;
      case 0x531:
      {      fuelTrim = ((rx_frame.data[1] << 8) | (rx_frame.data[0]));
        }
      break;
      case 0x534:
      {      
             errors = ((rx_frame.data[5] << 8) | (rx_frame.data[4]));
             syncerrors = ((rx_frame.data[7] << 8) | (rx_frame.data[6]));
        

        }
      break;
      case 0x536:
      {      oilPressure = ((rx_frame.data[5] << 8) | (rx_frame.data[4]));
             oilTemp = ((rx_frame.data[7] << 8) | (rx_frame.data[6]));
        }
      break;
      case 0x537:
      {      fuelPressure = ((rx_frame.data[1] << 8) | (rx_frame.data[0]));
        }
      break;
      default:
       //do nothing
      break;
    }
    }
		vTaskDelay(2); /* do something else until it is time to receive again. This is a simple delay task. */

  
	}

}

void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
   uint16_t touchX, touchY;
   bool touched = tft.getTouch( &touchX, &touchY);
   if( !touched )
   {
      data->state = LV_INDEV_STATE_REL;
   }
   else
   {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;

   }
}

// static void my_event_cb(lv_event_t * event)
//   {
//       Serial.println("Clicked\n");
//       detailBox();
//   }
  
// static void event_cb(lv_event_t * e)
// {
//     lv_obj_t * obj = lv_event_get_current_target(e);
//     LV_LOG_USER("Button %s clicked", lv_msgbox_get_active_btn_text(obj));
// }
// void detailBox(void)
// {
//     static const char * btns[] ={"Apply", "Close", ""};

//     lv_obj_t * mbox1 = lv_msgbox_create(NULL, "Hello", "This is a message box with two buttons.", btns, true);
//     lv_obj_add_event_cb(mbox1, event_cb, LV_EVENT_VALUE_CHANGED, NULL);
//     lv_obj_center(mbox1);
// }

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */

  ESP32Can.CANInit(GPIO_NUM_27, GPIO_NUM_22, ESP32CAN_SPEED_500KBPS);

  /* setup can receive RTOS task */
	xTaskCreatePinnedToCore(canReceive,      /* callback function */
                          "CAN RX",        /* name of task */
                          4048,            /* stack size (bytes in ESP32, words in FreeRTOS */
                          NULL,            /* parameter to pass to function */
                          CAN_RX_PRIORITY, /* task priority (0 to configMAX_PRIORITES - 1 */
                          &canRxTask,      /* task handle */
                          0);              /* CPU core, Arduino runs on 1 */



   
    lv_init();

    lv_log_register_print_cb( my_print ); /* register print function for debugging */

    tft.begin();          /* TFT init */
    tft.setRotation(1);//( 3 ); /* Landscape orientation, flipped */
    mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);

    ts.begin(mySpi);
    ts.setRotation(1);
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush; 
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );


   /*Initialize the (dummy) input device driver*/
   static lv_indev_drv_t indev_drv;
   lv_indev_drv_init(&indev_drv);
   indev_drv.type = LV_INDEV_TYPE_POINTER;
   indev_drv.read_cb = my_touchpad_read;
   lv_indev_drv_register(&indev_drv);

    static lv_style_t stylebar;
    lv_style_init(&stylebar);
    lv_style_set_bg_opa(&stylebar, LV_OPA_COVER);
    lv_style_set_bg_color(&stylebar, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&stylebar, lv_palette_main(LV_PALETTE_AMBER));
    lv_style_set_bg_grad_dir(&stylebar, LV_GRAD_DIR_HOR);
    lv_style_set_bg_main_stop(&stylebar, 195);

    bar = lv_bar_create(lv_scr_act());
    lv_obj_add_style(bar, &stylebar, LV_PART_INDICATOR);
    lv_obj_set_size(bar, 227, 25);
    lv_obj_center(bar);
    lv_bar_set_range(bar, 0, 7000);



  lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 5, 10);






 lv_style_init(&miniCaptionStyle);
 lv_style_set_text_font(&miniCaptionStyle, &lv_font_montserrat_12); 
 lv_style_set_text_color(&miniCaptionStyle,lv_palette_main(LV_PALETTE_GREY));


 lv_style_init(&medCaptionStyle);
 lv_style_set_text_font(&medCaptionStyle, &lv_font_montserrat_16); 
 lv_style_set_text_color(&medCaptionStyle,lv_palette_main(LV_PALETTE_GREY));

 lv_style_init(&warningText);
 lv_style_set_text_font(&miniCaptionStyle, &lv_font_montserrat_12); 
 lv_style_set_text_color(&warningText,lv_palette_main(LV_PALETTE_GREY));
    
  bar_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(bar_caption,  "0     10      20     30     40      50     60     70");
  lv_obj_set_style_text_align(bar_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(bar_caption, bar, LV_ALIGN_OUT_BOTTOM_LEFT, 4, 3);
  lv_obj_add_style(bar_caption,&miniCaptionStyle,0);


  //lv_obj_add_event_cb(bar, my_event_cb, LV_EVENT_CLICKED, NULL);   /*Assign an event callback*/





  static lv_style_t rpmlabel;

  lv_style_init(&rpmlabel);

 lv_style_set_text_font(&rpmlabel, &lv_font_montserrat_26); 
  /*Set a larger font*/
  rpm_label = lv_label_create(lv_scr_act());
  lv_label_set_text(rpm_label, "0");
  lv_obj_align(rpm_label, LV_ALIGN_TOP_RIGHT, -15, 5);

  lv_obj_add_style(rpm_label,&rpmlabel,0);
  rpm_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(rpm_caption,  "rpm");
  lv_obj_set_style_text_align(rpm_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(rpm_caption, rpm_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 0);
  lv_obj_add_style(rpm_caption,&medCaptionStyle,0);



    static lv_point_t line_points[] = { {5, 5}, {295, 5}};

    /*Create style*/
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 2);
    lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_line_rounded(&style_line, true);
    /*Create a line and apply the new style*/
    lv_obj_t * line1;
    line1 = lv_line_create(lv_scr_act());
    lv_line_set_points(line1, line_points, 2);     /*Set the points*/
    lv_obj_add_style(line1, &style_line, 0);
   lv_obj_align(line1, LV_ALIGN_TOP_LEFT, 10, 64);

    lambda_label = lv_label_create(lv_scr_act());
  lv_label_set_text(lambda_label,  "0");
  lv_obj_set_style_text_align(lambda_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(lambda_label, LV_ALIGN_TOP_LEFT, 15, 90);
  lambda_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(lambda_caption,  "Lambda");
  lv_obj_set_style_text_align(lambda_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(lambda_caption, lambda_label, LV_ALIGN_OUT_BOTTOM_MID, 23, 3);
  lv_obj_add_style(lambda_caption,&miniCaptionStyle,0);

  lambdaTarget_label = lv_label_create(lv_scr_act());
  lv_label_set_text(lambdaTarget_label,  "0");
  lv_obj_set_style_text_align(lambdaTarget_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(lambdaTarget_label, LV_ALIGN_TOP_LEFT, 205, 190);
  lambdaTarget_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(lambdaTarget_caption,  "Target");
  lv_obj_set_style_text_align(lambdaTarget_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(lambdaTarget_caption, lambdaTarget_label, LV_ALIGN_OUT_BOTTOM_MID, 15, 3);
  lv_obj_add_style(lambdaTarget_caption,&miniCaptionStyle,0);


      tps_label = lv_label_create(lv_scr_act());
  lv_label_set_text(tps_label,  "-");
  lv_obj_set_style_text_align(tps_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(tps_label, LV_ALIGN_TOP_LEFT, 270, 190);
  tps_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(tps_caption,  "TPS %");
  lv_obj_set_style_text_align(tps_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(tps_caption, tps_label, LV_ALIGN_OUT_BOTTOM_MID, 19, 3);
  lv_obj_add_style(tps_caption,&miniCaptionStyle,0);


  battery_label = lv_label_create(lv_scr_act());
  lv_label_set_text(battery_label,  "-");
  lv_obj_set_style_text_align(battery_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 15, 190);
  battery_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(battery_caption,  "Volt");
  lv_obj_set_style_text_align(battery_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(battery_caption, battery_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(battery_caption,&miniCaptionStyle,0);


      iat_label = lv_label_create(lv_scr_act());
  lv_label_set_text(iat_label,  "-");
  lv_obj_set_style_text_align(iat_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(iat_label, LV_ALIGN_TOP_LEFT, 70, 90);
  iat_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(iat_caption,  "IAT");
  lv_obj_set_style_text_align(iat_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(iat_caption, iat_label, LV_ALIGN_OUT_BOTTOM_MID, 9, 3);
  lv_obj_add_style(iat_caption,&miniCaptionStyle,0);

  clt_label = lv_label_create(lv_scr_act());
  lv_label_set_text(clt_label,  "-");
  lv_obj_set_style_text_align(clt_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(clt_label, LV_ALIGN_TOP_LEFT, 70, 140);
  clt_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(clt_caption,  "CLT");
  lv_obj_set_style_text_align(clt_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(clt_caption, clt_label, LV_ALIGN_OUT_BOTTOM_MID, 9, 3);
  lv_obj_add_style(clt_caption,&miniCaptionStyle,0);



  fp_label = lv_label_create(lv_scr_act());
  lv_label_set_text(fp_label,  "-");
  lv_obj_set_style_text_align(fp_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(fp_label, LV_ALIGN_TOP_LEFT, 140, 90);
  fp_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(fp_caption,  "Fuel kPa");
  lv_obj_set_style_text_align(fp_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(fp_caption, fp_label, LV_ALIGN_OUT_BOTTOM_MID, 17, 3);
  lv_obj_add_style(fp_caption,&miniCaptionStyle,0);

  oilp_label = lv_label_create(lv_scr_act());
  lv_label_set_text(oilp_label,  "-");
  lv_obj_set_style_text_align(oilp_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(oilp_label, LV_ALIGN_TOP_LEFT, 140, 140);
  oilp_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(oilp_caption,  "Oil kPa");
  lv_obj_set_style_text_align(oilp_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(oilp_caption, oilp_label, LV_ALIGN_OUT_BOTTOM_MID, 15, 3);
  lv_obj_add_style(oilp_caption,&miniCaptionStyle,0);

  baro_label = lv_label_create(lv_scr_act());
  lv_label_set_text(baro_label,  "-");
  lv_obj_set_style_text_align(baro_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(baro_label, LV_ALIGN_TOP_LEFT, 140, 190);
  baro_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(baro_caption,  "baro kPa");
  lv_obj_set_style_text_align(baro_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(baro_caption, baro_label, LV_ALIGN_OUT_BOTTOM_MID, 20, 3);
  lv_obj_add_style(baro_caption,&miniCaptionStyle,0);


  oilt_label = lv_label_create(lv_scr_act());
  lv_label_set_text(oilt_label,  "-");
  lv_obj_set_style_text_align(oilt_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(oilt_label, LV_ALIGN_TOP_LEFT, 70, 190);
  oilt_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(oilt_caption,  "Oil");
  lv_obj_set_style_text_align(oilt_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(oilt_caption, oilt_label, LV_ALIGN_OUT_BOTTOM_MID, 9, 3);
  lv_obj_add_style(oilt_caption,&miniCaptionStyle,0);

  advance_label = lv_label_create(lv_scr_act());
  lv_label_set_text(advance_label,  "-");
  lv_obj_set_style_text_align(advance_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(advance_label, LV_ALIGN_TOP_LEFT, 205, 90);
  advance_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(advance_caption,  "Adv");
  lv_obj_set_style_text_align(advance_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(advance_caption, advance_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(advance_caption,&miniCaptionStyle,0);

  gammaCorr_label = lv_label_create(lv_scr_act());
  lv_label_set_text(gammaCorr_label,  "-");
  lv_obj_set_style_text_align(gammaCorr_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(gammaCorr_label, LV_ALIGN_TOP_LEFT, 205, 140);
  gammaCorr_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(gammaCorr_caption,  "Trim");
  lv_obj_set_style_text_align(gammaCorr_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(gammaCorr_caption, gammaCorr_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(gammaCorr_caption,&miniCaptionStyle,0);
  ego_label = lv_label_create(lv_scr_act());
  lv_label_set_text(ego_label,  "-");
  lv_obj_set_style_text_align(ego_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(ego_label, LV_ALIGN_TOP_LEFT, 15, 140);
  ego_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(ego_caption,  "Corr");
  lv_obj_set_style_text_align(ego_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(ego_caption, ego_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(ego_caption,&miniCaptionStyle,0);

  syncLoss_label = lv_label_create(lv_scr_act());
  lv_label_set_text(syncLoss_label,  "0");
  lv_obj_set_style_text_align(syncLoss_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(syncLoss_label, LV_ALIGN_TOP_LEFT, 270, 90);
  syncLoss_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(syncLoss_caption,  "Loss");
  lv_obj_set_style_text_align(syncLoss_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(syncLoss_caption, syncLoss_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(syncLoss_caption,&miniCaptionStyle,0);

  fuelPw_label = lv_label_create(lv_scr_act());
  lv_label_set_text(fuelPw_label,  "0");
  lv_obj_set_style_text_align(fuelPw_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(fuelPw_label, LV_ALIGN_TOP_LEFT, 270, 140);
  fuelPw_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(fuelPw_caption,  "PW ms");
  lv_obj_set_style_text_align(fuelPw_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(fuelPw_caption, fuelPw_label, LV_ALIGN_OUT_BOTTOM_MID, 15, 3);
  lv_obj_add_style(fuelPw_caption,&miniCaptionStyle,0);


    static lv_style_t indicatorOff;
    lv_style_init(&indicatorOff);
     lv_style_set_text_font(&indicatorOff, &lv_font_montserrat_12); 
     lv_style_set_text_color(&indicatorOff,lv_palette_main(LV_PALETTE_BLUE_GREY));
     lv_style_set_bg_color(&indicatorOff, lv_palette_darken(LV_PALETTE_GREY, 2));

  

    syncIndicator = lv_btn_create(lv_scr_act());
    lv_obj_align(syncIndicator, LV_ALIGN_TOP_LEFT, 300, 307);
    syncIndicatorLabel = lv_label_create(syncIndicatorLabel);
    lv_label_set_text(syncIndicatorLabel, "wwww");
    lv_obj_set_size(syncIndicator, 55, 25);
    lv_obj_center(syncIndicatorLabel);
    lv_obj_add_style(syncIndicatorLabel,&indicatorOff,0);
    lv_obj_add_style(syncIndicator,&indicatorOff,0);

lv_obj_clear_flag( lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE );

   xTaskCreatePinnedToCore(lvglTimerHandler,
                           "lvglTimerHandler",
                           4096,
                           NULL,
                           2,
                           NULL,
                           1);

    Serial.println( "Setup done" );

    
}


void lvglTimerHandler(void *pvParameters) {

  while (1) {
    update_rpm(rpm);
    update_clt(clt);
    update_iat(iat);
    update_tps(tps);
    update_lambda(lambda, correction, fuelTrim);
    update_target(lambdaTarget);
    update_baro(baro);
    update_pw(fuelPw);
    update_adv(advance);
    update_pressures(fuelPressure, oilPressure, oilTemp);
    update_batt(voltage);
    update_syncLosses(syncerrors);
    lv_timer_handler();
    vTaskDelay(3);
  }
}


String formatDecimals(int number, int decimals){
  String dnumber= String(number);
  String formattedNumber;
     if (dnumber.length() == 4 && decimals == 3)
   {
     formattedNumber = dnumber.substring(0,1) + "." + dnumber.substring(1);

   }
   if (dnumber.length() == 4 && decimals == 2)
   {
    formattedNumber = dnumber.substring(0,2) + "." + dnumber.substring(2);
   }
   else if (dnumber.length() == 3 && decimals == 2)
   {
    formattedNumber = dnumber.substring(0,1) + "." + dnumber.substring(1);
   }
   else if (dnumber.length() == 4 && decimals == 1)
   {
    formattedNumber = dnumber.substring(0,3) + "." + dnumber.substring(3);
   }
   else if (dnumber.length() == 3 && decimals == 1)
   { 
     formattedNumber = dnumber.substring(0,2) + "." + dnumber.substring(2);
   }
   else if (dnumber.length() == 2)
   {
    formattedNumber = dnumber.substring(0,1) + "." + dnumber.substring(1);
   }
   else if (dnumber.length() == 1)
   {
     return dnumber;
   }
   else 
   {
     return dnumber;
   }
   return formattedNumber;
}

void loop()
{
}



void update_tps(int tps_percent) {
   String tps_text = formatDecimals(tps_percent,1)  ;
  lv_label_set_text(tps_label,  tps_text.c_str());
}

void update_rpm(int rpm) {
  String rpm_text = String("") + rpm ;
  lv_label_set_text(rpm_label,  rpm_text.c_str());
  set_rpm(bar, rpm);
}

void update_clt(int clt) {
   String clt_text = formatDecimals(clt, 1 ) + "°";
  lv_label_set_text(clt_label, clt_text.c_str());
}

void update_iat(int iat) {
   String iat_text = formatDecimals(iat, 1) + "°";
  lv_label_set_text(iat_label, iat_text.c_str());
}

void update_adv(int adv) {
   String adv_text = formatDecimals(adv, 1) + "°";
  lv_label_set_text(advance_label, adv_text.c_str());
}

void update_batt(int batt) {
   String batt_text = formatDecimals(batt, 2);
  lv_label_set_text(battery_label, batt_text.c_str());
}

void update_syncLosses(int syncLosses) {
   String syncLosses_text = String("") + syncLosses;
  lv_label_set_text(syncIndicatorLabel, syncLosses_text.c_str());
}

void update_baro(int baro) {
   String baro_text = formatDecimals(baro, 1);
  lv_label_set_text(baro_label, baro_text.c_str());
}

void update_pw(int pw) {
   String pw_text = formatDecimals(pw, 3);
  lv_label_set_text(fuelPw_label, pw_text.c_str());
}

void update_target(int target) {
   String target_text = formatDecimals(target, 3);
  lv_label_set_text(lambdaTarget_label, target_text.c_str());
}

void update_pressures(int fp, int oilp, int oilt) {
  String fp_text = formatDecimals(fp, 1);
  lv_label_set_text(fp_label, fp_text.c_str());
  String oilp_text = formatDecimals(oilp, 1);
  lv_label_set_text(oilp_label, oilp_text.c_str());
  String oilt_text = formatDecimals(oilt, 1) + "°";
  lv_label_set_text(oilt_label, oilt_text.c_str());
}

void update_lambda(int lambda, int correction, int fuelTrim) {
  String lambda_text = formatDecimals(lambda,3);
  lv_label_set_text(lambda_label, lambda_text.c_str());
  String gamma_text = formatDecimals(fuelTrim, 1);
  lv_label_set_text(gammaCorr_label, gamma_text.c_str());
  String ego_text = formatDecimals(correction,1);
  lv_label_set_text(ego_label, ego_text.c_str());
}
