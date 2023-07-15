#include <lvgl.h>
#include <TFT_eSPI.h>
#include <HardwareSerial.h>
#include <CAN.h>
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

#define TOUCH_CS 33     // Chip select pin (T_CS) of touch screen
#define SPI_FREQUENCY  55000000 // STM32 SPI1 only (SPI2 maximum is 27MHz)

#define SPI_READ_FREQUENCY  20000000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
 #define SPI_TOUCH_FREQUENCY  2500000

static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

HardwareSerial speeduino(2);
uint8_t speedyResponse[119]; //The data buffer for the Serial data. This is longer than needed, just in case
boolean sent = false;
boolean received = false;
uint32_t sendTimestamp;
unsigned int rpm; 
unsigned int rpmBefore;
float rps;
int prev_rpm = 0;


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif


static lv_obj_t *loading_screen;
static lv_obj_t * bar;
static lv_obj_t *date_time_label;
static lv_obj_t *battery_label;
static lv_obj_t *battery_caption;
static lv_obj_t *tps_label;
static lv_obj_t *tps_caption;
static lv_obj_t *afr_label;
static lv_obj_t *afr_caption;
static lv_obj_t *baro_label;
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
static lv_obj_t *syncLoss_caption;
static lv_obj_t *syncLoss_label;
static lv_meter_indicator_t *indic;
static lv_obj_t *process_list;
static lv_anim_t a;
static lv_obj_t * accelIndicator;
static lv_obj_t * accelIndicatorLabel;
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



void canReceiver() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    if (CAN.packetExtended()) {
      Serial.print ("extended ");
      Serial.print (CAN.packetId());
    }

  
    int message[packetSize] = {0};

        for (int i=0 ;i < packetSize; i++) {
           message[i] = CAN.read();
        }

    switch (CAN.packetId()) {
      case 790:
      {
        int rpm = ((message[3] << 8) | (message[2]));
        update_rpm(rpm);
        update_batt(message[0]/10);
        update_o2_fueltrim(message[4],message[5],message[6]);
        update_tps(message[1]);
        Serial.print(rpm);Serial.println(" niet");

        }
      break;
      case 809:
      {
              update_clt(message[0]);
              update_iat(message[1]);
          

        }
      break;
      case 804:
      {                   // fuel pressure, oil pressure
          update_pressures(message[1],message[0]);
      }
      break;
      default:
       //do nothing
      break;
    }

}

}


void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    CAN.setPins(21, 22);    
   // CAN.filter(809);
    //CAN.filter(790);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);  
  }
   // register the receive callback


    lv_init();

    lv_log_register_print_cb( my_print ); /* register print function for debugging */

    tft.begin();          /* TFT init */
    tft.setRotation(1);//( 3 ); /* Landscape orientation, flipped */


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



    static lv_style_t stylebar;

    lv_style_init(&stylebar);
    lv_style_set_bg_opa(&stylebar, LV_OPA_COVER);
    lv_style_set_bg_color(&stylebar, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&stylebar, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_dir(&stylebar, LV_GRAD_DIR_HOR);
    lv_style_set_bg_main_stop(&stylebar, 195);

    bar = lv_bar_create(lv_scr_act());
    lv_obj_add_style(bar, &stylebar, LV_PART_INDICATOR);
    lv_obj_set_size(bar, 227, 25);
    lv_obj_center(bar);
    lv_bar_set_range(bar, 0, 7000);



  lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 5, 10);






 static lv_style_t miniCaptionStyle;
 lv_style_init(&miniCaptionStyle);
 lv_style_set_text_font(&miniCaptionStyle, &lv_font_montserrat_12); 
 lv_style_set_text_color(&miniCaptionStyle,lv_palette_main(LV_PALETTE_GREY));
  static lv_style_t medCaptionStyle;
 lv_style_init(&medCaptionStyle);
 lv_style_set_text_font(&medCaptionStyle, &lv_font_montserrat_16); 
 lv_style_set_text_color(&medCaptionStyle,lv_palette_main(LV_PALETTE_GREY));
    
  bar_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(bar_caption,  "0     10      20     30     40      50     60     70");
  lv_obj_set_style_text_align(bar_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(bar_caption, bar, LV_ALIGN_OUT_BOTTOM_LEFT, 4, 3);
  lv_obj_add_style(bar_caption,&miniCaptionStyle,0);

  static lv_style_t rpmlabel;

  lv_style_init(&rpmlabel);

 lv_style_set_text_font(&rpmlabel, &lv_font_montserrat_26); 
  /*Set a larger font*/
  rpm_label = lv_label_create(lv_scr_act());
  lv_label_set_text(rpm_label, "1000");
     lv_obj_align(rpm_label, LV_ALIGN_TOP_RIGHT, -15, 5);

  lv_obj_add_style(rpm_label,&rpmlabel,0);
  rpm_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(rpm_caption,  "rpm");
  lv_obj_set_style_text_align(rpm_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(rpm_caption, rpm_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 0);
  lv_obj_add_style(rpm_caption,&medCaptionStyle,0);



    static lv_point_t line_points[] = { {5, 5}, {260, 5}};

    /*Create style*/
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 1);
    lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_BLUE_GREY));
    lv_style_set_line_rounded(&style_line, true);
    /*Create a line and apply the new style*/
    lv_obj_t * line1;
    line1 = lv_line_create(lv_scr_act());
    lv_line_set_points(line1, line_points, 5);     /*Set the points*/
    lv_obj_add_style(line1, &style_line, 0);
   // lv_obj_align_to(line1, rpm_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);
   lv_obj_align(line1, LV_ALIGN_TOP_LEFT, 10, 64);

    afr_label = lv_label_create(lv_scr_act());
  lv_label_set_text(afr_label,  "14.7");
  lv_obj_set_style_text_align(afr_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(afr_label, LV_ALIGN_TOP_LEFT, 15, 90);
  afr_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(afr_caption,  "AFR");
  lv_obj_set_style_text_align(afr_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(afr_caption, afr_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 3);
  lv_obj_add_style(afr_caption,&miniCaptionStyle,0);


      tps_label = lv_label_create(lv_scr_act());
  lv_label_set_text(tps_label,  "0%");
  lv_obj_set_style_text_align(tps_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(tps_label, LV_ALIGN_TOP_LEFT, 15, 140);
  tps_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(tps_caption,  "TPS");
  lv_obj_set_style_text_align(tps_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(tps_caption, tps_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 3);
  lv_obj_add_style(tps_caption,&miniCaptionStyle,0);

  battery_label = lv_label_create(lv_scr_act());
  lv_label_set_text(battery_label,  "12.1v");
  lv_obj_set_style_text_align(battery_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(battery_label, LV_ALIGN_TOP_LEFT, 15, 190);
  battery_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(battery_caption,  "Batt");
  lv_obj_set_style_text_align(battery_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(battery_caption, battery_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 3);
  lv_obj_add_style(battery_caption,&miniCaptionStyle,0);


      iat_label = lv_label_create(lv_scr_act());
  lv_label_set_text(iat_label,  "50°");
  lv_obj_set_style_text_align(iat_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(iat_label, LV_ALIGN_TOP_LEFT, 70, 90);
  iat_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(iat_caption,  "IAT");
  lv_obj_set_style_text_align(iat_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(iat_caption, iat_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 3);
  lv_obj_add_style(iat_caption,&miniCaptionStyle,0);

  clt_label = lv_label_create(lv_scr_act());
  lv_label_set_text(clt_label,  "50°");
  lv_obj_set_style_text_align(clt_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(clt_label, LV_ALIGN_TOP_LEFT, 70, 140);
  clt_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(clt_caption,  "CLT");
  lv_obj_set_style_text_align(clt_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(clt_caption, clt_label, LV_ALIGN_OUT_BOTTOM_MID, 4, 3);
  lv_obj_add_style(clt_caption,&miniCaptionStyle,0);



  fp_label = lv_label_create(lv_scr_act());
  lv_label_set_text(fp_label,  "3.5");
  lv_obj_set_style_text_align(fp_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(fp_label, LV_ALIGN_TOP_LEFT, 140, 90);
  fp_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(fp_caption,  "Fuel Bar");
  lv_obj_set_style_text_align(fp_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(fp_caption, fp_label, LV_ALIGN_OUT_BOTTOM_MID, 17, 3);
  lv_obj_add_style(fp_caption,&miniCaptionStyle,0);

  oilp_label = lv_label_create(lv_scr_act());
  lv_label_set_text(oilp_label,  "4.5");
  lv_obj_set_style_text_align(oilp_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(oilp_label, LV_ALIGN_TOP_LEFT, 140, 140);
  oilp_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(oilp_caption,  "Oil Bar");
  lv_obj_set_style_text_align(oilp_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(oilp_caption, oilp_label, LV_ALIGN_OUT_BOTTOM_MID, 15, 3);
  lv_obj_add_style(oilp_caption,&miniCaptionStyle,0);


  advance_label = lv_label_create(lv_scr_act());
  lv_label_set_text(advance_label,  " 10");
  lv_obj_set_style_text_align(advance_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(advance_label, LV_ALIGN_TOP_LEFT, 205, 90);
  advance_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(advance_caption,  "Adv");
  lv_obj_set_style_text_align(advance_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(advance_caption, advance_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(advance_caption,&miniCaptionStyle,0);

  gammaCorr_label = lv_label_create(lv_scr_act());
  lv_label_set_text(gammaCorr_label,  "100");
  lv_obj_set_style_text_align(gammaCorr_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(gammaCorr_label, LV_ALIGN_TOP_LEFT, 200, 140);
  gammaCorr_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(gammaCorr_caption,  "Corr");
  lv_obj_set_style_text_align(gammaCorr_caption, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align_to(gammaCorr_caption, gammaCorr_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 3);
  lv_obj_add_style(gammaCorr_caption,&miniCaptionStyle,0);
  ego_label = lv_label_create(lv_scr_act());
  lv_label_set_text(ego_label,  "100");
  lv_obj_set_style_text_align(ego_label, LV_TEXT_ALIGN_RIGHT, 0);
   lv_obj_align(ego_label, LV_ALIGN_TOP_LEFT, 200, 195);
  ego_caption = lv_label_create(lv_scr_act());
  lv_label_set_text(ego_caption,  "EGO");
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


    static lv_style_t indicatorOff;
    lv_style_init(&indicatorOff);
     lv_style_set_text_font(&indicatorOff, &lv_font_montserrat_12); 
     lv_style_set_text_color(&indicatorOff,lv_palette_main(LV_PALETTE_BLUE_GREY));
     lv_style_set_bg_color(&indicatorOff, lv_palette_darken(LV_PALETTE_GREY, 2));

    accelIndicator = lv_btn_create(lv_scr_act());
    lv_obj_align(accelIndicator, LV_ALIGN_TOP_LEFT, 255, 147);
    accelIndicatorLabel = lv_label_create(accelIndicator);
    lv_label_set_text(accelIndicatorLabel, "Accel");
    lv_obj_center(accelIndicatorLabel);
     lv_obj_set_size(accelIndicator, 55, 25);
     lv_obj_add_style(accelIndicatorLabel,&indicatorOff,0);
    lv_obj_add_style(accelIndicator,&indicatorOff,0);

    wueIndicator = lv_btn_create(lv_scr_act());
    lv_obj_align(wueIndicator, LV_ALIGN_TOP_LEFT, 255, 177);
    wueIndicatorLabel = lv_label_create(wueIndicator);
    lv_label_set_text(wueIndicatorLabel, "WUE");
    lv_obj_set_size(wueIndicator, 55, 25);
    lv_obj_center(wueIndicatorLabel);
    lv_obj_add_style(wueIndicatorLabel,&indicatorOff,0);
    lv_obj_add_style(wueIndicator,&indicatorOff,0);

    syncIndicator = lv_btn_create(lv_scr_act());
    lv_obj_align(syncIndicator, LV_ALIGN_TOP_LEFT, 255, 207);
    syncIndicatorLabel = lv_label_create(syncIndicatorLabel);
    lv_label_set_text(syncIndicatorLabel, "wwww");
    lv_obj_set_size(syncIndicator, 55, 25);
    lv_obj_center(syncIndicatorLabel);
    lv_obj_add_style(syncIndicatorLabel,&indicatorOff,0);
    lv_obj_add_style(syncIndicator,&indicatorOff,0);

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
    lv_timer_handler();
    vTaskDelay(5);
  }
}


void loop()
{
//Serial.println("Starting CAN failed!");
  canReceiver();
 
}

void processData() {
  update_afr(float(speedyResponse[12]));
  update_clt(speedyResponse[7]-40);
  update_iat(speedyResponse[6] -40);
  update_tps(speedyResponse[24]/ 2);
  update_batt(speedyResponse[9]/10);
  update_adv(speedyResponse[23]);
  update_rpm(((speedyResponse [15] << 8) | (speedyResponse [14])));
  }


void update_tps(int tps_percent) {
   String tps_text = String("") + tps_percent+ "%" ;
  lv_label_set_text(tps_label,  tps_text.c_str());
}

void update_rpm(int rpm) {
  String rpm_text = String("") + rpm ;
  lv_label_set_text(rpm_label,  rpm_text.c_str());
  set_rpm(bar, rpm);
}

void update_clt(int clt) {
   String clt_text = String("") + clt + "°";
  lv_label_set_text(clt_label, clt_text.c_str());
}

void update_iat(int iat) {
   String iat_text = String("") + iat + "°";
  lv_label_set_text(iat_label, iat_text.c_str());
}

void update_adv(float adv) {
   String adv_text = String("") + adv + "°";
  lv_label_set_text(advance_label, adv_text.c_str());
}

void update_afr(float afr) {
   String afr_text = String("afr: ") + afr ;
  lv_label_set_text(afr_label, afr_text.c_str());
}

void update_batt(float batt) {
   String batt_text = String("") + batt/10 + "v";
  lv_label_set_text(battery_label, batt_text.c_str());
}

void update_pressures(int fp, int oilp) {
  String fp_text = String() + fp;
  lv_label_set_text(fp_label, fp_text.c_str());
  String oilp_text = String() + oilp;
  lv_label_set_text(oilp_label, oilp_text.c_str());
}

void update_o2_fueltrim(int afr, int gamma, int ego) {
  String afr_text = String() + afr;
  lv_label_set_text(afr_label, afr_text.c_str());
  String gamma_text = String() + gamma;
  lv_label_set_text(gammaCorr_label, gamma_text.c_str());
    String ego_text = String() + ego;
  lv_label_set_text(ego_label, ego_text.c_str());
}
