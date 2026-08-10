/*
 * Pug Morse!!
 * Author: Tomio NARITA <nrt@ff.iij4u.or.jp>
 * License: GNU General Public License v3.0
 *
 * Powered by Arduino Audio Tools by Phil Schatzmann (pschatzmann)
 *   https://github.com/pschatzmann/arduino-audio-tools
 *   https://github.com/pschatzmann/arduino-libhelix
 *   https://github.com/pschatzmann/ESP32-A2DP
 * Also,
 *   https://github.com/Networking-for-Arduino/EthernetESP32
 */

#define APP_VERSION "1.68 (2026/08/10)"

/*
 * Copied the following part from morse-shield_nrt.ino (2024/05/18)
 */

/*
  Electric keyer for morse-shield on Arduino Uno
  designed by nksg JG5CBR/6
  2019/12/20

  http://www.nksg.net/usbif4cw/
  usbif4cw@nksg.net
*/

#include <Arduino.h>

#if 0
#include <Wire.h>
#include <FaBoLCDmini_AQM0802A.h>
#include <EEPROM.h>
#endif

String date = "19/12/20";
//#define DEBUG
#define SCROLL_ROW
//#define SERIAL_PRINT_DIT_DAH

#if 1 /* JG1WYW */
//String mod_date = "23/09/12";
String mod_date = "24/05/01";
/*
 * These options were added by JG1WYW
 * So far, off delay and look ahead memory is tested with improved Iambic_A.
 *
 * EEPROM:
 *  - Save settings by long press on Sw3
 *  - Load default by pressing on Sw4 during power on (not initializing EEPROM)
 *
 * Experimental features (need to be enabled explicitly):
 *  - Off delay
 *  - Look ahead memory
 *  - Avoid chattering
 *
 */
#define OPT_PUG_MORSE                   /* Pug Morse derived from Pug Radio */
#define OPT_OFF_DELAY			/* off delay timers */
//#define OPT_ON_DELAY			/* on delay timers */
//#define OPT_REPEAT_ADD                  /* addition by repeat */
//#define OPT_LOOK_AHEAD_MEMORY           /* look ahead memory */
//#define OPT_TWO_TONE                    /* two tone for marking the trailing dot */
//#define OPT_ESP32                       /* ESP32 instead of Arduino Uno */
//#define OPT_AVOID_CHATTERING            /* indend to avoid chattering */
//#define OPT_AVOID_CHATTERING_WATCH      /* indend to avoid chattering */
#define OPT_AUTO_KEYERTYPE              /* adapt to paddle/straight change */
#define OPT_INVADER_SOUND		/* invader sound */
//#define OPT_MEMORY_CLICK		/* memory click */
#define OPT_RUNNER			/* runner training of callsigns */
#define OPT_BETTER_DECODE		/* better decode */
#define OPT_BETTER_SRAND		/* random seed using timing */
//#define OPT_PARAM_EEPROM                /* load/save params from/to EEPROM */
//#define OPT_LCD_DISP                    /* on/off of LCD display */
#define OPT_TONE_FREQ                   /* selectable tone frequency */
#define OPT_LAST_MENUINDEX              /* memory the last menuIndex */
#define OPT_WORD_SPACE                  /* word space dots */
#define OPT_BETTER_IAMBIC_A             /* Improved behavior of Iambic_A */
#define OPT_DECODE_INVALID_IMMEDIATELY  /* '_' for invalid chars immediately when running */
//#define OPT_DECODE_DEBUG
//#define OPT_TIMING_DEBUG                /* Debug flag */
#ifdef OPT_ESP32
//#define OPT_VIBRATION  	                /* vibration feedback */
#endif

#define MOD_MISC      /* See comments for each modification */
//#define OPT_NO_SERIAL

#endif /* JG1WYW */

// keyer setting
#ifdef MOD_MISC
#define MIN_WPM 15
#define MAX_WPM 25
#else
#define MIN_WPM  5
#define MAX_WPM 50
#endif
#define L        0
#define R        1
#define UP       0
#define DOWN     1
int key = UP;

// default keyer value
int Wpm = 20;
float wait = 3.0;
int durationDot = 60;
int durationMark = 0;
int durationSpace = 0;
int durationLongPress = 1000;
int paddleRvs = 0; // paddle reverse

#ifdef OPT_RUNNER
#if 1
int runner_mode = 1; // Runner mode instead of just callsigns
#else
int runner_mode = 0; // Runner mode instead of just callsigns
#endif
int running = 0; // means Now running
int runner_repeat = 0;
int runner_miss = 0;
int runner_accel = 0;
int runner_retry = 0;
int runnerDurationDot = 60;
int runner_count = 0;
#define MAX_RUNNER_ACCEL 20
int runner_clear_count = 10;
#define HIGH_RUNNER_ACCEL runner_clear_count
#endif

#if defined(OPT_RUNNER) || defined(OPT_INVADER_SOUND)
int runner_beep = 0;

#define RUNNER_HIT      1
#define RUNNER_MISS     2
#define INVADER_BEEP    3
#define INVADER_ERRBEEP 4

#endif

#ifdef OPT_WORD_SPACE
int wordSpace = 4; /* XXX 7 is too short */
int charSpace = 3;
int menuIndexWordSpace;
int menuIndexCharSpace;
#endif

#ifdef OPT_OFF_DELAY
int offDelay = 1; // off-delay disabled by default (Experimental)

int off_timer_ratio_dot = 50;     /* XXX extra percent (of dot) for dot */
int off_timer_ratio_dash = 150;   /* XXX extra percent (of dot) for dash */

#define MAX_DOT_RATIO  100
//#define MAX_LAM_RATIO  200
#define MAX_LAM_RATIO  100
#define MAX_DASH_RATIO 200
#ifdef OPT_REPEAT_ADD
#define MAX_REPEAT_ADD 100
#endif

int menuIndexOffDelay;
int menuIndexOffDelayDot;
int menuIndexOffDelayDash;

#ifdef OPT_REPEAT_ADD
int off_repeat_add = 10;
int menuIndexOffRepeatAdd = 0;
int repeatingL = 0;
int repeatingR = 0;
#endif
#endif

#ifdef OPT_ON_DELAY
int onDelay = 1; // on-delay disabled by default (Experimental)

int on_timer_ratio_dot = 50;     /* XXX extra percent (of dot) for dot */
int on_timer_ratio_dash = 150;   /* XXX extra percent (of dot) for dash */

int menuIndexOnDelay;
int menuIndexOnDelayDot;
int menuIndexOnDelayDash;

#endif

#ifdef OPT_LOOK_AHEAD_MEMORY
int lookAheadMemory = 1; // lookAheadMemory is disabled by default (Experimental)
int look_ahead_memory_ratio = 25; // XXX percent (of dot) for look ahead memory
int menuIndexLookAheadMemory;
int menuIndexLookAheadMemoryRatio;
#endif

#ifdef OPT_AVOID_CHATTERING
int avoidChattering = 1; // avoid chattering disabled by default (Experimental)
int avoidChatteringCount = 10; // Stable count

#define MAX_CHATTERING_COUNT 100

int menuIndexAvoidChattering;
int menuIndexAvoidChatteringCount;
#endif

#ifdef OPT_AUTO_KEYERTYPE
int autoKeyerType = 1; // auto keyer type disabled by default (Experimental)
#ifdef OPT_BETTER_IAMBIC_A
int autoKeyerTypeElekey = 0; // XXX IambicA, auto keyer type when elekey changed from straight key
#else
int autoKeyerTypeElekey = 2; // XXX Supr, auto keyer type when elekey changed from straight key
#endif
unsigned long auto_keyer_start = 0;
#define MAX_STRAIGHT_OR_PADDLE 3000 /* 3sec */
int menuIndexAutoKeyertype;
int menuIndexAutoKeyertypeElekey;
#endif

#ifdef OPT_VIBRATION
//#define VIBRATION_PERIOD  (durationDot / 3)
#define VIBRATION_PERIOD  20
unsigned long vibration_end = 0;
#endif

#ifdef OPT_INVADER_SOUND
int invaderSound = 1;
#endif

#ifdef OPT_BETTER_SRAND
int first_srand = 1;
#endif

#ifdef OPT_TONE_FREQ
int toneFreqIndex = 0; // must be adjust to toneFreq whenever changed
int menuIndexToneFreq;
#endif

// side tone
int sidetone = 1; // 0: disable, 1: enable
int defToneFreq[] = {
#ifdef OPT_TONE_FREQ
  220, 233, 247, 261, 277, 294, 311, 330, 349, 370, 392, 415,
#endif
  440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831,
  880, 932, 988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661
};
#ifdef OPT_LCD_DISP
int menuIndexLcdDisp;
int lcdDisp = 1; // LCD display on/off flag
#endif
#ifdef OPT_TONE_FREQ
int toneFreq = 220; // side tone frequency 220Hz, A3
#else
int toneFreq = 440; // side tone frequency 440Hz, A4
#endif
int beepFreq = 880 * 2; // beep frequency 880Hz

#ifdef OPT_TWO_TONE
int twoTone = 0; // experimental
int toneTail = 0;
int durationTail = 0;
#define TONE_TAIL_RATIO 33
int menuIndexTwoTone;
#endif

#ifdef OPT_MEMORY_CLICK
static int memoryClick = 0;
int menuIndexMemoryClick;
#endif

// keyer selection
#define keyIambicA  0
#define keyIambicB  1
#define keySuper    2 // super keyer
#define keyStraight 3 // straight key
#define keyBug      4 // bug key
#ifdef OPT_BETTER_IAMBIC_A
int keyerType = keyIambicA;
#else
int keyerType = keySuper;
#endif
static int memory = -1;

char *keyName[] {
  "IbcA",
  "IbcB",
  "Supr",
  "Strt"
};

#ifdef OPT_PUG_MORSE

#define pinPaddleL   14/*27*//*36*//*39*//*27*//*1*/
#define pinPaddleR   12/*14*//*39*//*34*//*14*//*3*/
#define pinPaddleGND 13/*34*//*35*//*12*//*21*/
//#define pinPaddleGND /*35*//*32*//*GND*//*GND*/

#define EFFECT_BUZZER  1
#define EFFECT_SUCCESS 2
#define EFFECT_CLEAR   3
#define EFFECT_START   4
int effect_active = 0;
unsigned char *effect_dat;
unsigned int effect_len = 0;

#include "data.cpp"

#define FILE_START "/snd_start.raw"
uint8_t *snd_start_dat;
unsigned int snd_start_len;

#define FILE_BUZZER "/snd_buzzer.raw"
uint8_t *snd_buzzer_dat;
unsigned int snd_buzzer_len;

#define FILE_SUCCESS "/snd_success.raw"
uint8_t *snd_success_dat;
unsigned int snd_success_len;

#define FILE_CLEAR "/snd_clear.raw"
uint8_t *snd_clear_dat;
unsigned int snd_clear_len;

unsigned long runner_start = 0;
#define RUNNER_TYPE_EASY    0
#define RUNNER_TYPE_NORMAL  1
#define RUNNER_TYPE_HARD    2
int runner_type = 0;

#endif /* OPT_PUG_MORSE */

#ifdef OPT_ESP32

// I/O pins setting analog
#define pinVol      36
#define pinAudioIn  39
// I/O pins setting digital
#define pinPaddleL   4
#define pinPaddleR  16
#define pinBz       32 // audioOutPin
#ifdef OPT_VIBRATION
#define pinVb       33
#endif
#define pinLed      17
#define pinSw4      14
#define pinSw3      12
#define pinSw2      13
#define pinSw1      15

#else

// I/O pins setting analog
#define pinVol     A0
#define pinAudioIn A1
// I/O pins setting digital
//#define pinPaddleL  6
//#define pinPaddleR  7
#define pinBz       10/*9*/ // audioOutPin
#define pinLed     13
#define pinSw4      2
#define pinSw3      3
#define pinSw2      4
#define pinSw1      5

#endif

int pinBtn[4] = {pinSw1, pinSw2, pinSw3, pinSw4};

// keyer language
#define langAbc   0
#define langWabun 1
int lang = langAbc;

#ifdef OPT_BETTER_IAMBIC_A
int paddleR_pressed = 0; // to check if re-pressed once after press detected
int paddleL_pressed = 0;
#endif
int lastPaddle = L; // memory key param
int endingChar = 0; // space after letter(1) or dit/dah(0)

static int pos_read = 0;
unsigned int nextCode = 0x01; // letter to send
unsigned int presetMessage[128]; // ruber stamp
int IsSendingPresetMessages;

// mark | space control
static unsigned long timeStart = 0;
static unsigned long timeStartSw[4];

// beacon cq
unsigned int defCqInterval = 2000;
static unsigned long timeCqInterval = 0;
static unsigned long timeStartCq = 0;
unsigned int defCqTimes = 10;
unsigned int CqTimes = defCqTimes;

#if 0
// 8x2 LCD setting
FaBoLCDmini_AQM0802A lcd;
#endif
#define SPACE 0b00100000 // 32
#if 0
const int colums = 8;
const int rows = 2;
int lcdIndex = 0;
int line1[colums];
int line2[colums];
#endif

// goertzel calculation param
float magnitude ;
int magnitudelimit = 100;
int magnitudelimit_low = 100;
int realstate = LOW;
int realstatebefore = LOW;
int filteredstate = LOW;
int filteredstatebefore = LOW;

#if 0 /* not used */
// The sampling frq will be 8928 on a 16 mhz
// without any prescaler etc
// because we need the tone in the center of the bins
// you can set the tone to 496, 558, 744 or 992
// then n the number of samples which give the bandwidth
// can be (8928 / tone) * 1 or 2 or 3 or 4 etc
// init is 8928/558 = 16 *4 = 64 samples
// try to take n = 96 or 128 ;o)
// 48 will give you a bandwidth around 186 hz
// 64 will give you a bandwidth around 140 hz
// 96 will give you a bandwidth around 94 hz
// 128 will give you a bandwidth around 70 hz
// BUT remember that high n take a lot of time
// so you have to find the compromice - i use 48
float coeff;
float Q1 = 0;
float Q2 = 0;
float sine;
float cosine;
float sampling_freq = 8928.0;
float target_freq = 558.0; // adjust for your needs see above
float n = 48.0; // if you change  her please change next line also
int testData[48];

// Noise Blanker time which shall be computed so this is initial
int nbtime = 6;  // ms noise blanker
#endif

long starttimehigh;
long highduration;
long lasthighduration;
#ifdef MOD_MISC
long hightimesavg = 60; /* to begin with moderate highduration */
#else
long hightimesavg;
#endif
long lowtimesavg;
long startttimelow;
long lowduration;
long laststarttime = 0;

char code = 1;//[20]; estimated code
int flag_stop = LOW;
#ifdef MOD_MISC
#define INIT_DECODE_WPM 20
int wpm = INIT_DECODE_WPM; // initial wpm value of decoding
#else
int wpm;
#endif

// game setting
#define maxInvader 64
int restInvader = -1;
unsigned int presetInvader[maxInvader];
int gamePts = 0;
static int combo = 0;
int gameStarted = 0;

// setting menu
// ToDo: save param on EEPROM
char *setMenu[] {
  "Lang A/", // ABC or Wabun
  "Pdl Rev.", // paddle reverse
  "Key mode", // keyer mode
  "Wait 1:*", // dit dah wait
  "SideTone", // sidetone on, off / tone[Hz]
  "CQint:ms", // beacon interval
  "CQ times"  // beacon times
#ifdef OPT_LCD_DISP
  ,
  "LCD disp" // LCD display on/off flag
#endif
#ifdef OPT_TONE_FREQ
  ,
  "ToneFreq"  // sidetone frequency
#endif
#ifdef OPT_WORD_SPACE
  ,
  "Word Spc",  // word space dots
  "Char Spc"   // character space dots
#endif
#ifdef OPT_OFF_DELAY
  ,
  "OffDelay",  // off delay
  "OfDlyDot",  // off delay dot ratio
  "OfDlyDsh"   // off delay dash ratio
#ifdef OPT_REPEAT_ADD
  ,
  "OfRptAdd"   // off delay repeating add
#endif
#endif
#ifdef OPT_ON_DELAY
  ,
  "OnDelay",   // on delay
  "OnDlyDot",  // on delay dot ratio
  "OnDlyDsh"   // on delay dash ratio
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
  ,
  "LAMemory",  // look ahead memory
  "LAMratio"   // look ahead memory ratio
#endif
#ifdef OPT_TWO_TONE
  ,
  "Two-Tone"   // enable two tone sound
#endif
#ifdef OPT_MEMORY_CLICK
  ,
  "MemClick"   // memory click
#endif
#ifdef OPT_AVOID_CHATTERING
  ,
  "AvodChtr",  // to avoid chattering
  "StbleCnt"   // stable count
#endif
#ifdef OPT_AUTO_KEYERTYPE
  ,
  "AtoKyChg",  // auto keyer type change
  "AtoKyEle"   // auto keyer type when elekey
#endif
};

#define MENU_INDEX_EXTRA 7 // XXX the beginning of extra menu
#define MaxMenu (sizeof(setMenu) / sizeof(char *))

#ifdef OPT_PARAM_EEPROM

typedef struct PARAM_EEPROM {
  int cksum;
  int lang;
  int paddleRvs;
  int keyerType;
  float wait;
  int sidetone;
  unsigned int defCqInterval;
  unsigned int defCqTimes;
#ifdef OPT_LCD_DISP
  int lcdDisp;
#endif
#ifdef OPT_TONE_FREQ
  int toneFreq;
#endif
#ifdef OPT_WORD_SPACE
  int wordSpace;
  int charSpace;
#endif
#ifdef OPT_OFF_DELAY
  int offDelay;
  int off_timer_ratio_dot;
  int off_timer_ratio_dash;
#ifdef OPT_REPEAT_ADD
  int off_repeat_add;
#endif
#endif
#ifdef OPT_ON_DELAY
  int onDelay;
  int on_timer_ratio_dot;
  int on_timer_ratio_dash;
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
  int lookAheadMemory;
  int look_ahead_memory_ratio;
#endif
#ifdef OPT_TWO_TONE
  int twoTone;
#endif
#ifdef OPT_MEMORY_CLICK
  int memoryClick;
#endif
#ifdef OPT_AVOID_CHATTERING
  int avoidChattering;
  int avoidChatteringCount;
#endif
#ifdef OPT_AUTO_KEYERTYPE
  int autoKeyerType;
  int autoKeyerTypeElekey;
#endif
} param_eeprom_t;

#define PARAM_EEPROM_BASE (128 * 4)

int param_eeprom_load()
{
  param_eeprom_t param;
  byte *ptr;
  int cksum = 0;

#ifdef OPT_ESP32
  return 0;
#endif

  ptr = (byte *)&param;
  for (int i = 0 ; i < sizeof(param_eeprom_t) ; i++) {
    *ptr = EEPROM.read(PARAM_EEPROM_BASE + i);
    if (i >= sizeof(int)) { // excluding the first cksum
      cksum ^= *ptr;
      cksum <<= 1;
    }
    ptr++;
  }

  if (cksum == param.cksum) {
    lang = param.lang;
    paddleRvs = param.paddleRvs;
    keyerType = param.keyerType;
    wait = param.wait;
    sidetone = param.sidetone;
    defCqInterval = param.defCqInterval;
    defCqTimes = param.defCqTimes;
#ifdef OPT_LCD_DISP
    lcdDisp = param.lcdDisp;
#endif
#ifdef OPT_TONE_FREQ
    toneFreq = param.toneFreq;
#endif
#ifdef OPT_WORD_SPACE
    wordSpace = param.wordSpace;
    charSpace = param.charSpace;
#endif
#ifdef OPT_OFF_DELAY
    offDelay = param.offDelay;
    off_timer_ratio_dot = param.off_timer_ratio_dot;
    off_timer_ratio_dash = param.off_timer_ratio_dash;
#ifdef OPT_REPEAT_ADD
    off_repeat_add = param.off_repeat_add;
#endif
#endif
#ifdef OPT_ON_DELAY
    onDelay = param.onDelay;
    on_timer_ratio_dot = param.on_timer_ratio_dot;
    on_timer_ratio_dash = param.on_timer_ratio_dash;
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
    lookAheadMemory = param.lookAheadMemory;
    look_ahead_memory_ratio = param.look_ahead_memory_ratio;
#endif
#ifdef OPT_TWO_TONE
    twoTone = param.twoTone;
#endif
#ifdef OPT_MEMORY_CLICK
    memoryClick = param.memoryClick;
#endif
#ifdef OPT_AVOID_CHATTERING
    avoidChattering = param.avoidChattering;
    avoidChatteringCount = param.avoidChatteringCount;
#endif
#ifdef OPT_AUTO_KEYERTYPE
    autoKeyerType = param.autoKeyerType;
    autoKeyerTypeElekey = param.autoKeyerTypeElekey;
#endif

    return 0; // success
  }

  return -1; // return without setting params when fail
}

void param_eeprom_save()
{
  param_eeprom_t param;
  byte *ptr;
  int cksum = 0;

  param.lang = lang;
  param.paddleRvs = paddleRvs;
  param.keyerType = keyerType;
  param.wait = wait;
  param.sidetone = sidetone;
  param.defCqInterval = defCqInterval;
  param.defCqTimes = defCqTimes;
#ifdef OPT_LCD_DISP
  param.lcdDisp = lcdDisp;
#endif
#ifdef OPT_TONE_FREQ
  param.toneFreq = toneFreq;
#endif
#ifdef OPT_WORD_SPACE
  param.wordSpace = wordSpace;
  param.charSpace = charSpace;
#endif
#ifdef OPT_OFF_DELAY
  param.offDelay = offDelay;
  param.off_timer_ratio_dot = off_timer_ratio_dot;
  param.off_timer_ratio_dash = off_timer_ratio_dash;
#ifdef OPT_REPEAT_ADD
  param.off_repeat_add = off_repeat_add;
#endif
#endif
#ifdef OPT_ON_DELAY
  param.onDelay = onDelay;
  param.on_timer_ratio_dot = on_timer_ratio_dot;
  param.on_timer_ratio_dash = on_timer_ratio_dash;
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
  param.lookAheadMemory = lookAheadMemory;
  param.look_ahead_memory_ratio = look_ahead_memory_ratio;
#endif
#ifdef OPT_TWO_TONE
  param.twoTone = twoTone;
#endif
#ifdef OPT_MEMORY_CLICK
  param.memoryClick = memoryClick;
#endif
#ifdef OPT_AVOID_CHATTERING
  param.avoidChattering = avoidChattering;
  param.avoidChatteringCount = avoidChatteringCount;
#endif
#ifdef OPT_AUTO_KEYERTYPE
  param.autoKeyerType = autoKeyerType;
  param.autoKeyerTypeElekey = autoKeyerTypeElekey;
#endif

  ptr = (byte *)&param;
  for (int i = 0 ; i < sizeof(param_eeprom_t) ; i++) {
    if (i >= sizeof(int)) { // excluding the first cksum
      cksum ^= *ptr;
      cksum <<= 1;
    }
    ptr++;
  }
  param.cksum = cksum;

  ptr = (byte *)&param;
  for (int i = 0 ; i < sizeof(param_eeprom_t) ; i++) {
    EEPROM.write(PARAM_EEPROM_BASE + i, *ptr);
    ptr++;
  }
}

void tone_eeprom()
{
  buzzer(pinBz, 698, 100); // F4
  delay(100);
  buzzer(pinBz, 1397, 200); // F5
  delay(100);
}

#endif /* OPT_PARAM_EEPROM */

#ifdef OPT_TIMING_DEBUG
void serial_timing(char *s)
{
#if 1
  return;
#endif
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(s);
  Serial.println();
}
#endif

// morse code
// dit=0, dah=1, start flag=1, 1char=1byte, max7 dit/dah
// example: "A" -> ".-" -> "0000 0101" -> 0x05
unsigned char BinAbc[] = {
  0b00000101, // A
  0b00011000, // B
  0b00011010, // C
  0b00001100, // D
  0b00000010, // E
  0b00010010, // F
  0b00001110, // G
  0b00010000, // H
  0b00000100, // I
  0b00010111, // J
  0b00001101, // K
  0b00010100, // L
  0b00000111, // M
  0b00000110, // N
  0b00001111, // O
  0b00010110, // P
  0b00011101, // Q
  0b00001010, // R
  0b00001000, // S
  0b00000011, // T
  0b00001001, // U
  0b00010001, // V
  0b00001011, // W
  0b00011001, // X
  0b00011011, // Y
  0b00011100, // Z
  0b00111111, // 0
  0b00101111, // 1
  0b00100111, // 2
  0b00100011, // 3
  0b00100001, // 4
  0b00100000, // 5
  0b00110000, // 6
  0b00111000, // 7
  0b00111100, // 8
  0b00111110, // 9
  0b01010101, // .
  0b01110011, // ,
  0b01001100, // ?
  0b00110010, // /
  0b00101010, // + [AR]
  0b00110001, // = [BT]
  0b01100001, // -
  0b01011110, // '
  0b01010010, // "
  0b01111000, // :
  0b00110110, // (
  0b01101101, // )
  0b01011010, // @
  0b01000101, // [VA]
  0b00101000  // [AS]
};

unsigned char CharAbc[] = {
  0b01000001, // A 0x41
  0b01000010, // B
  0b01000011, // C
  0b01000100, // D
  0b01000101, // E
  0b01000110, // F
  0b01000111, // G
  0b01001000, // H
  0b01001001, // I
  0b01001010, // J
  0b01001011, // K
  0b01001100, // L
  0b01001101, // M
  0b01001110, // N
  0b01001111, // O
  0b01010000, // P
  0b01010001, // Q
  0b01010010, // R
  0b01010011, // S
  0b01010100, // T
  0b01010101, // U
  0b01010110, // V
  0b01010111, // W
  0b01011000, // X
  0b01011001, // Y
  0b01011010, // Z
  0b00110000, // 0
  0b00110001, // 1
  0b00110010, // 2
  0b00110011, // 3
  0b00110100, // 4
  0b00110101, // 5
  0b00110110, // 6
  0b00110111, // 7
  0b00111000, // 8
  0b00111001, // 9
  0b00101110, // .
  0b00101100, // ,
  0b00111111, // ?
  0b00101111, // /
  0b00101011, // + [AR]
  0b00111101, // = [BT]
  0b00101101, // -
  0b00100111, // '
  0b00100010, // "
  0b00111010, // :
  0b00101000, // (
  0b00101001, // )
  0b01000000  // @
};

unsigned char BinKana[] = {
  0b00111011, // a
  0b00000101, // i
  0b00001001, // u
  0b00110111, // e
  0b00101000, // o
  0b00010100, // ka
  0b00110100, // ki
  0b00010001, // ku
  0b00011011, // ke
  0b00011111, // ko
  0b00110101, // sa
  0b00111010, // si
  0b00111101, // su
  0b00101110, // se
  0b00011110, // so
  0b00000110, // ta
  0b00010010, // ti
  0b00010110, // tu
  0b00101011, // te
  0b00100100, // to
  0b00001010, // na
  0b00011010, // ni
  0b00010000, // nu
  0b00011101, // ne
  0b00010011, // no
  0b00011000, // ha
  0b00111001, // hi
  0b00011100, // hu
  0b00000010, // he
  0b00001100, // ho
  0b00011001, // ma
  0b00100101, // mi
  0b00000011, // mu
  0b00110001, // me
  0b00110010, // mo
  0b00001011, // ya
  0b00110011, // yu
  0b00000111, // yo
  0b00001000, // ra
  0b00001110, // ri
  0b00110110, // ru
  0b00001111, // re
  0b00010101, // ro
  0b00001101, // wa
  0b00101001, // wi
  0b00101100, // we
  0b00010111, // wo
  0b00101010, // n
  0b00000100, // dakuten
  0b00100110, // handakuten
  0b00111111, // 0
  0b00101111, // 1
  0b00100111, // 2
  0b00100011, // 3
  0b00100001, // 4
  0b00100000, // 5
  0b00110000, // 6
  0b00111000, // 7
  0b00111100, // 8
  0b00111110, // 9
  0b00101101, // -
  0b01010101, // touten
  0b01010100, // tojikakko
  0b01101101, // (
  0b01010010, // )
  0b01100111, // hore
  0b00100010  // rata
};

unsigned char CharWabun[] = {
  0b10110001, // a
  0b10110010, // i
  0b10110011, // u
  0b10110100, // e
  0b10110101, // o
  0b10110110, // ka
  0b10110111, // ki
  0b10111000, // ku
  0b10111001, // ke
  0b10111010, // ko
  0b10111011, // sa
  0b10111100, // si
  0b10111101, // su
  0b10111110, // se
  0b10111111, // so
  0b11000000, // ta
  0b11000001, // ti
  0b11000010, // tu
  0b11000011, // te
  0b11000100, // to
  0b11000101, // na
  0b11000110, // ni
  0b11000111, // nu
  0b11001000, // ne
  0b11001001, // no
  0b11001010, // ha
  0b11001011, // hi
  0b11001100, // hu
  0b11001101, // he
  0b11001110, // ho
  0b11001111, // ma
  0b11010000, // mi
  0b11010001, // mu
  0b11010010, // me
  0b11010011, // mo
  0b11010100, // ya
  0b11010101, // yu
  0b11010110, // yo
  0b11010111, // ra
  0b11011000, // ri
  0b11011001, // ru
  0b11011010, // re
  0b11011011, // ro
  0b11011100, // wa
  0b00000100, // wi
  0b00000101, // we
  0b10100110, // wo
  0b11011101, // n
  0b11011110, // dakuten
  0b11011111, // handakuten
  0b00110000, // 0
  0b00110001, // 1
  0b00110010, // 2
  0b00110011, // 3
  0b00110100, // 4
  0b00110101, // 5
  0b00110110, // 6
  0b00110111, // 7
  0b00111000, // 8
  0b00111001, // 9
  0b00101101, // -
  0b10100100, // touten
  0b10100011, // tojikakko
  0b00101000, // (
  0b00101001  // )
};

// original characters
byte invader1L[8] = {
  0b00100,
  0b00010,
  0b00111,
  0b01101,
  0b11111,
  0b10111,
  0b10100,
  0b00011
};

byte invader1R[8] = {
  0b00100,
  0b01000,
  0b11100,
  0b10110,
  0b11111,
  0b11101,
  0b00101,
  0b11000
};

byte invader2L[8] = {
  0b00100,
  0b10010,
  0b10111,
  0b11101,
  0b01111,
  0b01111,
  0b00100,
  0b01000
};

byte invader2R[8] = {
  0b00100,
  0b01001,
  0b11101,
  0b10111,
  0b11110,
  0b11110,
  0b00100,
  0b00010
};

byte wi[8] = {
  0b00010,
  0b01111,
  0b01010,
  0b01010,
  0b11111,
  0b00010,
  0b00010
};

byte we[8] = {
  0b00000,
  0b11111,
  0b00101,
  0b00110,
  0b00100,
  0b11111,
  0b00000
};

#ifdef OPT_ESP32

void startVibration()
{
#if 0
  ledcWriteTone(2, 120);
#endif
}

void stopVibration()
{
#if 0
  ledcWrite(2, 0);
#endif
}

void buzzer(int dummyPin, int freq, int duration)
{
#if 0
  ledcWriteTone(1, freq);
  delay(duration);
  ledcWrite(1, 0);
#endif
}

void tone(int dummyPin, int freq)
{
#if 0
  ledcWriteTone(1, freq);
#ifdef OPT_VIBRATION
  startVibration();
  vibration_end = millis() + VIBRATION_PERIOD;
#endif
#endif
}

void noTone(int dummyPin)
{
#if 0
  ledcWrite(1, 0);
  //stopVibration();
#endif
}

#else

void buzzer(int pin, int freq, int duration)
{
#if 0
  tone(pin, freq, duration);
#endif
}

#endif

void setup_runner() {

#ifdef OPT_ESP32
  /* tone setup */
  ledcSetup(1, 12000, 8); // Channel 1, Hz, 8bit
  ledcAttachPin(pinBz, 1); // Channel 1

#ifdef OPT_VIBRATION
  /* vibration setup */
  ledcSetup(2, 12000, 8); // Channel 2, Hz, 8bit
  ledcAttachPin(pinVb, 2); // Channel 2
#endif
#endif

#if 0 /* not used */
  // The basic goertzel calculation
  int  k;
  float omega;
  k = (int) (0.5 + ((n * target_freq) / sampling_freq));
  omega = (2.0 * PI * k) / n;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
#endif

  Serial.begin(115200);
  Serial.println(F("morse-shield for Arduino UNO"));
  Serial.println(F("Designed by nksg"));
  Serial.print(F("20"));
  Serial.println(date);

  // Create the random seed
  randomSeed(analogRead(pinVol));

  // I/O pins setup
  for (int i = 0; i < 4; i++) pinMode(pinBtn[i], INPUT_PULLUP);
  pinMode(pinPaddleL, INPUT_PULLUP);
  pinMode(pinPaddleR, INPUT_PULLUP);
  pinMode(pinBz, OUTPUT);
  pinMode(pinLed, OUTPUT);

#ifdef MOD_MISC
  /* turn off led */
  digitalWrite(pinLed, LOW);
#endif

  // clear 4x switchs hold time
  for (int i = 0; i < 4; i++) timeStartSw[i] = 0;

#if 0
  // Set up the LCD
  lcd.begin();
  // Print messages to the LCD
#if 1
  lcd.home();
  lcd.print(F("WakuWaku"));
  lcd.setCursor(0, 1);
  lcd.print(mod_date); // yy/mm/dd
  delay(2000);
#endif
  lcd.home();
  lcd.print(F("MorseKey"));
  lcd.setCursor(0, 1);
  lcd.print(F("JG5CBR/6"));
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print(date); // yy/mm/dd
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print(F("        "));

  // Create a original character
  lcd.createChar(0, invader1L);
  lcd.createChar(1, invader1R);
  lcd.createChar(2, invader2L);
  lcd.createChar(3, invader2R);
  lcd.createChar(4, wi);
  lcd.createChar(5, we);

  // Clear both lines of the LCD
  for (int i = 0; i < colums; i++) {
    line1[i] = SPACE;
    line2[i] = SPACE;
  }
#endif

#if 1 /* XXX dynamic menu indexing */
  /*
   * set menuIndex dynamically
   */
  int menuIndex = MENU_INDEX_EXTRA;
#ifdef OPT_LCD_DISP
  menuIndexLcdDisp = menuIndex++;
#endif
#ifdef OPT_TONE_FREQ
  menuIndexToneFreq = menuIndex++;
#endif
#ifdef OPT_WORD_SPACE
  menuIndexWordSpace = menuIndex++;
  menuIndexCharSpace = menuIndex++;
#endif
#ifdef OPT_OFF_DELAY
  menuIndexOffDelay = menuIndex++;
  menuIndexOffDelayDot = menuIndex++;
  menuIndexOffDelayDash = menuIndex++;
#ifdef OPT_REPEAT_ADD
  menuIndexOffRepeatAdd = menuIndex++;
#endif
#endif
#ifdef OPT_ON_DELAY
  menuIndexOnDelay = menuIndex++;
  menuIndexOnDelayDot = menuIndex++;
  menuIndexOnDelayDash = menuIndex++;
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
  menuIndexLookAheadMemory = menuIndex++;
  menuIndexLookAheadMemoryRatio = menuIndex++;
#endif
#ifdef OPT_TWO_TONE
  menuIndexTwoTone = menuIndex++;
#endif
#ifdef OPT_MEMORY_CLICK
  menuIndexMemoryClick = menuIndex++;
#endif
#ifdef OPT_AVOID_CHATTERING
  menuIndexAvoidChattering = menuIndex++;
  menuIndexAvoidChatteringCount = menuIndex++;
#endif
#ifdef OPT_AUTO_KEYERTYPE
  menuIndexAutoKeyertype = menuIndex++;
  menuIndexAutoKeyertypeElekey = menuIndex++;
#endif
#endif /* 1 */

#ifdef OPT_PARAM_EEPROM
  if (digitalRead(pinSw4) == LOW) {
    /*
     * XXX push pinSw4 in order not to load EEPROM
     */
    tone_eeprom();
  } else {
    param_eeprom_load();
  }

  // reset toneFreqIndex (no matter which result returned)
  for (int i = 0 ; i < (sizeof(defToneFreq) / sizeof(int)) ; i++) {
    toneFreqIndex = 0;
    if (toneFreq <= defToneFreq[i]) {
      toneFreqIndex = i;
      break;
    }
  }
#endif
}

void readPaddleValue();
void setNextChar();
void goertzel();
void decode();
void printascii(int asciinumber);
void setNewWpm(int wpm);
void clearPresetMessage();

#ifdef OPT_PUG_MORSE

int noiseFlag = 1;
int push_button = HIGH;
int tone_active = 0;

#endif

void tone_on(int dummyPin, int freq)
{
  tone_active = 1;
  //Serial.printf("o");
}

void tone_off(int dummyPin)
{
  tone_active = 0;
  //Serial.printf("x");
}

void buzzer_on(int pin, int freq, int duration)
{
  //Serial.printf("b");
}

void loop_runner() {
#if 0
  // read Potentiometer value and set new WPM
  readPotValue();
#endif

  // read Paddle LR status
  readPaddleValue();

#if 0
  // read Button status
  readButtonValue();
#endif

  // Put a new character when the keyer is waiting
  if (durationMark == 0 && durationSpace == 0) {
#if 0
#ifndef OPT_NO_SERIAL
#ifdef OPT_LCD_DISP
   if (lcdDisp)
#endif
    readFromSerial();
#endif
#endif
    setNextChar();
  }

#if 0
  // invader game
  morseInvader();
#endif

  // Messages from preset or keyboard
  if (pos_read > 0) return;

  // goertzel Calculation
  goertzel();
}

#if 0 /* not used */
void getMorseTone() {
  // The basic where we get the tone
  for (char index = 0; index < n; index++) {
    testData[index] = analogRead(pinAudioIn);
  }
  for (char index = 0; index < n; index++) {
    float Q0;
    Q0 = coeff * Q1 - Q2 + (float) testData[index];
    Q2 = Q1;
    Q1 = Q0;
  }
  // we do only need the real part
  float magnitudeSquared = (Q1 * Q1) + (Q2 * Q2) - Q1 * Q2 * coeff;
  magnitude = sqrt(magnitudeSquared);
  Q2 = 0;
  Q1 = 0;

  // here you can measure magnitude for setup..
  //Serial.println(magnitude);

  // here we will try to set the magnitude limit automatic
  if (magnitude > magnitudelimit_low) {
    // moving average filter
    magnitudelimit = (magnitudelimit + ((magnitude - magnitudelimit) / 6));
  }

  if (magnitudelimit < magnitudelimit_low)
    magnitudelimit = magnitudelimit_low;

  // now we check for the magnitude //
  // just to have some space up
  if (magnitude > magnitudelimit * 0.6)
    realstate = HIGH;
  else
    realstate = LOW;

  // here we clean up the state with a noise blanker
  if (realstate != realstatebefore) {
    laststarttime = millis();
  }
  if ((millis() - laststarttime) >= nbtime) {
    if (realstate != filteredstate) {
      filteredstate = realstate;
    }
  }
}
#endif

void goertzel() {
  //getMorseTone();
  filteredstate = key; // digitalRead(13);

  // Then we do want to have some durations on high and low
  if (filteredstate != filteredstatebefore) {
    if (filteredstate == HIGH) { /* key pushed */
      starttimehigh = millis();
      lowduration = (millis() - startttimelow);
#ifdef OPT_BETTER_DECODE
      flag_stop = HIGH;
#endif
    }

    if (filteredstate == LOW) { /* key released */
      startttimelow = millis();
#ifdef OPT_BETTER_DECODE
      flag_stop = LOW;
#endif
      highduration = (millis() - starttimehigh);
#ifdef OPT_BETTER_DECODE
      if ((highduration * 2) <= hightimesavg) {
        hightimesavg = highduration;  // if speed increased fast ..
      } else if (highduration < (2 * hightimesavg) && hightimesavg < (2 * highduration)) {
        hightimesavg = (highduration + 3 * hightimesavg) / 4; // now we know avg dit time (rolling 4 avg)
      } else if (highduration >= (2 * hightimesavg) && highduration < (4 * hightimesavg)) {
        hightimesavg = (highduration / wait + 3 * hightimesavg) / 4; // now we know avg dit time (rolling 4 avg)
      } else
#else
      if (highduration < (2 * hightimesavg) || hightimesavg == 0) {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3; // now we know avg dit time ( rolling 3 avg)
      }
#endif
      if (highduration > (5 * hightimesavg) ) {
#ifdef OPT_BETTER_DECODE
        hightimesavg = highduration;  // if speed decreased fast ..
#else
        hightimesavg = highduration + hightimesavg;   // if speed decrease fast ..
#endif
      }
    }
  }

  // now we will check which kind of baud we have - dit or dah
  // and what kind of pause we do have 1 - 3 or 7 pause
  // we think that hightimeavg = 1 bit
  if (filteredstate != filteredstatebefore) {
#ifdef OPT_BETTER_DECODE
    if (filteredstate == LOW && (unsigned int)code < 0b10000000) { // we did end a HIGH
#else
    flag_stop = LOW;
    if (filteredstate == LOW && code < 0b10000000) { // we did end a HIGH
#endif
      if (highduration < (hightimesavg * 2) && highduration > (hightimesavg * 0.6)) { /// 0.6 filter out false dits
        code = code << 1;//strcat(code, ".");
#ifdef SERIAL_PRINT_DIT_DAH
        Serial.print(F("."));
#endif
      }
#ifdef OPT_BETTER_DECODE
      if (highduration >= (hightimesavg * 2) && highduration < (hightimesavg * 6)) {
#else
      if (highduration > (hightimesavg * 2) && highduration < (hightimesavg * 6)) {
#endif
        code = code << 1;
        code += 1;
        //strcat(code, "-");
#ifdef SERIAL_PRINT_DIT_DAH
        Serial.print(F("-"));
#endif
        wpm = (wpm + (1200 / ((highduration) / 3))) / 2; // the most precise we can do ;o)
      }
    }
#if defined(OPT_RUNNER) && defined(OPT_DECODE_INVALID_IMMEDIATELY)
    else if (running && filteredstate == LOW) {
	// invoke decode expecting '_' during running to notify invalid keying (typically connecting more than two characters) immediately
#ifdef OPT_DECODE_DEBUG
	Serial.print(F("(1)"));
#endif
	decode();
	code = 1;
    }
#endif

    if (filteredstate == HIGH) { // we did end a LOW

#ifdef OPT_BETTER_DECODE
      float lacktime = 0.9; // a bit less pause when low speed under 25wpm
#else
      float lacktime = 1;
#endif
      if (wpm > 25)lacktime = 1.0; ///  when high speeds we have to have a little more pause before new letter or new word
      if (wpm > 30)lacktime = 1.2;
      if (wpm > 35)lacktime = 1.5;

      if (lowduration > (hightimesavg * (2 * lacktime)) && lowduration < hightimesavg * (5 * lacktime)) { // letter space
#ifdef OPT_DECODE_DEBUG
        Serial.print(F("(2)"));
#endif
        decode();
        code = 1;
#ifdef SERIAL_PRINT_DIT_DAH
        Serial.print(F("/"));
#endif
      }
      if (lowduration >= hightimesavg * (5 * lacktime)) { // word space
#ifdef OPT_DECODE_DEBUG
        Serial.print(F("(3)"));
#endif
        decode();
        code = 1;
#ifdef OPT_RUNNER
       if (!running) // to omit word space during running
#endif
        printascii(SPACE);
#ifdef SERIAL_PRINT_DIT_DAH
        Serial.println();
#endif
      }
    }
  }

  // write if no more letters
#ifdef OPT_BETTER_DECODE
  if (flag_stop == LOW && (millis() - startttimelow) >= (hightimesavg * 6)) {
#else
  if ((millis() - startttimelow) >= (highduration * 6) && flag_stop == LOW) {
#endif
#ifdef OPT_DECODE_DEBUG
    Serial.print(F("(4)"));
#endif
    decode();
    code = 1; //code[0] = '\0';
    flag_stop = HIGH;
  }

  // we will turn on and off the LED and the speaker
  /*
    if (filteredstate == HIGH) {
      digitalWrite(pinLed, HIGH);
      tone(audioOutPin, target_freq);
    }
    else {
      digitalWrite(pinLed, LOW);
      noTone(audioOutPin);
    }
  */
  // the end of main loop clean up//
#if 0
#ifdef OPT_RUNNER
 if (!running) // to keep lcd space as much as possible during running for verification purpose
#endif
  updateinfolinelcd();
#endif
  realstatebefore = realstate;
  lasthighduration = highduration;
  filteredstatebefore = filteredstate;
}

#if 0
// Read Potentiometer value and set new WPM
void readPotValue() {
#ifdef MOD_MISC
#define INIT_POT_VAL -11 // to update the first Wpm value read from potentio meter
  static int potValueOld = INIT_POT_VAL;
#else
  static int potValueOld = 0;
#endif
  int potStep = 10;
  int potValue = analogRead(pinVol);

  if (potValue - potValueOld > potStep || potValue - potValueOld < -potStep) {
#ifdef OPT_ESP32
    int wpm = map(potValue, 0, 4095 - potStep, MIN_WPM, MAX_WPM);
#else
    int wpm = map(potValue, 0, 1023 - potStep, MIN_WPM, MAX_WPM);
#endif
#ifdef MOD_MISC
    if (wpm != Wpm || potValueOld == INIT_POT_VAL) setNewWpm(wpm);
#else
    if (wpm != Wpm) setNewWpm(wpm);
#endif
    
#ifdef DEBUG
    Serial.print(F("pot(0-1023)="));
    Serial.println(potValue);
#endif
    potValueOld = potValue;
  }
}
#endif

void setNewWpm(int wpm) {
  Wpm = wpm;
  durationDot = (60.0 * 1000) / (Wpm * 50);
#ifdef OPT_RUNNER
  runner_accel = 0; // reset acceleration when Wpm is modified manually
#endif

#ifndef OPT_NO_SERIAL
  Serial.print(F("WPM="));
  Serial.println(Wpm);
#ifdef DEBUG
  Serial.print(F("duration dot(ms)="));
  Serial.println(durationDot);
#endif
#endif

#if 0
  // Print new WPM to the LCD
  lcd.home();
  if (Wpm < 10) lcd.print(F("0"));
  lcd.print(Wpm);
  lcd.print(F("WPM"));
  lcd.print(F("  "));
  if (lang == langAbc) {
    lcd.print(F("A"));
  } else if (lang == langWabun) {
    lcd.write(0b10110001); // wabun
  }
#endif
}

void beep() {
#if 0
  buzzer(pinBz, beepFreq, 20);
#endif
}

void errBeep() {
#if 0
  buzzer(pinBz, 220, 20);
#endif
}

#ifdef OPT_RUNNER
void tone_runner_mode()
{
#if 0
  buzzer(pinBz, 349, 100); // F3
  delay(100);
  buzzer(pinBz, 698, 100); // F4
  delay(100);
#endif
}

void tone_hit()
{
  Serial.printf("\r\nHIT!!\r\n");
  effect_dat = snd_success_dat;
  effect_len = snd_success_len;
  effect_active = EFFECT_SUCCESS;
#if 0
  //buzzer(pinBz, 262, 100); // C3
  //buzzer(pinBz, 698, 100); // F3
  buzzer(pinBz, 392, 100); // G3
  //buzzer(pinBz, 698, 100); // F4
  delay(100);
  //buzzer(pinBz, 523, 100); // C4
  //buzzer(pinBz, 698, 100); // F4
  buzzer(pinBz, 784, 100); // G4
  //buzzer(pinBz, 1397, 200); // F5

  delay(100);
#endif
}

void tone_miss()
{
  Serial.printf("\r\nMISS!!\r\n");
  effect_dat = snd_buzzer_dat;
  effect_len = snd_buzzer_len;
  effect_active = EFFECT_BUZZER;
#if 0
  buzzer(pinBz, 220, 100); // A2
  delay(100);
  buzzer(pinBz, 110, 200); // A1
  delay(100);
#endif
}
#endif

#ifdef OPT_INVADER_SOUND
void tone_invader_start()
{
#if 0
  buzzer(pinBz, 880, 100); // A4
  delay(100);
  buzzer(pinBz, 1760, 100); // A5
  delay(100);
#endif
}

void tone_invader_clear()
{
  unsigned long clear_time = millis() - runner_start;
  int sec = clear_time / 1000;

  Serial.printf("\r\nCLEAR!! Time = %02d:%02d.%03d\r\n",
                sec / 60,
                sec % 60,
                (clear_time % 1000)
                );
  effect_dat = snd_clear_dat;
  effect_len = snd_clear_len;
  effect_active = EFFECT_CLEAR;
#if 0
  buzzer(pinBz, 784, 100); // G4
  delay(100);
  buzzer(pinBz, 1047, 100); // C5
  delay(100);
  buzzer(pinBz, 1568, 100); // G5
  delay(100);
#endif
}
#endif

#ifdef OPT_MEMORY_CLICK
static unsigned long timeEnd = 0;

#define MEMORY_CLICK_RATIO 50 /* % */

void memory_click_start()
{
  if (timeEnd == 0) {
    tone(pinBz, beepFreq * 3);
    //tone(pinBz, toneFreq);
    //tone(pinBz, beepFreq * 2);
    //tone(pinBz, toneFreq * 2);
    timeEnd = millis() + durationDot * MEMORY_CLICK_RATIO / 100;
    //timeEnd = millis() + durationMark - durationDot * MEMORY_CLICK_RATIO / 100;
    if (timeEnd == 0) // zero means not started
      timeEnd = 1;
  }
}

void memory_click_stop()
{
  if (durationMark != 0) {
    if (sidetone)
      tone(pinBz, toneFreq); // side tone
      //tone(pinBz, beepFreq * 3);
    else
      noTone(pinBz);
  } else {
    noTone(pinBz);
  }
  timeEnd = 0;
}
#endif

#ifdef OPT_OFF_DELAY
int paddleL_prev = HIGH;
int paddleR_prev = HIGH;
unsigned long paddleL_highStart = 0;
unsigned long paddleR_highStart = 0;

int paddleR_off_delay; /* off delay ms */
int paddleL_off_delay; /* off delay ms */
#endif

#ifdef OPT_ON_DELAY
int paddleL_on_prev = HIGH;
int paddleR_on_prev = HIGH;
unsigned long paddleL_on_highStart = 0;
unsigned long paddleR_on_highStart = 0;

int paddleR_on_delay; /* on delay ms */
int paddleL_on_delay; /* on delay ms */
#endif

#ifdef OPT_BETTER_IAMBIC_A
int paddleL_iambic_prev = HIGH;
int paddleR_iambic_prev = HIGH;
#endif

#ifdef OPT_AVOID_CHATTERING
int paddleL_stable = HIGH;
int paddleR_stable = HIGH;
#ifdef OPT_AVOID_CHATTERING_WATCH
int paddleL_watching = 0;
int paddleR_watching = 0;
#endif
int paddleL_trans = 0;
int paddleR_trans = 0;

//#define AVG_THRESHOLD 0.25
#define AVG_THRESHOLD 0.125
//#define AVG_THRESHOLD 0.5

float paddleL_avg = 0;
float paddleR_avg = 0;
#endif

#ifdef OPT_RUNNER
unsigned int runner_call[8];
int runner_call_len = 0;
int runner_ptr = 0;

void reset_durationDot()
{
  if (Wpm != 0)
    durationDot = (60.0 * 1000) / (Wpm * 50); // reset in case accelerated
}

void accel_durationDot()
{
  if (Wpm != 0)
    durationDot = (60.0 * 1000) / (Wpm * 50) * 100 / (100 + 5 * runner_accel);
}

void runner_stop()
{
  running = 0; // stop running
  runner_repeat = 0;
  reset_durationDot();

  void clearPresetMessage();
  clearPresetMessage();
}

void print_runner_count()
{
#if 1
  Serial.printf("Score: %d (%d)\r\n\r\n", runner_accel, runner_count);
  return;
#endif
  printascii(SPACE);
  printascii('(');

  if (runner_accel >= 10)
    printascii('0' + (runner_accel % 100) / 10);
  printascii('0' + (runner_accel % 10));

  printascii('/');

  if (runner_count >= 100)
    printascii('0' + (runner_count % 1000) / 100);
  if (runner_count >= 10)
    printascii('0' + (runner_count % 100) / 10);
  printascii('0' + (runner_count % 10));

  printascii(')');
  printascii(SPACE);
}
#endif

void readPaddleValue() {
  int paddleL = digitalRead(pinPaddleL);
  int paddleR = digitalRead(pinPaddleR);
//if (paddleL == LOW) {
//    Serial.printf("L");
//}
//if (paddleR == LOW) {
//    Serial.printf("R");
//}
#ifdef OPT_TIMING_DEBUG
  static int phys_paddleL_prev = HIGH;
  static int phys_paddleR_prev = HIGH;
  static int logi_paddleL_prev = HIGH;
  static int logi_paddleR_prev = HIGH;

  if (phys_paddleL_prev != paddleL) {
    if (paddleL == LOW) {
      serial_timing("pL-D,,,");
    } else {
      serial_timing("pL-U,,,");
    }
    phys_paddleL_prev = paddleL;
  }
  if (phys_paddleR_prev != paddleR) {
    if (paddleR == LOW) {
      serial_timing(",pR-D,,");
    } else {
      serial_timing(",pR-U,,");
    }
    phys_paddleR_prev = paddleR;
  }
#endif

#ifdef OPT_AUTO_KEYERTYPE
  if (autoKeyerType) {
    /*
     * automatic change between keyStraight and keySuper
     */
    if (keyerType != keyStraight) {
      if (paddleL == HIGH && paddleR == LOW) {
	if (auto_keyer_start + MAX_STRAIGHT_OR_PADDLE <= millis()) {
	  keyerType = keyStraight;
          auto_keyer_start = millis();
	}
      } else {
        auto_keyer_start = millis();
      }
    }
    if (keyerType == keyStraight) {
      if (paddleL == HIGH && paddleR == HIGH) {
	if (auto_keyer_start + MAX_STRAIGHT_OR_PADDLE <= millis()) {
	  keyerType = autoKeyerTypeElekey;
          auto_keyer_start = millis();
	}
      } else {
        auto_keyer_start = millis();
      }
    }
  }
#endif

#ifdef OPT_AVOID_CHATTERING
  if (avoidChattering) {
    float weight = 1.0 / avoidChatteringCount;

    /*
     * avoid chattering, paddle R
     */
#ifdef OPT_AVOID_CHATTERING_WATCH
    if (paddleR_watching == 0 && (paddleR_stable != paddleR))
      paddleR_watching = avoidChatteringCount;
#endif

    if (paddleR == LOW)
        paddleR_avg += paddleR_avg * (/*1.0*/ - weight) /* + 0 * weight */;
    else
        paddleR_avg += paddleR_avg * (/*1.0*/ - weight) + /* 1 * */ weight;

#ifdef OPT_AVOID_CHATTERING_WATCH
    if (paddleR_watching > 0) {
      paddleR_watching--;
      if (paddleR_watching == 0) {
        if (paddleR_avg < 0.5)
            paddleR_stable = LOW;
        if (paddleR_avg >= 0.5)
            paddleR_stable = HIGH;
        paddleR = paddleR_stable;
      }
    }
#endif

    if (paddleR_stable == HIGH && paddleR_avg < AVG_THRESHOLD)
        paddleR_stable = LOW;
    if (paddleR_stable == LOW && paddleR_avg > (1.0 - AVG_THRESHOLD))
        paddleR_stable = HIGH;
    paddleR = paddleR_stable;

    /*
     * avoid chattering, paddle L
     */
#ifdef OPT_AVOID_CHATTERING_WATCH
    if (paddleL_watching == 0 && (paddleL_stable != paddleL))
      paddleL_watching = avoidChatteringCount;
#endif

    if (paddleL == LOW)
        paddleL_avg += paddleL_avg * (/*1.0*/ - weight) /* + 0 * weight */;
    else
        paddleL_avg += paddleL_avg * (/*1.0*/ - weight) + /* 1 * */ weight;

#ifdef OPT_AVOID_CHATTERING_WATCH
    if (paddleL_watching > 0) {
      paddleL_watching--;
      if (paddleL_watching == 0) {
        if (paddleL_stable == HIGH && paddleL_avg < 0.5)
            paddleL_stable = LOW;
        if (paddleL_stable == LOW && paddleL_avg >= 0.5)
            paddleL_stable = HIGH;
        paddleL = paddleL_stable;
      }
    }
#endif

    if (paddleL_stable == HIGH && paddleL_avg < AVG_THRESHOLD)
        paddleL_stable = LOW;
    if (paddleL_stable == LOW && paddleL_avg > (1.0 - AVG_THRESHOLD))
        paddleL_stable = HIGH;
    paddleL = paddleL_stable;

#if 0
    /*
     * avoid chattering, paddle R
     */
    if (paddleR == LOW && paddleR_stable == HIGH) {
      if (++paddleR_trans >= avoidChatteringCount) {
	paddleR_stable = LOW;
	paddleR_trans = 0;
      } else {
	/* stays HIGH until stable count reach */
	paddleR = HIGH;
      }
    } else if (paddleR == HIGH && paddleR_stable == LOW) {
      if (--paddleR_trans <= -avoidChatteringCount) {
	paddleR_stable = HIGH;
	paddleR_trans = 0;
      } else {
	/* stays LOW until stable count reach */
	paddleR = LOW;
      }
    }

    /*
     * avoid chattering, paddle L
     */
    if (paddleL == LOW && paddleL_stable == HIGH) {
      if (++paddleL_trans >= avoidChatteringCount) {
	paddleL_stable = LOW;
	paddleL_trans = 0;
      } else {
	/* stays HIGH until stable count reach */
	paddleL = HIGH;
      }
    } else if (paddleL == HIGH && paddleL_stable == LOW) {
      if (--paddleL_trans <= -avoidChatteringCount) {
	paddleL_stable = HIGH;
	paddleL_trans = 0;
      } else {
	/* stays LOW until stable count reach */
	paddleL = LOW;
      }
    }
#endif

  }
#endif /* OPT_AVOID_CHATTERING */

  // OK till here
  //return;
#ifdef OPT_OFF_DELAY
  /*
   * off delay is only for electric keyer. (i.e., not incl. straight key)
   */
  if (offDelay && keyerType != keyStraight) {
    /*
     * off delay, paddle R
     */
    if (paddleR == LOW) {
      /* immediately active, clear off delay timer */
      paddleR_prev = LOW;
      paddleR_highStart = 0;
    } else if (/*paddleR == HIGH &&*/ paddleR_prev == LOW) {
      /* R's falling edge */
      paddleR_prev = HIGH;
      paddleR_highStart = millis();
      if (paddleRvs) {
        /* reversed to DOT */
#ifdef OPT_REPEAT_ADD
        paddleR_off_delay = durationDot * (off_timer_ratio_dot + off_repeat_add * repeatingR) / 100;
        repeatingR = 0;
#else
        paddleR_off_delay = durationDot * off_timer_ratio_dot / 100;
#endif
      } else {
        /* normal DASH */
#ifdef OPT_REPEAT_ADD
        paddleR_off_delay = durationDot * (off_timer_ratio_dash + off_repeat_add * repeatingR) / 100;
        repeatingR = 0;
#else
        paddleR_off_delay = durationDot * off_timer_ratio_dash / 100;
#endif
      }
      /* paddleR stays active during off delay timer */
      paddleR = LOW;
    } else if (/*paddleR == HIGH && paddleR_prev != LOW &&*/ paddleR_highStart != 0) {
      if ((millis() - paddleR_highStart) < paddleR_off_delay) {
        /* paddleR stays active during off delay timer */
        paddleR = LOW;
      } else {
        /* off delay expired */
        paddleR_highStart = 0;
      }
    }

    /*
     * off delay, paddle L
     */
    if (paddleL == LOW) {
      /* immediately active, clear off delay timer */
      paddleL_prev = LOW;
      paddleL_highStart = 0;
    } else if (/*paddleL == HIGH &&*/ paddleL_prev == LOW) {
      /* L's falling edge */
      paddleL_prev = HIGH;
      paddleL_highStart = millis();
      if (paddleRvs) {
        /* reversed to DASH */
#ifdef OPT_REPEAT_ADD
        paddleL_off_delay = durationDot * (off_timer_ratio_dash + off_repeat_add * repeatingL) / 100;
        repeatingL = 0;
#else
        paddleL_off_delay = durationDot * off_timer_ratio_dash / 100;
#endif
      } else {
        /* normal DOT */
#ifdef OPT_REPEAT_ADD
        paddleL_off_delay = durationDot * (off_timer_ratio_dot + off_repeat_add * repeatingL) / 100;
        repeatingL = 0;
#else
        paddleL_off_delay = durationDot * off_timer_ratio_dot / 100;
#endif
      }
      /* paddleL stays active during off delay timer */
      paddleL = LOW;
    } else if (/*paddleL == HIGH && paddleL_prev != LOW &&*/ paddleL_highStart != 0) {
      if ((millis() - paddleL_highStart) < paddleL_off_delay) {
        /* paddleL stays active during off delay timer */
        paddleL = LOW;
      } else {
        /* off delay expired */
        paddleL_highStart = 0;
      }
    }

  } /* (offDelay && keyerType != keyStraight) */
#endif

  // OK till here
  //return;
/**************************************/
#ifdef OPT_ON_DELAY
  /*
   * on delay is only for electric keyer. (i.e., not incl. straight key)
   */
  if (onDelay && keyerType != keyStraight) {
    /*
     * on delay, paddle R
     */
    if (paddleR == HIGH) {
      /* immediately active, clear on delay timer */
      paddleR_on_prev = HIGH;
      paddleR_on_highStart = 0;
    } else if (/*paddleR == LOW &&*/ paddleR_on_prev == HIGH) {
      /* R's rising edge */
      paddleR_on_prev = LOW;
      paddleR_on_highStart = millis();
      if (paddleRvs) {
        /* reversed to DOT */
#ifdef OPT_REPEAT_ADD
        paddleR_on_delay = durationDot * (on_timer_ratio_dot + on_repeat_add * repeatingR) / 100;
        repeatingR = 0;
#else
        paddleR_on_delay = durationDot * on_timer_ratio_dot / 100;
#endif
      } else {
        /* normal DASH */
#ifdef OPT_REPEAT_ADD
        paddleR_on_delay = durationDot * (on_timer_ratio_dash + on_repeat_add * repeatingR) / 100;
        repeatingR = 0;
#else
        paddleR_on_delay = durationDot * on_timer_ratio_dash / 100;
#endif
      }
      /* paddleR stays inactive during on delay timer */
      paddleR = HIGH;
    } else if (/*paddleR == LOW && paddleR_on_prev != HIGH &&*/ paddleR_on_highStart != 0) {
      if ((millis() - paddleR_on_highStart) < paddleR_on_delay) {
        /* paddleR stays inactive during on delay timer */
        paddleR = HIGH;
      } else {
        /* on delay expired */
        paddleR_on_highStart = 0;
      }
    }

    /*
     * on delay, paddle L
     */
    if (paddleL == HIGH) {
      /* immediately active, clear on delay timer */
      paddleL_on_prev = HIGH;
      paddleL_on_highStart = 0;
    } else if (/*paddleL == LOW &&*/ paddleL_on_prev == HIGH) {
      /* L's rising edge */
      paddleL_on_prev = LOW;
      paddleL_on_highStart = millis();
      if (paddleRvs) {
        /* reversed to DASH */
#ifdef OPT_REPEAT_ADD
        paddleL_on_delay = durationDot * (on_timer_ratio_dash + on_repeat_add * repeatingL) / 100;
        repeatingL = 0;
#else
        paddleL_on_delay = durationDot * on_timer_ratio_dash / 100;
#endif
      } else {
        /* normal DOT */
#ifdef OPT_REPEAT_ADD
        paddleL_on_delay = durationDot * (on_timer_ratio_dot + on_repeat_add * repeatingL) / 100;
        repeatingL = 0;
#else
        paddleL_on_delay = durationDot * on_timer_ratio_dot / 100;
#endif
      }
      /* paddleL stays inactive during on delay timer */
      paddleL = HIGH;
    } else if (/*paddleL == LOW && paddleL_on_prev != HIGH &&*/ paddleL_on_highStart != 0) {
      if ((millis() - paddleL_on_highStart) < paddleL_on_delay) {
        /* paddleL stays inactive during on delay timer */
        paddleL = HIGH;
      } else {
        /* on delay expired */
        paddleL_on_highStart = 0;
      }
    }

  } /* (onDelay && keyerType != keyStraight) */
#endif
/**************************************/

  // OK till here
  //return;

#ifdef OPT_BETTER_IAMBIC_A
  if (keyerType != keyStraight) {

    /*
     * paddle R
     */
    if (paddleR == LOW) {
      if (paddleR_iambic_prev == HIGH) {
         paddleR_pressed = 1;
      }
      paddleR_iambic_prev = LOW;
    } else {
      paddleR_iambic_prev = HIGH;
    }

    /*
     * paddle L
     */
    if (paddleL == LOW) {
      if (paddleL_iambic_prev == HIGH) {
         paddleL_pressed = 1;
      }
      paddleL_iambic_prev = LOW;
    } else {
      paddleL_iambic_prev = HIGH;
    }

  }
#endif

#ifdef MOD_MISC
  if (paddleR == LOW && keyerType == keyStraight) {
    /* ignore R when straight key */
    paddleR = HIGH;
  }
#endif

#ifdef OPT_TIMING_DEBUG
  if (logi_paddleL_prev != paddleL) {
    if (paddleL == LOW) {
      serial_timing(",,L-D,");
    } else {
      serial_timing(",,L-U,");
    }
    logi_paddleL_prev = paddleL;
  }
  if (logi_paddleR_prev != paddleR) {
    if (paddleR == LOW) {
      serial_timing(",,,R-D");
    } else {
      serial_timing(",,,R-U");
    }
    logi_paddleR_prev = paddleR;
  }
#endif

#if 0
  // clear preset messages
  if ((paddleL == LOW || paddleR == LOW) && pos_read > 0) {
    clearPresetMessage();
    clearBeacon();
    return;
  }
#endif

#ifdef OPT_RUNNER
  if (running && (paddleR == LOW || paddleL == LOW)) {
    // normal speed for actual keying
    reset_durationDot();
  }
#endif

  // OK till here
  //timeStart = millis(); // OK
  //Serial.printf("Debug(0)\r\n");
  //return;

  /*
   * Paddle LOW = Pushed, HIGH = Released
   */

  // pressing both paddles
  if (durationSpace == 0 && paddleL == LOW && paddleR == LOW) {
    //Serial.printf("Debug(1)\r\n");
    if (durationMark == 0 && keyerType != keyStraight) {
      memory = -1;
#ifdef OPT_BETTER_IAMBIC_A
      /* reset memory and pressed status at the beginning of every mark */
      paddleR_pressed = 0;
      paddleL_pressed = 0;
#endif
      if (lastPaddle != L) {
        durationMark = !paddleRvs ? durationDot : durationDot * wait;
        lastPaddle = L;
#if 1 /* PUG_MORSE */
        if (push_button == LOW) {
          if (!paddleRvs) {
            if (Wpm < MAX_WPM)
                Wpm++;
          } else {
            if (Wpm > MIN_WPM)
                Wpm--;
          }
          setNewWpm(Wpm);
        }
#endif
#ifdef OPT_TWO_TONE
	if (twoTone) {
	  if (!paddleRvs)
	    toneTail = 1;
	  else
	    toneTail = 0;
	  //durationTail = !paddleRvs ? durationDot * (100 - TONE_TAIL_RATIO) / 100 : durationDot * wait - durationDot * TONE_TAIL_RATIO / 100;
	  durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	  toneTail = 0;
	}
#endif
      } else {
        durationMark = !paddleRvs ? durationDot * wait : durationDot;
        lastPaddle = R;
#if 1 /* PUG_MORSE */
        if (push_button == LOW) {
          if (paddleRvs) {
            if (Wpm < MAX_WPM)
                Wpm++;
          } else {
            if (Wpm > MIN_WPM)
                Wpm--;
          }
          setNewWpm(Wpm);
        }
#endif
#ifdef OPT_TWO_TONE
	if (twoTone) {
	  if (!paddleRvs)
	    toneTail = 0;
	  else
	    toneTail = 1;
	  //durationTail = !paddleRvs ? durationDot * wait - durationDot * TONE_TAIL_RATIO / 100 : durationDot * (100 - TONE_TAIL_RATIO) / 100;
	  durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	  toneTail = 0;
	}
#endif
      }
#if 0
      digitalWrite(pinLed, HIGH);
#endif
      key = DOWN;
      timeStart = millis();
#ifdef OPT_TIMING_DEBUG
      serial_timing(",,,,t1-D,");
#endif
#ifdef OPT_TWO_TONE
      if (sidetone) {
	tone(pinBz, toneFreq); // side tone
	//if (twoTone)
	//toneTail = 0;
      }
#else
#ifdef OPT_MEMORY_CLICK
      if (memoryClick)
        memory_click_start();
      else
#endif
#if 0
      if (sidetone) tone(pinBz, toneFreq); // side tone
#else
      if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
#endif
    }
    // pressing the L paddle
  } else if (durationSpace == 0 && paddleL == LOW) {
    //Serial.printf("Debug(2)\r\n");
    if (durationMark == 0) {
      if (keyerType == keyStraight) {
        durationMark = 10000; // key down straight key
      } else {
        durationMark = !paddleRvs ? durationDot : durationDot * wait;
#if 1 /* PUG_MORSE */
        if (push_button == LOW) {
          if (!paddleRvs) {
            if (Wpm < MAX_WPM)
                Wpm++;
          } else {
            if (Wpm > MIN_WPM)
                Wpm--;
          }
          setNewWpm(Wpm);
        }
#endif
#ifdef OPT_TWO_TONE
	if (twoTone) {
	  if (!paddleRvs)
	    toneTail = 1;
	  else
	    toneTail = 0;
	  //durationTail = !paddleRvs ? durationDot * (100 - TONE_TAIL_RATIO) / 100 : durationDot * wait - durationDot * TONE_TAIL_RATIO / 100;
	  durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	  toneTail = 0;
	}
#endif
      }
      lastPaddle = L;
      memory = -1;
#ifdef OPT_BETTER_IAMBIC_A
      /* reset memory and pressed status at the beginning of every mark */
      paddleR_pressed = 0;
      paddleL_pressed = 0;
#endif
#if 0
      digitalWrite(pinLed, HIGH);
#endif
      key = DOWN;
      timeStart = millis();
#ifdef OPT_TIMING_DEBUG
      serial_timing(",,,,t2-D,");
#endif
#ifdef OPT_TWO_TONE
      if (sidetone) {
	tone(pinBz, toneFreq); // side tone
	//if (twoTone)
	//toneTail = 0;
      }
#else
#ifdef OPT_MEMORY_CLICK
      if (memoryClick)
        memory_click_start();
      else
#endif
#if 0
      if (sidetone) tone(pinBz, toneFreq); // side tone
#else
      if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
#endif
    }
    // pressing the R paddle
  } else if (paddleR == LOW && durationSpace == 0) {
    //Serial.printf("Debug(3)\r\n");
    if (durationMark == 0 && keyerType != keyStraight) {
      durationMark = !paddleRvs ? durationDot * wait : durationDot;
#if 1 /* PUG_MORSE */
        if (push_button == LOW) {
          if (paddleRvs) {
            if (Wpm < MAX_WPM)
                Wpm++;
          } else {
            if (Wpm > MIN_WPM)
                Wpm--;
          }
          setNewWpm(Wpm);
        }
#endif
      lastPaddle = R;
      memory = -1;
#ifdef OPT_BETTER_IAMBIC_A
      /* reset memory and pressed status at the beginning of every mark */
      paddleR_pressed = 0;
      paddleL_pressed = 0;
#endif
#ifdef OPT_TWO_TONE
      if (twoTone) {
	if (!paddleRvs)
	  toneTail = 0;
	else
	  toneTail = 1;
	//durationTail = !paddleRvs ? durationDot * wait - durationDot * TONE_TAIL_RATIO / 100 : durationDot * (100 - TONE_TAIL_RATIO) / 100;
	durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	toneTail = 0;
      }
#endif
#if 0
      digitalWrite(pinLed, HIGH);
#endif
      key = DOWN;
      timeStart = millis();
#ifdef OPT_TIMING_DEBUG
      serial_timing(",,,,t3-D,");
#endif
#ifdef OPT_TWO_TONE
      if (sidetone) {
	tone(pinBz, toneFreq); // side tone
	//if (twoTone)
	//toneTail = 0;
      }
#else
#ifdef OPT_MEMORY_CLICK
      if (memoryClick)
        memory_click_start();
      else
#endif
#if 0
      if (sidetone) tone(pinBz, toneFreq); // side tone
#else
      if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
#endif
    }
  }

#if 1
  // XXX force reboot loop here
  //return;
#endif

  if (durationSpace != 0) {
#ifdef OPT_BETTER_IAMBIC_A
    if (keyerType == keyIambicA) {
      if (memory == -1 && paddleL == LOW && paddleL_pressed) {
	memory = L;
        paddleL_pressed = 0;
      } else if (memory == -1 && paddleR == LOW && paddleR_pressed) {
	memory = R;
        paddleR_pressed = 0;
      }
#ifdef OPT_LOOK_AHEAD_MEMORY
      else if (lookAheadMemory && memory == -1 && (millis() - timeStart >= durationDot * (100 - min(100,look_ahead_memory_ratio)) / 100)) {
        // memory paddle LR
	if (paddleL == LOW && paddleR == LOW) {
          if (lastPaddle == R) {
            memory = L;
            paddleL_pressed = 0;
          } else {
            memory = R;
            paddleR_pressed = 0;
          }
	} else if (paddleL == LOW) {
          memory = L;
          paddleL_pressed = 0;
	} else if (paddleR == LOW) {
          memory = R;
          paddleR_pressed = 0;
	}
      }
#endif
    } else
#endif
    if (keyerType == keyIambicB || keyerType == keySuper) {

      // memory paddle LR
      if (memory == -1 && paddleL == LOW && lastPaddle == R) {
        memory = L;
      } else if (memory == -1 && paddleR == LOW && lastPaddle == L) {
        memory = R;
      }
#ifdef OPT_LOOK_AHEAD_MEMORY
      else if (lookAheadMemory && memory == -1 && (millis() - timeStart >= durationDot * (100 - min(100,look_ahead_memory_ratio)) / 100)) {
        // memory paddle LR
	if (paddleL == LOW && paddleR == LOW) {
          if (lastPaddle == R) {
            memory = L;
          } else {
            memory = R;
          }
	} else if (paddleL == LOW) {
          memory = L;
	} else if (paddleR == LOW) {
          memory = R;
	}
      }
#endif
    }
    // ending of space
    if (millis() - timeStart >= durationSpace) {
      durationSpace = 0;
      // output memory LR
#ifdef OPT_REPEAT_ADD
      if (off_repeat_add > 0) {
        if (memory == L) {
          if (lastPaddle == L) {
            if (repeatingL < 4) {
              repeatingL++;
            } else if (repeatingL >= 4) {
              repeatingL = 0;
              memory = -1;
              //Serial.print(F("MemoryClear"));
            }
            repeatingR = 0;
            //Serial.println();
            //Serial.print(F("repeatingL="));
            //Serial.println(repeatingL);
          } else {
            repeatingL = 0;
            repeatingR = 0;
          }
        } else if (memory == R) {
          if (lastPaddle == R) {
            if (repeatingR < 4) {
              repeatingR++;
            } else if (repeatingR >= 4) {
              repeatingR = 0;
              memory = -1;
              //Serial.print(F("MemoryClear"));
            }
            repeatingL = 0;
            //Serial.println();
            //Serial.print(F("repeatingR="));
            //Serial.println(repeatingR);
          } else {
            repeatingL = 0;
            repeatingR = 0;
          }
        }
      }
#endif
      if (memory != -1) {
        if (memory == L) {
          durationMark = !paddleRvs ? durationDot : durationDot * wait;
          lastPaddle = L;
#ifdef OPT_TWO_TONE
	  if (twoTone) {
	    if (!paddleRvs)
	      toneTail = 1;
	    else
	      toneTail = 0;
	    //durationTail = !paddleRvs ? durationDot * (100 - TONE_TAIL_RATIO) / 100 : durationDot * wait - durationDot * TONE_TAIL_RATIO / 100;
	    durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	    toneTail = 0;
	  }
#endif
        } else if (memory == R) {
          durationMark = !paddleRvs ? durationDot * wait : durationDot;
          lastPaddle = R;
#ifdef OPT_TWO_TONE
	  if (twoTone) {
	    if (!paddleRvs)
	      toneTail = 0;
	    else
	      toneTail = 1;
	    //durationTail = !paddleRvs ? durationDot * wait - durationDot * TONE_TAIL_RATIO / 100 : durationDot * (100 - TONE_TAIL_RATIO) / 100;
	    durationTail = durationMark * (100 - TONE_TAIL_RATIO) / 100;
	    toneTail = 0;
	  }
#endif
        }
        memory = -1;
#if 0
        digitalWrite(pinLed, HIGH);
#endif
        key = DOWN;
        timeStart = millis();
#ifdef OPT_TIMING_DEBUG
        serial_timing(",,,,t4-D,");
#endif
#ifdef OPT_TWO_TONE
	if (sidetone) {
	  tone(pinBz, toneFreq); // side tone
	  //if (twoTone)
	  //toneTail = 0;
	}
#else
#ifdef OPT_MEMORY_CLICK
        if (memoryClick)
          memory_click_start();
        else
#endif
#if 0
        if (sidetone) tone(pinBz, toneFreq); // side tone
#else
        if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
#endif
#ifdef OPT_REPEAT_ADD
      } else {
        repeatingL = 0;
        repeatingR = 0;
#endif
      }
    }
  }
  if (durationMark != 0) {
#ifdef OPT_TWO_TONE
    if (twoTone && sidetone && toneTail == 0 && (millis() - timeStart) >= durationTail) {
      toneTail = 1;
#if 0
      noTone(pinBz);
      tone(pinBz, toneFreq*5/8);
#endif
    }
#endif
    // memory paddle LR
#ifdef OPT_BETTER_IAMBIC_A
    if (keyerType == keyIambicA) {
      if (memory == -1 && paddleL_pressed && lastPaddle == R) {
	memory = L;
        paddleL_pressed = 0;
      } else if (memory == -1 && paddleR_pressed && lastPaddle == L) {
	memory = R;
        paddleR_pressed = 0;
      }
#ifdef OPT_LOOK_AHEAD_MEMORY
      else if (lookAheadMemory && look_ahead_memory_ratio > 100 && memory == -1 && (timeStart + durationMark - millis() < durationDot * (look_ahead_memory_ratio - 100) / 100)) {
        // memory paddle LR
	if (paddleL == LOW && paddleR == LOW) {
          if (lastPaddle == R) {
            memory = L;
            paddleL_pressed = 0;
          } else {
            memory = R;
            paddleR_pressed = 0;
          }
	} else if (paddleL == LOW) {
          memory = L;
          paddleL_pressed = 0;
	} else if (paddleR == LOW) {
          memory = R;
          paddleR_pressed = 0;
	}
      }
#endif
    } else
#endif
    if (keyerType == keyIambicB || ((keyerType == keySuper) && (millis() - timeStart >= durationDot))) {
      if (memory == -1 && paddleL == LOW && lastPaddle == R) {
        memory = L;
      } else if (memory == -1 && paddleR == LOW && lastPaddle == L) {
        memory = R;
      }
#ifdef OPT_LOOK_AHEAD_MEMORY
      else if (lookAheadMemory && look_ahead_memory_ratio > 100 && memory == -1 && (timeStart + durationMark - millis() < durationDot * (look_ahead_memory_ratio - 100) / 100)) {
        // memory paddle LR
	if (paddleL == LOW && paddleR == LOW) {
          if (lastPaddle == R) {
            memory = L;
          } else {
            memory = R;
          }
	} else if (paddleL == LOW) {
          memory = L;
	} else if (paddleR == LOW) {
          memory = R;
	}
      }
#endif
    }
    // ending of mark
    if (millis() - timeStart >= durationMark) {
#ifdef OPT_WORD_SPACE
      durationSpace = endingChar ? durationDot * charSpace : durationDot;
#else
      durationSpace = endingChar ? durationDot * 3 : durationDot;
#endif
      durationMark = 0;
      endingChar = 0;
#if 0
      noTone(pinBz);
#else
      tone_off(pinBz);
#endif

      timeStart = millis();
#if 0
      digitalWrite(pinLed, LOW);
#endif
      key = UP;
#ifdef OPT_TIMING_DEBUG
      serial_timing(",,,,,t1-U");
#endif
    }
    // key up straight key
    if (keyerType == keyStraight) {
#ifdef MOD_MISC
      if (paddleL == HIGH && (pos_read == 0) && (millis() - timeStart >= durationDot)) { /* pos_read condition prevents from all-dots message output */
#else
      if (paddleL == HIGH && (millis() - timeStart >= durationDot)) {
#endif
        durationSpace = durationDot;
        durationMark = 0;
#if 0
        noTone(pinBz);
#else
        tone_off(pinBz);
#endif
        timeStart = millis();
#if 0
        digitalWrite(pinLed, LOW);
#endif
        key = UP;
#ifdef OPT_TIMING_DEBUG
        serial_timing(",,,,,t2-U");
#endif
      }
    }
  }
#if defined(OPT_RUNNER) || defined(OPT_INVADER_SOUND)
  /*
   * to avoid conflict among tone()
   */
  if (runner_beep != 0 && durationMark == 0 && filteredstate == LOW) {
    /* intending no impact on sidetone and highduration */
    switch (runner_beep) {
#ifdef OPT_RUNNER
    case RUNNER_HIT:
      tone_hit();
      break;
    case RUNNER_MISS:
      tone_miss();
      break;
#endif
#ifdef OPT_INVADER_SOUND
    case INVADER_BEEP:
      beep();
      break;
    case INVADER_ERRBEEP:
      errBeep();
      break;
#endif
    }
    runner_beep = 0;
  }
#endif /* defined(OPT_RUNNER) || defined(OPT_INVADER_SOUND) */
#ifdef OPT_MEMORY_CLICK
  if (timeEnd && timeEnd <= millis()) {
    memory_click_stop();
  }
#endif

#ifdef OPT_VIBRATION
  if (vibration_end > 0 && vibration_end <= millis()) {
    stopVibration();
    vibration_end = 0;
  }
#endif
}

#if 0
void readButtonValue() {
  int sw[4];

  for (int i = 0; i < 4; i++) {
    sw[i] = digitalRead(pinBtn[i]);
    // button was pressed
    if (sw[i] == LOW && timeStartSw[i] == 0) {
#ifdef OPT_BETTER_SRAND
      if (first_srand) {
        first_srand = 0;
        srand(millis()); /* seed for rand() */
      }
#endif
      //digitalWrite(pinLed, HIGH);
      timeStartSw[i] = millis();
      beep();
      // button was released
    } else if (sw[i] == HIGH && timeStartSw[i] != 0) {
      //digitalWrite(pinLed, LOW);
      if (millis() - timeStartSw[i] >= durationLongPress)
        fireLongFunction(i); // long press
      else
        fireFunction(i); // short press
      timeStartSw[i] = 0;
      // detect long press
    } else if (millis() - timeStartSw[i] >= durationLongPress && millis() - timeStartSw[i] < durationLongPress + 10) {
      //digitalWrite(pinLed, LOW);
      beep();
#ifdef OPT_RUNNER
      runner_mode = 0;
    } else if (millis() - timeStartSw[i] >= durationLongPress*2 && millis() - timeStartSw[i] < durationLongPress*2 + 10) { /* much longer press */
      //digitalWrite(pinLed, LOW);
      tone_runner_mode();
      runner_mode = 1;
      runner_count = 0;
#endif
    }
  }
}
#endif

#if 0
void fireFunction(int no) {
  Serial.println();
  Serial.print(F("Press="));
  Serial.println(no + 1);

#if 1 // XXX stop side tone
  noTone(pinBz);
#endif
#ifdef OPT_RUNNER
  runner_stop(); /* pushing button to disable runner */
#endif
  // clear preset messages
  if (pos_read > 0) {
    clearPresetMessage();
    clearBeacon();
    return;
  }
  // stop game
  if (restInvader >= 0) {
    restInvader = 0;
    return;
  }

  // play preset messages
  lcd.setCursor(0, 0);
  lcd.print(F("Msg"));
  lcd.print(no + 1);
  lcd.print(F("    "));
  setPresetMessage(no);
  printascii(SPACE);
  IsSendingPresetMessages = true;
}

void fireLongFunction(int no) {
  Serial.println();
  Serial.print(F("LongPress="));
  Serial.println(no + 1);

#if 1 // XXX stop side tone
  noTone(pinBz);
#endif
#ifdef OPT_RUNNER
  runner_stop(); /* pushing button to disable runner */
#endif
  // clear preset messages
  if (pos_read > 0) {
    clearPresetMessage();
    clearBeacon();
    return;
  }
  // stop game
  if (restInvader >= 0) {
    restInvader = 0;
    return;
  }

  int pressedSw;
  long lasttime = millis();
  switch (no) {
    case 0: // beacon cq
      setPresetMessage(0);
      printascii(SPACE);
      timeCqInterval = defCqInterval;
      lcd.setCursor(0, 0);
      lcd.print(F("Beacon"));
      lcd.print(defCqTimes);
      CqTimes = defCqTimes;
      break;
    case 1: // recv training mode
      lcd.setCursor(0, 0);
      lcd.print(F("Training"));
      lcd.setCursor(0, 1);
      lcd.print(F("A "));
      lcd.write(0b10110001);
#ifdef OPT_RUNNER
      if (runner_mode) {
        lcd.print(F(" r "));
      } else {
        lcd.print(F(" c "));
      }
#else
      lcd.print(F(" c "));
#endif
      lcd.write(0); // invader
      lcd.write(1); // invader
      // wait for the selection
      lcd.setCursor(5, 0);
      for (;;) {
        lcd.setCursor(6, 1);
        if ((millis() - lasttime) >= 1000) {
          lcd.write((millis() / 1000) % 3 ? 0 : 2);
          lcd.write((millis() / 1000) % 2 ? 1 : 3);
          lasttime = millis();
        }
        if (digitalRead(pinSw1) == LOW) {
          pressedSw = pinSw1;
          lcd.setCursor(0, 0);
          lcd.print(F("5x10 ABC"));
          break;
        } else if (digitalRead(pinSw2) == LOW) {
          pressedSw = pinSw2;
          lcd.setCursor(0, 0);
          lcd.print(F("5x10"));
          lcd.write(0b11011100); // wabun
          lcd.write(0b11001100);
          lcd.write(0b11011110);
          lcd.write(0b11011101);
          break;
        } else if (digitalRead(pinSw3) == LOW) {
          pressedSw = pinSw3;
          lcd.setCursor(0, 0);
#ifdef OPT_RUNNER
          lcdIndex = 0;
          if (runner_mode) {
            lcd.print(F("Runner  "));
	  } else {
            lcd.print(F("10x CALL"));
	  }
#else
          lcd.print(F("10x CALL"));
#endif
          break;
        } else if (digitalRead(pinSw4) == LOW) {
          pressedSw = pinSw4;
#ifdef OPT_INVADER_SOUND
          if (invaderSound) {
            beep();
	  }
#endif
          invaderAnimation();
          break;
        }
      }
      lcd.setCursor(0, 1);
      lcd.print(F("        "));
#ifdef OPT_INVADER_SOUND
     if (!invaderSound) {
#endif
      beep();
      delay(200);
#ifdef OPT_INVADER_SOUND
     }
#endif
      for (;;) {
        if (digitalRead(pressedSw) == HIGH) break;
      }
      trainingMode(pressedSw);
      break;
    case 2: // record messages
      //ToDo: save original messages on EEPROM
      Serial.println(F("recording mode..."));
#ifdef OPT_PARAM_EEPROM
      param_eeprom_save();
      lcd.setCursor(0, 0);
      lcd.print(F("Saved   "));
      tone_eeprom();
#endif
      break;
    case 3: // setting
      Serial.println(F("setting mode..."));
      editSetting();
      break;
    default:
      Serial.println(F("default"));
      break;
  }
}
#endif

// recv training mode (abc, kana, ja-callsign, invader)
void trainingMode(int pressedSw) {
  int i, j, k = 0;
#if 0
  if (pressedSw == pinSw1) {
    // 5x10 random letters ABC
    lang = langAbc;
    for (i = 0; i < 10; i++) {
      for (j = 0; j < 5; j++) {
        presetMessage[k] = CharAbc[rand() / (RAND_MAX / 26 + 1)] ;
        k++;
      }
      presetMessage[k] = ' ';
      k++;
    }
  } else if (pressedSw == pinSw2) {
    // 5x10 random letters KANA
    lang = langWabun;
    for (i = 0; i < 10; i++) {
      for (j = 0; j < 5; j++) {
        presetMessage[k] = CharWabun[rand() / (RAND_MAX / 48 + 1)] ;
#if 0 /* XXX debug (made wabun same to abc) */
        Serial.print(presetMessage[k]);
        Serial.print(F(","));
#endif
        k++;
      }
#if 0 /* XXX debug (made wabun same to abc) */
      Serial.println();
#endif
      presetMessage[k] = ' ';
      k++;
    }
  } else if (pressedSw == pinSw3) {
#endif
    // 10 random callsigns JA
    lang = langAbc;
    for (i = 0; i < 10; i++) {
#ifdef MOD_MISC
#ifdef OPT_RUNNER
     int nr = 0;
     if (runner_mode == 0 /*rand() / (RAND_MAX / 2 + 1) == 0*/) { /* contest number */
      runner_repeat = 1;
      int ch;
      if (rand() / (RAND_MAX / 2 + 1) == 0) {
        ch = 'M';
      } else if (rand() / (RAND_MAX / 2 + 1) == 0) {
        ch = 'H';
      } else {
        ch = 'L';
      }

      nr = 1;
      presetMessage[k] = '0';
      k++;
      presetMessage[k] = '0' + rand() / (RAND_MAX / 10 + 1);
      k++;

      if (rand() / (RAND_MAX / 2 + 1) == 0) { /* JCC/JCG */
        /* JCC */
        presetMessage[k] = '0';
        k++;
        presetMessage[k] = '0' + rand() / (RAND_MAX / 15 + 1);
        k++;

        if (rand() / (RAND_MAX / 2 + 1) == 0) {
          presetMessage[k] = '0';
          k++;
          presetMessage[k] = '0' + rand() / (RAND_MAX / 15 + 1);
          k++;

          presetMessage[k] = ch;
          k++;

          runner_call_len = 7;
        } else {
          presetMessage[k] = ch;
          k++;

          runner_call_len = 5;
        }
      } else {
        /* JCG */
        presetMessage[k] = '0';
                k++;
        presetMessage[k] = '0';
        k++;
        presetMessage[k] = '0' + rand() / (RAND_MAX / 15 + 1);
        k++;

        presetMessage[k] = ch;
        k++;

        runner_call_len = 6;
      }

      int j;
      for (j = 0 ; j < k ; j++) {
        if (presetMessage[j] == '0')
          presetMessage[j] = 'T';
        else if (presetMessage[j] >= '9' && presetMessage[j] <= '>')
          presetMessage[j] = 'N';
      }
     } else
#endif /* OPT_RUNNER */
     if (rand() / (RAND_MAX / 4 + 1) == 0) { /* begin with 7/8 */
      presetMessage[k] = '7' + rand() / (RAND_MAX / 2 + 1); /* 7-8 */
      k++;
      presetMessage[k] = 'J' + rand() / (RAND_MAX / 5 + 1); /* J-N */
      k++;
     } else { /* begin with J */
#endif /* MOD_MISC */
      presetMessage[k] = 'J';
      k++;
      presetMessage[k] = 'A' + rand() / (RAND_MAX / 17 + 1) ; /* JA, JD-JS */
      if (presetMessage[k] == 'B' || presetMessage[k] == 'C') {
        presetMessage[k] += 16;
      }
      k++;
#ifdef MOD_MISC
     } /* end of if 7/8 or J */
#endif
#ifndef OPT_RUNNER /* !OPT_RUNNDER */
      if (presetMessage[k - 1] == 'S') {
        presetMessage[k] = '6';
      } else if (presetMessage[k - 1] == 'D') {
        presetMessage[k] = '1';
      } else {
#endif
        presetMessage[k] = '0' + rand() / (RAND_MAX / 10 + 1) ;
#ifndef OPT_RUNNER /* !OPT_RUNNDER */
      }
#endif
      k++;
#ifdef OPT_RUNNER
      if (nr == 1) {
        k--;
        presetMessage[k] = 0x00;
      } else
      if (presetMessage[k - 3] == '7') {
	if (presetMessage[k - 2] >= 'K' && presetMessage[k - 2] <= 'N') { // omitting 7{K-N}{0|5-9}
	  presetMessage[k - 1] = '1' + rand() / (RAND_MAX / 4 + 1) ; /* 1-4 */
	}
      } else if (presetMessage[k - 3] == '8') {
	if (presetMessage[k - 2] == 'K') { // omitting 8K
	  presetMessage[k - 2] = 'J';
	} else if (presetMessage[k - 2] == 'L') { // omitting 8L
	  presetMessage[k - 2] = 'N';
	}
      } else if (presetMessage[k - 2] == 'D') { // omitting JD{0|2-9}
	presetMessage[k - 1] = '1';
      }
      if (nr == 0)
#endif
      for (j = 0; j < 3; j++) {
        presetMessage[k] = 'A' + rand() / (RAND_MAX / 26 + 1) ;
        k++;
      }
#ifdef OPT_RUNNER
      if (runner_mode) { /* runner */
        int idx;

	running = 1;
	runner_ptr = 0;

        if (nr == 1)
          ; /* fall through */
        else
        if (rand() / (RAND_MAX / 5 + 1) == 0) {
          presetMessage[k] = '/'; /* stroke */
          k++;
          presetMessage[k] = '0' + rand() / (RAND_MAX / 10 + 1) ;
          k++;
	  runner_call_len = 8;
	} else {
	  runner_call_len = 6;
	}

	for (idx = 0 ; idx < runner_call_len ; idx++) {
	  runner_call[idx] = presetMessage[k - runner_call_len + idx];
	}

        break;
      }
#endif
      presetMessage[k] = ' ';
      k++;
    }
#if 0
  } else if (pressedSw == pinSw4) {
    // morse-invader
    SetupMorseInvader();
  }
#endif
}

// translate cw code to ascii //
void decode() {
  int i, j, k;

#ifdef MOD_MISC
  if (code == 1) // there is no decoded characters
    return;
#endif

  if (lang == langAbc) {
    if (code == 0b01000101) { // [VA]
      printascii(0b01011011);
      printascii(0b01010110);
      printascii(0b01000001);
      printascii(0b01011101);
    } else if (code == 0b00101000) { // [AS]
      printascii(0b01011011);
      printascii(0b01000001);
      printascii(0b01010011);
      printascii(0b01011101);
    } else if (code == 0b01100111) { // hore
      printascii(0b01011011);
      printascii(0b11001110);
      printascii(0b11011010);
      printascii(0b01011101);
      lang = langWabun;
      Serial.println(F("(switching to Wabun)"));
    } else {
      for (i = 0; i < sizeof(CharAbc) / sizeof(unsigned char); i++) {
        if (code == BinAbc[i]) {
          if (restInvader < 0) {
            printascii(CharAbc[i]);
#ifdef OPT_TIMING_DEBUG
            Serial.println();
#endif
#ifdef OPT_RUNNER
            if (running == 1) {
              if (code == 0b01001100) { /* '?' */
	        int idx;
		clearPresetMessage();
		for (idx = 0 ; idx < runner_call_len ; idx++) {
		  presetMessage[idx] = runner_call[idx];
		}
                runner_retry++;
                // HARD class
                // retrying 2 attempts (total 3 calls) are allowed.
                // 3rd retry (total 4th attempts) becomes 1 penalty,
                // and runner_accel will be immediately decreaased (slowed).
		if (runner_retry > 2 && runner_accel > 0) {
#if 1
                  if (runner_type == RUNNER_TYPE_HARD) {
#endif
		   runner_accel--; // '?' becomes 5% slower
		   runner_retry = 0;
#if 1
                  }
#endif
		}
                accel_durationDot(); // accelerated speed for callsigns
		runner_ptr = 0;
                print_runner_count();
		//printascii(SPACE);
              } else if (CharAbc[i] == runner_call[runner_ptr]) {
	        if (++runner_ptr == runner_call_len) {
                  // increment runner_accel every after 1-QSO
                  // if there were misses and retries, they have already decreased runner_accel at this moment.
                  runner_accel++;

                  runner_count++;
                  if (runner_accel >= HIGH_RUNNER_ACCEL) {
                    tone_invader_clear();
                  } else {
                    tone_hit();
                  }
                  print_runner_count();

		  accel_durationDot(); // accelerated speed for callsigns
		  runner_miss = 0;
                  runner_retry = 0;
#if 0
		  delay(1000);
#endif
		  printascii(SPACE);
                  trainingMode(pinSw3);
		}
	      } else {
	        //tone_miss();
		runner_beep = RUNNER_MISS;
		runner_miss++;
#if 1
                // HARD and NORMAL class
                // 3-miss becomes penalty.
                // Up to 2 miss is allowed.
                if (runner_type != RUNNER_TYPE_EASY) {
                  if (runner_miss > 2 && runner_accel > 0) {
                    runner_accel--; // miss(es) becomes 5% slower
                    runner_miss = 0;
                  }
                }
#endif
		runner_ptr = 0;
		printascii(SPACE);
	      }
	    }
#endif
#if 1
          }
#else
          } else { // Invader
            gameStarted = 1;
            lcd.setCursor(0, 1);
            lcd.write(CharAbc[i]);
            lcd.noCursor();
            for (j = 1; j < colums; j++) {
              if (line1[j] == CharAbc[i]) { // hit
                gamePts += (1 + combo);
                if (combo < 4) combo++;
                lcd.write(SPACE);
#ifdef OPT_INVADER_SOUND
                if (invaderSound) {
		  runner_beep = INVADER_BEEP;
		}
#endif
                for (k = j; k > 1; k--) {
                  line1[k] = line1[k - 1];
                  lcd.setCursor(k, 1);
                  lcd.write(line1[k]);
                }
                break;
              }
            }
#ifdef OPT_INVADER_SOUND
            if (j == colums) {
	      combo = 0; // wrong
	      if (invaderSound) {
                runner_beep = INVADER_ERRBEEP;
	      }
	    }
#else
            if (j == colums) combo = 0; // wrong
#endif
          }
#endif /* invader */
          break;
        }
      }
#ifdef OPT_INVADER_SOUND
      if (restInvader >=0 && i == sizeof(CharAbc) / sizeof(unsigned char)) {
        if (invaderSound) {
          runner_beep = INVADER_ERRBEEP;
	}
      }
#endif
#ifdef OPT_RUNNER
      if (running == 1 && i == sizeof(CharAbc) / sizeof(unsigned char)) {
        //tone_miss();
	runner_beep = RUNNER_MISS;
	runner_miss++;
#if 1
        // HARD and NORMAL class
        // 3-miss becomes penalty.
        // Up to 2 miss is allowed.
        if (runner_type != RUNNER_TYPE_EASY) {
          if (runner_miss > 2 && runner_accel > 0) {
            runner_accel--; // miss(es) becomes 5% slower
            runner_miss = 0;
          }
        }
#endif
	runner_ptr = 0;
	printascii('_');
      }
#endif
    }
  } else if (lang == langWabun) {
    if (code == 0b00100010) { // rata
      printascii(0b01011011);
      printascii(0b11010111);
      printascii(0b11000000);
      printascii(0b01011101);
      lang = langAbc;
      Serial.println(F("(switching to Abc)"));
    } else {
      for (i = 0; i < sizeof(CharWabun) / sizeof(unsigned char); i++) {
        if (code == BinKana[i]) {
          if (restInvader < 0) {
            printascii(CharWabun[i]);
#if 1
          }
#else
          } else { // Invader
            gameStarted = 1;
            lcd.setCursor(0, 1);
            lcd.write(CharWabun[i]);
            lcd.noCursor();
#if 1 /* XXX made wabun same to abc */
            for (j = 1; j < colums; j++) {
              if (line1[j] == CharWabun[i]) { // hit
#else
            for (j = 1; j < colums - 1; j++) {
              if (line1[j] == CharWabun[i]) {
#endif
                gamePts += (1 + combo);
                if (combo < 4) combo++;
                lcd.write(SPACE);
#ifdef OPT_INVADER_SOUND
                if (invaderSound) {
		  runner_beep = INVADER_BEEP;
		}
#endif
                for (k = j; k > 1; k--) {
                  line1[k] = line1[k - 1];
                  lcd.setCursor(k, 1);
                  lcd.write(line1[k]);
                }
                break;
              }
            }
#ifdef OPT_INVADER_SOUND /* made wabun same to abc */
            if (j == colums) {
	      combo = 0; // wrong
              if (invaderSound) {
                runner_beep = INVADER_ERRBEEP;
	      }
	    }
#else
            if (j == colums) combo = 0; // wrong
#endif
          }
#endif /* invader */
          break;
        }
      }
#ifdef OPT_INVADER_SOUND
      if (restInvader >=0 && i == sizeof(CharWabun) / sizeof(unsigned char)) {
        if (invaderSound) {
          runner_beep = INVADER_ERRBEEP;
	}
      }
#endif
    }
  }
}

// print the ascii code to the lcd one a time
void printascii(int asciinumber) {
  // Do nothing when playing the game
  if (restInvader >= 0) return;

  static int lastLetter;
  if (lastLetter == SPACE && asciinumber == SPACE) return;
  lastLetter = asciinumber;

#ifdef OPT_PUG_MORSE
  Serial.printf("%c", asciinumber);
#endif

#if 0
#ifndef OPT_NO_SERIAL
#ifdef OPT_LCD_DISP
 if (lcdDisp)
#endif
  Serial.write(asciinumber); // Monitor characters
#endif

#ifdef OPT_LCD_DISP
  if (lcdDisp == 0)
    return;
#endif

#ifdef SCROLL_ROW
  if (lcdIndex > colums - 1) {
    lcdIndex = 0;
    for (int i = 0; i <= colums - 1 ; i++) {
      lcd.setCursor(i, rows - 2);
      lcd.write(line1[i]);
      lcd.setCursor(i, rows - 1);
      lcd.write(SPACE);
    }
  }
  line1[lcdIndex] = asciinumber;
  lcd.setCursor(lcdIndex, rows - 1);
  lcd.write(asciinumber);
  lcdIndex += 1;
#endif

#ifndef SCROLL_ROW
  if (lcdIndex > colums - 1) {
    lcdIndex = 7;
    lcd.setCursor(0, rows - 2);
    for (int i = 0; i < colums - 1 ; i++) {
      lcd.write(line2[i + 1]);
      line2[i] = line2[i + 1];
    }
    line2[7] = line1[0];
    lcd.write(line2[7]);
    lcd.setCursor(0, rows - 1);
    for (int i = 0; i < colums - 1 ; i++) {
      lcd.write(line1[i + 1]);
      line1[i] = line1[i + 1];
    }
    //lcd.write(SPACE);
  }
  line1[lcdIndex] = asciinumber;
  lcd.setCursor(lcdIndex, rows - 1);
  lcd.write(asciinumber);
  lcdIndex += 1;
#endif SCROLL_ROW
#endif
}

#if 0
// update the upper line with the speed
void updateinfolinelcd() {
#ifdef MOD_MISC
  static int oldWpm = INIT_DECODE_WPM; // to surpress initial "wpm" (for initial "WPM" displayed, instead)
#else
  static int oldWpm;
#endif
  if (oldWpm == wpm) return;
  lcd.home();
  if (wpm < 10) lcd.print(F("0"));
  lcd.print(wpm);
  lcd.print(F("wpm"));
  oldWpm = wpm;
}
#endif

#if 0
void editSetting() {
  lcd.setCursor(0, 1);
  lcd.write(0b00000110);
  lcd.print(F("<    >"));
  lcd.write(0b11110111);
#ifdef OPT_LAST_MENUINDEX
  static int menuIndex = -1; // memory the last menuIndex
  int right_after_invoked = 1;
  int backward = 0;
#define DURATION_SEMILONG_PRESS 300
#else
  int menuIndex = -1;
#endif

  lcd.setCursor(0, 0);
  lcd.print(F("Set Mode"));
  for (;;) {
    if (digitalRead(pinSw4) == LOW) break;
#ifdef OPT_LAST_MENUINDEX
    if (backward || (menuIndex != -1 && right_after_invoked) || digitalRead(pinSw1) == LOW) {
      if (digitalRead(pinSw1) == LOW) {
	backward = 0;
	timeStart = millis();
	while (digitalRead(pinSw1) == LOW) {
	  if (millis() - timeStart >= DURATION_SEMILONG_PRESS) {
	    backward = 1;
	    buzzer(pinBz, beepFreq * 2, 20);
	    break;
	  }
	}
	while (digitalRead(pinSw1) == LOW)
	  /* wait until pinSw1 becomes HIGH */;
      }
      if (menuIndex == -1 || !right_after_invoked) {
	beep();
	if (backward) {
	  backward = 0;
	  menuIndex--;
	  if (menuIndex < 0) menuIndex = MaxMenu -1;
	} else {
	  menuIndex++;
	  if (menuIndex == MaxMenu) menuIndex = 0;
	}
      } else {
	right_after_invoked = 0;
	// menuIndex is not incremented
      }
#else
    if (digitalRead(pinSw1) == LOW) {
      beep();
      menuIndex++;
      if (menuIndex == MaxMenu) menuIndex = 0;
#endif
      delay(200);
      lcd.setCursor(0, 0);
      lcd.print(setMenu[menuIndex]);
      if (menuIndex == 0) lcd.write(0b10110001); // a
      lcd.setCursor(2, 1);
      if (menuIndex == 0) {
        if (lang == langAbc)
          lcd.print(F("ABC "));
        else {
          lcd.write(0b11011100); // wabun
          lcd.write(0b11001100);
          lcd.write(0b11011110);
          lcd.write(0b11011101);
        }
      } else if (menuIndex == 1) {
        if (paddleRvs)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 2) {
        lcd.print(keyName[keyerType]);
      } else if (menuIndex == 3) {
        lcd.print(wait);
      } else if (menuIndex == 4) {
        if (sidetone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 5) {
        lcd.print(defCqInterval);
      } else if (menuIndex == 6) {
        if (defCqTimes < 10) lcd.print(F("  "));
        else lcd.print(F(" "));
        lcd.print(defCqTimes);
        lcd.print(F(" "));
#ifdef OPT_LCD_DISP
      } else if (menuIndex == menuIndexLcdDisp) {
        if (lcdDisp)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_TONE_FREQ
      } else if (menuIndex == menuIndexToneFreq) {
        lcd.print(toneFreq);
	if (toneFreq < 1000)
          lcd.print(F(" "));
#endif
#ifdef OPT_WORD_SPACE
      } else if (menuIndex == menuIndexWordSpace) {
        lcd.print(wordSpace);
	if (wordSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
      } else if (menuIndex == menuIndexCharSpace) {
        lcd.print(charSpace);
	if (charSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
#endif
#ifdef OPT_OFF_DELAY
      } else if (menuIndex == menuIndexOffDelay) {
        if (offDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOffDelayDot) {
        lcd.print(off_timer_ratio_dot);
	if (off_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOffDelayDash) {
        lcd.print(off_timer_ratio_dash);
	if (off_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#ifdef OPT_REPEAT_ADD
      } else if (menuIndex == menuIndexOffRepeatAdd) {
        lcd.print(off_repeat_add);
	if (off_repeat_add < 10)
          lcd.print(F(" "));
	if (off_repeat_add < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#endif
#ifdef OPT_ON_DELAY
      } else if (menuIndex == menuIndexOnDelay) {
        if (onDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOnDelayDot) {
        lcd.print(on_timer_ratio_dot);
	if (on_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOnDelayDash) {
        lcd.print(on_timer_ratio_dash);
	if (on_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
      } else if (menuIndex == menuIndexLookAheadMemory) {
        if (lookAheadMemory)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexLookAheadMemoryRatio) {
        lcd.print(look_ahead_memory_ratio);
	if (look_ahead_memory_ratio < 10)
          lcd.print(F(" "));
	if (look_ahead_memory_ratio < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_TWO_TONE
      } else if (menuIndex == menuIndexTwoTone) {
        if (twoTone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_MEMORY_CLICK
      } else if (menuIndex == menuIndexMemoryClick) {
        if (memoryClick)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_AVOID_CHATTERING
      } else if (menuIndex == menuIndexAvoidChattering) {
        if (avoidChattering)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAvoidChatteringCount) {
        lcd.print(avoidChatteringCount);
	if (avoidChatteringCount < 10)
	  lcd.print(F(" "));
	if (avoidChatteringCount < 100)
	  lcd.print(F(" "));
	lcd.print(F(" "));
#endif
#ifdef OPT_AUTO_KEYERTYPE
      } else if (menuIndex == menuIndexAutoKeyertype) {
        if (autoKeyerType)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAutoKeyertypeElekey) {
        lcd.print(keyName[autoKeyerTypeElekey]);
#endif
      }
    }
    if (digitalRead(pinSw2) == LOW) {
      beep();
      delay(200);
      lcd.setCursor(2, 1);
      if (menuIndex == 0) {
        if (lang == langWabun) {
          lang = langAbc;
          lcd.print(F("ABC "));
        } else {
          lang = langWabun;
          lcd.write(0b11011100); // wabun
          lcd.write(0b11001100);
          lcd.write(0b11011110);
          lcd.write(0b11011101);
        }
      } else if (menuIndex == 1) {
        paddleRvs = !paddleRvs;
        if (paddleRvs)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 2) {
        keyerType -= 1;
#ifdef OPT_BETTER_IAMBIC_A
        if (keyerType < 0) keyerType = 0;
#else
        if (keyerType < 1) keyerType = 1;
#endif
        lcd.print(keyName[keyerType]);
      } else if (menuIndex == 3) {
        wait -= 0.1;
        if (wait < 2.0) wait = 3.0;
        lcd.print(wait);
      } else if (menuIndex == 4) {
        sidetone = !sidetone;
        if (sidetone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 5) {
        defCqInterval -= 200;
        if (defCqInterval < 1000) defCqInterval = 1000;
        lcd.print(defCqInterval);
      } else if (menuIndex == 6) {
        defCqTimes--;
        if (defCqTimes < 3) defCqTimes = 3;
        if (defCqTimes < 10) lcd.print(F("  "));
        else lcd.print(F(" "));
        lcd.print(defCqTimes);
        lcd.print(F(" "));
#ifdef OPT_LCD_DISP
      } else if (menuIndex == menuIndexLcdDisp) {
        lcdDisp = !lcdDisp;
        if (lcdDisp)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_TONE_FREQ
      } else if (menuIndex == menuIndexToneFreq) {
	if (toneFreqIndex > 0)
	  toneFreqIndex--;
	toneFreq = defToneFreq[toneFreqIndex];
	buzzer(pinBz, toneFreq, 100);
	delay(100);
        lcd.print(toneFreq);
	if (toneFreq < 1000)
          lcd.print(F(" "));
#endif
#ifdef OPT_WORD_SPACE
      } else if (menuIndex == menuIndexWordSpace) {
	if (wordSpace > 1)
	  wordSpace--;
        lcd.print(wordSpace);
	if (wordSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
      } else if (menuIndex == menuIndexCharSpace) {
	if (charSpace > 1)
	  charSpace--;
        lcd.print(charSpace);
	if (charSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
#endif
#ifdef OPT_OFF_DELAY
      } else if (menuIndex == menuIndexOffDelay) {
        offDelay = !offDelay;
        if (offDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOffDelayDot) {
	if (off_timer_ratio_dot > 0)
	  off_timer_ratio_dot -= 5;
        lcd.print(off_timer_ratio_dot);
	if (off_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOffDelayDash) {
	if (off_timer_ratio_dash > 0)
	  off_timer_ratio_dash -= 5;
        lcd.print(off_timer_ratio_dash);
	if (off_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#ifdef OPT_REPEAT_ADD
      } else if (menuIndex == menuIndexOffRepeatAdd) {
	if (off_repeat_add > 0)
	  off_repeat_add -= 1;
	if (off_repeat_add < 0)
	  off_repeat_add = 0;
        lcd.print(off_repeat_add);
	if (off_repeat_add < 10)
          lcd.print(F(" "));
	if (off_repeat_add < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#endif
#ifdef OPT_ON_DELAY
      } else if (menuIndex == menuIndexOnDelay) {
        onDelay = !onDelay;
        if (onDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOnDelayDot) {
	if (on_timer_ratio_dot > 0)
	  on_timer_ratio_dot -= 5;
        lcd.print(on_timer_ratio_dot);
	if (on_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOnDelayDash) {
	if (on_timer_ratio_dash > 0)
	  on_timer_ratio_dash -= 5;
        lcd.print(on_timer_ratio_dash);
	if (on_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
      } else if (menuIndex == menuIndexLookAheadMemory) {
        lookAheadMemory = !lookAheadMemory;
        if (lookAheadMemory)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexLookAheadMemoryRatio) {
	if (look_ahead_memory_ratio > 0)
	  look_ahead_memory_ratio -= 5;
        lcd.print(look_ahead_memory_ratio);
	if (look_ahead_memory_ratio < 10)
          lcd.print(F(" "));
	if (look_ahead_memory_ratio < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_TWO_TONE
      } else if (menuIndex == menuIndexTwoTone) {
	twoTone = !twoTone;
        if (twoTone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_MEMORY_CLICK
      } else if (menuIndex == menuIndexMemoryClick) {
        memoryClick = !memoryClick;
        if (memoryClick)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_AVOID_CHATTERING
      } else if (menuIndex == menuIndexAvoidChattering) {
	avoidChattering = !avoidChattering;
        if (avoidChattering)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAvoidChatteringCount) {
	if (avoidChatteringCount > 5)
	  avoidChatteringCount -= 5;
        lcd.print(avoidChatteringCount);
	if (avoidChatteringCount < 10)
	  lcd.print(F(" "));
	if (avoidChatteringCount < 100)
	  lcd.print(F(" "));
	lcd.print(F(" "));
#endif
#ifdef OPT_AUTO_KEYERTYPE
      } else if (menuIndex == menuIndexAutoKeyertype) {
	autoKeyerType = !autoKeyerType;
        if (autoKeyerType)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAutoKeyertypeElekey) {
        if (autoKeyerTypeElekey > 0) {
	  autoKeyerTypeElekey--;
	}
        lcd.print(keyName[autoKeyerTypeElekey]);
#endif
      }
    }
    if (digitalRead(pinSw3) == LOW) {
      beep();
      delay(200);
      lcd.setCursor(2, 1);
      if (menuIndex == 0) {
        if (lang == langWabun) {
          lang = langAbc;
          lcd.print(F("ABC "));
        } else {
          lang = langWabun;
          lcd.write(0b11011100); // wabun
          lcd.write(0b11001100);
          lcd.write(0b11011110);
          lcd.write(0b11011101);
        }
      } else if (menuIndex == 1) {
        paddleRvs = !paddleRvs;
        if (paddleRvs)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 2) {
        keyerType += 1;
        if (keyerType > 3) keyerType = 3;
        lcd.print(keyName[keyerType]);
      } else if (menuIndex == 3) {
        wait += 0.1;
        if (wait > 4.0) wait = 4.0;
        lcd.print(wait);
      } else if (menuIndex == 4) {
        sidetone = !sidetone;
        if (sidetone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == 5) {
        defCqInterval += 200;
        if (defCqInterval > 9000) defCqInterval = 9000;
        lcd.print(defCqInterval);
      } else if (menuIndex == 6) {
        defCqTimes++;
        if (defCqTimes > 30) defCqTimes = 30;
        if (defCqTimes < 10) lcd.print(F("  "));
        else lcd.print(F(" "));
        lcd.print(defCqTimes);
        lcd.print(F(" "));
#ifdef OPT_LCD_DISP
      } else if (menuIndex == menuIndexLcdDisp) {
        lcdDisp = !lcdDisp;
        if (lcdDisp)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_TONE_FREQ
      } else if (menuIndex == menuIndexToneFreq) {
	if (toneFreqIndex < (sizeof(defToneFreq) / sizeof(int)) - 1)
	  toneFreqIndex++;
	toneFreq = defToneFreq[toneFreqIndex];
	buzzer(pinBz, toneFreq, 100);
	delay(100);
        lcd.print(toneFreq);
	if (toneFreq < 1000)
          lcd.print(F(" "));
#endif
#ifdef OPT_WORD_SPACE
      } else if (menuIndex == menuIndexWordSpace) {
	if (wordSpace < 10)
	  wordSpace++;
        lcd.print(wordSpace);
	if (wordSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
      } else if (menuIndex == menuIndexCharSpace) {
	if (charSpace < 10)
	  charSpace++;
        lcd.print(charSpace);
	if (charSpace < 10)
          lcd.print(F(" "));
	lcd.print(F("  "));
#endif
#ifdef OPT_OFF_DELAY
      } else if (menuIndex == menuIndexOffDelay) {
        offDelay = !offDelay;
        if (offDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOffDelayDot) {
	if (off_timer_ratio_dot < MAX_DOT_RATIO)
	  off_timer_ratio_dot += 5;
        lcd.print(off_timer_ratio_dot);
	if (off_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOffDelayDash) {
	if (off_timer_ratio_dash < MAX_DASH_RATIO)
	  off_timer_ratio_dash += 5;
        lcd.print(off_timer_ratio_dash);
	if (off_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (off_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#ifdef OPT_REPEAT_ADD
      } else if (menuIndex == menuIndexOffRepeatAdd) {
	if (off_repeat_add < MAX_REPEAT_ADD)
	  off_repeat_add += 1;
        lcd.print(off_repeat_add);
	if (off_repeat_add < 10)
          lcd.print(F(" "));
	if (off_repeat_add < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#endif
#ifdef OPT_ON_DELAY
      } else if (menuIndex == menuIndexOnDelay) {
        onDelay = !onDelay;
        if (onDelay)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexOnDelayDot) {
	if (on_timer_ratio_dot < MAX_DOT_RATIO)
	  on_timer_ratio_dot += 5;
        lcd.print(on_timer_ratio_dot);
	if (on_timer_ratio_dot < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dot < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
      } else if (menuIndex == menuIndexOnDelayDash) {
	if (on_timer_ratio_dash < MAX_DASH_RATIO)
	  on_timer_ratio_dash += 5;
        lcd.print(on_timer_ratio_dash);
	if (on_timer_ratio_dash < 10)
          lcd.print(F(" "));
	if (on_timer_ratio_dash < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_LOOK_AHEAD_MEMORY
      } else if (menuIndex == menuIndexLookAheadMemory) {
        lookAheadMemory = !lookAheadMemory;
        if (lookAheadMemory)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexLookAheadMemoryRatio) {
	if (look_ahead_memory_ratio < MAX_LAM_RATIO)
	  look_ahead_memory_ratio += 5;
        lcd.print(look_ahead_memory_ratio);
	if (look_ahead_memory_ratio < 10)
          lcd.print(F(" "));
	if (look_ahead_memory_ratio < 100)
          lcd.print(F(" "));
        lcd.print(F(" "));
#endif
#ifdef OPT_TWO_TONE
      } else if (menuIndex == menuIndexTwoTone) {
	twoTone = !twoTone;
        if (twoTone)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_MEMORY_CLICK
      } else if (menuIndex == menuIndexMemoryClick) {
        memoryClick = !memoryClick;
        if (memoryClick)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
#endif
#ifdef OPT_AVOID_CHATTERING
      } else if (menuIndex == menuIndexAvoidChattering) {
	avoidChattering = !avoidChattering;
        if (avoidChattering)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAvoidChatteringCount) {
	if (avoidChatteringCount < MAX_CHATTERING_COUNT)
	  avoidChatteringCount += 5;
        lcd.print(avoidChatteringCount);
	if (avoidChatteringCount < 10)
	  lcd.print(F(" "));
	if (avoidChatteringCount < 100)
	  lcd.print(F(" "));
	lcd.print(F(" "));
#endif
#ifdef OPT_AUTO_KEYERTYPE
      } else if (menuIndex == menuIndexAutoKeyertype) {
	autoKeyerType = !autoKeyerType;
        if (autoKeyerType)
          lcd.print(F(" on "));
        else
          lcd.print(F("off "));
      } else if (menuIndex == menuIndexAutoKeyertypeElekey) {
	if (autoKeyerTypeElekey < (sizeof(keyName)/sizeof(char *)-2)) { // except for the last index (straight key)
	  autoKeyerTypeElekey++;
	}
        lcd.print(keyName[autoKeyerTypeElekey]);
#endif
      }
    }
  }

  for (;;) {
    if (digitalRead(pinSw4) == HIGH) break;
  }
  lcd.setCursor(0, 1);
  lcd.print(F("        "));
  beep();
  delay(200);
  setNewWpm(Wpm);
}
#endif

#if 0
void readFromSerial() {
  static bool combineChar = false; // [AR] etc.
  int readChar = 0;
  static int posWriting = 0, bank = -1;
  static bool memoryWriting = false;

  // generator is still working
  if (nextCode != 0x01) return;

  // Read 1 character from serial port
  readChar = Serial.read();
  if (readChar >= 'a' && readChar <= 'z') {
    readChar -= 0x20; // 'a'->'A'
  }
  // Succeeded in reading 1 character from serial port
  if (readChar > 0) {
    //Serial.write(readChar);
    if (IsSendingPresetMessages) {
      // clear preset messages
      if (pos_read > 0) {
        clearPresetMessage();
        clearBeacon();
        return;
      }
      IsSendingPresetMessages = false;
    }
    // stop game
    if (restInvader >= 0) {
      restInvader = 0;
      return;
    }
    if (!memoryWriting && readChar == '[') {
      combineChar = true;
      printascii('[');
    } else if (!memoryWriting && readChar == ']') {
      combineChar = false;
      printascii(']');
    } else if (readChar == '#') {
      memoryWriting = true;
      posWriting = 0;
      bank = -1;
      Serial.println(F("\nwriting mode"));
      Serial.println(F("bank? (1-4)"));
      return;
      // CR or LF
    } else if (readChar == 10 || readChar == 13) {
      return;
    }
    if (memoryWriting) {
      if (bank == -1) { // Start writing to memory
        if (readChar >= '1' && readChar <= '4') {
          bank = readChar - '1';
          Serial.print(F("\nbank = "));
          Serial.println(bank + 1);
          Serial.println(F("type ';' to finish"));
          for (int i = 0; i < 128; i++) EEPROM.write(i + 128 * bank, 0x00);
        } else { // Bank number is wrong
          Serial.println(F("\nbank unknown"));
          memoryWriting = false;
          Serial.print(readChar);
        }
        // Find terminal character
      } else if (readChar == ';' ) {
        Serial.println(F("\nfinish writing"));
        memoryWriting = false;
      } else {
        EEPROM.write(posWriting + 128 * bank, readChar);
        Serial.print(readChar);
        posWriting++;
      }
      return;
    }
  }
  if (readChar > 0 && !memoryWriting) {
    presetMessage[pos_read] = readChar;
  }
}
#endif

void setNextChar() {
  static bool combineChar = false; // [AR] etc.
  int readChar = 0;

  if (nextCode == 0x01) {
    // Get next character from preset message
    readChar = presetMessage[pos_read];
    //Serial.write(readChar);
    //Serial.println(pos_read);
    presetMessage[pos_read] = 0;
    if (pos_read < 126) pos_read++;

    // Preset message has not been prepared
    if (readChar == 0) {
      readChar = -1;
      clearPresetMessage();
      //Serial.println(F("no preset messages"));

#ifdef OPT_RUNNER
      if (runner_repeat == 1) {
        trainingMode(pinSw3);
      }
#endif
#if 0
      // set next beacon message
      if (timeCqInterval > 0) {
        if (timeStartCq == 0) { // 1st time
          timeStartCq = millis();
        } else {
          if ((millis() - timeStartCq) >= timeCqInterval && CqTimes > 1) {
            setPresetMessage(0);
            printascii(SPACE);
            timeStartCq = 0;
            CqTimes--;
            Serial.print(F(" #beacon"));
            Serial.println(CqTimes);
          }
        }
      }
#endif
    }
  }

  // Put a new character into the keyer
  if (readChar != -1 && nextCode == 0x01) {
    int i;
    if (lang == langAbc) {
      if (readChar >= 'a' && readChar <= 'z') {
        readChar -= 0x20;
      }
      for (i = 0; i < sizeof(CharAbc) / sizeof(unsigned char); i++) {
        if (readChar == CharAbc[i]) {
          nextCode = BinAbc[i]; // Convert to morse code
          printascii(CharAbc[i]);
        }
      }
      if (readChar == '[') {
        combineChar = true;
        printascii('[');
      } else if (readChar == ']') {
        combineChar = false;
        printascii(']');
#ifdef OPT_TIMING_DEBUG
        Serial.println();
#endif
      }
      switch (readChar) {
        case ' ':
          nextCode = 0xFF;
          printascii(SPACE);
          break;
      }
#ifdef OPT_RUNNER
      if (running && presetMessage[pos_read] == 0) {
        // space after callsign
        printascii(SPACE);
      }
#endif
    } else if (lang == langWabun) {
      for (i = 0; i < sizeof(CharWabun) / sizeof(unsigned char); i++) {
        if (readChar == CharWabun[i]) {
          nextCode = BinKana[i];
          printascii(CharWabun[i]);
          break;
        }
      }
      if (readChar == '[') {
        combineChar = true;
        printascii('[');
      } else if (readChar == ']') {
        combineChar = false;
        printascii(']');
      }
      switch (readChar) {
        case ' ':
          nextCode = 0xFF;
          printascii(SPACE);
          break;
      }
    }
  }
  unsigned int tmpChar, cnt = 0;

  // send space
  if (nextCode == 0xFF) {
#ifdef OPT_WORD_SPACE
    durationSpace = durationDot * wordSpace;
#else
    durationSpace = durationDot * 7;
#endif
    timeStart = millis();
    nextCode = 0x01;
    // An effective code
  } else if (nextCode >= B00000010) {
    tmpChar = nextCode;//, cnt = 0;
    while (tmpChar != 0x01) {
      tmpChar = tmpChar >> 1;
      cnt++;
    }
#if 1
    tmpChar = nextCode & (0x01 << (cnt - 1));
#else
    tmpChar = nextCode & (0x01 << cnt - 1);
#endif
    //cntSpace = 0;
    if (tmpChar > 0) {
      if (durationSpace > 0) {
        memory = R;
      } else {
        durationMark = durationDot * wait;
#if 0
        digitalWrite(pinLed, HIGH);
#endif
        key = DOWN;
        timeStart = millis();
#if 0
        if (sidetone) tone(pinBz, toneFreq); // side tone
#else
        if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
      }
    } else {
      if (durationSpace > 0) {
        memory = L;
      } else {
        durationMark = durationDot;
#if 0
        digitalWrite(pinLed, HIGH);
#endif
        key = DOWN;
        timeStart = millis();
#if 0
        if (sidetone) tone(pinBz, toneFreq); // side tone
#else
        if (sidetone) tone_on(pinBz, toneFreq); // side tone
#endif
      }
    }
    nextCode = nextCode ^ (0x01 << cnt);
#if 1
    nextCode = nextCode | (0x01 << (cnt - 1));
#else
    nextCode = nextCode | (0x01 << cnt - 1);
#endif
    if (nextCode == 0x01) {
      if (combineChar) {
        endingChar = 0;
      } else {
        endingChar = 1;
      }
    }
  }
  //Serial.print(F("Ready\n"));
}

void clearPresetMessage() {
  for (int i = 0; i < 128; i++) {
    presetMessage[i] = 0x00;
  }
  nextCode = 0x01;
  pos_read = 0;
}

#if 0
void clearBeacon() {
  CqTimes = 0;
  timeStartCq = 0;
  timeCqInterval = 0;
}
#endif

#if 0
// preset message have 4 banks (offset 0-3)
void setPresetMessage(int offset) {
  // read messages from eeprom
  for (int i = 0; i < 128; i++) {
    presetMessage[i] = EEPROM.read(i + 128 * offset);
    //Serial.print(presetMessage[i]);
    //Serial.print(F(","));
  }
  // last colum is language option
  if (presetMessage[127] == langAbc ||
      presetMessage[127] == langAbc) {
    lang = presetMessage[127];
  }
}

void invaderAnimation() {
  lcd.setCursor(0, 0);
  lcd.print(F("Ready..3")); // count down 3s
  lcd.setCursor(5, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  delay(500);
  lcd.setCursor(7, 0);
  lcd.print(F("3"));
  lcd.setCursor(4, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  delay(500);
#ifdef OPT_INVADER_SOUND
  if (invaderSound) {
    beep();
  }
#endif
  lcd.setCursor(7, 0);
  lcd.print(F("2"));
  lcd.setCursor(3, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  lcd.write(1); // invader
  delay(500);
  lcd.setCursor(7, 0);
  lcd.print(F("2"));
  lcd.setCursor(2, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  delay(500);
#ifdef OPT_INVADER_SOUND
  if (invaderSound) {
    beep();
  }
#endif
  lcd.setCursor(7, 0);
  lcd.print(F("1"));
  lcd.setCursor(1, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  delay(500);
  lcd.setCursor(7, 0);
  lcd.print(F("1"));
  lcd.setCursor(0, 1);
  lcd.write(0); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  lcd.write(1); // invader
  lcd.write(SPACE);
  lcd.write(2); // invader
  lcd.write(3); // invader
  delay(500);
#ifdef OPT_INVADER_SOUND
  if (invaderSound) {
    tone_invader_start();
    delay(500);
  }
#endif
  lcd.setCursor(0, 0);
  lcd.print(F("        "));
}

void SetupMorseInvader() {
  int i;
  gamePts = 0;
  restInvader = maxInvader;
  combo = 0;
  lcdIndex = 0;
  gameStarted = 0;

  // clear display and buffers
  lcd.setCursor(0, 1);
  for (i = 0; i < colums; i++) {
    line1[i] = SPACE; // fill with space
    lcd.write(line1[i]); // clear colum
  }
  // keying position
  lcd.setCursor(0, 1);
  lcd.write(0b00010000);
}

void morseInvader() {
  // other functoin is running
  if (restInvader == -1) return;

  // this is refresh interval
  static long lastInvaderTime;
  if ((millis() - lastInvaderTime) <= (1500 / maxInvader * restInvader + 500)) {
    return;
  }

  int i;
  lcd.setCursor(1, 1);

  // move left 1 colum
  for (i = 0; i < colums - 1; i++) {
    line1[i] = line1[i + 1];
    if (i < 1) continue; // estimated code
    lcd.write(line1[i]);
  }

  // still invaders are waiting
  if (restInvader > 0) {
    if (lang == langAbc) { // ABC
      line1[colums - 1] = CharAbc[rand() / (RAND_MAX / 26 + 1)];
    } else if (lang == langWabun) { // Wabun
      line1[colums - 1] = CharWabun[rand() / (RAND_MAX / 48 + 1)];
    }
    // no more invaders
  } else {
    line1[colums - 1] = SPACE; // fill with 1 space
  }
  lcd.write(line1[colums - 1]);

  // display pts
  lcd.setCursor(5, 0);
  if (gamePts < 100) lcd.print(F("0"));
  if (gamePts < 10) lcd.print(F("0"));
  lcd.print(gamePts);

  // Game Over
  if (line1[0] != SPACE) {
    lcd.setCursor(0, 1);
    lcd.print(F("GameOver"));
    lcd.noCursor();
    restInvader = -1;
    lastInvaderTime = 0;
    for (i = 0; i < 5; i++) {
      errBeep();
      delay(50);
    }
    // no letters in line1
  } else if (SPACE == (line1[1] | line1[2] | line1[3] | line1[4] | line1[5] | line1[6] | line1[7])) {
    lcd.setCursor(2, 1);
    lcd.print(F("Clear!"));
#ifdef OPT_INVADER_SOUND
    if (invaderSound) {
      tone_invader_clear();
    }
#endif
    restInvader = -1;
    lastInvaderTime = 0;
    // still invaders are waiting
  } else {
    if (restInvader > 0) restInvader--;
    lastInvaderTime = millis();
    if (!gameStarted) {
      lcd.cursor();
      lcd.setCursor(0, 1);
    }
  }
}
#endif

#ifdef OPT_PUG_MORSE
/*
 * to reconsile succeeding compile errors, undef some macros here
 */
#undef L
#undef R
#endif

/*
 * Copied the following part from Pug Radio
 */

#if 1 /* 1 if enabling Ethernet port */
#define BT_WIFI_ETHER
#endif

#if 1 /* 1 if enabling Radio like BT */
#define BT_RADIO
#endif

#ifdef  BT_WIFI_ETHER
#define FEATURE "(BT-WiFi-Ether)"
#else
#define FEATURE "(BT-WiFi)"
#endif

#define USE_BT_SERIAL

#ifdef USE_BT_SERIAL
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
static int bt_initialized = 0;

//#define DEF_VOL "1.0"
//#define DEF_VOL "0.9"
#define DEF_VOL "0.8" // to prevent sound cracking on D-109

void Serial_printf(const char *format, ...)
{
  va_list va;

  va_start(va, format);
  Serial.vprintf(format, va);
  va_end(va);

  if (bt_initialized) {
    va_start(va, format);
    SerialBT.vprintf(format, va);
    va_end(va);
  }
}

void Serial_write(int ch)
{
  SerialBT.write(ch);
  if (bt_initialized) {
    Serial.write(ch);
  }
}

#else
#define SerialBT Serial
#define Serial_printf  Serial.printf
#define Serial_write  Serial.write
#endif

#define USE_ETHER /* Use W5500 Ethernet module */
//#define BUF_16KB /* Use 16KB buffer */

//#include <A2DPStream.h>

//#define USE_RESAMPLE_BUFFER false
#define HELIX_PCM_CORRECTED

//#define A2DP_BUFFER_COUNT 8 /* A2DP_BUFFER_SIZE(512) * 8 becomes 4KB */
//#define A2DP_BUFFER_COUNT 40 /* A2DP_BUFFER_SIZE(512) * 40 becomes 20KB */
//#define A2DP_BUFFER_COUNT 768 /* A2DP_BUFFER_SIZE(512) * 768 becomes 384KB */

#include "AudioTools.h"
#include "AudioLibs/AudioA2DP.h"
#include "AudioCodecs/CodecMP3Helix.h"

//#include "AudioCodecs/CodecMTS.h"
#include "AudioCodecs/CodecAACHelix.h"
//#include "AudioLibs/HLSStream.h"

const char *urls[] = {
//char *urls[] = {
  //"https://pd.npr.org/anon.npr-mp3/npr/news/newscast.mp3", // NPR News and Talk (44.1kHz/16bit/stereo)
  "https://npr-ice.streamguys1.com/live.mp3", // KRVS-HD3, WKCR-HD3, NPR News and Talk (32kHz/16bit/stereo)

  //"https://war.streamguys1.com:7031/kuaf1", // KUAF
  //"https://vpr.streamguys1.com/vpr96.mp3", // Vermont Public
  //"https://playerservices.streamtheworld.com/api/livestream-redirect/WGVUFM.mp3", // WGVU NPR
  //"https://radio2.wqln.org:8443/", // WQLN NPR
  //"https://wamu.cdnstream1.com/wamu.mp3", // WAMU 88.5 NPR (44.1kHz/16bit/stereo)

  //"https://wgbh-live.streamguys1.com/wgbh", // GBH 89.7 NPR (44.1kHz/16bit/stereo)

  "https://war.streamguys1.com:7785/wuis.mp3", // NPR Illinois 91.9 UIS
  "https://whyy.streamguys1.com/whyy-mp3", // WHYY NPR

  //"https://npr-ice.streamguys1.com/live.mp3", // KRVS-HD3, WKCR-HD3, NPR News and Talk

  "https://wpr-ice.streamguys1.com/wpr-ideas-mp3-64", // WISCONSIN PUBLIC RADIO (44.1kHz/16bit/mono)

  "https://streams.kqed.org/kqedradio", // KQED, 32kbps, mono
  "https://playerservices.streamtheworld.com/api/livestream-redirect/KVPRFM.mp3", // Valley Public Radio (KVPR)

  //"https://streaming.azpm.org/kuaz128.mp3", // NPR 89.1 Arizona Public Media (48kHz,16bit,stereo)

  "https://utulsa.streamguys1.com/KWGSHD3.mp3", // KWGS HD3 BBC (44.1kHz/16bit/mono)

  //"https://streaming.live365.com/a77334", // Smooth Jazz Global Radio

  "https://live.radioart.com/fLove_smooth_jazz.mp3", // Radio Art - Love Smooth Jazz
  "https://live.radioart.com/fSmooth_jazz.mp3", // Radio Art - Smooth Jazz
  "https://live.radioart.com/fSmooth_bossa_nova.mp3", // Radio Art - Smooth Bossa Nova

  "https://kpop.onlyhit.us/play",     // OnlyHit K-Pop
  "https://j.onlyhit.us/play",        // OnlyHit Japan
  "https://api.onlyhit.us/tophits",   // OnlyHit Top Hits
  "https://api.onlyhit.us/play",      // OnlyHit
  "https://gold.onlyhit.us/play",     // OnlyHit Gold

  //"https://wgbh-live.streamguys1.com/classical-hi/", // Classic, CRB 99.5

  "https://wtmd-ice.streamguys1.com/wypr-3-mp3", // WYPR PRESENTS ALL CLASSICAL
};


#include <SPI.h>

#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Update.h>

WebServer webserver(80);


#ifdef USE_ETHER
#include <EthernetESP32.h>
#include <MacAddress.h>
#endif /* USE_ETHER */

#include "WifiProv.h"
#include <WiFi.h>
//#include "esp_system.h"

#ifdef USE_ETHER
// #include <ESP_SSLClient.h>
// ESP_SSLClient ssl_client;

EthernetClient ether_client;

#include <NetworkClientSecure.h>
NetworkClientSecure ssl_client;
#endif

WiFiClient wifi_client;

#define BUF_SIZE (DEFAULT_BUFFER_SIZE)

//const char *wifi = "xxx";
//const char *password = "xxx";

#ifndef USE_ETHER
//URLStream urlStream(BUF_SIZE);
#endif

#if 1
AnalogAudioStream out;
#define STREAM_BUFFER_SIZE (512*2)
RingBufferStream stream(STREAM_BUFFER_SIZE);
StreamCopy copier(out, stream); 
#endif

#ifdef USE_ETHER
#if 0
//URLStreamBuffered urlStream(/*ssl_client,*/ BUF_SIZE/*DEFAULT_BUFFER_SIZE*/);
#else
//URLStream urlStream(/*ssl_client,*/ BUF_SIZE/*DEFAULT_BUFFER_SIZE*/);
#endif
#endif /* USE_ETHER */

//AudioSourceURL source(urlStream, (const char **)urls, "audio/mp3");
// MP3 -> AAC
//AudioSourceURL source(urlStream, urls, "audio/mp3,audio/mpeg,audio/aac");
//AudioSourceURL source(urlStream, urls, "audio/mpeg");
//AudioSourceURL source(urlStream, urls, "audio/aac");


AudioInfo info(44100,2,16);

//A2DPStream out;

// MP3 -> AAC
MP3DecoderHelix decoder;
AACDecoderHelix decoderAAC;

//FormatConverterStream conv(out);
//AudioPlayer player(source, out, decoder, conv);

#define PIN_CS    33
byte mac[] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x01 };

W5500Driver driver(PIN_CS);
//W5500Driver driver;

#include "FS.h"
#include "SPIFFS.h"

#include "esp_mac.h"

uint8_t macaddr_wifi[8];
uint8_t macaddr_eth[8];
uint8_t macaddr_bt[8];

#define MACADDR_DIGITS(p)  p[0], p[1], p[2], p[3], p[4], p[5]
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

void scramble_passwd(char *passwd, int passwd_len, char *macaddr)
{
  for (int i = 0 ; i < passwd_len ; i++) {
    passwd[i] ^= macaddr[i % 6];
  }
}

char *read_cfg(char *cfg_file, char *buf, int buf_len, char *default_value, int flag_print, int flag_url)
{
  File fp = SPIFFS.open(cfg_file, FILE_READ);
  char *ptr = default_value;

  if (fp) {
    int len = fp.read((uint8_t *)buf, buf_len);
    fp.close();
    if (len == buf_len) {
      len--;
    }
    if (len > 0) {
      buf[len] = 0x0;
      for (int i = 0 ; i < len ; i++) {
        if (buf[i] == 0x0a || buf[i] == 0x0d) {
          buf[i] = 0x0;
        }
      }
      if (buf[0] == 0x0) {
        // ignore file when zero length
      } else {
        ptr = buf;
        if (!flag_print) {
          scramble_passwd(ptr, strlen(ptr), macaddr_wifi);
        }
      }
    } else {
      //Serial_printf("empty file: [%s]\r\n", cfg_file);
    }
  } else {
    Serial_printf("cannot open: [%s]\r\n", cfg_file);
  }

  if (flag_url) {
    if (!strstr(ptr, "https://") && !strstr(ptr, "http://")) {
      ptr = default_value;
    }
  }

  return ptr;
}

int write_cfg(char *cfg_file, char *buf)
{
  File fp = SPIFFS.open(cfg_file, FILE_WRITE);
  int rv = -1;

  if (fp) {
    rv = fp.write((uint8_t *)buf, strlen(buf));
    rv = fp.write((uint8_t *)"\r\n", 2);
    fp.close();
  }

  return rv;
}

int write_cfg_passwd(char *cfg_file, char *buf)
{
  File fp = SPIFFS.open(cfg_file, FILE_WRITE);
  int rv = -1;

  if (fp) {
    char passwd[65];
    strncpy(passwd, buf, MIN(64, strlen(buf)));
    passwd[MIN(64, strlen(buf))] = 0x00;
    scramble_passwd(passwd, strlen(passwd), macaddr_wifi);
    buf = passwd;

    rv = fp.write((uint8_t *)buf, strlen(buf));
    rv = fp.write((uint8_t *)"\r\n", 2);
    fp.close();
  }

  return rv;
}

char *read_serial(char *buf, int buf_len, char *default_value)
{
  int idx = 0;
#define SERIAL_BUF_SIZE 256
  char serial_buf[SERIAL_BUF_SIZE];
  char *ptr = default_value;

  while (idx < SERIAL_BUF_SIZE - 1) {
    // Read 1 character from serial port
    char ch = 0;
    do {
      //esp_task_wdt_reset();

      if (Serial.available()) {
        //disableCore0WDT();
        ch = Serial.read();
        //enableCore0WDT();
      } else if (SerialBT.available()) {
        //disableCore0WDT();
        ch = SerialBT.read();
        //enableCore0WDT();
      }
    } while (ch == 0);

    if (ch == 0x0d) { // CR
      Serial_write(ch);
      Serial_write(0x0a); // LF
      break;
    } else if (ch == 0x08) { // BS
      if (idx > 0) {
        idx--;
        Serial_write(ch);
        Serial_write(' ');
        Serial_write(ch);
      }
    } else if (ch >= ' ' and ch <= 0x7f) {
      Serial_write(ch);
      serial_buf[idx++] = ch;
    }
  }

  serial_buf[idx] = 0x0;

  if (idx > 0) {
    strncpy(buf, serial_buf, min(idx + 1, buf_len));
    buf[buf_len - 1] = 0x0;
    ptr = buf;
  }

  return ptr;
}

// UUID dda79c1b-9c5a-4c02-894f-31e8487d9ad0

uint8_t uuid[16] = {0xdd, 0xa7, 0x9c, 0x1b, 0x9c, 0x5a, 0x4c, 0x02,
                    0x89, 0x4f, 0x31, 0xe8, 0x48, 0x7d, 0x9a, 0xd0 };

void SysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("\nConnected IP address : ");
      Serial.println(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: Serial.println("\nDisconnected. Connecting to the AP again... "); break;
    case ARDUINO_EVENT_PROV_START:            Serial.println("\nProvisioning started\nGive Credentials of your access point using smartphone app"); break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
    {
      Serial.println("\nReceived Wi-Fi credentials");
      Serial.print("\tSSID : ");
      Serial.println((const char *)sys_event->event_info.prov_cred_recv.ssid);
      Serial.print("\tPassword : ");
//      Serial.println((char const *)sys_event->event_info.prov_cred_recv.password);
      Serial.println("*****");

      write_cfg("/ssid.txt", sys_event->event_info.prov_cred_recv.ssid);
      write_cfg_passwd("/passwd.txt", sys_event->event_info.prov_cred_recv.password);

      //Serial.println("\nRestarting...");
      //ESP.restart();

      break;
    }
    case ARDUINO_EVENT_PROV_CRED_FAIL:
    {
      Serial.println("\nProvisioning failed!\nPlease reset to factory and retry provisioning\n");
/*
      if (sys_event->event_info.prov_fail_reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) {
        Serial.println("\nWi-Fi AP password incorrect");
      } else {
        Serial.println("\nWi-Fi AP not found....Add API \" nvs_flash_erase() \" before beginProvision()");
      }
*/

      //Serial.println("\nRestarting...");
      //ESP.restart();

      break;
    }
    case ARDUINO_EVENT_PROV_CRED_SUCCESS: Serial.println("\nProvisioning Successful"); break;
    case ARDUINO_EVENT_PROV_END:
        Serial.println("\nProvisioning Ends");

        Serial.println("\nRestarting...");
        ESP.restart();

        break;
    default:                              break;
  }
}

#if 0
void processEndCallback(URLStream& stream, HttpRequest& request, Url& url)
{
  //Serial.printf("connectCallback: URL = %s\n", url.url());
  //Serial.printf("connectCallback: request.reply().statusCode() %d\n", request.reply().statusCode());
  Serial.printf("Content-Type: %s\r\n", request.reply().get(CONTENT_TYPE));

#if 0 /* decided not to allow AAC */
  if (strstr(request.reply().get(CONTENT_TYPE), "audio/aac")) {
    Serial.printf("Changing decoder to AAC\r\n");
    player.setDecoder(decoderAAC);
  } else {
    Serial.printf("Changing decoder to MP3\r\n");
    player.setDecoder(decoder);
  }
#endif
}
#endif

#if 1
const static char *def_yes = "yes";
const static char *def_no  = "no";
const static char *def_0   = "0";
const static char *def_1   = "1";
const static char *def_2   = "2";
const static char *def_3   = "3";
const static char *def_20  = "20";
const static char *def_50  = "50";
const static char *def_150 = "150";

#define DEF_YES   def_yes
#define DEF_NO    def_no
#define DEF_0     def_0
#define DEF_1     def_1
#define DEF_2     def_2
#define DEF_3     def_3
#define DEF_20    def_20
#define DEF_50    def_50
#define DEF_150   def_150
#endif

#if 0

#define BT_BUF_SIZE 32
//#define BT_BUF_SIZE 128
static char *bt = "any", bt_buf[BT_BUF_SIZE];

#define PAIRING_BUF_SIZE 32
//#define PAIRING_BUF_SIZE 128
static char *pairing = "yes", pairing_buf[PAIRING_BUF_SIZE];

#define RSSI_BUF_SIZE 32
//#define RSSI_BUF_SIZE 128
#define DEF_RSSI "-75"
static char *rssi = DEF_RSSI, rssi_buf[RSSI_BUF_SIZE];

#define URL_BUF_SIZE 256
#define URL_NUM (sizeof(urls) / sizeof(char *))
static char *urlp[URL_NUM], url_buf[URL_NUM][URL_BUF_SIZE];

#endif /* 0 */

#define PORT_BUF_SIZE 32
#if 0 /*def BT_WIFI_ETHER*/ /* default "wifi" even with Ether */
static char *port = "eth", port_buf[PORT_BUF_SIZE];
#else
static char *port = "wifi", port_buf[PORT_BUF_SIZE];
#endif

#define SSID_BUF_SIZE 32
static char *ssid = "ssid", ssid_buf[SSID_BUF_SIZE];

#define PASSWD_BUF_SIZE 64
static char *passwd = "passwd", passwd_buf[PASSWD_BUF_SIZE];

#if 0
#define VOL_BUF_SIZE 32
static char *vol = DEF_VOL, vol_buf[VOL_BUF_SIZE];
#endif

#if 1 /* PUG_MORSE */

#define ADJ_WPM_BUF_SIZE 32
#define ADJ_WPM_DEF DEF_20
static char *adj_wpm = ADJ_WPM_DEF, adj_wpm_buf[ADJ_WPM_BUF_SIZE];

#define REVERSE_BUF_SIZE 32
#define REVERSE_DEF DEF_NO
static char *reverse = REVERSE_DEF, reverse_buf[REVERSE_BUF_SIZE];

#define KEYER_IAMBICA  "IambicA"
#define KEYER_IAMBICB  "IambicB"
#define KEYER_SUPER    "Super"
#define KEYER_STRAIGHT "Straight"

#define KEYER_BUF_SIZE 32
#define KEYER_DEF DEF_0
static char *keyer = KEYER_DEF, keyer_buf[KEYER_BUF_SIZE];

#define OFFDELAY_BUF_SIZE 32
#define OFFDELAY_DEF DEF_NO
static char *offdelay = OFFDELAY_DEF, offdelay_buf[OFFDELAY_BUF_SIZE];

#define DOT_RATIO_BUF_SIZE 32
#define DOT_RATIO_DEF DEF_50
static char *dot_ratio = DOT_RATIO_DEF, dot_ratio_buf[DOT_RATIO_BUF_SIZE];

#define DASH_RATIO_BUF_SIZE 32
#define DASH_RATIO_DEF DEF_150
static char *dash_ratio = DASH_RATIO_DEF, dash_ratio_buf[DASH_RATIO_BUF_SIZE];

#define NOISE_BUF_SIZE 32
#define NOISE_DEF DEF_YES
static char *noise = NOISE_DEF, noise_buf[NOISE_BUF_SIZE];

#define AUTOKEYER_FLAG_BUF_SIZE 32
#define AUTOKEYER_FLAG_DEF DEF_NO
static char *autokeyer_flag = AUTOKEYER_FLAG_DEF, autokeyer_flag_buf[AUTOKEYER_FLAG_BUF_SIZE];

#define AUTOKEYER_TYPE_BUF_SIZE 32
#define AUTOKEYER_TYPE_DEF DEF_0
static char *autokeyer_type = AUTOKEYER_TYPE_DEF, autokeyer_type_buf[AUTOKEYER_TYPE_BUF_SIZE];

#endif /* 1 */

#define OTA_BUF_SIZE 32
static char *ota = "no", ota_buf[OTA_BUF_SIZE];

#define RESET_BUF_SIZE 32
static char *reset = "no", reset_buf[RESET_BUF_SIZE];

#if 0
#define IDX_BUF_SIZE 32
#define DEF_IDX "0"
static char *idx = DEF_IDX, idx_buf[IDX_BUF_SIZE];
#endif

static int playing_idx = 0;

void Serial_print_config()
{
#if 0
    Serial_printf("  BT: %s\r\n", bt);
    Serial_printf("  PAIRING: %s\r\n", pairing);
    Serial_printf("  RSSI: %s\r\n", rssi);
#endif
//#ifndef USE_ETHER
    Serial_printf("  PORT: %s\r\n", port);
    Serial_printf("  SSID: %s\r\n", ssid);
    Serial_printf("  PASSWD: *****\r\n");
//#endif /* !USE_ETHER */
#if 0
    for (int i = 0 ; i < URL_NUM ; i++) {
      Serial_printf("  URL%d: %s\r\n", i, urlp[i]);
    }
    Serial_printf("  INDEX: %s\r\n", idx);
    Serial_printf("  VOLUME: %s\r\n", vol);
#endif
#if 1 /* PUG_MORSE */
    Serial_printf("  Adjusted WPM: %s\r\n", adj_wpm);
    Serial_printf("  Reverse flag: %s\r\n", reverse);
    Serial_printf("  Keyer type: %s (0:Iambic A, 1:Iambic B, 2:Super keyer, 3:Straight key)\r\n", keyer);
    Serial_printf("  Off-delay flag: %s\r\n", offdelay);
    Serial_printf("  Off-delay Dot Ratio: %s\r\n", dot_ratio);
    Serial_printf("  Off-delay Dash Ratio: %s\r\n", dash_ratio);
    Serial_printf("  Background Noise flag: %s\r\n", noise);
    Serial_printf("  Auto Keyer flag: %s\r\n", autokeyer_flag);
    Serial_printf("  Auto Keyer type: %s  (0:Iambic A, 1:Iambic B, 2:Super keyer)\r\n", autokeyer_type);
#endif
    Serial_printf("  OTA: %s\r\n", ota);
    Serial_printf("  RESET: %s\r\n", reset);

    Serial_printf("\r\n");
}

void Serial_print_usage()
{
  Serial.printf("\r\n");
  Serial.printf("*** Welcome to Pug Morse!! Ver. %s\r\n", APP_VERSION);
  Serial.printf("\r\n");
  Serial.printf("Short Press (Button count):\r\n");
  Serial.printf("  1: 10-QSO Challenge (NORMAL class, %dWPM, 3-miss becomes 1-penalty)\r\n", Wpm);
  Serial.printf("  2: Stop game\r\n");
  Serial.printf("\r\n");
  Serial.printf("  3: 1-QSO Challenge (EASY class, 15WPM)\r\n");
  Serial.printf("  4: 10-QSO Challenge (HARD calss, 25WPM, 3-miss & 3-'?' becomes 1-penalty)\r\n");
  Serial.printf("\r\n");
  Serial.printf("  5: (RV) Toggle Reverse flag\r\n");
  Serial.printf("  6: (KY) Toggle Keyer Type (Iambic A, Iambic B, Super keyer, Straight)\r\n");
  Serial.printf("  7: (OD) Toggle Off-Delay flag\r\n");
  Serial.printf("  8: (BN) Toggle Background Noise flag\r\n");
  Serial.printf("  9: (AK) Toggle Auto Keyer detection flag\r\n");
  Serial.printf(" 10: (AT) Toggle Auto Keyer Type (Iambic A, Iambic B, Super keyer)\r\n");
  Serial.printf("\r\n");
  Serial.printf("Long Press (Button count):\r\n");
  Serial.printf("  1: WPM adjustment during 1 long press (3sec)\r\n");
  Serial.printf("  2: Menu and Status after 1 short press plus 1 long press (3sec)\r\n");

  Serial.printf("\r\n");
}

unsigned int sizeSpiffsFile(char *filename)
{
  File file = SPIFFS.open(filename, "r");
  if (!file || file.isDirectory()) return -1;

  size_t fileSize = file.size();

  file.close();

  return fileSize;
}

uint8_t *readSpiffsFile(char *filename)
{
  File file = SPIFFS.open(filename, "r");
  if (!file || file.isDirectory()) return NULL;

  size_t fileSize = file.size();
  // ファイルサイズ分のメモリを確保（テキスト用なら+1）
  uint8_t* buffer = (uint8_t*)malloc(fileSize);

  if (buffer) {
    file.read(buffer, fileSize);
    // bufferを使ってデータを処理
  }

  file.close();

  return buffer;
}

void setup() {
  Serial.begin(115200);

  esp_read_mac(macaddr_wifi, ESP_MAC_WIFI_STA);
  esp_read_mac(macaddr_eth,  ESP_MAC_ETH);
  esp_read_mac(macaddr_bt,   ESP_MAC_BT);

  Serial.printf("\r\n************************************************************************\r\n\r\n");
  Serial.printf("  Welcome to Pug Morse!!\ " FEATURE " Ver. %s\r\n", APP_VERSION);
  Serial.printf("  Wi-Fi MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_wifi));
  Serial.printf("\r\n************************************************************************\r\n\r\n");

#if 0
  disableCore0WDT();
  //disableCore1WDT();
  Serial.printf("Available memory: %d bytes\r\n", availableMemory());
  enableCore0WDT();
  //enableCore1WDT();
#endif


  SPIFFS.begin(true);

#if 0
  bt = read_cfg("/bt.txt", bt_buf, BT_BUF_SIZE, bt, 1, 0);
  pairing = read_cfg("/pairing.txt", pairing_buf, PAIRING_BUF_SIZE, pairing, 1, 0);
  rssi = read_cfg("/rssi.txt", rssi_buf, RSSI_BUF_SIZE, rssi, 1, 0);
#endif
  port = read_cfg("/port.txt", port_buf, PORT_BUF_SIZE, port, 1, 0);

  ssid = read_cfg("/ssid.txt", ssid_buf, SSID_BUF_SIZE, ssid, 1, 0);
  passwd = read_cfg("/passwd.txt", passwd_buf, PASSWD_BUF_SIZE, passwd, 0, 0);

#if 0
  for (int i = 0 ; i < URL_NUM ; i++) {
    urlp[i] = (char *)urls[i];
#define PATH_BUF_SIZE 32
    char buf[PATH_BUF_SIZE];
    sprintf(buf, "/url%d.txt", i);
    urlp[i] = read_cfg(buf, url_buf[i], URL_BUF_SIZE, urlp[i], 1, 1);
  }

  idx = read_cfg("/index.txt", idx_buf, IDX_BUF_SIZE, idx, 1, 0);
  if (atoi(idx) < 0 || atoi(idx) >= URL_NUM) {
    idx = DEF_IDX;
  }

  vol = read_cfg("/volume.txt", vol_buf, VOL_BUF_SIZE, vol, 1, 0);
#endif

#if 1 /* PUG_MORSE */
  adj_wpm = read_cfg("/adj_wpm.txt", adj_wpm_buf, ADJ_WPM_BUF_SIZE, adj_wpm, 1, 0);
  reverse = read_cfg("/reverse.txt", reverse_buf, REVERSE_BUF_SIZE, reverse, 1, 0);
  offdelay = read_cfg("/offdelay.txt", offdelay_buf, OFFDELAY_BUF_SIZE, offdelay, 1, 0);
  dot_ratio = read_cfg("/dot_ratio.txt", dot_ratio_buf, DOT_RATIO_BUF_SIZE, dot_ratio, 1, 0);
  dash_ratio = read_cfg("/dash_ratio.txt", dash_ratio_buf, DASH_RATIO_BUF_SIZE, dash_ratio, 1, 0);
  keyer = read_cfg("/keyer.txt", keyer_buf, KEYER_BUF_SIZE, keyer, 1, 0);
  noise = read_cfg("/noise.txt", noise_buf, NOISE_BUF_SIZE, noise, 1, 0);
  autokeyer_flag = read_cfg("/autokeyer_flag.txt", autokeyer_flag_buf, AUTOKEYER_FLAG_BUF_SIZE, autokeyer_flag, 1, 0);
  autokeyer_type = read_cfg("/autokeyer_type.txt", autokeyer_type_buf, AUTOKEYER_TYPE_BUF_SIZE, autokeyer_type, 1, 0);
#endif

#if 0
  ota = read_cfg("/ota.txt", ota_buf, OTA_BUF_SIZE, ota, 1, 0);

  reset = read_cfg("/reset.txt", reset_buf, RESET_BUF_SIZE, reset, 1, 0);
#endif

#if 1
  pinMode(0/*Boot button*/, INPUT_PULLUP);
  int state = digitalRead(0);

  if (state == LOW) {

    esp_err_t ret = nvs_flash_init();
    if (ret == 0) {
      ret = nvs_flash_erase();
    }
    Serial.printf("\r\nRemoving BT pairing... (%d)\r\n", ret);

#ifdef USE_BT_SERIAL
      (void)nvs_flash_init();
      WiFi.begin();
      WiFi.onEvent(SysProvEvent);

// Cannot start provisioning service with BLE...
      WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BLE, WIFI_PROV_SECURITY_1, "abcd1234", "PROV_PugRadio", NULL, uuid, true);
      WiFiProv.printQR("PROV_PugRadio", "abcd1234", "ble");

//      WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, "abcd1234", "PROV_PugRadio", NULL, uuid, true);
//      WiFiProv.printQR("PROV_PugRadio", "abcd1234", "softap");

//    if (Serial.available()) {
//      while (Serial.available()) {
//        int ch = Serial.read();
//      }
//    } else if (bt_initialized) {
      SerialBT.begin("PROV_PugRadio");
      bt_initialized = 1;
//      delay(3000);
//    }

    //disableCore0WDT();
    //disableCore1WDT();
    esp_task_wdt_deinit();

#define ANSWER_BUF_SIZE 32
//#define ANSWER_BUF_SIZE 128
    static char *answer = "n", answer_buf[ANSWER_BUF_SIZE];

    // Wait the first key input
    answer = read_serial(answer_buf, ANSWER_BUF_SIZE, answer);
#endif

   do {

    Serial_printf("\r\n************************************************************************\r\n\r\n");
    Serial_printf("  Welcome to Pug Morse!!\ " FEATURE " Ver. %s\r\n", APP_VERSION);
    Serial_printf("  Wi-Fi MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_wifi));
    Serial_printf("\r\n  Setup mode. Please input values.\r\n");
    Serial_printf("\r\n************************************************************************\r\n\r\n");

#if 0
#ifdef BT_RADIO
    Serial_printf("BT (%s) [comma separated BT names, or \"any\" when any is OK]: ", bt);
    bt = read_serial(bt_buf, BT_BUF_SIZE, bt);
#endif

#ifdef BT_RADIO
    Serial_printf("PAIRING (%s) [Y/n]: ", pairing);
    pairing = read_serial(pairing_buf, PAIRING_BUF_SIZE, pairing);
    if (pairing[0] == 'n' || pairing[0] == 'N') {
      pairing = "no";
    } else {
      pairing = "yes";
    }
#endif

#ifdef BT_RADIO
    Serial_printf("RSSI (%s) [more than -75]: ", rssi);
    rssi = read_serial(rssi_buf, RSSI_BUF_SIZE, rssi);
    if (-75 > atoi(rssi) || atoi(rssi) >= 0) {
      rssi = DEF_RSSI;
    }
#endif
#endif

#ifdef BT_WIFI_ETHER
//#ifndef USE_ETHER
    Serial_printf("PORT (%s) [wifi or eth]: ", port);
    port = read_serial(port_buf, PORT_BUF_SIZE, port);
    if (strcmp("wifi", port) && strcmp("eth", port)) {
      port = "wifi";
    }
#endif

    Serial_printf("SSID (%s): ", ssid);
    ssid = read_serial(ssid_buf, SSID_BUF_SIZE, ssid);

    Serial_printf("PASSWD (*****): ");
    passwd = read_serial(passwd_buf, PASSWD_BUF_SIZE, passwd);
//#endif /* !USE_ETHER */

#if 0
    for (int i = 0 ; i < URL_NUM ; i++) {
      Serial_printf("URL%d (%s): ", i, urlp[i]);
      urlp[i] = read_serial(url_buf[i], URL_BUF_SIZE, urlp[i]);
      if (!strstr(urlp[i], "https://") && !strstr(urlp[i], "http://")) {
          urlp[i] = (char *)urls[i];
      }
    }

    Serial_printf("INDEX (%s) [0 - %d]: ", idx, URL_NUM - 1);
    idx = read_serial(idx_buf, IDX_BUF_SIZE, idx);
    if (atoi(idx) < 0 || atoi(idx) >= URL_NUM) {
      idx = DEF_IDX;
    }

    Serial_printf("VOLUME (%s) [0.0 - 1.0]: ", vol);
    vol = read_serial(vol_buf, VOL_BUF_SIZE, vol);
    if (0.0 >= atof(vol) || 1.0 < atof(vol)) {
      vol = DEF_VOL;
    }
#endif

#if 1 /* PUG_RATIO */
    Serial_printf("WPM (%s) [%d - %d]: ", adj_wpm, MIN_WPM, MAX_WPM);
    adj_wpm = read_serial(adj_wpm_buf, ADJ_WPM_BUF_SIZE, adj_wpm);
    if (adj_wpm[0] < '0' || adj_wpm[0] > '9' || atoi(adj_wpm) < MIN_WPM || atoi(adj_wpm) > MAX_WPM) {
      adj_wpm = ADJ_WPM_DEF;
    }

    Serial_printf("Reverse flag (%s) [yes or no]: ", reverse);
    reverse = read_serial(reverse_buf, REVERSE_BUF_SIZE, reverse);
    if (strcmp("no", reverse) && strcmp("yes", reverse)) {
      reverse = REVERSE_DEF;
    }

    Serial_printf("Keyer type (%s) [0:Iambic A, 1:Iambic B, 2:Super keyer, 3:Straight key]: ", keyer);
    keyer = read_serial(keyer_buf, KEYER_BUF_SIZE, keyer);
    if (keyer[0] < '0' || keyer[0] > '3' || atoi(keyer) < 0 || atoi(keyer) > 3) {
      /* 0:Iambic A, 1:Iambic B, 2:Super keyer, 3:Straight key */
      keyer = KEYER_DEF;
    }

    Serial_printf("Off-Delay flag (%s) [yes or no]: ", offdelay);
    offdelay = read_serial(offdelay_buf, OFFDELAY_BUF_SIZE, offdelay);
    if (strcmp("no", offdelay) && strcmp("yes", offdelay)) {
      offdelay = OFFDELAY_DEF;
    }

    Serial_printf("Off-Delay Dot ratio (%s) [0-%d]: ", dot_ratio, MAX_DOT_RATIO);
    dot_ratio = read_serial(dot_ratio_buf, DOT_RATIO_BUF_SIZE, dot_ratio);
    if (dot_ratio[0] < '0' || dot_ratio[0] > '9' || atoi(dot_ratio) < 0 || atoi(dot_ratio) > MAX_DOT_RATIO) {
      dot_ratio = DOT_RATIO_DEF;
    }

    Serial_printf("Off-Delay Dash ratio (%s) [0-%d]: ", dash_ratio, MAX_DASH_RATIO);
    dash_ratio = read_serial(dash_ratio_buf, DASH_RATIO_BUF_SIZE, dash_ratio);
    if (dash_ratio[0] < '0' || dash_ratio[0] > '9' || atoi(dash_ratio) < 0 || atoi(dash_ratio) > MAX_DASH_RATIO) {
      dash_ratio = DASH_RATIO_DEF;
    }

    Serial_printf("Background Noise flag (%s) [yes or no]: ", noise);
    noise = read_serial(noise_buf, NOISE_BUF_SIZE, noise);
    if (strcmp("no", noise) && strcmp("yes", noise)) {
      noise = NOISE_DEF;
    }

    Serial_printf("Autokey flag (%s) [yes or no]: ", autokeyer_flag);
    autokeyer_flag = read_serial(autokeyer_flag_buf, AUTOKEYER_FLAG_BUF_SIZE, autokeyer_flag);
    if (strcmp("no", autokeyer_flag) && strcmp("yes", autokeyer_flag)) {
      autokeyer_flag = AUTOKEYER_FLAG_DEF;
    }

    Serial_printf("Autokeyer type (%s) [0:Iambic A, 1:Iambic B, 2:Super keyer]: ", autokeyer_type);
    autokeyer_type = read_serial(autokeyer_type_buf, AUTOKEYER_TYPE_BUF_SIZE, autokeyer_type);
    if (autokeyer_type[0] < '0' || autokeyer_type[0] > '2' || atoi(autokeyer_type) < 0 || atoi(autokeyer_type) > 2) {
      /* 0:Iambic A, 1:Iambic B, 2:Super keyer (Straight key is not included) */
      autokeyer_type = "0";
    }
#endif

    Serial_printf("OTA (%s): ", ota);
    ota = read_serial(ota_buf, OTA_BUF_SIZE, ota);
    if (strcmp("no", ota) && strcmp("yes", ota)) {
      ota = "no";
    }

    Serial_printf("RESET (%s): ", reset);
    reset = read_serial(reset_buf, RESET_BUF_SIZE, reset);
    if (strcmp("no", reset) && strcmp("yes", reset)) {
      reset = "no";
    }

    Serial_printf("\r\n*** Results (to be new values) ***\r\n");
    Serial_print_config();

    Serial_printf("Are you sure to overwrite settings? [y/N]: ");
    answer = read_serial(answer_buf, ANSWER_BUF_SIZE, answer);
    if (answer[0] == 'y' || answer[0] == 'Y') {
     if (!strcmp(reset, "yes")) {
#if 0
      SPIFFS.remove("/bt.txt");
      SPIFFS.remove("/pairing.txt");
      SPIFFS.remove("/rssi.txt");
#endif
//#ifndef USE_ETHER
      SPIFFS.remove("/port.txt");
      SPIFFS.remove("/ssid.txt");
      SPIFFS.remove("/passwd.txt");
//#endif /* !USE_ETHER */
#if 0
      for (int i = 0 ; i < URL_NUM ; i++) {
        char buf[PATH_BUF_SIZE];
        sprintf(buf, "/url%d.txt", i);
        SPIFFS.remove(buf);
      }
      SPIFFS.remove("/index.txt");
      SPIFFS.remove("/volume.txt");
#endif
#if 1 /* PUG_MORSE */
      SPIFFS.remove("/adj_wpm.txt");
      SPIFFS.remove("/reverse.txt");
      SPIFFS.remove("/keyer.txt");
      SPIFFS.remove("/offdelay.txt");
      SPIFFS.remove("/dot_ratio.txt");
      SPIFFS.remove("/dash_ratio.txt");
      SPIFFS.remove("/noise.txt");
      SPIFFS.remove("/autokeyer_flag.txt");
      SPIFFS.remove("/autokeyer_type.txt");
#endif
#if 0
      SPIFFS.remove("/ota.txt");
      SPIFFS.remove("/reset.txt");
#endif

      Serial_printf("\r\n*** Initialized.\r\n\r\n");

     } else {
#if 0
      write_cfg("/bt.txt", bt);
      write_cfg("/pairing.txt", pairing);
      write_cfg("/rssi.txt", rssi);
#endif
//#ifndef USE_ETHER
      write_cfg("/port.txt", port);
      write_cfg("/ssid.txt", ssid);
      write_cfg_passwd("/passwd.txt", passwd);
//#endif /* !USE_ETHER */
#if 0
      for (int i = 0 ; i < URL_NUM ; i++) {
        char buf[PATH_BUF_SIZE];
        sprintf(buf, "/url%d.txt", i);
        write_cfg(buf, urlp[i]);
      }
      write_cfg("/index.txt", idx);
      write_cfg("/volume.txt", vol);
#endif
#if 1 /* PUG_MORSE */
      write_cfg("/adj_wpm.txt", adj_wpm);
      write_cfg("/reverse.txt", reverse);
      write_cfg("/keyer.txt", keyer);
      write_cfg("/offdelay.txt", offdelay);
      write_cfg("/dot_ratio.txt", dot_ratio);
      write_cfg("/dash_ratio.txt", dash_ratio);
      write_cfg("/noise.txt", noise);
      write_cfg("/autokeyer_flag.txt", autokeyer_flag);
      write_cfg("/autokeyer_type.txt", autokeyer_type);
#endif
      //write_cfg("/ota.txt", ota);
      //write_cfg("/reset.txt", reset);

      Serial_printf("\r\n*** Updated.\r\n\r\n");
     }

      //Serial_printf("SPIFFS was overwritten.\r\n");
      break;
    } else {
      //Serial_printf("Aborted.\r\n");
    }
   } while (!(answer[0] == 'y' || answer[0] == 'Y'));

    SPIFFS.end();

#if 1
    if (!strcmp(ota, "yes")) {
      Serial_printf("Are you sure to update firmware? [y/N]: ");
      answer = read_serial(answer_buf, ANSWER_BUF_SIZE, answer);
      if (answer[0] == 'y' || answer[0] == 'Y') {
          if (!strcmp(port, "eth") || !strcmp(port, "ETH")) {
            Ethernet.init(driver);
            Ethernet.begin();
            Serial_printf("\r\n----------------------------------------\r\nPlease open http://%s/\r\n----------------------------------------\r\n\r\n", Ethernet.localIP().toString().c_str());
          } else {
            WiFi.begin(ssid, passwd);
            while (WiFi.status() != WL_CONNECTED) {
              delay(500);
              Serial_printf(".");
            }
            Serial_printf(".\r\nWiFi connected.\r\n");
            Serial_printf("\r\n----------------------------------------\r\nPlease open URL: http://%s/\r\n----------------------------------------\r\n\r\n", WiFi.localIP().toString().c_str());
          }

          ArduinoOTA.begin();

          webserver.on("/", HTTP_GET, []() {
              webserver.send(200, "text/html",
                             "<html>"
                             "<body>"
                             "<hr>"
                             "<h1>Welcome to PugRadio firmware update!</h1>"
                             "<form method='POST' action='/update' enctype='multipart/form-data'>"
                             "First, please select the firmware: <input type='file' name='update'><br>"
                             "<p>"
                             "After that, press <input type='submit' value='Update'> button, and Wait for the page to refresh!"
                             "</form>"
                             "<hr>"
                             "</body>"
                             "</html>");
          });

          webserver.on("/update", HTTP_POST, []() {
              webserver.send(200, "text/html",
                             "<html>"
                             "<body>"
                             "<hr>"
                             "<h1>PugRadio Firmware Update Success!!</h1>"
                             "<hr>"
                             "</body>"
                             "</html>");
              delay(3000);
              ESP.restart();
          }, []() {
              HTTPUpload& upload = webserver.upload();
              if (upload.status == UPLOAD_FILE_START) {
                  Serial_printf("Updating: %s...\r\n", upload.filename.c_str());
                  if (!Update.begin()) {
                      Update.printError(Serial);
                  }
              } else if (upload.status == UPLOAD_FILE_WRITE) {
                  if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                      Update.printError(Serial);
                  }
              } else if (upload.status == UPLOAD_FILE_END) {
                  if (Update.end(true)) {
                      Serial_printf("Update finished.\r\n");
                  } else {
                      Update.printError(Serial);
                  }
              }
          });

          webserver.begin();

          for ( ; ; ) {
            ArduinoOTA.handle();
            webserver.handleClient();
          }
      }
    }
#endif

    Serial_printf("\r\n************************************************************************\r\n\r\n");
    Serial_printf("Restarting...\r\n");
    Serial_printf("\r\n************************************************************************\r\n\r\n");
    Serial_printf("\r\n");

    delay(3000);

    ESP.restart();
  }
#endif

  Serial_printf("\r\n*** Configuration ***\r\n");
  Serial_print_config();

#if 0
//#define URL_BUF_SIZE 128
  for (int i = 0 ; i < URL_NUM ; i++) {
    ((char **)urls)[i] = urlp[i];
  }
#endif

#if 0
  SPIFFS.end();
#endif

#if 1 /* PUG_MORSE */
  if (reverse[0] == 'y') {
    paddleRvs = 1;
  } else {
    paddleRvs = 0;
  }

  if (offdelay[0] == 'y') {
    offDelay = 1;
  } else {
    offDelay = 0;
  }

  off_timer_ratio_dot = atoi(dot_ratio);
  off_timer_ratio_dash = atoi(dash_ratio);

  keyerType = atoi(keyer);

  if (noise[0] == 'y') {
    noiseFlag = 1;
  } else {
    noiseFlag = 0;
  }

  if (autokeyer_flag[0] == 'y') {
    autoKeyerType = 1;
  } else {
    autoKeyerType = 0;
  }

  autoKeyerTypeElekey = atoi(autokeyer_type);

#endif

  int auto_reconnect = 0;

#if 0
  if (pairing[0] == 'y') {
    auto_reconnect = 1;
  }
#endif

  int use_wifi = 1;
#ifdef USE_ETHER
  int use_ether = 0;

  if (!strcmp(port, "eth") || !strcmp(port, "ETH")) {
    use_wifi = 0;
    use_ether = 1;
  }
#endif

  Serial.printf("Wi-Fi MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_wifi));

  if (use_ether) {
    Serial.printf("Ether MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_eth));
  }

  Serial.printf("Bluetooth MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_bt));

//#ifndef USE_ETHER
//  if (use_wifi) {
    //urlStream.setSSID(ssid);
    //urlStream.setPassword(passwd);
//  }
//#endif /* !USE_ETHER */

  //disableCore0WDT();
  //disableCore1WDT();

  //default
  //decoder.setMaxPCMSize(1024 * 5);
  //decoder.setMaxFrameSize(1024 * 2);
  // 2025/03/30

  //modified (leads to playing on and off)
  //decoder.setMaxPCMSize(1024 * 10);
  //decoder.setMaxFrameSize(3200);

// does not work for playing on and off
//  decoder.setMaxPCMSize(1024 * 8);
//  decoder.setMaxFrameSize(1024 * 4);

  //decoder.setFilterMetaData(true);

//  esp_read_mac(mac, ESP_MAC_ETH);
//  Serial.printf("[Ethernet] Mac : %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // leads to playing on and off
  //esp_netif_init();

#if 0
  esp_netif_init();
#endif

#if 0
  int res = (int)esp_wifi_stop();
  // returnes ESP_ERR_WIFI_NOT_INIT 
  Serial.printf("res : %d\r\n", res);
#endif

#if 0

#ifdef USE_ETHER
 if (use_ether) {
  //use_ether = 0;

  // Ethernet.init(PIN_CS);
  // Ethernet.begin(mac);
  Ethernet.init(driver);
  Ethernet.begin();

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet hardware was not found.");
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is NOT connected.");
  } else {
    use_ether = 1;
    Serial.println("Ethernet cable is connected.");

    Serial.print("IP: ");
    Serial.println(Ethernet.localIP());

    Serial.print("gateway: ");
    Serial.println(Ethernet.gatewayIP());

    Serial.print("DNS server: ");
    Serial.println(Ethernet.dnsServerIP());
  }

  //if (use_ether == 0) {
  //  Ethernet.end();
  //}

 }
#endif /* USE_ETHER */

#if 1
  if (use_wifi == 1) {
    int status = WL_IDLE_STATUS;
    WiFi.begin(ssid, passwd);
#if 0
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
    int wait_count = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (++wait_count > 20) { // 10sec
        Serial.println("WiFi connection failed.");
        Serial.println("Rebooting...");
        delay(1000);
        ESP.restart();
      }
    }
    Serial.println(".");
    Serial.println("WiFi connected.");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
#endif

#endif

  snd_start_len = sizeSpiffsFile(FILE_START);
  snd_start_dat = readSpiffsFile(FILE_START);

  snd_buzzer_len = sizeSpiffsFile(FILE_BUZZER);
  snd_buzzer_dat = readSpiffsFile(FILE_BUZZER);

  snd_success_len = sizeSpiffsFile(FILE_SUCCESS);
  snd_success_dat = readSpiffsFile(FILE_SUCCESS);

  snd_clear_len = sizeSpiffsFile(FILE_CLEAR);
  snd_clear_dat = readSpiffsFile(FILE_CLEAR);

  //urlStream.setProcessEndCallback(processEndCallback);

  //AudioLogger::instance().begin(Serial, AudioLogger::Info);
  AudioLogger::instance().begin(Serial, AudioLogger::Warning);
  //AudioLogger::instance().begin(Serial, AudioLogger::Debug);

#if 0
  auto cfg = out.defaultConfig(TX_MODE);
  //cfg.copyFrom(info);
  //cfg.name = "[Unknown]";
  cfg.name = bt;
  //cfg.name = "SR1323";
  //cfg.auto_reconnect = true;  // if this is use we just quickly connect to the last device ignoring cfg.name
  //cfg.auto_reconnect = false;  // if this is use we just quickly connect to the last device ignoring cfg.name
  cfg.auto_reconnect = auto_reconnect;

  cfg.delay_ms = 1;

  //cfg.startup_logic = StartOnConnect; //StartWhenBufferFull;
#if 1
  cfg.startup_nodata = A2DPWhoosh;
#endif
  //cfg.buffer_size = 1024 * 8;
#ifdef BUF_16KB
  cfg.buffer_size = 1024 * 16; // moderate
#else
  cfg.buffer_size = 1024 * 1024 * 2; // 2MB
#endif /* BUF_16KB */
  //cfg.silence_on_nodata = true;

  //cfg.sample_rate = 22050;
  //cfg.channels = 1;
  //cfg.bits_per_sample = 16;

  out.set_a2dp_rssi(atoi(rssi));

  out.begin(cfg);
#endif

#if 1
  //AnalogConfigESP32 cfg;
  //cfg.sample_rate = 44100;
  //cfg.channels = 2; 
  //cfg.bits_per_sample = 16; // Library auto-converts 16-bit to internal 8-bit DAC
  //out.begin(config);


  // Define audio configuration (2 Channels = Stereo)
  auto config = out.defaultConfig(TX_MODE);
  config.sample_rate = 44100;
  config.channels = 2;
  config.bits_per_sample = 16; // Library auto-converts 16-bit to internal 8-bit DAC
  out.begin(config);

  stream.begin();
#endif

#if 0
  // setup player
  float vol_f = atof(vol);
  player.setVolume(vol_f);
  //player.setVolume(0.9);
  //player.setVolume(1.0);
  //player.setBufferSize(128 * 1024); // no effect too prevent buffering

  playing_idx = atoi(idx);
  Serial.printf("Starting Audio Player: index = %d\r\n", playing_idx);
  bool rv = player.begin(playing_idx, true);
  //bool rv = player.begin();
#if 1
  int retry_count = 1;
  while (rv == false) {
    Serial.println("Starting Audio Player failed. Retrying...");
    if (++playing_idx >= URL_NUM)
      playing_idx = 0;
    Serial.printf("Play index = %d\r\n", playing_idx);
    rv = player.next();
    if (++retry_count == URL_NUM)
      break;
  }
#endif
  if (rv == false) {
    Serial.println("Starting Audio Player failed.");
    Serial.println("Rebooting...");
    delay(1000);
    ESP.restart();
  }
#endif

  //setNewWpm(20);
  setNewWpm(atoi(adj_wpm));
  Serial.printf("Finishing setup\r\n\r\n");

  Serial_print_usage();

#if 1
  randomSeed(esp_random());

  pinMode(pinPaddleL,   INPUT_PULLUP);
  pinMode(pinPaddleR,   INPUT_PULLUP);

  pinMode(pinPaddleGND, OUTPUT_OPEN_DRAIN);
  digitalWrite(pinPaddleGND, LOW);

  digitalRead(pinPaddleL);
  digitalRead(pinPaddleR);


#define pinDacL    26
#define pinDacR    25
#define pinDacGND1 33
#define pinDacGND2 32

  pinMode(pinDacL,    ANALOG);
  pinMode(pinDacR,    ANALOG);

  pinMode(pinDacGND1, OUTPUT_OPEN_DRAIN);
  digitalWrite(pinDacGND1, LOW);
  pinMode(pinDacGND2, OUTPUT_OPEN_DRAIN);
  digitalWrite(pinDacGND2, LOW);

#endif
}

#if 1
//#define PUSH_TIMEOUT 500 /* msec */
#define PUSH_TIMEOUT 1000 /* msec */
#if 1
#define LONG_PUSH_TIMEOUT 1000 /* msec */
#endif
int push_last_button = HIGH;
unsigned long push_time = 0;
int push_duration = 0;
int push_time_over  = 0;
int push_count = 0;

#endif

void push_button_reset()
{
  push_last_button = HIGH;
  push_time = 0;
  push_time_over = 0;
  push_count = 0;
}

int push_button_check()
{
#if 0
  int push_button = digitalRead(0);
#endif

  push_button = digitalRead(0);

  if (push_button == LOW) {
    if (push_last_button == HIGH) {
      // button HIGH -> LOW
      push_last_button = LOW;

      if (push_time_over == 0) {
        // previous push_count was already read

        // extend push_time
        push_time = millis();

        push_count++;
      }
    }
  } else if (/* here push_button == HIGH && */ push_last_button == LOW) {
    // button LOW -> HIGH
    push_last_button = HIGH;

#if 1
    push_duration = millis() - push_time;
    if (push_duration > LONG_PUSH_TIMEOUT) {
      // long button push
      push_time = 0;

      push_time_over = -1;
    }
#endif

    // button released

  } else if (/* here button released && */ push_time > 0 && millis() - push_time > PUSH_TIMEOUT) {
    // button push timeout
    push_time = 0;

    push_time_over = 1;
  }

  if (push_time_over > 0)
    return push_count;
#if 1
  else if (push_time_over < 0)
    return - push_count;
#endif
  else
    return 0;
}

#if 1
#define PERIOD_SAMPLES 88  // 1周期をぴったり88サンプルに固定
#define HALF_PERIOD    44  // 半周期（44サンプル）

/**
 * 44.1kHzサンプリング・約501.1Hz矩形波
 * 3点加重移動平均（WMA）ローパスフィルタ適用版（完全整数演算）
 */
static int16_t generate_88sample_wma_square(uint32_t sample_index) {
  // 1周期（88サンプル）の中での現在の位置を、高速な余り算で求める
  uint32_t idx_curr = sample_index % PERIOD_SAMPLES;
    
  // 3点WMAのために「1つ前」と「1つ後」のインデックスをループ対応で計算
  uint32_t idx_prev = (idx_curr == 0) ? (PERIOD_SAMPLES - 1) : (idx_curr - 1);
  uint32_t idx_next = (idx_curr == (PERIOD_SAMPLES - 1)) ? 0 : (idx_curr + 1);

  // デューティ比50%（44サンプル）で 1 と -1 を振り分ける（素朴な矩形波）
  int32_t s_prev = (idx_prev < HALF_PERIOD) ? 1 : -1;
  int32_t s_curr = (idx_curr < HALF_PERIOD) ? 1 : -1;
  int32_t s_next = (idx_next < HALF_PERIOD) ? 1 : -1;

  // 【3点加重移動平均（WMA）】
  // 22.05kHzを完全カットする重み付け [1:2:1]
  // 割り算をビットシフト「>> 2」（4で割る）に置き換えて超高速化
  int32_t out = (s_prev + (s_curr << 1) + s_next) >> 2;

  // 16bit PCM 振幅（32000）を掛けて出力
  //return (int16_t)(out * 16000/*32000*/);
  return (int16_t)(out * 8192/*32000*/);
}
#endif

void loop() {
#if 1
  static uint8_t data[512];
  static unsigned long gap2next = 0;
#define DATA_LEN 512

  //Serial.printf("Available: %d\r\n", stream.availableForWrite());
  if (stream.availableForWrite() >= DATA_LEN) {
    static uint32_t tone_index = 0;
    static float pink_state = 0;
    static float noise_old = 0;
    static float noise = 0;
    static float raw_old2 = 0;
    //static float raw_old = 0;

    int16_t *data16 = (int16_t *)data;
    int32_t len = DATA_LEN;
    for (int j = 0 ; j < len / 2 ; j += 2) {

#if 0
      if (effect_active) {
        data16[j] = (int16_t)((effect_dat[1] << 8) | effect_dat[0]);
        effect_dat += 2;
        effect_len -= 2;

        data16[j+1] = (int16_t)((effect_dat[1] << 8) | effect_dat[0]);
        effect_dat += 2;
        effect_len -= 2;

        if (effect_len <= 0) {
          if (effect_active == EFFECT_SUCCESS || effect_active == EFFECT_CLEAR) {
            gap2next = millis() + 1000;
          }
          effect_active = 0;
        }

        continue;
      }
#endif

//      float raw_noise = random(16384/2) * 2 - 16383/2;
      float raw_noise;
     if (noiseFlag) {
      raw_noise = random(16384/2) * 2 - 16383/2;
      //float raw_noise = random(16384) * 2 - 16383;
      //float raw_noise = random(16384*2) * 2 - 16383*2;
      //float raw_noise = random(16384*4) * 2 - 16383*4;

      // 500Hz
      //float next_noise = 1.9288f * noise - 0.9312f * noise_old + 0.0344f * raw_noise - 0.0344f * raw_old2;

      // 100Hz
      //float next_noise = 1.9566f * noise - 0.9719f * noise_old + 0.0141f * raw_noise - 0.0141f * raw_old2;

      //float next_noise = 1.966927f * noise - 0.971929f * noise_old + 0.014036f * raw_noise - 0.014036f * raw_old2;

      // 50Hz
      //float next_noise = 1.987853f * noise - 0.992908f * noise_old + 0.003546f * raw_noise - 0.003546f * raw_old2;

      // 10Hz
      //float next_noise = 1.993514f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // 200 +- 10Hz
      //float next_noise = 1.997765f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // 300 +- 10Hz
      //float next_noise = 1.996173f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // 400 +- 10Hz
      //float next_noise = 1.995004f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // 400 +- 20Hz
      //float next_noise = 1.993574f * noise - 0.997152f * noise_old + 0.001424f * raw_noise - 0.001424f * raw_old2;

      // 500 +- 10Hz
      //float next_noise = 1.993514f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // White noise 400 +- 10Hz
      //float next_noise = 1.995004f * noise - 0.998576f * noise_old + 0.000712f * raw_noise - 0.000712f * raw_old2;

      // Pink noise 500 +- 10Hz
      //pink_state = 0.95f * pink_state + 0.05f * raw_noise;
      //float next_noise = 1.993514f * noise - 0.998576f * noise_old + 0.000712f * pink_state - 0.000712f * raw_old2;

      // Pink noise 400 +- 10Hz
      //pink_state = 0.95f * pink_state + 0.05f * raw_noise;
      //float next_noise = 1.995332f * noise - 0.998577f * noise_old + 0.000711f * pink_state - 0.000711f * raw_old2;

      // Pink noise 400 +- 100Hz
      //pink_state = 0.95f * pink_state + 0.05f * raw_noise;
      //float next_noise = 1.982637f * noise - 0.985861f * noise_old + 0.028278f * pink_state - 0.028278f * raw_old2;

      // Pink noise 400 +- 50Hz
      //pink_state = 0.95f * pink_state + 0.05f * raw_noise;
      //float next_noise = 1.988960f * noise - 0.992911f * noise_old + 0.014178f * pink_state - 0.014178f * raw_old2;

      // Pink noise 400 +- 20Hz
      pink_state = 0.95f * pink_state + 0.05f * raw_noise;
      float next_noise = 1.993206f * noise - 0.997157f * noise_old + 0.005686f * pink_state - 0.005686f * raw_old2;

      noise_old = noise;
      noise = next_noise;

      //raw_old2 = raw_old;
      //raw_old = raw_noise;
      raw_old2 = pink_state;

      data16[j+1] = data16[j] = (int16_t)noise * 10;
     } else {
      data16[j+1] = data16[j] = 0;
     }

      if (tone_active) {
        data16[j+1] = data16[j] += generate_88sample_wma_square(tone_index++);
      }
      //data16[j+1] = data16[j] = (int16_t)noise * 100;
      //data16[j+1] = data16[j] = (int16_t)noise * 20;

#if 1
      if (effect_active) {
#if 0
        Serial.printf("high: %02x, low: %02x, res: %04x\r\n",
                      effect_dat[1],
                      effect_dat[0],
                      (uint16_t)(int16_t)((effect_dat[1] << 8) | effect_dat[0])
                      );
#endif
        data16[j] += (int16_t)((effect_dat[1] << 8) | effect_dat[0]);
        effect_dat += 2;
        effect_len -= 2;

        data16[j+1] += (int16_t)((effect_dat[1] << 8) | effect_dat[0]);
        effect_dat += 2;
        effect_len -= 2;

        if (effect_len <= 0) {
          /*
           * End of sound effect
           */
          if (effect_active == EFFECT_SUCCESS || effect_active == EFFECT_CLEAR) {
            gap2next = millis() + 1000;
          }
          if (effect_active == EFFECT_CLEAR) {
            void tone_off(int dummyPin);
            void runner_stop();

            tone_off(0);
            runner_stop();
          }
          if (effect_active == EFFECT_START) {
            /* timer start after sound effect finished */
            runner_start = millis();

            void trainingMode(int pressedSw);
            trainingMode(3);
          }
          effect_active = 0;
        }
      }
#endif

    }

    stream.write(data, len);
    size_t res = copier.copy();
    //Serial.printf("res: %d\r\n", res);
    //delay(1);
  }
#endif

  if (effect_active) {
    return;
  }
  if (gap2next != 0) {
    if (millis() >= gap2next) {
      gap2next = 0;
    }
    return;
  }

#if 1
  //extern int push_button;
  //extern int push_last_button;
  //extern unsigned long push_time;
  //extern int push_time_over;
  //extern int push_count;

#if 0
  if (!player.isActive()) {
    Serial.printf("Audio Player is inactive\r\n");
    Serial.printf("Rebooting...\r\n");
    delay(1000);
    ESP.restart();
  }
#endif

  int press_count = push_button_check();
  int flag_reverse = 0;
  int idx = 0;

  extern int bt_prev_pushed, bt_next_pushed;

  if (bt_next_pushed) {
    press_count = 1;
    bt_next_pushed = 0;
  }
  if (bt_prev_pushed) {
    press_count = 2;
    bt_prev_pushed = 0;
  }

  //if (press_count || !player.isActive()) {
  if (press_count != 0) {
#if 0
    if (press_count == 0 || press_count == 1) {
      // 0 means "player is not active"

      if (++playing_idx >= URL_NUM)
          playing_idx = 0;

    } else if (press_count == 2) {
      flag_reverse = 1;

      if (--playing_idx < 0)
          playing_idx = URL_NUM - 1;

    } else if (press_count == 3) {
      playing_idx = 0;

    } else {
      playing_idx = press_count;
      if (playing_idx >= URL_NUM)
        playing_idx = 0;
    }
#endif

#if 0
    if (press_count > 0) {
      // saving index into file
      sprintf(idx_buf, "%d", playing_idx);
      idx = idx_buf;
      write_cfg("/index.txt", idx);
    }
#endif

   if (press_count < 0) {
    // press_count < 0
    press_count = - press_count;
    Serial.printf("\r\n\r\nLong Press count = %d\r\n", press_count);

    if (press_count > 0) {
      tone_off(0);
      runner_stop();

      switch (press_count) {
      default:
      case 1:
        /* WPM adjustment */
        sprintf(adj_wpm_buf, "%d", Wpm);
        adj_wpm = adj_wpm_buf;
        write_cfg("/adj_wpm.txt", adj_wpm);

        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = '0' + (Wpm / 10);
        presetMessage[idx++] = '0' + (Wpm % 10);
        presetMessage[idx++] = 'W';
        presetMessage[idx++] = 'P';
        presetMessage[idx++] = 'M';

        break;

      case 2:
        /* Menu and Status */
        Serial_print_usage();

        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'R';
        presetMessage[idx++] = 'V';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '5';
        presetMessage[idx++] = ' ';
        if (paddleRvs) {
          presetMessage[idx++] = 'C';
        } else {
          presetMessage[idx++] = 'N';
        }
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = 'K';
        presetMessage[idx++] = 'Y';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '6';
        presetMessage[idx++] = ' ';
        switch (keyerType) {
        default:
        case 0:
          keyer = DEF_0;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'A';
          break;
        case 1:
          keyer = DEF_1;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'B';
          break;
        case 2:
          keyer = DEF_2;
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'P';
          break;
        case 3:
          keyer = DEF_3;
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'T';
          break;
        }
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = 'O';
        presetMessage[idx++] = 'D';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '7';
        presetMessage[idx++] = ' ';
        if (offDelay) {
          presetMessage[idx++] = 'C';
        } else {
          presetMessage[idx++] = 'N';
        }
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = 'B';
        presetMessage[idx++] = 'N';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '8';
        presetMessage[idx++] = ' ';
        if (noiseFlag) {
          presetMessage[idx++] = 'C';
        } else {
          presetMessage[idx++] = 'N';
        }
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = 'A';
        presetMessage[idx++] = 'K';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '9';
        presetMessage[idx++] = ' ';
        if (autoKeyerType) {
          presetMessage[idx++] = 'C';
        } else {
          presetMessage[idx++] = 'N';
        }
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = 'A';
        presetMessage[idx++] = 'T';
        presetMessage[idx++] = ' ';
        presetMessage[idx++] = '1';
        presetMessage[idx++] = '0';
        presetMessage[idx++] = ' ';
        switch (autoKeyerTypeElekey) {
        default:
        case 0:
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'A';
          break;
        case 1:
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'B';
          break;
        case 2:
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'P';
          break;
        }

        break;
      }

    }

   } else {
    // press_count >= 0
    Serial.printf("\r\n\r\nShort Press count = %d\r\n", press_count);

    if (press_count > 0) {
      tone_off(0);
      runner_stop();

      switch (press_count) {
      case 1:
      case 3:
      case 4:
        /*
         * Press 1 = Start 20WPM (medium class)
         * Press 3 = Start 15WPM (light class)
         * Press 4 = Start 25WPM (heavy class)
         */
        switch (press_count) {
        case 1:
          runner_type = RUNNER_TYPE_NORMAL;
          runner_clear_count = 10;
          //setNewWpm(20);
          setNewWpm(atoi(adj_wpm));
          
          Serial.printf("Starting 10-QSO Challenge!! (NORMAL class)\r\n");
          break;
        case 3:
          runner_type = RUNNER_TYPE_EASY;
          runner_clear_count = 1;
          setNewWpm(15);
          Serial.printf("Starting 1-QSO Challenge!! (EASY class)\r\n");
          break;
        case 4:
          runner_type = RUNNER_TYPE_HARD;
          runner_clear_count = 10;
          setNewWpm(25);
          Serial.printf("Starting 10-QSO Challenge!! (HARD class)\r\n");
          break;
        }

        effect_dat = snd_start_dat;
        effect_len = snd_start_len;
        effect_active = EFFECT_START;

        break;

      case 2:
      default:
        /* just stop morse runner */
        Serial.printf("Stopped.\r\n");

        break;

      case 5:
        /* reverse */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'R';
        presetMessage[idx++] = 'V';
        presetMessage[idx++] = ' ';

        if (paddleRvs) {
          paddleRvs = 0;
          reverse = "no";
          presetMessage[idx++] = 'N';
        } else {
          paddleRvs = 1;
          reverse = "yes";
          presetMessage[idx++] = 'C';
        }

        Serial.printf("Reverse flag = %s (%d)\r\n", reverse, paddleRvs);

        write_cfg("/reverse.txt", reverse);

        break;

      case 6:
        /* keyer type */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'K';
        presetMessage[idx++] = 'Y';
        presetMessage[idx++] = ' ';

        keyerType++;
        if (keyerType > keyStraight) { // 0:Iambic A, 1:Iambic B, 2:Super keyer, 3:Straight key
          keyerType = 0;
        }

        switch (keyerType) {
        default:
        case 0:
          keyer = DEF_0;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'A';
          break;
        case 1:
          keyer = DEF_1;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'B';
          break;
        case 2:
          keyer = DEF_2;
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'P';
          break;
        case 3:
          keyer = DEF_3;
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'T';
          break;
        }

        Serial.printf("Keyer Type = %d (0:Iambic A, 1:Iambic B, 2:Super keyer)\r\n", keyerType);

        write_cfg("/keyer.txt", keyer);
        break;

      case 7:
        /* off-delay */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'O';
        presetMessage[idx++] = 'D';
        presetMessage[idx++] = ' ';

        if (offDelay) {
          offDelay = 0;
          offdelay = DEF_NO;
          presetMessage[idx++] = 'N';
        } else {
          offDelay = 1;
          offdelay = DEF_YES;
          presetMessage[idx++] = 'C';
        }

        Serial.printf("Off-Delay flag = %s (%d)\r\n", offdelay, offDelay);

        write_cfg("/offdelay.txt", offdelay);
        break;

      case 8:
        /* noise */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'B';
        presetMessage[idx++] = 'N';
        presetMessage[idx++] = ' ';

        if (noiseFlag) {
          noiseFlag = 0;
          noise = DEF_NO;
          presetMessage[idx++] = 'N';
        } else {
          noiseFlag = 1;
          noise = DEF_YES;
          presetMessage[idx++] = 'C';
        }

        Serial.printf("Background Noise flag = %s (%d)\r\n", noise, noiseFlag);

        write_cfg("/noise.txt", noise);
        break;

      case 9:
        /* Autokey flag */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'A';
        presetMessage[idx++] = 'K';
        presetMessage[idx++] = ' ';

        if (autoKeyerType) {
          autoKeyerType = 0;
          autokeyer_flag = DEF_NO;
          presetMessage[idx++] = 'N';
        } else {
          autoKeyerType = 1;
          autokeyer_flag = DEF_YES;
          presetMessage[idx++] = 'C';
        }

        Serial.printf("Auto Keyer detection flag = %s (%d)\r\n", autokeyer_flag, autoKeyerType);

        write_cfg("/autokeyer_flag.txt", autokeyer_flag);
        break;

      case 10:
        /* Autokeyer type */
        clearPresetMessage();
        idx = 0;
        presetMessage[idx++] = 'A';
        presetMessage[idx++] = 'T';
        presetMessage[idx++] = ' ';

        autoKeyerTypeElekey++;
        if (autoKeyerTypeElekey > keySuper) { // 0:Iambic A, 1:Iambic B, 2:Super keyer (keyStraight is not included)
          autoKeyerTypeElekey = 0;
        }

        switch (autoKeyerTypeElekey) {
        default:
        case 0:
          autokeyer_type = DEF_0;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'A';
          break;
        case 1:
          autokeyer_type = DEF_1;
          presetMessage[idx++] = 'I';
          presetMessage[idx++] = 'B';
          break;
        case 2:
          autokeyer_type = DEF_2;
          presetMessage[idx++] = 'S';
          presetMessage[idx++] = 'P';
          break;
        }

        Serial.printf("Auto Keyer Type = %d (0:Iambic A, 1:Iambic B, 2:Super keyer)\r\n", autoKeyerTypeElekey);

        write_cfg("/autokeyer_type.txt", autokeyer_type);
        break;

      }

      Serial.printf("\r\n");
    }
   }

#if 1
    push_button_reset();
#endif

      //out.reset_a2dp_buf();

#if 0
      int rv;
      do {
        //rv = player.next();
        Serial.printf("Play index = %d\r\n", playing_idx);
        rv = player.setIndex(playing_idx);
        //Serial.printf("Result: %d\r\n", rv);
        if (rv != 1) {
          if (flag_reverse) {
            if (--playing_idx < 0)
              playing_idx = URL_NUM - 1;
          } else {
            if (++playing_idx >= URL_NUM)
              playing_idx = 0;
          }
        }
      } while (rv != 1);
#endif

  }

  //player.copy();
#endif

#if 1
  void loop_runner();
  loop_runner();
  delay(1);
#endif
}
