/*
 * Pug Radio!!
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

#define APP_VERSION "1.42 (2025/06/08)"

#if 0 /* 1 if enabling Ethernet port */
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
  "https://wgbh-live.streamguys1.com/wgbh", // GBH 89.7 NPR (44.1kHz/16bit/stereo)

  "https://war.streamguys1.com:7785/wuis.mp3", // NPR Illinois 91.9 UIS
  "https://whyy.streamguys1.com/whyy-mp3", // WHYY NPR

  "https://npr-ice.streamguys1.com/live.mp3", // KRVS-HD3, WKCR-HD3, NPR News and Talk

  "https://streams.kqed.org/kqedradio", // KQED, 32kbps, mono
  "https://playerservices.streamtheworld.com/api/livestream-redirect/KVPRFM.mp3", // Valley Public Radio (KVPR)

  //"https://streaming.azpm.org/kuaz128.mp3", // NPR 89.1 Arizona Public Media (48kHz,16bit,stereo)

  "https://utulsa.streamguys1.com/KWGSHD3.mp3", // KWGS HD3 BBC (44.1kHz/16bit/mono)

  "https://streaming.live365.com/a77334", // Smooth Jazz Global Radio

  "https://live.radioart.com/fSmooth_jazz.mp3", // Radio Art - Smooth Jazz
  "https://live.radioart.com/fSmooth_bossa_nova.mp3", // Radio Art - Smooth Bossa Nova

  "https://kpop.onlyhit.us/play",     // OnlyHit K-Pop
  "https://j.onlyhit.us/play",        // OnlyHit Japan
  "https://api.onlyhit.us/tophits",   // OnlyHit Top Hits
  "https://api.onlyhit.us/play",      // OnlyHit
  "https://gold.onlyhit.us/play",     // OnlyHit Gold

  "https://wgbh-live.streamguys1.com/classical-hi/", // Classic, CRB 99.5
};


#include <SPI.h>

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

//typedef int16_t sound_t;                                   // sound will be represented as int16_t (with 2 bytes)
//AudioInfo sineInfo(44100, 2, 16);
//SineWaveGenerator<sound_t> sineWave(32000);                // subclass of SoundGenerator with max amplitude of 32000
//GeneratedSoundStream<sound_t> sound(sineWave);             // Stream generated from sine wave



//#include <SSLClient.h>
//#include "trust_anchors.h"
////SSLClient ssl_client(client, TAs, (size_t)TAs_NUM, A7);
//SSLClient ssl_client(client, TAs, (size_t)TAs_NUM, A6);

//#define BUF_SIZE (DEFAULT_BUFFER_SIZE / 2)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE)
/* attempting to increase BUF_SIZE from "* 4" to "* 8" ; does not seem effective */

#define BUF_SIZE (DEFAULT_BUFFER_SIZE)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 2)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 4)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 8)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 16)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 32)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 64)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 128)
//#define BUF_SIZE (DEFAULT_BUFFER_SIZE * 256)

//const char *wifi = "xxx";
//const char *password = "xxx";

#ifndef USE_ETHER
URLStream urlStream(BUF_SIZE);
#endif
//URLStream urlStream(wifi, password, BUF_SIZE);
//URLStream urlStream(wifi, password);

//#include "AudioHttp/URLStreamBuffered.h"

#ifdef USE_ETHER
#if 0
URLStreamBuffered urlStream(/*ssl_client,*/ BUF_SIZE/*DEFAULT_BUFFER_SIZE*/);
#else
URLStream urlStream(/*ssl_client,*/ BUF_SIZE/*DEFAULT_BUFFER_SIZE*/);
#endif
#endif /* USE_ETHER */

//HLSStream urlStream(ssl_client, DEFAULT_BUFFER_SIZE);
//URLStream urlStream(client);

//HLSStream hls_stream(wifi, password);

//AudioSourceURL source(urlStream, (const char **)urls, "audio/mp3");
// MP3 -> AAC
AudioSourceURL source(urlStream, urls, "audio/mp3,audio/mpeg,audio/aac");
//AudioSourceURL source(urlStream, urls, "audio/mpeg");
//AudioSourceURL source(urlStream, urls, "audio/aac");

AudioInfo info(44100,2,16);
//AudioInfo info(44100,1,16);
//AudioInfo info(22050,1,16);

//#include <driver/dac.h>
//AnalogAudioStream out;

A2DPStream out;
//ResampleStream resample(out);

//StreamCopy copier(out, sound);                             // copies sound into i2s

//MTSDecoder mts;
//AACDecoderHelix aac;

//EncodedAudioStream aac_stream(&out, &aac); 
//EncodedAudioStream mts_stream(&aac_stream, &mts);
//StreamCopy copier(mts_stream, hls_stream);

// MP3 -> AAC
MP3DecoderHelix decoder;
AACDecoderHelix decoderAAC;
//AudioPlayer player(source, out, decoder);
//AudioPlayer player(source, resample, decoder);

FormatConverterStream conv(out);
AudioPlayer player(source, out, decoder, conv);
//AudioPlayer player(source, out, decoder);

//AudioPlayer player(source, conv, decoder);
//AudioPlayer player(source, out, aac);

//addNotifyAudioChange(AudioInfoSupport *notify)
//addNotifyAudioChange(AudioInfoSupport *notify)

#define PIN_CS    33
byte mac[] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x01 };

W5500Driver driver(PIN_CS);
//W5500Driver driver;

#include "FS.h"
#include "SPIFFS.h"

#if 0
size_t availableMemory() {
  
  size_t size = 1024 * 1024; // 最大サイズ
  size_t step = 128 * 1024; // 探索ステップ
  
  while (true) {
    byte *buf;
    while ((buf = (byte *) malloc(size)) == NULL) {
      size -= step;
    }
    free(buf);
    //Serial.printf("step=%d, size=%d\n", step, size);
    step /= 2;
    if (step == 0) break;
    size += step * 2;
    delay(1);
  }

  return size;
}
#endif

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
      }
    } else {
      //Serial_printf("empty file: [%s]\r\n", cfg_file);
    }
  } else {
    Serial_printf("cannot open: [%s]\r\n", cfg_file);
  }

#if 0
  if (flag_print)
    //Serial_printf("%s\r\n", ptr);
    Serial_printf("%s\r\n", cfg_file, ptr);
  else
    //Serial_printf("*****\r\n", ptr);
    Serial_printf("*****\r\n", cfg_file, ptr);
#endif

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
      write_cfg("/passwd.txt", sys_event->event_info.prov_cred_recv.password);

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

void processEndCallback(URLStream& stream, HttpRequest& request, Url& url)
{
  //Serial.printf("connectCallback: URL = %s\n", url.url());
  //Serial.printf("connectCallback: request.reply().statusCode() %d\n", request.reply().statusCode());
  Serial.printf("Content-Type: %s\r\n", request.reply().get(CONTENT_TYPE));

  if (strstr(request.reply().get(CONTENT_TYPE), "audio/aac")) {
    Serial.printf("Changing decoder to AAC\r\n");
    player.setDecoder(decoderAAC);
  } else {
    Serial.printf("Changing decoder to MP3\r\n");
    player.setDecoder(decoder);
  }
}

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

#define PORT_BUF_SIZE 32
#ifdef BT_WIFI_ETHER
static char *port = "eth", port_buf[PORT_BUF_SIZE];
#else
static char *port = "wifi", port_buf[PORT_BUF_SIZE];
#endif

#define SSID_BUF_SIZE 32
static char *ssid = "ssid", ssid_buf[SSID_BUF_SIZE];

#define PASSWD_BUF_SIZE 32
static char *passwd = "passwd", passwd_buf[PASSWD_BUF_SIZE];

#define VOL_BUF_SIZE 32
static char *vol = DEF_VOL, vol_buf[VOL_BUF_SIZE];

#define IDX_BUF_SIZE 32
#define DEF_IDX "0"
static char *idx = DEF_IDX, idx_buf[IDX_BUF_SIZE];

static int playing_idx = 0;

void Serial_print_config()
{
    Serial_printf("  BT: %s\r\n", bt);
    Serial_printf("  PAIRING: %s\r\n", pairing);
    Serial_printf("  RSSI: %s\r\n", rssi);
//#ifndef USE_ETHER
    Serial_printf("  PORT: %s\r\n", port);
    Serial_printf("  SSID: %s\r\n", ssid);
    Serial_printf("  PASSWD: *****\r\n");
//#endif /* !USE_ETHER */
    for (int i = 0 ; i < URL_NUM ; i++) {
      Serial_printf("  URL%d: %s\r\n", i, urlp[i]);
    }
    Serial_printf("  INDEX: %s\r\n", idx);
    Serial_printf("  VOLUME: %s\r\n", vol);

    Serial_printf("\r\n");
}

#include "esp_mac.h"

uint8_t macaddr_wifi[8];
uint8_t macaddr_eth[8];
uint8_t macaddr_bt[8];

#define MACADDR_DIGITS(p)  p[0], p[1], p[2], p[3], p[4], p[5]

void setup() {
  Serial.begin(115200);

  esp_read_mac(macaddr_wifi, ESP_MAC_WIFI_STA);
  esp_read_mac(macaddr_eth,  ESP_MAC_ETH);
  esp_read_mac(macaddr_bt,   ESP_MAC_BT);

  Serial.printf("\r\n************************************************************************\r\n\r\n");
  Serial.printf("  Welcome to Pug Radio!!\ " FEATURE " Ver. %s\r\n", APP_VERSION);
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

  bt = read_cfg("/bt.txt", bt_buf, BT_BUF_SIZE, bt, 1, 0);
  pairing = read_cfg("/pairing.txt", pairing_buf, PAIRING_BUF_SIZE, pairing, 1, 0);
  rssi = read_cfg("/rssi.txt", rssi_buf, RSSI_BUF_SIZE, rssi, 1, 0);
  port = read_cfg("/port.txt", port_buf, PORT_BUF_SIZE, port, 1, 0);

  ssid = read_cfg("/ssid.txt", ssid_buf, SSID_BUF_SIZE, ssid, 1, 0);
  passwd = read_cfg("/passwd.txt", passwd_buf, PASSWD_BUF_SIZE, passwd, 0, 0);

#if 1
  for (int i = 0 ; i < URL_NUM ; i++) {
    urlp[i] = (char *)urls[i];
#define PATH_BUF_SIZE 32
    char buf[PATH_BUF_SIZE];
    sprintf(buf, "/url%d.txt", i);
    urlp[i] = read_cfg(buf, url_buf[i], URL_BUF_SIZE, urlp[i], 1, 1);
  }
#endif

  idx = read_cfg("/index.txt", idx_buf, IDX_BUF_SIZE, idx, 1, 0);
  if (atoi(idx) < 0 || atoi(idx) >= URL_NUM) {
    idx = DEF_IDX;
  }

  vol = read_cfg("/volume.txt", vol_buf, VOL_BUF_SIZE, vol, 1, 0);

#if 1
  pinMode(0/*Boot button*/, INPUT_PULLUP);
  int state = digitalRead(0);

  if (state == LOW) {

    esp_err_t ret = nvs_flash_init();
    if (ret == 0) {
      ret = nvs_flash_erase();
    }
    Serial.printf("\r\nRemoving BT pairing... (%d)\r\n", ret);

#if 0
    if (btStarted() || btStart()) {
      esp_bluedroid_init();
      esp_bluedroid_enable();

      int expected_dev_num = esp_bt_gap_get_bond_device_num();
      Serial.printf("expected_dev_num: %d\r\n", expected_dev_num);

      if (expected_dev_num > 0) {
        esp_bd_addr_t *dev_list = (esp_bd_addr_t *)malloc(sizeof(esp_bd_addr_t) * expected_dev_num);

        if (dev_list != NULL) {
          int dev_num = expected_dev_num;
          int ret = esp_bt_gap_get_bond_device_list(&dev_num, dev_list);

          Serial.printf("dev_num: %d\r\n", dev_num);
          Serial.printf("ret: %d\r\n", ret);

          if (ret == ESP_OK) {
            for (int i = 0 ; i < dev_num ; i++) {
              ret = esp_bt_gap_remove_bond_device(dev_list[i]);
              if (ret == ESP_OK) {
                Serial.printf("Removed: %02x:%02x:%02x:%02x:%02x\r\n", dev_list[i][0], dev_list[i][1], dev_list[i][2], dev_list[i][3], dev_list[i][4], dev_list[i][5]);
              } else {
                Serial.printf("Failed to remove: %02x:%02x:%02x:%02x:%02x\r\n", dev_list[i][0], dev_list[i][1], dev_list[i][2], dev_list[i][3], dev_list[i][4], dev_list[i][5]);
              }
            }
          }
        }
      }
    } else {
      Serial.printf("BT could not be initialized...\r\n");
    }
#endif

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
    Serial.printf("  Welcome to Pug Radio!!\ " FEATURE " Ver. %s\r\n", APP_VERSION);
    Serial.printf("  Wi-Fi MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", MACADDR_DIGITS(macaddr_wifi));
    Serial_printf("Setup mode. Please input values.\r\n");
    Serial_printf("\r\n************************************************************************\r\n\r\n");

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

#ifdef BT_WIFI_ETHER
//#ifndef USE_ETHER
    Serial_printf("PORT (%s) [wifi or eth]: ", port);
    port = read_serial(port_buf, PORT_BUF_SIZE, port);
    if (!strcmp("wifi", port) and !strcmp("eth", port)) {
      port = "wifi";
    }
#endif

    Serial_printf("SSID (%s): ", ssid);
    ssid = read_serial(ssid_buf, SSID_BUF_SIZE, ssid);

    Serial_printf("PASSWD (*****): ");
    passwd = read_serial(passwd_buf, PASSWD_BUF_SIZE, passwd);
//#endif /* !USE_ETHER */

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

    Serial_printf("\r\n*** Results (to be new values) ***\r\n");
    Serial_print_config();

    Serial_printf("Are you sure to overwrite settings? [y/N]: ");
    answer = read_serial(answer_buf, ANSWER_BUF_SIZE, answer);
    if (answer[0] == 'y' || answer[0] == 'Y') {
     if (!strcmp(ssid, "reset")) {
      SPIFFS.remove("/bt.txt");
      SPIFFS.remove("/pairing.txt");
      SPIFFS.remove("/rssi.txt");
//#ifndef USE_ETHER
      SPIFFS.remove("/port.txt");
      SPIFFS.remove("/ssid.txt");
      SPIFFS.remove("/passwd.txt");
//#endif /* !USE_ETHER */
      for (int i = 0 ; i < URL_NUM ; i++) {
        char buf[PATH_BUF_SIZE];
        sprintf(buf, "/url%d.txt", i);
        SPIFFS.remove(buf);
      }
      SPIFFS.remove("/index.txt");
      SPIFFS.remove("/volume.txt");
     } else {
      write_cfg("/bt.txt", bt);
      write_cfg("/pairing.txt", pairing);
      write_cfg("/rssi.txt", rssi);
//#ifndef USE_ETHER
      write_cfg("/port.txt", port);
      write_cfg("/ssid.txt", ssid);
      write_cfg("/passwd.txt", passwd);
//#endif /* !USE_ETHER */
      for (int i = 0 ; i < URL_NUM ; i++) {
        char buf[PATH_BUF_SIZE];
        sprintf(buf, "/url%d.txt", i);
        write_cfg(buf, urlp[i]);
      }
      write_cfg("/index.txt", idx);
      write_cfg("/volume.txt", vol);
     }

      //Serial_printf("SPIFFS was overwritten.\r\n");
      break;
    } else {
      //Serial_printf("Aborted.\r\n");
    }
   } while (!(answer[0] == 'y' || answer[0] == 'Y'));

    SPIFFS.end();

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

#if 1
//#define URL_BUF_SIZE 128
  for (int i = 0 ; i < URL_NUM ; i++) {
    ((char **)urls)[i] = urlp[i];
  }
#endif

#if 0
  SPIFFS.end();
#endif

  int auto_reconnect = 0;

  if (pairing[0] == 'y') {
    auto_reconnect = 1;
  }

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
  }
#endif

#if 0 /*def BT_WIFI_ETHER*/
  /* XXX */
  ssl_client.setInsecure();
  ssl_client.setHandshakeTimeout(URL_HANDSHAKE_TIMEOUT / 1000); /* seconds */

  /* XXX narita 2025/3/9 */
  if (strstr(urls[0], "http://") || strstr(urls[0], "HTTP://")) {
    if (use_ether) {
      //urlStream.setClient(ether_client);
    } else {
      //urlStream.setClient(wifi_client);
    }
  } else {
    //urlStream.setClient(ssl_client);
  }
#endif

  urlStream.setProcessEndCallback(processEndCallback);

  /* XXX narita 2025/3/29 */
  //if (strstr(urls[0], ".aac") || strstr(urls[0], ".AAC")) {
  //  player.setDecoder(decoderAAC);
  //}

#if 0 /*def USE_ETHER*/
  if (use_ether == 1) {
    ssl_client.setInsecure();
  // ssl_client.setBufferSizes(2048 /* rx */, 512 /* tx */);
// does not work for playing on and off
//  ssl_client.setBufferSizes(2048 /* rx */, 1024 /* tx */);

    if (use_ether) {
      //ssl_client.setClient(&ether_client);
    } else {
      //ssl_client.setClient(&wifi_client);
    }
  }
#endif /* USE_ETHER */

#if 0
  if (client.available() > 0) {
    Serial.println("Client is available.");
  } else {
    Serial.println("Client is NOT available.");
  }
#endif

  //esp_netif_init();
  //urlStream.begin("http://stream.srg-ssr.ch/m/rsj/mp3_128","audio/mp3");

#if 0
  client.setConnectionTimeout(60000);
#endif

#if 0
  Serial.println("Test connecting...");

  client.setConnectionTimeout(60000);
  if (client.connect("ice1.somafm.com", 80)) {
    Serial.println("connected");
    client.stop();
  } else {
    Serial.println("connection failed");
  }
#endif

  //AudioLogger::instance().begin(Serial, AudioLogger::Info);
  AudioLogger::instance().begin(Serial, AudioLogger::Warning);
  //AudioLogger::instance().begin(Serial, AudioLogger::Debug);

  // setup output - We send the test signal via A2DP - so we conect to a Bluetooth Speaker

#if 0
  // define resampling info
  auto rcfg = resample.defaultConfig();
  rcfg.copyFrom(info);
  rcfg.to_sample_rate = 44100;
  rcfg.buffer_size = DEFAULT_BUFFER_SIZE;
  //rcfg.buffer_size = DEFAULT_BUFFER_SIZE * 2;
  //rcfg.step_size = 22050.0 / 44100.0;
  rcfg.step_size = 32000.0 / 44100.0;
  //rcfg.step_size = 0.5;
  //rcfg.step_size = 0.25;
  resample.begin(rcfg);
#endif

#if 0
  //dac_output_enable(DAC_CHANNEL_1);
  //dac_output_enable(DAC_CHANNEL_2);

  auto cfg = out.defaultConfig(TX_MODE);
  cfg.copyFrom(info);
  out.begin(cfg);
#endif

#if 1
  auto cfg = out.defaultConfig(TX_MODE);
  //cfg.copyFrom(info);
  //cfg.name = "[Unknown]";
  cfg.name = bt;
  //cfg.name = "SR1323";
  //cfg.auto_reconnect = true;  // if this is use we just quickly connect to the last device ignoring cfg.name
  //cfg.auto_reconnect = false;  // if this is use we just quickly connect to the last device ignoring cfg.name
  cfg.auto_reconnect = auto_reconnect;

  //cfg.delay_ms = 0;
  cfg.delay_ms = 1;
  //cfg.delay_ms = 2;
  //cfg.delay_ms = 3;
  //cfg.delay_ms = 10;

  //cfg.startup_logic = StartOnConnect; //StartWhenBufferFull;
  //cfg.startup_nodata = A2DPWhoosh;
  //cfg.buffer_size = 1024 * 8;
#ifdef BUF_16KB
  cfg.buffer_size = 1024 * 16; // moderate
#else
  //cfg.buffer_size = 1024 * 256;
  //cfg.buffer_size = 1024 * 512; // 512KB
  //cfg.buffer_size = 1024 * 1024 * 1; // 1MB
  //cfg.buffer_size = 1024 * 1024 * 2; // 2MB
  //cfg.buffer_size = 1024 * 1024 * 5 / 2; // 2.5MB
  cfg.buffer_size = 1024 * 1024 * 3; // 3MB (larger buffer especially for AAC)
#endif /* BUF_16KB */
  //cfg.buffer_size = 1024 * 20; // too many putu putu without underrun
  //cfg.buffer_size = 1024 * 24; // badly worked on+off
  //cfg.buffer_size = 1024 * 32; // failed
  //cfg.buffer_size = 1024 * 64; // allocatable??, but does not proceed
  //cfg.buffer_size = 1024 * 256; // failed
  //cfg.buffer_size = 1024 * 32;
  //cfg.buffer_size = 1024 * 20;
  //cfg.buffer_size = 1024 * 8;
  //cfg.buffer_size = 1024 * 4;
  //cfg.silence_on_nodata = true;

  //cfg.sample_rate = 22050;
  //cfg.channels = 1;
  //cfg.bits_per_sample = 16;

  out.set_a2dp_rssi(atoi(rssi));

  out.begin(cfg);
#endif

#if 0
  //conv.begin(info);

//  mts_stream.begin();
//  aac_stream.begin();
#endif

#if 1
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
  if (rv == false) {
    Serial.println("Starting Audio Player failed.");
    Serial.println("Rebooting...");
    delay(1000);
    ESP.restart();
  }
#if 0
  while (rv == false) {
    rv = player.next();
    Serial.printf("setup(): next() rv: %d\r\n", rv);
    Serial.println("setup(): Play Next");
  }
#endif
#endif

#if 0
  //out.begin(TX_MODE);
  mts_stream.begin();
  aac_stream.begin();

  hls_stream.begin("https://hls.kqed.org/hls/kqed_app/playlist.m3u8");

#endif

#if 0
  // Setup sine wave
  sound.begin(sineInfo);
  sineWave.begin(sineInfo, N_B4);
#endif

#if 0
  char *ptr = (char *)malloc(1024 * 8);
  if (ptr) {
    Serial.println("MALLOC success!!");
  } else {
    Serial.println("MALLOC failed.");
  }
#endif
}

#if 1
//#define PUSH_TIMEOUT 500 /* msec */
#define PUSH_TIMEOUT 1000 /* msec */
int push_last_button = HIGH;
unsigned long push_time = 0;
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
  int push_button = digitalRead(0);

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

    // button released

  } else if (/* here button released && */ push_time > 0 && millis() - push_time > PUSH_TIMEOUT) {
    // button push timeout
    push_time = 0;

    push_time_over = 1;
  }

  if (push_time_over)
    return push_count;
  else
    return 0;
}

void loop() {
#if 0
  static int a2dp_ready = 0;

  if (a2dp_ready == 0) {
    if (out.is_a2dp_connected()) {
      Serial.println("A2DP connected");
      a2dp_ready = 1;
    }
  } else {
    if (out.is_a2dp_disconnected()) {
      a2dp_ready = 0;
      Serial.println("A2DP disconnected");
      delay(1000);
      abort();
    }
  }
#endif

#if 1
  //extern int push_button;
  //extern int push_last_button;
  //extern unsigned long push_time;
  //extern int push_time_over;
  //extern int push_count;

  if (!player.isActive()) {
    Serial.printf("Audio Player is inactive\r\n");
    Serial.printf("Rebooting...\r\n");
    delay(1000);
    ESP.restart();
  }

  int press_count = push_button_check();

  //if (press_count || !player.isActive()) {
  if (press_count) {
    if (press_count == 0 || press_count == 1) {
      // 0 means "player is not active"

      if (++playing_idx >= URL_NUM)
          playing_idx = 0;

    } else if (press_count == 2) {
      if (--playing_idx < 0)
          playing_idx = URL_NUM - 1;

    } else if (press_count == 3) {
      playing_idx = 0;

    } else {
      playing_idx = press_count;
      if (playing_idx >= URL_NUM)
        playing_idx = 0;
    }

#if 1
    if (press_count > 0) {
      // saving index into file
      sprintf(idx_buf, "%d", playing_idx);
      idx = idx_buf;
      write_cfg("/index.txt", idx);
    }
#endif

#if 1
    push_button_reset();
#endif

#if 0
      UBaseType_t uxPriority_org;
      uxPriority_org = uxTaskPriorityGet(NULL);
      //printf("(1) Loop Task is running with %d priority\r\n", uxPriority_org);

      vTaskPrioritySet(NULL, 24);

      UBaseType_t uxPriority_new;
      uxPriority_new = uxTaskPriorityGet(NULL);
      //printf("(2) Loop Task is running with %d priority\r\n", uxPriority_new);

#endif

      out.reset_a2dp_buf();

#if 0
      vTaskPrioritySet(NULL, uxPriority_org);

      uxPriority_new = uxTaskPriorityGet(NULL);
      //printf("(3) Loop Task is running with %d priority\r\n", uxPriority_new);
#endif

      int rv;
      do {
        //rv = player.next();
        Serial.printf("Play index = %d\r\n", playing_idx);
        rv = player.setIndex(playing_idx);
        Serial.printf("Result: %d\r\n", rv);
        if (rv != 1) {
          if (++playing_idx >= URL_NUM)
            playing_idx = 0;
        }
      } while (rv != 1);

  }

  player.copy();

#if 0
  if (!player.isActive()) {
    Serial.printf("Audio Player is inactive\r\n");
    delay(1000);
    ESP.restart();
  }
#endif
#endif

#if 0
  static bool flag_buffered = false;
//  copier.copy();
  if (flag_buffered) {
    player.copy();
  } else {
    delay(1);
    Serial.printf("Buffer: %d\r\n", player.available());
    if (player.available() >= BUF_SIZE * 8 / 10) {
      Serial.println("Buffered!");
      flag_buffered = true;
    }
  }
#endif

#if 0
  copier.copy();
#endif

#if 0
  copier.copy();
#endif

  //delay(1);
}
