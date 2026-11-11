/*
 * Texecom Premier 412 - Seri Dinleyici / Köprü
 * Donanım: NodeMCU V3 (ESP8266). RX = D7 (GPIO13, lojik dönüştürücü üzerinden), TX = D5 (GPIO14, doğrudan).
 * Panel hattı 19200 8N2. 8N1 gönderilirse panel her Z'ye 03 0F ED (kapat) der.
 *
 * Neden yazılımsal seri: lojik analizör (scope) D7'deki sinyalin kusursuz olduğunu, panelin her Z'ye cevap
 * verdiğini gösterdi; ama donanım UART0 (swap, GPIO13) cevapların başını bozuyor ya da hiç görmüyordu.
 * GPIO3 (asıl RX) kullanılamaz: kart üstündeki CH340'ın TX'i hattı 1'de tutuyor.
 *
 * Serial'e debug yazdırmıyoruz; tüm kayıtlar Wi-Fi üzerinden gelir.
 *
 * İki port:
 *   23     -> MONİTÖR (dinleme + komutlar). Windows telnet yerine PuTTY "Raw" veya `nc <ip> 23` kullan.
 *             Varsayılan PASİF: panele HİÇBİR ŞEY göndermez. Göndermek için "tx on" gerekir.
 *   10001  -> KÖPRÜ (ham TCP <-> UART). Wintex'i buradan bağlayınca iki yöndeki
 *             tüm trafik monitörde TX/RX olarak kaydedilir (ters mühendislik için).
 *
 * Wintex çözücü: iki yöndeki trafikte geçerli Wintex çerçevelerini (uzunluk + checksum)
 * yakalar, ham satırın altına ">> WX ..." açıklaması basar (komut, adres, uzunluk).
 *
 * Güncellemeler Wi-Fi (OTA) ile yapılır. Alternatif pinlerde USB ile yüklerken kablo sökmek gerekmez.
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <time.h>

// ---------------- AYARLAR ----------------
#include "secrets.h"  // WIFI_SSID, WIFI_PASS, OTA_PASS, PANEL_UDL, TG_TOKEN, TG_CHAT_ID
#include "telegram_root.h"
#include <EEPROM.h>
#include <stdarg.h>
static const char*    HOSTNAME    = "texecom-sniffer";
static const uint16_t MON_PORT    = 23;
static const uint16_t BRIDGE_PORT = 10001;

// RX: donanım UART0, Serial.swap() ile D7 (GPIO13).
// TX: çalışma anında "txpin" ile seçilir. d4 = donanım UART1 (GPIO2); d5/d6/d1/d2 = EspSoftwareSerial
//     (bayt gönderirken kesmeler kapalı, zamanlama hassas).
// 2026-11-08: panel D4'ten gelen çerçeveleri kabul etmedi, aynı baytları Pico'dan kabul etti → varsayılan D5.
// D5 açılışta sinyal üretmez, yani kutu yeniden başlarken panele çöp gitmez.
struct TxPinDef { const char* name; int8_t gpio; };
// "tx" = GPIO1 (kartın TX pini, CH340'ın RX'ine bağlı): USB takılıyken gönderilen veri bilgisayarda COM portundan okunabilir.
static const TxPinDef TX_PINS[] = {{"d4", 2}, {"d5", 14}, {"d6", 12}, {"d1", 5}, {"d2", 4}, {"tx", 1}};
static int8_t g_txGpio = 14;   // D5
static SoftwareSerial swTx;
static bool g_swActive = false;
// Açık drenaj: pin sadece 0'a çeker, 1 için hattı bırakır; panelin RX'i hattı kendisi ~4.8V'a çekiyor
// (2026-11-08 ölçümü). Push-pull 3.3V yüksek seviye panel girişine yetmiyor olabilir. "txod on|off".
static bool g_txOpenDrain = false;
static const uint8_t ACT_LED = 16;  // etkinlik LED'i D0 (kartta LED yoksa etkisiz)

// Panel tarafı seri ayarlar: 19200 8N2 (lojik analizörle doğrulandı, iki yönde de 2 stop biti).
// Komutla değiştirilebilir: "cfg 8n1"
static uint32_t     g_baud      = 19200;
static SerialConfig g_cfg       = SERIAL_8N2;
static const bool   g_rxSw      = true;   // D7'den yazılımsal alıcı. Donanım UART0 (swap) cevapların başını bozuyordu.
static uint16_t     g_gapMs     = 25;     // bu kadar sessizlik = çerçeve sonu
static bool         g_txEnabled = false;  // monitörden panele gönderme kilidi

// ---------------- DEĞİŞKENLER ----------------
WiFiServer monServer(MON_PORT);
WiFiServer bridgeServer(BRIDGE_PORT);
WiFiClient mon;
WiFiClient bridge;

static const size_t RING_SZ = 1536;      // Son ~1.5 KB log bellekte tutulur, bağlanınca gösterilir (RAM TLS'e lazım)
static char   ring[RING_SZ];
static size_t ringHead = 0;
static bool   ringFull = false;

struct Frame {
  uint8_t  buf[128];
  size_t   n = 0;
  uint32_t firstMs = 0;
  uint32_t lastMs = 0;
  time_t   firstWall = 0;
  // Wintex çözücü tamponu (ham gösterimden bağımsız)
  uint8_t  wb[256];
  uint16_t wn = 0;
  uint32_t wLast = 0;
};
static Frame rxF, txF;

static uint32_t g_rxTotal = 0, g_txTotal = 0, g_wxFrames = 0;
static uint32_t ledOffAt = 0;

// ---------------- ÖN TANIMLAR ----------------
static void emitFrame(Frame& f, const char* dir);
static void frameAdd(Frame& f, uint8_t b, const char* dir);
static void applyUart();
static void handleCommand(String line);
static void sendToPanel(const uint8_t* data, size_t n);

// ---------------- LOG ----------------
static void ringPut(const char* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    ring[ringHead] = s[i];
    ringHead = (ringHead + 1) % RING_SZ;
    if (ringHead == 0) ringFull = true;
  }
}

static void ringDump(Print& out) {
  if (ringFull) out.write((const uint8_t*)ring + ringHead, RING_SZ - ringHead);
  out.write((const uint8_t*)ring, ringHead);
}

// ---------------- SESSİZ PENCERE ----------------
// Wi-Fi yayını panelin cevabına denk gelince RX hattı bozuluyor: cevabın başı kayboluyor ya da çöp geliyor.
// Panele yazdıktan sonra cevap bitene kadar TCP yazmaları ertelenir, pencere kapanınca topluca gönderilir.
static uint16_t g_quietMs = 40;            // TX sonrası en çok bu kadar ms Wi-Fi'ye yazma (0 = kapalı); panel 25 ms'de de cevap verebiliyor
static const uint8_t QUIET_RX_TAIL_MS = 4; // son RX baytından sonra bu kadar ms daha bekle
static bool     g_quietActive = false;
static uint32_t g_quietUntil = 0;
static char     monPend[512];
static size_t   monPendN = 0;
static uint8_t  brPend[512];
static size_t   brPendN = 0;

static void quietArm(uint32_t ms) {
  if (!g_quietMs) return;
  g_quietUntil = millis() + ms;
  g_quietActive = true;
}

static void quietFlush() {
  g_quietActive = false;
  if (brPendN && bridge && bridge.connected()) bridge.write(brPend, brPendN);
  brPendN = 0;
  if (monPendN && mon && mon.connected()) mon.write((const uint8_t*)monPend, monPendN);
  monPendN = 0;
}

static void monSend(const char* s, size_t n) {
  if (!(mon && mon.connected())) return;
  if (g_quietActive) {
    size_t k = n < sizeof monPend - monPendN ? n : sizeof monPend - monPendN;
    memcpy(monPend + monPendN, s, k);
    monPendN += k;
    return;
  }
  mon.write((const uint8_t*)s, n);
}

static void bridgeSend(const uint8_t* d, size_t n) {
  if (!(bridge && bridge.connected())) return;
  if (g_quietActive && n <= sizeof brPend - brPendN) {
    memcpy(brPend + brPendN, d, n);
    brPendN += n;
    return;
  }
  bridge.write(d, n);
}

// Hem kayda (ring) hem bağlı monitöre yazar
static void logOut(const char* s, size_t n) {
  ringPut(s, n);
  monSend(s, n);
}

static void monWrite(const char* s) {
  monSend(s, strlen(s));
}

static void emitFrame(Frame& f, const char* dir) {
  if (!f.n) return;
  char line[800];
  char wall[16] = "--:--:--";
  if (f.firstWall > 1700000000) {
    struct tm t;
    localtime_r(&f.firstWall, &t);
    snprintf(wall, sizeof wall, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  }
  int p = snprintf(line, sizeof line, "[%s +%lu.%03lu] %s %3u B: ", wall,
                   (unsigned long)(f.firstMs / 1000), (unsigned long)(f.firstMs % 1000),
                   dir, (unsigned)f.n);
  for (size_t i = 0; i < f.n && p < (int)sizeof line - 8; i++)
    p += snprintf(line + p, sizeof line - p, "%02X ", f.buf[i]);
  p += snprintf(line + p, sizeof line - p, "| ");
  for (size_t i = 0; i < f.n && p < (int)sizeof line - 4; i++) {
    uint8_t c = f.buf[i];
    line[p++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
  }
  line[p++] = '\r';
  line[p++] = '\n';
  line[p] = 0;
  logOut(line, p);
  f.n = 0;
}

// ---------------- WINTEX ÇÖZÜCÜ ----------------
// Çerçeve: [uzunluk][komut][veri...][checksum]
//   uzunluk : kendisi ve checksum dahil toplam bayt
//   checksum: önceki tüm baytların toplamı XOR 0xFF
// O/R/I/W verisi: [adres 3 bayt, büyük uç][n][n bayt veri (yazma/cevap)]
// Kaynak: shuckc/pialarm wintex-protocol.md, RoganDawes/esphome wintex bileşeni.
static const char* wintexName(uint8_t c, bool fromPanel) {
  if (fromPanel) {
    switch (c) {
      case 'Z': return "oturum-cevap";
      case 'I': return "konfig-veri";
      case 'W': return "durum-veri";
      case 'P': return "pong";
      case 0x06: return "ACK";
      case 0x0F: return "kapat";
    }
  } else {
    switch (c) {
      case 'Z': return "oturum/giris";
      case 'O': return "konfig-oku";
      case 'R': return "durum-oku";
      case 'I': return "KONFIG-YAZ";
      case 'W': return "DURUM-YAZ";
      case 'U': return "COMMIT";
      case 'K': return "tus";
      case 'A': return "KUR";
      case 'D': return "COZ";
      case 'S': return "KISMI-KUR";
      case 'C': return "reset";
      case 'H': return "kapat";
      case 'P': return "ping";
    }
  }
  return "?";
}

static void wintexAnnotate(const uint8_t* m, uint8_t len, bool fromPanel) {
  char line[160];
  uint8_t cmd = m[1];
  const uint8_t* p = m + 2;
  int pn = len - 3;
  int k = snprintf(line, sizeof line, "    >> WX %s %c %s", fromPanel ? "panel   " : "istemci ",
                   (cmd >= 0x20 && cmd < 0x7F) ? (char)cmd : '.', wintexName(cmd, fromPanel));
  bool addrCmd = fromPanel ? (cmd == 'I' || cmd == 'W')
                           : (cmd == 'O' || cmd == 'R' || cmd == 'I' || cmd == 'W');
  if (addrCmd && pn >= 4) {
    uint32_t a = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
    k += snprintf(line + k, sizeof line - k, " adr=0x%06lX n=%u", (unsigned long)a, p[3]);
    if (pn > 4) k += snprintf(line + k, sizeof line - k, " veri=%d B", pn - 4);
  } else if (pn > 0) {
    k += snprintf(line + k, sizeof line - k, " veri=%d B", pn);
  }
  k += snprintf(line + k, sizeof line - k, "\r\n");
  logOut(line, k);
}

static void wbShift(Frame& f, uint16_t k) {
  memmove(f.wb, f.wb + k, f.wn - k);
  f.wn -= k;
}

static void wintexFeed(Frame& f, uint8_t b, const char* dir) {
  uint32_t now = millis();
  // Sessizlik (gap) geldiğinde yarım kalan çerçeve çöptür: at ve bu bayttan yeniden senkronlan.
  // Aksi halde başta gelen tek bir 0xFF çöp baytı çözücüyü 255 bayt boyunca kilitler.
  if (f.wn && now - f.wLast >= g_gapMs) f.wn = 0;
  f.wLast = now;
  if (f.wn < sizeof f.wb) f.wb[f.wn++] = b;
  while (f.wn) {
    uint8_t L = f.wb[0];
    if (L < 3) { wbShift(f, 1); continue; }
    if (f.wn < L) return;
    uint8_t s = 0;
    for (uint8_t i = 0; i < L - 1; i++) s += f.wb[i];
    if ((uint8_t)(s ^ 0xFF) == f.wb[L - 1]) {
      emitFrame(f, dir);  // ham satırı çerçeve sınırında kapat
      wintexAnnotate(f.wb, L, dir[0] == 'R');
      g_wxFrames++;
      wbShift(f, L);
    } else {
      wbShift(f, 1);
    }
  }
}

static void frameAdd(Frame& f, uint8_t b, const char* dir) {
  // Ters yönde bekleyen çerçeve varsa önce onu yaz: sıralama doğru kalsın
  if (&f == &rxF) emitFrame(txF, "TX"); else emitFrame(rxF, "RX");
  if (f.n == sizeof f.buf) emitFrame(f, dir);
  if (f.n == 0) {
    f.firstMs = millis();
    f.firstWall = time(nullptr);
  }
  f.buf[f.n++] = b;
  f.lastMs = millis();
  wintexFeed(f, b, dir);
}

// ---------------- UART ----------------
static const char* txPinName() {
  for (const auto& p : TX_PINS) if (p.gpio == g_txGpio) return p.name;
  return "?";
}

static EspSoftwareSerial::Config swCfg() {
  if (g_cfg == SERIAL_8N1) return SWSERIAL_8N1;
  if (g_cfg == SERIAL_8E1) return SWSERIAL_8E1;
  if (g_cfg == SERIAL_8O1) return SWSERIAL_8O1;
  return SWSERIAL_8N2;
}

static void applyUart() {
  if (g_swActive) { swTx.end(); g_swActive = false; }
  Serial.end();
  Serial1.end();
  bool swRx = g_rxSw && g_txGpio != 2;          // TX D4'teyken (UART1) yazılımsal nesne yok: donanım RX
  if (!swRx) {
    Serial.setRxBufferSize(1024);
    Serial.begin(g_baud, g_cfg, SERIAL_RX_ONLY);  // GPIO1'e (ve swap sonrası GPIO15'e) hiç dokunma
    Serial.swap();                                // RX -> GPIO13 (D7)
  }
  if (g_txGpio == 2) {
    Serial1.begin(g_baud, g_cfg);               // TX -> GPIO2 (D4), donanım UART1
  } else {
    swTx.enableTxGPIOOpenDrain(g_txOpenDrain);
    // D7'de dahili pull-up KAPALI olmalı: açıkken cevabın başı bozuluyor. Hattı dönüştürücünün pull-up'ı tutuyor.
    swTx.enableRxGPIOPullUp(false);
    // Tampon: 128 bayt; kesme tamponu 384 kenar (panel cevabı en çok ~70 bayt, döngü sık okuyor)
    swTx.begin(g_baud, swCfg(), swRx ? 13 : -1, g_txGpio, false, 128, 384);
    swTx.enableIntTx(false);                    // bit zamanlaması Wi-Fi kesmelerinden etkilenmesin
    g_swActive = true;
  }
}

static void sendToPanel(const uint8_t* data, size_t n) {
  quietArm(g_quietMs);                 // yazma sırasında da Wi-Fi'ye dokunma
  if (g_txGpio == 2) Serial1.write(data, n);
  else swTx.write(data, n);
  quietArm(g_quietMs);                 // pencere TX bittikten sonra başlar
  g_txTotal += n;
  for (size_t i = 0; i < n; i++) frameAdd(txF, data[i], "TX");
}

// ---------------- LOJİK ANALİZÖR ----------------
// "scope [z2_ms] [pencere_ms]": yazılımsal TX pininden bit-bang ile bare Z (03 5A A2) gönderir, gönderim
// sırasında ve sonrasında D7 ile TX pinindeki her değişimi kesmeler kapalıyken kaydeder. z2_ms > 0 ise
// Wintex'in yaptığı gibi ikinci Z'yi o kadar ms sonra gönderir. Yalnızca Z gönderir, ayar değiştirmez.
static const uint16_t SCOPE_MAX = 900;
static uint32_t* scopeT = nullptr;    // yalnızca yakalama sırasında ayrılır, RAM TLS'e lazım
static uint8_t*  scopeV = nullptr;    // bit0 = D7, bit1 = TX pini
static uint16_t scopeN = 0;
static uint32_t scopeEdgeT[72];
static uint8_t  scopeEdgeV[72];
static uint16_t scopeEdges = 0;

static void scopeAddFrame(uint32_t baseCycles, uint32_t bitQ8) {
  static const uint8_t z[] = {0x03, 0x5A, 0xA2};
  uint32_t k = 0;
  const int8_t lastBit = (g_cfg == SERIAL_8N2) ? 9 : 8;   // 8N2'de iki stop biti
  for (uint8_t b : z) {
    for (int8_t i = -1; i <= lastBit; i++) {   // -1 = start, 0..7 = veri, 8.. = stop
      uint8_t lvl = (i < 0) ? 0 : (i > 7) ? 1 : ((b >> i) & 1);
      scopeEdgeT[scopeEdges] = baseCycles + ((k * bitQ8) >> 8);
      scopeEdgeV[scopeEdges] = lvl;
      scopeEdges++;
      k++;
    }
  }
}

static void IRAM_ATTR scopeRun(uint32_t windowCycles) {
  const uint32_t txMask = 1UL << g_txGpio;
  const uint8_t  txBit  = (uint8_t)g_txGpio;
  uint16_t ei = 0;
  uint8_t  last = 0xFF;
  scopeN = 0;
  uint32_t ps = xt_rsil(15);
  uint32_t t0 = ESP.getCycleCount();
  for (;;) {
    uint32_t now = ESP.getCycleCount() - t0;
    if (ei < scopeEdges && now >= scopeEdgeT[ei]) {
      if (scopeEdgeV[ei]) GPOS = txMask; else GPOC = txMask;
      ei++;
    }
    uint32_t g = GPI;
    uint8_t v = ((g >> 13) & 1) | (((g >> txBit) & 1) << 1);
    if (v != last && scopeN < SCOPE_MAX) {
      scopeT[scopeN] = now;
      scopeV[scopeN] = v;
      scopeN++;
      last = v;
    }
    if (ei >= scopeEdges && now >= windowCycles) break;
  }
  xt_wsr_ps(ps);
}

static void scopeCapture(uint32_t z2Ms, uint32_t winMs) {
  const uint32_t mhz = ESP.getCpuFreqMHz();
  const uint32_t bitQ8 = (uint32_t)(((uint64_t)mhz * 1000000ULL * 256ULL) / g_baud);
  scopeT = (uint32_t*)malloc(SCOPE_MAX * sizeof(uint32_t));
  scopeV = (uint8_t*)malloc(SCOPE_MAX);
  if (!scopeT || !scopeV) {
    free(scopeT); free(scopeV); scopeT = nullptr; scopeV = nullptr;
    monWrite("scope: bellek yok\r\n");
    return;
  }
  scopeEdges = 0;
  scopeAddFrame(mhz * 20, bitQ8);
  if (z2Ms) scopeAddFrame(mhz * 20 + z2Ms * 1000 * mhz, bitQ8);
  pinMode(13, INPUT);          // UART fonksiyonundayken GPI hep 0 okunuyor: yakalama süresince GPIO yap
  scopeRun(winMs * 1000 * mhz);
  applyUart();                 // pin işlevini ve alıcıyı baştan kur

  char line[32];
  snprintf(line, sizeof line, "SCOPE n=%u mhz=%lu\r\n", scopeN, (unsigned long)mhz);
  monWrite(line);
  char buf[420];
  size_t p = 0;
  for (uint16_t i = 0; i < scopeN; i++) {
    p += snprintf(buf + p, sizeof buf - p, "%lu:%u ",
                  (unsigned long)(scopeT[i] / (mhz / 10)), scopeV[i]);   // 0.1 µs birimi
    if (p > sizeof buf - 24) { monWrite(buf); p = 0; }
  }
  if (p) monWrite(buf);
  monWrite("\r\nSCOPE son\r\n");
  free(scopeT); free(scopeV); scopeT = nullptr; scopeV = nullptr;
}

// ---------------- PANEL İSTEMCİSİ ----------------
// ESP panele kendi oturumunu açar: boş Z -> Z + UDL -> R ile durum okuma, 10 sn'de bir boş Z.
// Yalnızca Z, R ve H gönderir; panele yazma komutu yoktur. Wintex köprüye bağlanınca oturumu kapatıp bekler.
enum PnlState : uint8_t { PNL_OFF, PNL_WAIT, PNL_HELLO, PNL_LOGIN, PNL_RUN };
static PnlState pnlState = PNL_OFF;
static bool     g_pollEnabled = true;
static bool     pnlInflight = false;
static uint32_t pnlSentAt = 0, pnlNextAt = 0, pnlLastHb = 0, pnlLastVolt = 0, pnlLastOk = 0, pnlCloseUntil = 0;
static uint32_t pnlBackoff = 30000;
static uint8_t  pnlRetries = 0, pnlStep = 0;
static uint8_t  pnlLastReq[24];
static uint8_t  pnlLastReqN = 0;
static uint8_t  pnlRx[96];
static uint8_t  pnlRxN = 0;
static uint32_t pnlRxAt = 0;
static char     pnlProduct[20] = "";
static bool     pnlDownNotified = false;
// Uzaktan kurma: yalnızca Telegram'daki onaylı düğmeyle, tek atış, otomatik tekrar yok. Kapatma komutu yok.
static bool     pnlArmWait = false;
// Panel uzaktan kurmada çıkış süresi vermiyor (kayıt: 0,2 sn'de kurulu): geri sayımı bot yapar.
static const uint32_t ARM_EXIT_MS = 30000, ARM_WAIT_MS = 30000;
static uint32_t pnlArmAt = 0, pnlArmDeadline = 0;
static uint32_t pnlArmCheckAt = 0;
static uint8_t  pnlArmBy = 0;          // kurmayı isteyen kullanıcı + 1
// Uzaktan kapatma: yalnızca PIN'li /kapat ile, tek atış
static bool     pnlDisarmReq = false, pnlDisarmWait = false;
static uint32_t pnlDisarmCheckAt = 0, pnlDisarmByAt = 0;
static uint8_t  pnlDisarmBy = 0;
static uint32_t pnlArmByAt = 0;

static const uint32_t ADR_ZONES = 0x0004F0, ADR_AREA = 0x000D6B, ADR_VOLT = 0x000DCE;
static const uint8_t  ZONE_COUNT = 8, AREA_LEN = 0x13;
static const uint32_t PNL_TIMEOUT_MS = 200, PNL_GAP_MS = 150;

// Bildirimlerde görünen zon adları; boş olanlar "Zon N" olarak yazılır
static const char* const ZONE_NAMES[ZONE_COUNT] = {"Giriş", "Koridor", "", "", "", "", "", ""};

static uint8_t  stZones[ZONE_COUNT], stArea[AREA_LEN], stVolt[4];
static bool     stZonesOk = false, stAreaOk = false, stVoltOk = false;

// Olay takibi: önceki durum
static bool     evInit = false, evArmed = false, evEntry = false, evAlarm = false, evArmFail = false;
static uint8_t  evZones[ZONE_COUNT];
static uint32_t evZoneAt[ZONE_COUNT];

// Öğrenme kaydı: anlamı henüz çözülmemiş adresleri de okur, değişen her bloğu "DEG" satırı olarak monitöre yazar.
static bool g_diag = true;
static const struct { uint32_t adr; uint8_t n; } DIAG_READS[] = {
  {0x0000E1, 1}, {0x0000E9, 1}, {0x0000EC, 1}, {0x000CB4, 1}, {0x000DCF, 1}, {0x0004D0, 8},
};
static const uint8_t DIAG_N = sizeof DIAG_READS / sizeof DIAG_READS[0];
static uint8_t diagLast[DIAG_N][8];
static bool    diagOk[DIAG_N];
static uint8_t diagIdx = 0;

static void tgQueue(const char* s);
static void fmtStatus(char* out, size_t sz);

static void zoneLabel(uint8_t i, char* out, size_t sz) {
  if (ZONE_NAMES[i][0]) strlcpy(out, ZONE_NAMES[i], sz);
  else snprintf(out, sz, "Zon %u", i + 1);
}

static void pnlLogBlock(uint32_t adr, const uint8_t* d, uint8_t n) {
  char b[100];
  const time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  int k = snprintf(b, sizeof b, "DEG %02d:%02d:%02d 0x%06lX:", t.tm_hour, t.tm_min, t.tm_sec, (unsigned long)adr);
  for (uint8_t i = 0; i < n && k < (int)sizeof b - 6; i++) k += snprintf(b + k, sizeof b - k, " %02X", d[i]);
  k += snprintf(b + k, sizeof b - k, "\r\n");
  logOut(b, k);
}

static void pnlRaw(const uint8_t* d, size_t n) {
  quietArm(g_quietMs);
  swTx.write(d, n);
  quietArm(g_quietMs);
}

static void pnlTx(uint8_t cmd, const uint8_t* data, uint8_t n) {
  uint8_t m[24];
  m[0] = n + 3;
  m[1] = cmd;
  if (n) memcpy(m + 2, data, n);
  uint8_t s = 0;
  for (uint8_t i = 0; i < n + 2; i++) s += m[i];
  m[n + 2] = s ^ 0xFF;
  memcpy(pnlLastReq, m, n + 3);
  pnlLastReqN = n + 3;
  pnlRaw(m, n + 3);
  pnlInflight = true;
  pnlSentAt = millis();
}

static void pnlRead(uint32_t adr, uint8_t n) {
  uint8_t d[4] = {(uint8_t)(adr >> 16), (uint8_t)(adr >> 8), (uint8_t)adr, n};
  pnlTx('R', d, 4);
}

static void pnlLost(const char* why, bool loginFail) {
  char b[80];
  int k = snprintf(b, sizeof b, "panel: oturum kayip (%s)\r\n", why);
  logOut(b, k);
  pnlState = PNL_WAIT;
  pnlInflight = false;
  pnlRxN = 0;
  stZonesOk = stAreaOk = false;
  pnlArmAt = 0; pnlArmWait = pnlDisarmReq = pnlDisarmWait = false;
  if (loginFail) {            // yanlış UDL ile paneli yormamak için giderek seyrekleş
    pnlNextAt = millis() + pnlBackoff;
    pnlBackoff = pnlBackoff >= 300000 ? 600000 : pnlBackoff * 2;
  } else {
    pnlNextAt = millis() + 3000;
  }
}

static void pnlStop(bool sendH) {
  if (pnlState == PNL_OFF) return;
  if (sendH && g_swActive) {
    static const uint8_t h[] = {0x03, 0x48, 0xB4};
    pnlRaw(h, sizeof h);
    pnlCloseUntil = millis() + 60;   // H'nin cevabı Wintex'e sızmasın
  }
  pnlState = PNL_OFF;
  pnlInflight = false;
  pnlRxN = 0;
  stZonesOk = stAreaOk = false;
  pnlArmAt = 0; pnlArmWait = pnlDisarmReq = pnlDisarmWait = false;
}

// Ofset 6 = siren/flaş (2026-11-08 doğrulandı: kurma hatasında 11 oldu, kod girilince 00). Siren çalıyorsa her zaman haber ver.
// Ofset 0/1 (panik/hırsız) 832 tablosundan; 412'de doğrulanmadı.
static bool pnlAlarmNow() {
  return (stArea[0] | stArea[1] | stArea[6]) != 0;
}

// ---------------- OLAY GEÇMİŞİ, İSTATİSTİK, AYARLAR ----------------
enum : uint8_t { EV_BOOT, EV_ARM, EV_DISARM, EV_ENTRY, EV_ALARM, EV_ALARM_END, EV_ARMFAIL, EV_ZONE, EV_LOST, EV_BACK, EV_VLOW, EV_VOK, EV_REMARM };
struct HistEnt { uint32_t t; uint8_t type; uint8_t arg; };
static const uint8_t HIST_N = 16;
static HistEnt hist[HIST_N];
static uint8_t histHead = 0, histCount = 0;

struct DayStat { uint16_t arms, disarms, alarms, vMin, vMax; uint16_t zone[ZONE_COUNT]; };
static DayStat dayNow, dayPrev;
static int     dayYday = -1;
static bool    remDone = false, sumDone = false;

static uint32_t zoneLast[ZONE_COUNT];
static uint16_t armedCnt[ZONE_COUNT];
static uint32_t armedSince = 0;
static uint8_t  voltLowReads = 0;
static bool     voltLow = false;

// Bildirim tercihleri, EEPROM'da saklanır. Alarm/siren bildirimi kapatılamaz.
enum : uint8_t { CF_ARM = 1, CF_ENTRY = 2, CF_ZONE = 4, CF_REMIND = 8, CF_SUMMARY = 16, CF_VOLT = 32 };
static uint8_t cfgFlags = CF_ARM | CF_ENTRY | CF_ZONE | CF_VOLT;
static uint8_t cfgRemHour = 23;
static const uint8_t CFG_MAGIC = 0xA7;
// İzinli Telegram hesapları: [0] = TG_CHAT_ID, diğerleri numara paylaşımıyla eklenir ve EEPROM'da saklanır.
static const uint8_t TG_MAX_USERS = 5;
static const size_t  CFG_SIZE = 4 + (TG_MAX_USERS - 1) * 20;
static char    tgUsers[TG_MAX_USERS][20];
static uint8_t tgUserN = 0;
static uint8_t tgUserName[TG_MAX_USERS];   // TG_PHONE_NAMES sırası + 1, 0 = bilinmiyor

static const char* tgUserLabel(uint8_t i) {
  if (i == 0) return TG_OWNER_NAME;
  const uint8_t n = i < TG_MAX_USERS ? tgUserName[i] : 0;
  if (n && n <= sizeof TG_PHONE_NAMES / sizeof TG_PHONE_NAMES[0]) return TG_PHONE_NAMES[n - 1];
  return "Bilinmeyen";
}

static void cfgLoad() {
  strlcpy(tgUsers[0], TG_CHAT_ID, sizeof tgUsers[0]);
  tgUserN = 1;
  EEPROM.begin(CFG_SIZE);
  if (EEPROM.read(0) == CFG_MAGIC) {
    cfgFlags = EEPROM.read(1);
    const uint8_t h = EEPROM.read(2);
    if (h < 24) cfgRemHour = h;
    const uint8_t n = EEPROM.read(3);
    for (uint8_t i = 0; n < TG_MAX_USERS && i < n; i++) {
      char id[20];
      for (uint8_t k = 0; k < sizeof id; k++) id[k] = (char)EEPROM.read(4 + i * 20 + k);
      const uint8_t nm = (uint8_t)id[sizeof id - 1];
      id[sizeof id - 1] = 0;
      if (id[0] >= '0' && id[0] <= '9') { tgUserName[tgUserN] = nm; strlcpy(tgUsers[tgUserN++], id, sizeof tgUsers[0]); }
    }
  }
  EEPROM.end();
}

static void cfgSave() {
  EEPROM.begin(CFG_SIZE);
  EEPROM.write(0, CFG_MAGIC);
  EEPROM.write(1, cfgFlags);
  EEPROM.write(2, cfgRemHour);
  EEPROM.write(3, tgUserN > 0 ? tgUserN - 1 : 0);
  for (uint8_t i = 1; i < TG_MAX_USERS; i++)
    for (uint8_t k = 0; k < 20; k++) EEPROM.write(4 + (i - 1) * 20 + k, i >= tgUserN ? 0 : (k == 19 ? tgUserName[i] : tgUsers[i][k]));
  EEPROM.end();     // commit + belleği geri ver
}

static uint32_t nowEpoch() {
  const time_t t = time(nullptr);
  return t > 1700000000 ? (uint32_t)t : 0;
}

static void histAdd(uint8_t type, uint8_t arg) {
  hist[(histHead + histCount) % HIST_N] = HistEnt{nowEpoch(), type, arg};
  if (histCount < HIST_N) histCount++;
  else histHead = (histHead + 1) % HIST_N;
}

static void fmtDur(uint32_t s, char* out, size_t sz) {
  if (s < 60) snprintf(out, sz, "%lu sn", (unsigned long)s);
  else if (s < 3600) snprintf(out, sz, "%lu dk", (unsigned long)(s / 60));
  else if (s < 86400) snprintf(out, sz, "%lu sa %lu dk", (unsigned long)(s / 3600), (unsigned long)((s % 3600) / 60));
  else snprintf(out, sz, "%lu gün %lu sa", (unsigned long)(s / 86400), (unsigned long)((s % 86400) / 3600));
}

static uint32_t voltHundredths(uint16_t raw) { return (uint32_t)raw * 1800UL / 256; }

static void voltSeen() {
  const uint16_t raw = (uint16_t)stVolt[0] | ((uint16_t)stVolt[1] << 8);
  if (!dayNow.vMin || raw < dayNow.vMin) dayNow.vMin = raw;
  if (raw > dayNow.vMax) dayNow.vMax = raw;
  const uint32_t h = voltHundredths(raw);
  if (h < 1200) {
    if (voltLowReads < 3) voltLowReads++;
    if (voltLowReads >= 2 && !voltLow) {
      voltLow = true;
      histAdd(EV_VLOW, 0);
      if (cfgFlags & CF_VOLT) tgQueue("🪫 <b>Voltaj düşük</b> (12 V altında). Elektrik kesik ya da akü zayıf olabilir.");
    }
  } else {
    voltLowReads = 0;
    if (voltLow && h > 1270) {
      voltLow = false;
      histAdd(EV_VOK, 0);
      if (cfgFlags & CF_VOLT) tgQueue("🔋 Voltaj normale döndü");
    }
  }
}

// Kurma komutu ancak panel gerçekten hazırken gönderilir: zon açıkken kurma sireni çaldırıyor (2026-11-08).
static bool armAllowedNow() {
  if (pnlState != PNL_RUN || !stZonesOk || !stAreaOk) return false;
  if (bridge && bridge.connected()) return false;
  if (stArea[8] & 0x01) return false;            // zaten kurulu
  if (stArea[7] & 0x11) return false;            // çıkış ya da giriş süresi sayıyor
  if (!(stArea[9] & 0x01)) return false;         // hazır değil: bir zon aktif
  for (uint8_t i = 0; i < ZONE_COUNT; i++) if (stZones[i]) return false;
  return !pnlAlarmNow();
}

// Geri sayım başlatılabilir mi? Zonlar burada aranmaz, çünkü kişi hâlâ evin içinde; asıl kontrol süre sonunda.
static bool armStartAllowed() {
  if (pnlState != PNL_RUN || !stZonesOk || !stAreaOk) return false;
  if (bridge && bridge.connected()) return false;
  return !(stArea[8] & 0x01) && !(stArea[7] & 0x11) && !pnlAlarmNow() && !pnlArmAt;
}

static bool disarmAllowedNow() {
  if (pnlState != PNL_RUN || !stAreaOk) return false;
  if (bridge && bridge.connected()) return false;
  return (stArea[8] & 0x01) || (stArea[7] & 0x11) || pnlAlarmNow();
}

static void pnlEvents() {
  if (!stZonesOk || !stAreaOk) return;
  const bool armed = stArea[8] & 0x01;
  const bool entry = stArea[7] & 0x01;
  const bool armFail = stArea[0x12] & 0x01;
  const bool alarm = pnlAlarmNow();
  const uint32_t e = nowEpoch();
  char m[170];
  if (!evInit) {
    evInit = true;
    evArmFail = armFail;
    evArmed = armed; evEntry = entry; evAlarm = alarm;
    memcpy(evZones, stZones, ZONE_COUNT);
    armedSince = armed ? e : 0;
    histAdd(EV_BOOT, 0);
    tgQueue("~🛡 <b>Alarm izleme başladı</b>");
    return;
  }
  if (armFail && !evArmFail && !alarm) {
    histAdd(EV_ARMFAIL, 0);
    tgQueue("⚠️ <b>Alarm KURULAMADI</b>\\nÇıkış süresi bitince bir zon aktifti");
  }
  evArmFail = armFail;
  if (armed && !evArmed) {
    armedSince = e;
    memset(armedCnt, 0, sizeof armedCnt);
    dayNow.arms++;
    const bool byTg = pnlArmBy && millis() - pnlArmByAt < 60000;
    histAdd(EV_ARM, byTg ? pnlArmBy : 0);
    if ((cfgFlags & CF_ARM) || byTg) {
      if (byTg) snprintf(m, sizeof m, "🔒 <b>Alarm KURULDU</b>\\n👤 %s (Telegram)", tgUserLabel(pnlArmBy - 1));
      else strlcpy(m, "🔒 <b>Alarm KURULDU</b>\\n⌨️ Klavyeden", sizeof m);
      tgQueue(m);
    }
    pnlArmBy = 0;
  }
  if (!armed && evArmed) {
    dayNow.disarms++;
    const bool byTg = pnlDisarmBy && millis() - pnlDisarmByAt < 60000;
    histAdd(EV_DISARM, byTg ? pnlDisarmBy : 0);
    if ((cfgFlags & CF_ARM) || byTg) {
      char who[50];
      if (byTg) snprintf(who, sizeof who, "👤 %s (Telegram)", tgUserLabel(pnlDisarmBy - 1));
      else strlcpy(who, "⌨️ Klavyeden", sizeof who);
      char dur[24] = "";
      if (armedSince && e > armedSince) fmtDur(e - armedSince, dur, sizeof dur);
      char z[70] = "";
      size_t p = 0;
      for (uint8_t i = 0; i < ZONE_COUNT && p < sizeof z - 30; i++) {
        if (!armedCnt[i]) continue;
        char zn[24];
        zoneLabel(i, zn, sizeof zn);
        p += snprintf(z + p, sizeof z - p, "%s%s %u", p ? ", " : "", zn, armedCnt[i]);
      }
      snprintf(m, sizeof m, "🔓 <b>Alarm KAPATILDI</b>\\n%s%s%s%s", who, dur[0] ? "\\n⏱ " : "", dur,
               dur[0] ? " kurulu kaldı" : "");
      if (p) { strlcat(m, "\\n👣 ", sizeof m); strlcat(m, z, sizeof m); }
      tgQueue(m);
    }
    armedSince = 0;
    pnlDisarmBy = 0;
  }
  if (entry && !evEntry) {
    histAdd(EV_ENTRY, 0);
    if (cfgFlags & CF_ENTRY) tgQueue("🚪 <b>Giriş süresi başladı</b>");
  }
  if (alarm && !evAlarm) {
    dayNow.alarms++;
    histAdd(EV_ALARM, armFail && !armed ? 1 : 0);
    if (armFail && !armed) strlcpy(m, "🚨 <b>SİREN ÇALIYOR</b>\\nAlarm kurulamadı: çıkış süresi bitince bir zon aktifti", sizeof m);
    else snprintf(m, sizeof m, "🚨🚨 <b>ALARM!</b> 🚨🚨\\nSiren çalıyor (ham: %02X %02X %02X)", stArea[0], stArea[1], stArea[6]);
    tgQueue(m);
  }
  if (!alarm && evAlarm) {
    histAdd(EV_ALARM_END, 0);
    tgQueue("✅ Alarm durumu bitti");
  }
  const uint32_t ms = millis();
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    if (!stZones[i] || evZones[i]) continue;
    zoneLast[i] = e;
    dayNow.zone[i]++;
    if (!armed) continue;
    armedCnt[i]++;
    if (ms - evZoneAt[i] > 30000 || !evZoneAt[i]) {
      evZoneAt[i] = ms;
      histAdd(EV_ZONE, i);
      if (cfgFlags & CF_ZONE) {
        char zn[24];
        zoneLabel(i, zn, sizeof zn);
        snprintf(m, sizeof m, "⚠️ <b>Kuruluyken hareket:</b> %s", zn);
        tgQueue(m);
      }
    }
  }
  evArmed = armed; evEntry = entry; evAlarm = alarm;
  memcpy(evZones, stZones, ZONE_COUNT);
}

static void pnlOnFrame(const uint8_t* f, uint8_t L) {
  if (pnlState == PNL_OFF) return;
  const uint8_t cmd = f[1];
  const uint8_t* p = f + 2;
  const int pn = L - 3;
  pnlInflight = false;
  pnlRetries = 0;
  if (cmd == 0x0F) { pnlLost("panel kapatti", pnlState == PNL_LOGIN); return; }
  switch (pnlState) {
    case PNL_HELLO:
      if (cmd == 'Z') { pnlState = PNL_LOGIN; pnlTx('Z', PANEL_UDL, sizeof PANEL_UDL); }
      break;
    case PNL_LOGIN: {
      bool text = (cmd == 'Z' && pn >= 8);
      for (int i = 0; text && i < 8; i++) if (p[i] < 0x20 || p[i] >= 0x7F) text = false;
      if (!text) { pnlLost("giris reddedildi", true); break; }
      int k = pn < (int)sizeof pnlProduct - 1 ? pn : (int)sizeof pnlProduct - 1;
      memcpy(pnlProduct, p, k);
      pnlProduct[k] = 0;
      pnlState = PNL_RUN;
      pnlBackoff = 30000;
      pnlLastHb = pnlLastOk = millis();
      pnlLastVolt = pnlLastOk - 30000;
      pnlNextAt = pnlLastOk + PNL_GAP_MS;
      logOut("panel: oturum acildi\r\n", 22);
      if (pnlDownNotified) { pnlDownNotified = false; histAdd(EV_BACK, 0); tgQueue("📶 Panel bağlantısı geri geldi"); }
      break;
    }
    case PNL_RUN:
      pnlLastOk = millis();
      pnlNextAt = pnlLastOk + PNL_GAP_MS;
      if (pnlDisarmWait) {
        pnlDisarmWait = false;
        if (cmd == 0x06) {
          pnlDisarmCheckAt = pnlLastOk + 20000;
          char note[90];
          snprintf(note, sizeof note, "~📨 %s kapatma komutu gönderdi", pnlDisarmBy ? tgUserLabel(pnlDisarmBy - 1) : "Telegram");
          tgQueue(note);
        } else {
          tgQueue("⚠️ Panel kapatma komutunu reddetti");
        }
        break;
      }
      if (pnlArmWait) {
        pnlArmWait = false;
        if (cmd == 0x06) {
          pnlArmCheckAt = pnlLastOk + 45000;
          char note[90];
          snprintf(note, sizeof note, "~📨 %s kurma komutu gönderdi", pnlArmBy ? tgUserLabel(pnlArmBy - 1) : "Telegram");
          tgQueue(note);
        } else {
          tgQueue("⚠️ Panel kurma komutunu reddetti");
        }
        break;
      }
      if (cmd == 'W' && pn >= 5) {
        const uint32_t a = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
        const uint8_t n = p[3];
        if (pn != n + 4) break;
        if (a == ADR_ZONES && n == ZONE_COUNT) {
          if (g_diag && (!stZonesOk || memcmp(stZones, p + 4, n))) pnlLogBlock(a, p + 4, n);
          memcpy(stZones, p + 4, n); stZonesOk = true; pnlEvents();
        } else if (a == ADR_AREA && n == AREA_LEN) {
          if (g_diag && (!stAreaOk || memcmp(stArea, p + 4, n))) pnlLogBlock(a, p + 4, n);
          memcpy(stArea, p + 4, n); stAreaOk = true; pnlEvents();
        } else if (a == ADR_VOLT && n == 4) {
          if (g_diag && (!stVoltOk || memcmp(stVolt, p + 4, n))) pnlLogBlock(a, p + 4, n);
          memcpy(stVolt, p + 4, 4); stVoltOk = true; voltSeen();
        } else {
          for (uint8_t i = 0; i < DIAG_N; i++) {
            if (a != DIAG_READS[i].adr || n != DIAG_READS[i].n) continue;
            if (!diagOk[i] || memcmp(diagLast[i], p + 4, n)) pnlLogBlock(a, p + 4, n);
            memcpy(diagLast[i], p + 4, n);
            diagOk[i] = true;
          }
        }
      }
      break;
    default:
      break;
  }
}

static void pnlFeed(uint8_t b) {
  const uint32_t now = millis();
  if (pnlRxN && now - pnlRxAt > 8) pnlRxN = 0;   // yarım çerçeve: at
  pnlRxAt = now;
  if (pnlRxN < sizeof pnlRx) pnlRx[pnlRxN++] = b;
  while (pnlRxN) {
    const uint8_t L = pnlRx[0];
    if (L < 3 || L > sizeof pnlRx) { memmove(pnlRx, pnlRx + 1, --pnlRxN); continue; }
    if (pnlRxN < L) return;
    uint8_t s = 0;
    for (uint8_t i = 0; i < L - 1; i++) s += pnlRx[i];
    if ((uint8_t)(s ^ 0xFF) == pnlRx[L - 1]) {
      uint8_t f[sizeof pnlRx];
      memcpy(f, pnlRx, L);
      pnlRxN -= L;
      memmove(pnlRx, pnlRx + L, pnlRxN);
      pnlOnFrame(f, L);
    } else {
      memmove(pnlRx, pnlRx + 1, --pnlRxN);
    }
  }
}

static void pnlService() {
  const uint32_t now = millis();
  if ((bridge && bridge.connected()) || !g_pollEnabled || !g_swActive) {
    if (pnlState != PNL_OFF) { pnlStop(true); logOut("panel: izleme duraklatildi\r\n", 28); }
    pnlLastOk = now;                 // Wintex bağlıyken "koptu" bildirimi gitmesin
    return;
  }
  if (pnlState == PNL_OFF) { pnlState = PNL_WAIT; pnlNextAt = now + 2000; }
  if (!pnlDownNotified && now - pnlLastOk > 60000) {
    pnlDownNotified = true;
    histAdd(EV_LOST, 0);
    tgQueue("📵 <b>Panelle bağlantı koptu</b>");
  }
  if (pnlDisarmCheckAt && (int32_t)(now - pnlDisarmCheckAt) >= 0) {
    pnlDisarmCheckAt = 0;
    if (stAreaOk && (stArea[8] & 0x01)) tgQueue("⚠️ Kapatma komutu gitti ama panel hâlâ kurulu görünüyor");
  }
  if (pnlArmCheckAt && (int32_t)(now - pnlArmCheckAt) >= 0) {
    pnlArmCheckAt = 0;
    if (stAreaOk && !(stArea[8] & 0x01) && !pnlAlarmNow()) tgQueue("⚠️ Kurma komutu gitti ama panel kurulu görünmüyor");
  }
  if (pnlInflight) {
    if (now - pnlSentAt > PNL_TIMEOUT_MS) {
      if (pnlDisarmWait) {              // yazma komutunu tekrarlama
        pnlDisarmWait = false;
        pnlInflight = false;
        pnlNextAt = now + PNL_GAP_MS;
        pnlDisarmCheckAt = now + 20000;
        tgQueue("⚠️ Panel kapatma komutuna cevap vermedi, durumu kontrol et");
        return;
      }
      if (pnlArmWait) {                 // yazma komutunu tekrarlama
        pnlArmWait = false;
        pnlInflight = false;
        pnlNextAt = now + PNL_GAP_MS;
        pnlArmCheckAt = now + 45000;
        tgQueue("⚠️ Panel kurma komutuna cevap vermedi, durumu kontrol et");
        return;
      }
      if (++pnlRetries > 3) { pnlRetries = 0; pnlLost("cevap yok", pnlState == PNL_LOGIN); return; }
      pnlRaw(pnlLastReq, pnlLastReqN);
      pnlSentAt = millis();
    }
    return;
  }
  if ((int32_t)(now - pnlNextAt) < 0) return;
  switch (pnlState) {
    case PNL_WAIT:
      pnlState = PNL_HELLO;
      pnlTx('Z', nullptr, 0);
      break;
    case PNL_RUN:
      if (pnlDisarmReq) {
        pnlDisarmReq = false;
        if (disarmAllowedNow()) {
          static const uint8_t mask = 0x01;     // Alan 1 (Wintex kaydı: 04 44 01 B6)
          logOut("panel: uzaktan kapatma komutu\r\n", 31);
          pnlDisarmWait = true;
          pnlTx('D', &mask, 1);
        } else {
          tgQueue("ℹ️ Alarm zaten kapalı, komut gönderilmedi");
        }
        break;
      }
      if (pnlArmAt && (int32_t)(now - pnlArmAt) >= 0) {
        if ((stAreaOk && ((stArea[8] & 0x01) || pnlAlarmNow()))) {
          pnlArmAt = 0;             // bu arada klavyeden kurulmuş ya da alarm çalıyor: dokunma
        } else if (armAllowedNow()) {
          static const uint8_t mask = 0x01;     // Alan 1 (Wintex kaydı: 04 41 01 B9)
          pnlArmAt = 0;
          pnlArmByAt = now;
          histAdd(EV_REMARM, pnlArmBy);
          logOut("panel: uzaktan kurma komutu\r\n", 29);
          pnlArmWait = true;
          pnlTx('A', &mask, 1);
          break;
        } else if ((int32_t)(now - pnlArmDeadline) >= 0) {
          pnlArmAt = 0;
          pnlArmBy = 0;
          tgQueue("⛔ <b>Alarm kurulmadı</b>\\nÇıkış süresi bitti ama sensörler sakinleşmedi");
        }
      }
      if (now - pnlLastHb >= 10000) {
        pnlLastHb = now;
        pnlTx('Z', nullptr, 0);
      } else if (now - pnlLastVolt >= 30000) {
        pnlLastVolt = now;
        pnlRead(ADR_VOLT, 4);
      } else {
        const uint8_t s = pnlStep++ & 3;
        if (s == 2 && g_diag) {
          pnlRead(DIAG_READS[diagIdx].adr, DIAG_READS[diagIdx].n);
          diagIdx = (diagIdx + 1) % DIAG_N;
        } else if (s & 1) {
          pnlRead(ADR_AREA, AREA_LEN);
        } else {
          pnlRead(ADR_ZONES, ZONE_COUNT);
        }
      }
      break;
    default:
      break;
  }
}

static void fmtStatus(char* out, size_t sz) {
  if (bridge && bridge.connected()) { strlcpy(out, "Wintex bağlı, izleme duraklatıldı.", sz); return; }
  if (pnlState != PNL_RUN || !stZonesOk || !stAreaOk) { strlcpy(out, "Panel durumu henüz okunamadı.", sz); return; }
  const bool armed = stArea[8] & 0x01, ready = stArea[9] & 0x01;
  const bool alarm = pnlAlarmNow();
  char z[72] = "";
  size_t p = 0;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    if (!stZones[i] || p >= sizeof z - 12) continue;
    char zn[24];
    zoneLabel(i, zn, sizeof zn);
    p += snprintf(z + p, sizeof z - p, "%s%s", p ? ", " : "", zn);
  }
  char v[48] = "";
  if (stVoltOk) {
    const uint32_t a = ((uint32_t)stVolt[0] | ((uint32_t)stVolt[1] << 8)) * 1800UL / 256;
    const uint32_t b = ((uint32_t)stVolt[2] | ((uint32_t)stVolt[3] << 8)) * 1800UL / 256;
    snprintf(v, sizeof v, "\\nVoltaj: %lu.%02lu V / %lu.%02lu V",
             (unsigned long)(a / 100), (unsigned long)(a % 100), (unsigned long)(b / 100), (unsigned long)(b % 100));
  }
  snprintf(out, sz, "%sAlarm: %s\\nHazır: %s\\nAktif zon: %s%s", alarm ? "🚨 ALARM ÇALIYOR\\n" : "",
           armed ? "KURULU" : "KAPALI", ready ? "evet" : "hayır", p ? z : "yok", v);
}

// ---------------- TELEGRAM ----------------
// Tek mesajda düzenlenen, düğmeli arayüz. Bot yalnızca durum okur; panele hiçbir şey yazmaz. Sadece TG_CHAT_ID'ye cevap verir.
#define TG_HOST "api.telegram.org"
static BearSSL::WiFiClientSecure tg;
static BearSSL::X509List* tgRoot = nullptr;
static const uint8_t TG_Q = 5;
static char     tgQ[TG_Q][170];
static uint8_t  tgQHead = 0, tgQCount = 0;
static uint32_t tgNextAt = 0, tgLastPoll = 0, tgOffset = 0;
static int      tgLastCode = 0;

static char     tgBody[1500];
static size_t   tgB = 0;
static char     tgResp[900];
static char     tgTail[160];

enum : uint8_t { V_NONE, V_HOME, V_ZONES, V_LOG, V_DAY, V_SET, V_HELP, V_SUMMARY, V_ARM, V_DISARM };
static uint8_t  tgPendView = V_NONE;
static uint32_t tgPendEdit = 0;
static char     tgPendCb[48] = "";
static char     tgCurChat[20] = "", tgReqChat[20] = "", tgPendChat[20] = "", tgPanelChat[20] = "";
static char     tgWelcome[20] = "", tgPromptChat[20] = "";
static uint32_t tgPromptAt = 0;
static uint8_t  tgQRcpt = 0;
static uint8_t  tgReqUser = 0;
static uint32_t tgArmAskAt = 0;
static uint8_t  pinFails = 0;
static uint32_t pinLockUntil = 0;
// Tuş takımı: aynı anda bir kişi, 60 sn hareketsizlikte sıfırlanır
static char     pinBuf[17] = "";
static uint8_t  pinLen = 0, pinOwner = 0;
static uint32_t pinAt = 0;
static bool     tgPadForce = false;
static const uint8_t  PIN_MAX_FAILS = 3;
static const uint32_t PIN_LOCK_MS = 900000;
static char     tgDelChat[20] = "";
static uint32_t tgDelMsg = 0;
static uint32_t tgPanelMsg = 0, tgLiveUntil = 0, tgLastLive = 0, tgLiveHash = 0;
static uint8_t  tgPanelView = V_NONE;
static const uint32_t TG_LIVE_MS = 180000;

static bool tgEnabled() { return TG_TOKEN[0] && TG_CHAT_ID[0]; }

static void tgQueue(const char* s) {
  char b[200];
  int k = snprintf(b, sizeof b, "tg: %s\r\n", s);
  logOut(b, k < (int)sizeof b ? k : (int)sizeof b - 1);
  if (!tgEnabled()) return;
  if (tgQCount == TG_Q) { tgQHead = (tgQHead + 1) % TG_Q; tgQCount--; tgQRcpt = 0; }  // en eskiyi at
  strlcpy(tgQ[(tgQHead + tgQCount) % TG_Q], s, sizeof tgQ[0]);
  tgQCount++;
}

static void tgPrepare() {
  if (!tgRoot) tgRoot = new BearSSL::X509List(TG_ROOT_PEM);
  tg.setTrustAnchors(tgRoot);       // sertifika doğrulanıyor: token ağda ele geçirilemesin
  tg.setBufferSizes(6144, 1024);
  tg.setTimeout(5000);
}

static bool tgEnsure() {
  if (tg.connected()) return true;
  if (time(nullptr) < 1700000000) return false;   // saat yoksa sertifika doğrulanamaz
  tg.stop();
  tgPrepare();
  return tg.connect(TG_HOST, 443);
}

// HTTP tamamlandıysa true döner; API sonucu tgLastCode'da, cevap tgResp (baş) ve tgTail (son) içinde.
static bool tgCall(const char* method, const char* body) {
  if (!tgEnsure()) return false;
  char head[220];
  const size_t blen = strlen(body);
  int hn = snprintf_P(head, sizeof head,
                      PSTR("POST /bot%s/%s HTTP/1.1\r\nHost: " TG_HOST "\r\nContent-Type: application/json\r\nConnection: keep-alive\r\nContent-Length: %u\r\n\r\n"),
                      TG_TOKEN, method, (unsigned)blen);
  if (tg.write((const uint8_t*)head, hn) != (size_t)hn || tg.write((const uint8_t*)body, blen) != blen) { tg.stop(); return false; }
  String line = tg.readStringUntil('\n');
  if (!line.startsWith(F("HTTP/1.1 "))) { tg.stop(); return false; }
  tgLastCode = line.substring(9, 12).toInt();
  long cl = -1;
  for (;;) {
    line = tg.readStringUntil('\n');
    line.trim();
    if (!line.length()) break;
    if (line.length() > 15 && strncasecmp(line.c_str(), "content-length:", 15) == 0) cl = line.substring(15).toInt();
  }
  if (cl < 0) { tg.stop(); return false; }
  const uint32_t t0 = millis();
  long got = 0;
  size_t rn = 0, tn = 0;
  while (got < cl && millis() - t0 < 5000) {
    if (!tg.available()) { if (!tg.connected()) break; delay(1); continue; }
    int c = tg.read();
    if (c < 0) continue;
    got++;
    if (rn < sizeof tgResp - 1) tgResp[rn++] = (char)c;
    if (tn == sizeof tgTail - 1) { memmove(tgTail, tgTail + 1, tn - 1); tn--; }
    tgTail[tn++] = (char)c;
  }
  tgResp[rn] = 0;
  tgTail[tn] = 0;
  if (got < cl) { tg.stop(); return false; }
  if (tgLastCode != 200 && !strstr(tgResp, "not modified")) {
    char e[170];
    int k = snprintf(e, sizeof e, "tg hata %s %d govde=%u: %.100s\r\n", method, tgLastCode, (unsigned)blen, tgResp);
    logOut(e, k < (int)sizeof e ? k : (int)sizeof e - 1);
  }
  return true;
}

// ---- gövde kurucu ----
static void bp(PGM_P fmt, ...) {
  if (tgB >= sizeof tgBody - 1) return;
  va_list a;
  va_start(a, fmt);
  const int n = vsnprintf_P(tgBody + tgB, sizeof tgBody - tgB, fmt, a);
  va_end(a);
  if (n > 0) tgB = (tgB + (size_t)n < sizeof tgBody) ? tgB + n : sizeof tgBody - 1;
}

static void bodyBegin(uint32_t editId, bool silent) {
  tgB = 0;
  bp(PSTR("{\"chat_id\":%s,"), tgCurChat);
  if (editId) bp(PSTR("\"message_id\":%lu,"), (unsigned long)editId);
  if (silent) bp(PSTR("\"disable_notification\":true,"));
  bp(PSTR("\"parse_mode\":\"HTML\",\"text\":\""));
}

static void kbBegin() { bp(PSTR("\",\"reply_markup\":{\"inline_keyboard\":[")); }
static void kbEnd() { bp(PSTR("]}}")); }
static void kbBack() { bp(PSTR("[{\"text\":\"⬅️ Ana ekran\",\"callback_data\":\"home\"}]")); }

static void fmtClock(uint32_t e, char* out, size_t sz) {
  const time_t t = e;
  struct tm v;
  localtime_r(&t, &v);
  snprintf(out, sz, "%02d:%02d", v.tm_hour, v.tm_min);
}

static void fmtAgo(uint32_t e, char* out, size_t sz) {
  const uint32_t n = nowEpoch();
  if (!e || !n) { strlcpy(out, "hareket yok", sz); return; }
  if (n <= e + 45) { strlcpy(out, "az önce", sz); return; }
  char d[20];
  fmtDur(n - e, d, sizeof d);
  snprintf(out, sz, "%s önce", d);
}

static bool panelReady() { return pnlState == PNL_RUN && stZonesOk && stAreaOk && !(bridge && bridge.connected()); }

static void bpStateLine() {
  if (bridge && bridge.connected()) { bp(PSTR("🔧 <b>Wintex bağlı</b>\\nİzleme duraklatıldı")); return; }
  if (!panelReady()) { bp(PSTR("⏳ <b>Panele bağlanıyor…</b>")); return; }
  const bool armed = stArea[8] & 0x01;
  if (pnlAlarmNow()) { bp(PSTR("🚨 <b>ALARM ÇALIYOR</b>")); return; }
  if (armed) {
    bp(PSTR("🔒 <b>KURULU</b>"));
    const uint32_t e = nowEpoch();
    if (armedSince && e > armedSince) { char d[20]; fmtDur(e - armedSince, d, sizeof d); bp(PSTR(" · %s"), d); }
    return;
  }
  if (stArea[7] & 0x10) { bp(PSTR("⏳ <b>ÇIKIŞ SÜRESİ</b>")); return; }
  if (stArea[7] & 0x01) { bp(PSTR("🚪 <b>GİRİŞ SÜRESİ</b>")); return; }
  if (pnlArmAt) {
    const int32_t left = (int32_t)(pnlArmAt - millis());
    bp(PSTR("⏳ <b>KURULUYOR</b> · %d sn"), left > 0 ? (int)(left / 1000 + 1) : 0);
    return;
  }
  bp(PSTR("🔓 <b>KAPALI</b>"));
  if (stArea[9] & 0x01) bp(PSTR(" · Kurmaya hazır ✅"));
  else bp(PSTR(" · Zon açık ⚠️"));
}

static void bpVoltLine() {
  if (!stVoltOk) return;
  const uint32_t h = voltHundredths((uint16_t)stVolt[0] | ((uint16_t)stVolt[1] << 8));
  int bars = ((int)h - 1150) / 46;     // 11.50 V .. 13.80 V -> 0..5
  if (bars < 0) bars = 0;
  if (bars > 5) bars = 5;
  bp(PSTR("\\n🔋 %lu.%02lu V  "), (unsigned long)(h / 100), (unsigned long)(h % 100));
  for (int i = 0; i < 5; i++) bp(i < bars ? PSTR("▰") : PSTR("▱"));
}

static void renderHome(uint32_t editId, bool live) {
  bodyBegin(editId, true);
  bp(PSTR("🛡 <b>EV ALARMI</b>\\n━━━━━━━━━━━━\\n"));
  bpStateLine();
  if (panelReady()) {
    char z[72] = "";
    size_t p = 0;
    uint16_t total = 0;
    for (uint8_t i = 0; i < ZONE_COUNT; i++) {
      total += dayNow.zone[i];
      if (!stZones[i] || p >= sizeof z - 12) continue;
      char zn[24];
      zoneLabel(i, zn, sizeof zn);
      p += snprintf(z + p, sizeof z - p, "%s%s", p ? ", " : "", zn);
    }
    if (p) bp(PSTR("\\n\\n🔴 Hareket: <b>%s</b>"), z);
    else bp(PSTR("\\n\\n🟢 Tüm zonlar sakin"));
    bpVoltLine();
    bp(PSTR("\\n👣 Bugün %u hareket · %u kurma"), total, dayNow.arms);
  }
  const uint32_t e = nowEpoch();
  char c[8] = "--:--";
  if (e) fmtClock(e, c, sizeof c);
  bp(PSTR("\\n\\n%s %s"), live ? "🟢 Canlı ·" : "🕐", c);
  kbBegin();
  if (panelReady() && pnlArmAt)
    bp(PSTR("[{\"text\":\"⚡ Hemen kur\",\"callback_data\":\"armn\"},{\"text\":\"❌ İptal\",\"callback_data\":\"armx\"}],"));
  else if (panelReady() && !(stArea[8] & 0x01) && !(stArea[7] & 0x10))
    bp(PSTR("[{\"text\":\"🔒 Alarmı kur\",\"callback_data\":\"arm\"}],"));
  else if (panelReady())
    bp(PSTR("[{\"text\":\"🔓 Alarmı kapat\",\"callback_data\":\"dis\"}],"));
  bp(PSTR("[{\"text\":\"🔄 Yenile\",\"callback_data\":\"home\"},{\"text\":\"👁 Zonlar\",\"callback_data\":\"zon\"}],"));
  bp(PSTR("[{\"text\":\"📜 Geçmiş\",\"callback_data\":\"log\"},{\"text\":\"📊 Bugün\",\"callback_data\":\"day\"}],"));
  bp(PSTR("[{\"text\":\"⚙️ Bildirimler\",\"callback_data\":\"set\"},{\"text\":\"❓ Yardım\",\"callback_data\":\"hlp\"}]"));
  kbEnd();
}

static void renderZones(uint32_t editId) {
  bodyBegin(editId, true);
  bp(PSTR("👁 <b>ZONLAR</b>\\n\\n"));
  uint8_t hidden = 0;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    const bool active = stZonesOk && stZones[i];
    if (!ZONE_NAMES[i][0] && !active && !dayNow.zone[i]) { hidden++; continue; }
    char zn[24], ago[26];
    zoneLabel(i, zn, sizeof zn);
    fmtAgo(zoneLast[i], ago, sizeof ago);
    if (active) bp(PSTR("🔴 <b>%s</b> · şimdi aktif"), zn);
    else bp(PSTR("🟢 <b>%s</b> · %s"), zn, ago);
    bp(PSTR(" · bugün %u\\n"), dayNow.zone[i]);
  }
  if (hidden) bp(PSTR("\\n▫️ Diğer %u zon sakin"), hidden);
  kbBegin();
  bp(PSTR("[{\"text\":\"🔄 Yenile\",\"callback_data\":\"zon\"}],"));
  kbBack();
  kbEnd();
}

static void bpHistLine(uint8_t idx) {
  const HistEnt& h = hist[idx];
  char c[8] = "--:--";
  if (h.t) fmtClock(h.t, c, sizeof c);
  bp(PSTR("<b>%s</b>  "), c);
  char zn[24];
  switch (h.type) {
    case EV_BOOT:      bp(PSTR("🛡 İzleme başladı")); break;
    case EV_ARM:       bp(PSTR("🔒 Kuruldu")); if (h.arg) bp(PSTR(" · %s"), tgUserLabel(h.arg - 1)); break;
    case EV_DISARM:    bp(PSTR("🔓 Kapatıldı")); if (h.arg) bp(PSTR(" · %s"), tgUserLabel(h.arg - 1)); break;
    case EV_ENTRY:     bp(PSTR("🚪 Giriş süresi")); break;
    case EV_ALARM:     bp(h.arg ? PSTR("🚨 Siren: kurulamadı") : PSTR("🚨 ALARM")); break;
    case EV_ALARM_END: bp(PSTR("✅ Alarm bitti")); break;
    case EV_ARMFAIL:   bp(PSTR("⚠️ Kurulamadı")); break;
    case EV_ZONE:      zoneLabel(h.arg, zn, sizeof zn); bp(PSTR("⚠️ Kuruluyken: %s"), zn); break;
    case EV_LOST:      bp(PSTR("📵 Panel bağlantısı koptu")); break;
    case EV_BACK:      bp(PSTR("📶 Panel bağlantısı geldi")); break;
    case EV_VLOW:      bp(PSTR("🪫 Voltaj düşük")); break;
    case EV_VOK:       bp(PSTR("🔋 Voltaj normal")); break;
    case EV_REMARM:    bp(PSTR("📲 Telegram'dan kurma")); break;
    default: break;
  }
  bp(PSTR("\\n"));
}

static void renderLog(uint32_t editId) {
  bodyBegin(editId, true);
  bp(PSTR("📜 <b>OLAY GEÇMİŞİ</b>\\n\\n"));
  if (!histCount) bp(PSTR("Henüz olay yok."));
  for (int i = histCount - 1; i >= 0; i--) bpHistLine((histHead + i) % HIST_N);
  bp(PSTR("\\n<i>Son %u olay, yeniden başlayınca sıfırlanır</i>"), HIST_N);
  kbBegin();
  bp(PSTR("[{\"text\":\"🔄 Yenile\",\"callback_data\":\"log\"}],"));
  kbBack();
  kbEnd();
}

static void bpDayBody(bool prev) {
  const DayStat& d = prev ? dayPrev : dayNow;
  bp(PSTR("🔒 Kurma <b>%u</b>  ·  🔓 Kapatma <b>%u</b>  ·  🚨 Alarm <b>%u</b>\\n\\n"), d.arms, d.disarms, d.alarms);
  uint16_t mx = 0;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) if (d.zone[i] > mx) mx = d.zone[i];
  bool any = false;
  for (uint8_t i = 0; i < ZONE_COUNT; i++) {
    if (!d.zone[i]) continue;
    any = true;
    char zn[24];
    zoneLabel(i, zn, sizeof zn);
    const uint8_t bars = 1 + (uint32_t)(d.zone[i] - 1) * 7 / (mx ? mx : 1);
    bp(PSTR("<b>%s</b> %u\\n"), zn, d.zone[i]);
    for (uint8_t k = 0; k < bars; k++) bp(PSTR("▇"));
    bp(PSTR("\\n"));
  }
  if (!any) bp(PSTR("Hareket kaydı yok.\\n"));
  if (d.vMin) {
    const uint32_t a = voltHundredths(d.vMin), b = voltHundredths(d.vMax);
    bp(PSTR("\\n🔋 %lu.%02lu – %lu.%02lu V"), (unsigned long)(a / 100), (unsigned long)(a % 100), (unsigned long)(b / 100), (unsigned long)(b % 100));
  }
}

static void renderDay(uint32_t editId, bool summary) {
  bodyBegin(editId, summary ? false : true);
  if (summary) { bp(PSTR("☀️ <b>DÜNÜN ÖZETİ</b>\\n\\n")); bpDayBody(true); }
  else { bp(PSTR("📊 <b>BUGÜN</b>\\n\\n")); bpDayBody(false); }
  kbBegin();
  if (!summary) bp(PSTR("[{\"text\":\"🔄 Yenile\",\"callback_data\":\"day\"},{\"text\":\"📅 Dün\",\"callback_data\":\"yes\"}],"));
  kbBack();
  kbEnd();
}

static void bpToggle(const char* data, PGM_P label, uint8_t bit) {
  bp(PSTR("[{\"text\":\"%s "), (cfgFlags & bit) ? "✅" : "⬜");
  bp(label);
  bp(PSTR("\",\"callback_data\":\"%s\"}],"), data);
}

static void renderSet(uint32_t editId) {
  bodyBegin(editId, true);
  bp(PSTR("⚙️ <b>BİLDİRİMLER</b>\\n\\n🚨 Alarm ve siren bildirimi her zaman açık.\\nDiğerlerini aşağıdan aç/kapat:"));
  kbBegin();
  bpToggle("t0", PSTR("Kurma / kapatma"), CF_ARM);
  bpToggle("t1", PSTR("Giriş süresi"), CF_ENTRY);
  bpToggle("t2", PSTR("Kuruluyken hareket"), CF_ZONE);
  bpToggle("t5", PSTR("Düşük voltaj"), CF_VOLT);
  bpToggle("t4", PSTR("Sabah 09:00 günlük özet"), CF_SUMMARY);
  bpToggle("t3", PSTR("Gece kurma hatırlatması"), CF_REMIND);
  bp(PSTR("[{\"text\":\"➖\",\"callback_data\":\"hdn\"},{\"text\":\"🌙 %02u:00\",\"callback_data\":\"set\"},{\"text\":\"➕\",\"callback_data\":\"hup\"}],"), cfgRemHour);
  kbBack();
  kbEnd();
}

static void renderArm(uint32_t editId) {
  bodyBegin(editId, true);
  if (armStartAllowed()) {
    bp(PSTR("🔒 <b>Alarm kurulsun mu?</b>\\n\\n⏳ <b>30 saniye çıkış süren var.</b> Süre sonunda sensörler sakin olmalı, olmazsa alarm kurulmaz.\\n\\n<i>30 saniye içinde onayla.</i>"));
    kbBegin();
    bp(PSTR("[{\"text\":\"✅ Evet, kur\",\"callback_data\":\"armgo\"},{\"text\":\"❌ Vazgeç\",\"callback_data\":\"home\"}]"));
  } else {
    bp(PSTR("⛔ <b>Şu an kurulamaz</b>\\n\\n"));
    if (!panelReady()) bp(PSTR("Panel bağlantısı hazır değil."));
    else if (stArea[8] & 0x01) bp(PSTR("Alarm zaten kurulu."));
    else if (pnlAlarmNow()) bp(PSTR("Alarm çalıyor."));
    else if (stArea[7] & 0x11) bp(PSTR("Giriş ya da çıkış süresi sayıyor."));
    else {
      char z[72] = "";
      size_t p = 0;
      for (uint8_t i = 0; i < ZONE_COUNT; i++) {
        if (!stZones[i] || p >= sizeof z - 12) continue;
        char zn[24];
        zoneLabel(i, zn, sizeof zn);
        p += snprintf(z + p, sizeof z - p, "%s%s", p ? ", " : "", zn);
      }
      if (p) bp(PSTR("Aktif zon: <b>%s</b>\\nSensör seni görmeyecek şekilde bekleyip tekrar dene."), z);
      else bp(PSTR("Panel henüz hazır değil, birkaç saniye sonra dene."));
    }
    kbBegin();
    bp(PSTR("[{\"text\":\"🔄 Tekrar dene\",\"callback_data\":\"arm\"}],"));
    kbBack();
  }
  kbEnd();
}

static void renderDisarm(uint32_t editId) {
  bodyBegin(editId, true);
  if (!TG_DISARM_PIN[0]) bp(PSTR("🔐 Kapatma PIN'i tanımlı değil."));
  else if (pinLockUntil && (int32_t)(pinLockUntil - millis()) > 0)
    bp(PSTR("🔐 <b>Kapatma kilitli</b>\\n\\nÇok fazla yanlış PIN. %lu dakika sonra tekrar dene."), (unsigned long)((pinLockUntil - millis()) / 60000 + 1));
  else if (!disarmAllowedNow() && !tgPadForce) bp(PSTR("🔓 Alarm zaten kapalı."));
  else {
    tgPadForce = false;
    bp(PSTR("🔓 <b>Alarmı kapat</b>\\n\\nPIN'ini tuşla:  "));
    for (uint8_t i = 0; i < pinLen; i++) bp(PSTR("●"));
    if (!pinLen) bp(PSTR("<i>boş</i>"));
    kbBegin();
    bp(PSTR("[{\"text\":\"1\",\"callback_data\":\"p1\"},{\"text\":\"2\",\"callback_data\":\"p2\"},{\"text\":\"3\",\"callback_data\":\"p3\"}],"));
    bp(PSTR("[{\"text\":\"4\",\"callback_data\":\"p4\"},{\"text\":\"5\",\"callback_data\":\"p5\"},{\"text\":\"6\",\"callback_data\":\"p6\"}],"));
    bp(PSTR("[{\"text\":\"7\",\"callback_data\":\"p7\"},{\"text\":\"8\",\"callback_data\":\"p8\"},{\"text\":\"9\",\"callback_data\":\"p9\"}],"));
    bp(PSTR("[{\"text\":\"⌫\",\"callback_data\":\"pb\"},{\"text\":\"0\",\"callback_data\":\"p0\"},{\"text\":\"✅ Kapat\",\"callback_data\":\"pk\"}],"));
    kbBack();
    kbEnd();
    return;
  }
  kbBegin();
  kbBack();
  kbEnd();
}

static void renderHelp(uint32_t editId) {
  bodyBegin(editId, true);
  bp(PSTR("❓ <b>YARDIM</b>\\n\\n"
          "/panel  Ana ekran\\n/kur  Alarmı kur\\n/kapat  Alarmı kapat\\n/zonlar  Sensörler\\n/gecmis  Olay geçmişi\\n/bugun  Günlük istatistik\\n/ayarlar  Bildirimler\\n\\n"
          "Ana ekran açıldıktan sonra 3 dakika kendini canlı günceller.\\n\\n"
          "🔐 Kurma onay, kapatma PIN ister. Bot panel ayarlarını değiştiremez."));
  kbBegin();
  kbBack();
  kbEnd();
}

static void renderView(uint8_t v, uint32_t editId, bool live) {
  switch (v) {
    case V_HOME:    renderHome(editId, live); break;
    case V_ZONES:   renderZones(editId); break;
    case V_LOG:     renderLog(editId); break;
    case V_DAY:     renderDay(editId, false); break;
    case V_SET:     renderSet(editId); break;
    case V_HELP:    renderHelp(editId); break;
    case V_SUMMARY: renderDay(editId, true); break;
    case V_ARM:     renderArm(editId); break;
    case V_DISARM:  renderDisarm(editId); break;
    default: tgB = 0; tgBody[0] = 0; break;
  }
}

static uint32_t liveHash() {
  uint32_t h = (uint32_t)pnlState + (pnlArmAt ? (millis() - pnlArmDeadline) / 5000 + 7 : 0);
  if (stAreaOk) h = h * 31 + stArea[7] + (stArea[8] << 8) + (stArea[9] << 16) + ((uint32_t)stArea[6] << 24);
  if (stZonesOk) for (uint8_t i = 0; i < ZONE_COUNT; i++) h = h * 31 + stZones[i];
  return h;
}

static int tgFindUser(const char* p) {   // p'deki sayı izinli hesaplardan biri mi? -1 = hayır
  for (uint8_t i = 0; i < tgUserN; i++) {
    const size_t n = strlen(tgUsers[i]);
    if (n && strncmp(p, tgUsers[i], n) == 0 && !isdigit((unsigned char)p[n])) return i;
  }
  return -1;
}

// Listede olmayan hesap: yalnızca Telegram'ın doğruladığı kendi numarasını paylaşırsa ve numara listedeyse eklenir.
static void tgHandleStranger(const char* chat, const char* u) {
  char id[20];
  size_t n = 0;
  while (isdigit((unsigned char)chat[n]) && n < sizeof id - 1) { id[n] = chat[n]; n++; }
  id[n] = 0;
  if (!n) return;                        // grup sohbetleri (eksi kimlik) yok sayılır
  const char* ct = strstr(u, "\"contact\":{");
  if (!ct) {
    if (!tgPromptAt || millis() - tgPromptAt > 60000) { tgPromptAt = millis(); strlcpy(tgPromptChat, id, sizeof tgPromptChat); }
    return;
  }
  const char* ph = strstr(ct, "\"phone_number\":\"");
  const char* uid = strstr(ct, "\"user_id\":");
  if (!ph || !uid || strncmp(uid + 10, id, n) || isdigit((unsigned char)uid[10 + n])) {
    logOut("tg: baskasinin numarasi, reddedildi\r\n", 37);
    return;
  }
  ph += 16;
  char digits[20];
  size_t d = 0;
  for (; *ph && *ph != '"' && d < sizeof digits - 1; ph++) if (isdigit((unsigned char)*ph)) digits[d++] = *ph;
  digits[d] = 0;
  if (d < 10) return;
  const char* last = digits + d - 10;
  bool ok = false;
  uint8_t nameIdx = 0;
  for (uint8_t i = 0; i < sizeof TG_PHONES / sizeof TG_PHONES[0]; i++)
    if (!strcmp(TG_PHONES[i], last)) { ok = true; nameIdx = i + 1; }
  if (!ok || tgUserN >= TG_MAX_USERS) { logOut("tg: numara listede yok\r\n", 24); return; }
  tgUserName[tgUserN] = nameIdx;
  strlcpy(tgUsers[tgUserN++], id, sizeof tgUsers[0]);
  cfgSave();
  strlcpy(tgWelcome, id, sizeof tgWelcome);
  logOut("tg: yeni kullanici eklendi\r\n", 28);
}

static void tgShow(uint8_t v, uint32_t editId) {
  strlcpy(tgPendChat, tgReqChat[0] ? tgReqChat : tgUsers[0], sizeof tgPendChat);
  tgPendView = v;
  tgPendEdit = editId;
}

static void pinPadStart(uint8_t who) {
  memset(pinBuf, 0, sizeof pinBuf);
  pinLen = 0;
  pinOwner = who;
  pinAt = millis();
}

static void pinPadEnd() {
  memset(pinBuf, 0, sizeof pinBuf);
  pinLen = 0;
  pinAt = 0;
}

// PIN doğruysa kapatma isteğini başlatır ve true döner.
static bool tgTryDisarm(const char* pin, uint8_t who) {
  if (!TG_DISARM_PIN[0] || (pinLockUntil && (int32_t)(pinLockUntil - millis()) > 0)) return false;
  pinLockUntil = 0;
  char note[110];
  if (strcmp(pin, TG_DISARM_PIN)) {
    if (++pinFails >= PIN_MAX_FAILS) {
      pinFails = 0;
      pinLockUntil = millis() + PIN_LOCK_MS;
      snprintf(note, sizeof note, "🚫 <b>Yanlış PIN:</b> %s\\nKapatma 15 dakika kilitlendi", tgUserLabel(who));
    } else {
      snprintf(note, sizeof note, "⚠️ <b>Yanlış PIN:</b> %s (%u/%u)", tgUserLabel(who), pinFails, PIN_MAX_FAILS);
    }
    tgQueue(note);
    return false;
  }
  pinFails = 0;
  if (!disarmAllowedNow()) return false;
  pnlDisarmReq = true;
  pnlDisarmBy = who + 1;
  pnlDisarmByAt = millis();
  return true;
}

static void tgOnCallback(const char* data, uint32_t msg) {
  if (data[0] == 'p' && data[1] && !data[2]) {     // tuş takımı
    if (pinOwner != tgReqUser || !pinAt || millis() - pinAt > 60000) pinPadStart(tgReqUser);
    pinAt = millis();
    if (data[1] >= '0' && data[1] <= '9') {
      if (pinLen < sizeof pinBuf - 1) { pinBuf[pinLen++] = data[1]; pinBuf[pinLen] = 0; }
      tgShow(V_DISARM, msg);
    } else if (data[1] == 'b') {
      if (pinLen) pinBuf[--pinLen] = 0;
      tgShow(V_DISARM, msg);
    } else if (data[1] == 'k') {
      const bool ok = pinLen && tgTryDisarm(pinBuf, tgReqUser);
      pinPadEnd();
      if (ok) tgShow(V_HOME, msg);
      else { pinPadStart(tgReqUser); tgShow(V_DISARM, msg); }
    }
    return;
  }
  if (!strcmp(data, "home")) { pinPadEnd(); tgShow(V_HOME, msg); }
  else if (!strcmp(data, "new")) tgShow(V_HOME, 0);
  else if (!strcmp(data, "zon")) tgShow(V_ZONES, msg);
  else if (!strcmp(data, "log")) tgShow(V_LOG, msg);
  else if (!strcmp(data, "day")) tgShow(V_DAY, msg);
  else if (!strcmp(data, "yes")) tgShow(V_SUMMARY, msg);
  else if (!strcmp(data, "set")) tgShow(V_SET, msg);
  else if (!strcmp(data, "hlp")) tgShow(V_HELP, msg);
  else if (!strcmp(data, "dis")) { pinPadStart(tgReqUser); tgShow(V_DISARM, msg); }
  else if (!strcmp(data, "armn")) {
    // Geri sayımı bitir; sensör kontrolü aynen geçerli, zon aktifse 30 sn içinde sakinleşmesi beklenir
    if (pnlArmAt) { pnlArmAt = millis(); pnlArmDeadline = pnlArmAt + ARM_WAIT_MS; pnlArmBy = tgReqUser + 1; }
    tgShow(V_HOME, msg);
  }
  else if (!strcmp(data, "armx")) {
    if (pnlArmAt) {
      pnlArmAt = 0;
      pnlArmBy = 0;
      char note[90];
      snprintf(note, sizeof note, "❌ %s kurmayı iptal etti", tgUserLabel(tgReqUser));
      tgQueue(note);
    }
    tgShow(V_HOME, msg);
  }
  else if (!strcmp(data, "arm")) { tgArmAskAt = millis(); tgShow(V_ARM, msg); }
  else if (!strcmp(data, "armgo")) {
    if (tgArmAskAt && millis() - tgArmAskAt < 30000 && armStartAllowed()) {
      pnlArmAt = millis() + ARM_EXIT_MS;
      pnlArmDeadline = pnlArmAt + ARM_WAIT_MS;
      pnlArmBy = tgReqUser + 1;
      char note[100];
      snprintf(note, sizeof note, "⏳ <b>%s alarmı kuruyor</b>\\n30 saniye çıkış süresi", tgUserLabel(tgReqUser));
      tgQueue(note);
    } else {
      tgQueue("⏱ Onay süresi doldu ya da panel hazır değil, kurma iptal edildi");
    }
    tgArmAskAt = 0;
    tgShow(V_HOME, msg);
  }
  else if (data[0] == 't' && data[1] >= '0' && data[1] <= '5' && !data[2]) {
    cfgFlags ^= (uint8_t)(1 << (data[1] - '0'));
    cfgSave();
    tgShow(V_SET, msg);
  } else if (!strcmp(data, "hup") || !strcmp(data, "hdn")) {
    cfgRemHour = (cfgRemHour + (data[1] == 'u' ? 1 : 23)) % 24;
    remDone = false;
    cfgSave();
    tgShow(V_SET, msg);
  }
}

static void tgHandleUpdate() {
  const char* u = strstr(tgResp, "\"update_id\":");
  if (!u) return;
  tgOffset = strtoul(u + 12, nullptr, 10) + 1;
  const char* cb = strstr(u, "\"callback_query\":{\"id\":\"");
  if (cb) {
    const char* id = cb + 24;
    size_t k = 0;
    while (id[k] && id[k] != '"' && k < sizeof tgPendCb - 1) { tgPendCb[k] = id[k]; k++; }
    tgPendCb[k] = 0;
    const char* from = strstr(cb, "\"from\":{\"id\":");
    const int who = from ? tgFindUser(from + 13) : -1;
    if (who < 0) { tgPendCb[0] = 0; logOut("tg: yabanci dugme, yok sayildi\r\n", 32); return; }
    strlcpy(tgReqChat, tgUsers[who], sizeof tgReqChat);
    tgReqUser = (uint8_t)who;
    const char* mid = strstr(cb, "\"message\":{\"message_id\":");
    const uint32_t msg = mid ? strtoul(mid + 24, nullptr, 10) : 0;
    const char* d = strstr(tgTail, "\"data\":\"");
    if (!d) return;
    char data[12];
    d += 8;
    k = 0;
    while (d[k] && d[k] != '"' && k < sizeof data - 1) { data[k] = d[k]; k++; }
    data[k] = 0;
    tgOnCallback(data, msg);
    return;
  }
  const char* c = strstr(u, "\"chat\":{\"id\":");
  if (!c) return;
  const int who = tgFindUser(c + 13);
  if (who < 0) { tgHandleStranger(c + 13, u); return; }
  strlcpy(tgReqChat, tgUsers[who], sizeof tgReqChat);
  tgReqUser = (uint8_t)who;
  const char* t = strstr(c, "\"text\":\"");
  if (!t) return;
  t += 8;
  if (!strncasecmp(t, "/kapat", 6)) {
    if (t[6] != ' ') { pinPadStart((uint8_t)who); tgShow(V_DISARM, 0); return; }
    // PIN'li mesaj sohbette kalmasın
    const char* mid = strstr(u, "\"message\":{\"message_id\":");
    if (mid) { tgDelMsg = strtoul(mid + 24, nullptr, 10); strlcpy(tgDelChat, tgUsers[who], sizeof tgDelChat); }
    if (!TG_DISARM_PIN[0] || (pinLockUntil && (int32_t)(pinLockUntil - millis()) > 0)) { tgShow(V_DISARM, 0); return; }
    char pin[17];
    size_t k = 0;
    for (const char* q = t + 7; *q && *q != '"' && *q != ' ' && k < sizeof pin - 1; q++) pin[k++] = *q;
    pin[k] = 0;
    if (!tgTryDisarm(pin, (uint8_t)who)) tgShow(V_DISARM, 0);
    memset(pin, 0, sizeof pin);
    return;
  }
  if (!strncasecmp(t, "/kur", 4)) { tgArmAskAt = millis(); tgShow(V_ARM, 0); }
  else if (!strncasecmp(t, "/zon", 4)) tgShow(V_ZONES, 0);
  else if (!strncasecmp(t, "/gecmis", 7)) tgShow(V_LOG, 0);
  else if (!strncasecmp(t, "/bugun", 6)) tgShow(V_DAY, 0);
  else if (!strncasecmp(t, "/ayar", 5)) tgShow(V_SET, 0);
  else if (!strncasecmp(t, "/yardim", 7) || !strncasecmp(t, "/help", 5)) tgShow(V_HELP, 0);
  else tgShow(V_HOME, 0);     // /start, /panel, /durum ve her şey ana ekranı açar
}

static void statsTick() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  const uint32_t e = nowEpoch();
  if (!e) return;
  const time_t t = e;
  struct tm v;
  localtime_r(&t, &v);
  if (dayYday < 0) dayYday = v.tm_yday;
  else if (v.tm_yday != dayYday) {
    dayPrev = dayNow;
    memset(&dayNow, 0, sizeof dayNow);
    dayYday = v.tm_yday;
    remDone = false;
    sumDone = false;
  }
  if (v.tm_hour == cfgRemHour && !remDone) {
    remDone = true;
    if ((cfgFlags & CF_REMIND) && panelReady() && !(stArea[8] & 0x01) && !(stArea[7] & 0x10)) {
      char m[120];
      snprintf(m, sizeof m, "🌙 <b>Saat %02u:00, alarm kurulu değil.</b>\\nKurmayı unutma.", cfgRemHour);
      tgQueue(m);
    }
  }
  if (v.tm_hour == 9 && !sumDone) {
    sumDone = true;
    if ((cfgFlags & CF_SUMMARY) && dayPrev.vMin) { tgReqChat[0] = 0; tgShow(V_SUMMARY, 0); }
  }
}

static void tgService() {
  statsTick();
  if (!tgEnabled() || WiFi.status() != WL_CONNECTED || pnlInflight) return;
  const uint32_t now = millis();
  if ((int32_t)(now - tgNextAt) < 0) return;

  if (tgPendCb[0]) {                     // düğmedeki yükleniyor simgesini hemen kapat
    tgB = 0;
    bp(PSTR("{\"callback_query_id\":\"%s\"}"), tgPendCb);
    const bool ok = tgCall("answerCallbackQuery", tgBody);
    tgPendCb[0] = 0;
    if (!ok) tgNextAt = millis() + 15000;
    return;
  }

  if (tgDelMsg) {                        // PIN'li mesajı önce sil
    tgB = 0;
    bp(PSTR("{\"chat_id\":%s,\"message_id\":%lu}"), tgDelChat, (unsigned long)tgDelMsg);
    tgDelMsg = 0;
    if (!tgCall("deleteMessage", tgBody)) tgNextAt = millis() + 15000;
    return;
  }

  if (tgWelcome[0]) {
    tgB = 0;
    bp(PSTR("{\"chat_id\":%s,\"parse_mode\":\"HTML\",\"text\":\"✅ <b>Erişim açıldı.</b>\\nAna ekran için /panel yaz.\",\"reply_markup\":{\"remove_keyboard\":true}}"), tgWelcome);
    tgWelcome[0] = 0;
    if (!tgCall("sendMessage", tgBody)) tgNextAt = millis() + 15000;
    return;
  }

  if (tgPromptChat[0]) {
    tgB = 0;
    bp(PSTR("{\"chat_id\":%s,\"text\":\"🔐 Bu bot özeldir. Erişim için numaranı paylaş.\",\"reply_markup\":{\"keyboard\":[[{\"text\":\"📱 Numaramı paylaş\",\"request_contact\":true}]],\"resize_keyboard\":true,\"one_time_keyboard\":true}}"), tgPromptChat);
    tgPromptChat[0] = 0;
    if (!tgCall("sendMessage", tgBody)) tgNextAt = millis() + 15000;
    return;
  }

  if (tgPendView) {
    const uint8_t v = tgPendView;
    const uint32_t edit = tgPendEdit;
    strlcpy(tgCurChat, tgPendChat, sizeof tgCurChat);
    const bool live = (v == V_HOME);
    renderView(v, edit, live);
    const bool ok = tgCall(edit ? "editMessageText" : "sendMessage", tgBody);
    if (!ok) { tgNextAt = millis() + 15000; return; }
    tgPendView = V_NONE;
    if (tgLastCode == 200) {
      if (!edit) {
        const char* m = strstr(tgResp, "\"message_id\":");
        if (m) tgPanelMsg = strtoul(m + 13, nullptr, 10);
      } else {
        tgPanelMsg = edit;
      }
      tgPanelView = v;
      strlcpy(tgPanelChat, tgCurChat, sizeof tgPanelChat);
      if (v == V_HOME) { tgLiveUntil = millis() + TG_LIVE_MS; tgLiveHash = liveHash(); }
      else tgLiveUntil = 0;
      tgLastLive = millis();
    } else if (edit && !strstr(tgResp, "not modified")) {
      tgShow(v, 0);                      // eski mesaj düzenlenemedi: yenisini gönder
    }
    tgNextAt = millis() + 200;
    return;
  }

  if (tgQCount) {
    const char* s = tgQ[tgQHead];
    const bool silent = s[0] == '~';
    if (tgQRcpt >= tgUserN) tgQRcpt = 0;
    strlcpy(tgCurChat, tgUsers[tgQRcpt], sizeof tgCurChat);
    bodyBegin(0, silent);
    bp(PSTR("%s"), silent ? s + 1 : s);
    kbBegin();
    bp(PSTR("[{\"text\":\"🛡 Ana ekran\",\"callback_data\":\"new\"}]"));
    kbEnd();
    if (!tgCall("sendMessage", tgBody) || tgLastCode == 429 || tgLastCode >= 500) { tgNextAt = millis() + 15000; return; }
    if (++tgQRcpt >= tgUserN) {          // herkese gitti (400 gibi kalıcı hatada da geç, kuyruk tıkanmasın)
      tgQRcpt = 0;
      tgQHead = (tgQHead + 1) % TG_Q;
      tgQCount--;
    }
    tgNextAt = millis() + 300;
    return;
  }

  // Ana ekran açıkken değişiklikleri aynı mesajda göster
  if (tgPanelMsg && tgPanelView == V_HOME && tgLiveUntil) {
    const bool running = (int32_t)(tgLiveUntil - now) > 0;
    strlcpy(tgCurChat, tgPanelChat, sizeof tgCurChat);
    if (!running) {
      tgLiveUntil = 0;
      renderHome(tgPanelMsg, false);
      if (!tgCall("editMessageText", tgBody)) { tgNextAt = millis() + 15000; }
      return;
    }
    if (now - tgLastLive > 2500) {
      const uint32_t h = liveHash();
      if (h != tgLiveHash) {
        tgLiveHash = h;
        tgLastLive = now;
        renderHome(tgPanelMsg, true);
        if (!tgCall("editMessageText", tgBody)) tgNextAt = millis() + 15000;
        return;
      }
    }
  }

  // Tuş takımı açıkken basışlar gecikmesin
  if (now - tgLastPoll < ((pinAt && now - pinAt < 60000) ? 300 : 2000)) return;
  tgLastPoll = now;
  tgB = 0;
  bp(PSTR("{\"offset\":%lu,\"limit\":1,\"timeout\":0,\"allowed_updates\":[\"message\",\"callback_query\"]}"), (unsigned long)tgOffset);
  if (!tgCall("getUpdates", tgBody)) { tgNextAt = millis() + 15000; return; }
  if (tgLastCode == 200) tgHandleUpdate();
  else tgNextAt = millis() + 15000;
}

// ---------------- KOMUTLAR ----------------
static const char* cfgName() {
  if (g_cfg == SERIAL_8N1) return "8N1";
  if (g_cfg == SERIAL_8N2) return "8N2";
  if (g_cfg == SERIAL_8E1) return "8E1";
  if (g_cfg == SERIAL_8O1) return "8O1";
  return "?";
}

static void monWriteP(PGM_P s) {
  char b[128];
  const size_t n = strlen_P(s);
  for (size_t i = 0; i < n; ) {
    const size_t k = n - i < sizeof b ? n - i : sizeof b;
    memcpy_P(b, s + i, k);
    monSend(b, k);
    i += k;
  }
}

static void printHelp() {
  monWriteP(PSTR("Komutlar:\r\n"
           "  help | stats | dump | clear | reboot\r\n"
           "  baud <n>            ornek: baud 9600\r\n"
           "  cfg <8n1|8n2|8e1|8o1>\r\n"
           "  gap <ms>            cerceve ayirma suresi (varsayilan 25)\r\n"
           "  quiet <ms>          TX sonrasi Wi-Fi sessiz pencere (varsayilan 40, 0 = kapali)\r\n"
           "  durum               panel ve Telegram durumu\r\n"
           "  diag on|off         degisen bloklari DEG satiri olarak kaydet\r\n"
           "  poll on|off         panel izleme (wx/hex/scope icin once poll off)\r\n"
           "  tgtest              Telegram sunucusuna TLS baglanti testi\r\n"
           "  scope [z2 ms] [pencere ms]  Z gonderir, D7 hattini bit bit kaydeder (tx on gerekir)\r\n"
           "  tx on|off           panele gonderme kilidi (varsayilan OFF)\r\n"
           "  hex <AA BB ..>      hex bayt gonder (tx on gerekir)\r\n"
           "  ascii <metin>       metin gonder, \\r ve \\n kullanilabilir (tx on gerekir)\r\n"
           "  wx <Z|O|R|H|P> [AA BB ..]  Wintex cercevesi gonder, uzunluk+checksum otomatik\r\n"
           "                      sadece okuma/oturum komutlari. ornek: wx R 00 04 F8 20\r\n"
           "  txpin <d4|d5|d6|d1|d2|tx>  panele gonderim pini (varsayilan d5)\r\n"
           "  txod on|off         TX acik drenaj (d4 haric; panel hatti kendisi 5V'a ceker)\r\n"));
}

static void printBanner() {
  char b[200];
  snprintf(b, sizeof b, "\r\n== Texecom dinleyici == %lu baud %s | RX=D7 TX=%s%s | TX kilidi: %s | gap %u ms\r\n",
           (unsigned long)g_baud, cfgName(), txPinName(), g_txOpenDrain ? " (OD)" : "",
           g_txEnabled ? "ACIK" : "KAPALI", g_gapMs);
  monWrite(b);
  printHelp();
  monWrite("-- gecmis kayit --\r\n");
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  String cmd = line, arg = "";
  int sp = line.indexOf(' ');
  if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
  cmd.toLowerCase();
  char out[200];

  if (cmd == "help") {
    printHelp();
  } else if (cmd == "stats") {
    snprintf(out, sizeof out,
             "uptime %lus | heap %u | rssi %d dBm | RX %lu B | TX %lu B | WX %lu cerceve | %lu baud %s | tx %s\r\n",
             (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(), (int)WiFi.RSSI(),
             (unsigned long)g_rxTotal, (unsigned long)g_txTotal, (unsigned long)g_wxFrames,
             (unsigned long)g_baud, cfgName(), g_txEnabled ? "ACIK" : "KAPALI");
    monWrite(out);
  } else if (cmd == "dump") {
    if (mon && mon.connected()) ringDump(mon);
  } else if (cmd == "clear") {
    ringHead = 0; ringFull = false;
    monWrite("kayit temizlendi\r\n");
  } else if (cmd == "reboot") {
    monWrite("yeniden basliyor...\r\n");
    delay(200);
    ESP.restart();
  } else if (cmd == "baud") {
    long v = arg.toInt();
    if (v < 1200 || v > 115200) { monWrite("gecersiz baud\r\n"); return; }
    g_baud = (uint32_t)v;
    applyUart();
    snprintf(out, sizeof out, "baud = %lu\r\n", (unsigned long)g_baud);
    monWrite(out);
  } else if (cmd == "cfg") {
    arg.toLowerCase();
    if (arg == "8n1") g_cfg = SERIAL_8N1;
    else if (arg == "8n2") g_cfg = SERIAL_8N2;
    else if (arg == "8e1") g_cfg = SERIAL_8E1;
    else if (arg == "8o1") g_cfg = SERIAL_8O1;
    else { monWrite("8n1 | 8n2 | 8e1 | 8o1\r\n"); return; }
    applyUart();
    snprintf(out, sizeof out, "cfg = %s\r\n", cfgName());
    monWrite(out);
  } else if (cmd == "gap") {
    long v = arg.toInt();
    if (v < 2 || v > 2000) { monWrite("2..2000 ms\r\n"); return; }
    g_gapMs = (uint16_t)v;
    monWrite("tamam\r\n");
  } else if (cmd == "poll") {
    arg.toLowerCase();
    g_pollEnabled = (arg != "off");
    monWrite(g_pollEnabled ? "panel izleme: ACIK\r\n" : "panel izleme: KAPALI (oturum kapatiliyor)\r\n");
  } else if (cmd == "diag") {
    arg.toLowerCase();
    g_diag = (arg != "off");
    monWrite(g_diag ? "ogrenme kaydi: ACIK\r\n" : "ogrenme kaydi: KAPALI\r\n");
  } else if (cmd == "durum") {
    char st[230];
    fmtStatus(st, sizeof st);
    monWrite(st);
    snprintf(out, sizeof out, "\r\n[panel durum=%u urun=%s | tg %s, kuyruk %u, son http %d | heap %u]\r\n",
             (unsigned)pnlState, pnlProduct, tgEnabled() ? "acik" : "kapali (token yok)", tgQCount, tgLastCode, (unsigned)ESP.getFreeHeap());
    monWrite(out);
  } else if (cmd == "tgusers") {
    if (arg == "clear") { tgUserN = 1; cfgSave(); }
    snprintf(out, sizeof out, "izinli hesap: %u\r\n", tgUserN);
    monWrite(out);
    for (uint8_t i = 0; i < tgUserN; i++) {
      snprintf(out, sizeof out, "  #%u %s %s\r\n", i, tgUsers[i], tgUserLabel(i));
      monWrite(out);
    }
  } else if (cmd == "tgname") {
    int i = -1, n = -1;
    sscanf(arg.c_str(), "%d %d", &i, &n);
    if (i < 1 || i >= tgUserN || n < 0 || n > (int)(sizeof TG_PHONE_NAMES / sizeof TG_PHONE_NAMES[0])) { monWrite("tgname <hesap> <isim sirasi>\r\n"); return; }
    tgUserName[i] = (uint8_t)n;
    cfgSave();
    monWrite("tamam\r\n");
  } else if (cmd == "tgsim") {
    // Test: bir sayfayı Telegram'a gönderir. tgsim <1-7> [e] (e = son mesajı düzenle)
    const int v = arg.toInt();
    if (v < 1 || v > 9) { monWrite("tgsim 1..9 [e]\r\n"); return; }
    tgReqChat[0] = 0;
    tgPadForce = arg.indexOf('p') > 0;
    tgShow((uint8_t)v, arg.indexOf('e') > 0 ? tgPanelMsg : 0);
    monWrite("tamam\r\n");
  } else if (cmd == "tgtest") {
    const uint32_t t0 = millis();
    const unsigned h0 = ESP.getFreeHeap();
    tg.stop();
    tgPrepare();
    const bool ok = time(nullptr) > 1700000000 && tg.connect(TG_HOST, 443);
    const unsigned h1 = ESP.getFreeHeap();
    int err = tg.getLastSSLError();
    tg.stop();
    snprintf(out, sizeof out, "tgtest: %s, %lu ms, heap %u -> %u, ssl hata %d, saat %s\r\n", ok ? "BAGLANDI" : "BASARISIZ",
             (unsigned long)(millis() - t0), h0, h1, err, time(nullptr) > 1700000000 ? "tamam" : "YOK");
    monWrite(out);
  } else if (cmd == "scope") {
    if (pnlState != PNL_OFF) { monWrite("once: poll off\r\n"); return; }
    if (!g_txEnabled) { monWrite("once: tx on\r\n"); return; }
    if (!g_swActive)  { monWrite("scope sadece yazilimsal TX pininde calisir\r\n"); return; }
    long z2 = 0, win = 25;
    sscanf(arg.c_str(), "%ld %ld", &z2, &win);
    if (z2 < 0 || z2 > 100 || win < 5 || win > 150) { monWrite("scope [z2_ms 0..100] [pencere_ms 5..150]\r\n"); return; }
    scopeCapture((uint32_t)z2, (uint32_t)win);
  } else if (cmd == "quiet") {
    long v = arg.toInt();
    if (v < 0 || v > 500) { monWrite("0..500 ms (0 = kapali)\r\n"); return; }
    g_quietMs = (uint16_t)v;
    snprintf(out, sizeof out, "sessiz pencere = %u ms\r\n", g_quietMs);
    monWrite(out);
  } else if (cmd == "tx") {
    arg.toLowerCase();
    g_txEnabled = (arg == "on");
    monWrite(g_txEnabled ? "TX ACIK: panele veri gonderebilirsin\r\n" : "TX KAPALI (pasif dinleme)\r\n");
  } else if (cmd == "hex") {
    if (pnlState != PNL_OFF) { monWrite("once: poll off\r\n"); return; }
    if (!g_txEnabled) { monWrite("once: tx on\r\n"); return; }
    char tmp[256];
    strlcpy(tmp, arg.c_str(), sizeof tmp);
    uint8_t data[64];
    size_t n = 0;
    for (char* t = strtok(tmp, " ,"); t && n < sizeof data; t = strtok(nullptr, " ,"))
      data[n++] = (uint8_t)strtol(t, nullptr, 16);
    if (!n) { monWrite("bos\r\n"); return; }
    sendToPanel(data, n);
    monWrite("gonderildi\r\n");
  } else if (cmd == "ascii") {
    if (pnlState != PNL_OFF) { monWrite("once: poll off\r\n"); return; }
    if (!g_txEnabled) { monWrite("once: tx on\r\n"); return; }
    arg.replace("\\r", "\r");
    arg.replace("\\n", "\n");
    sendToPanel((const uint8_t*)arg.c_str(), arg.length());
    monWrite("gonderildi\r\n");
  } else if (cmd == "u0test" || cmd == "swtest") {
    // Teşhis: GPIO1'den (kart TX pini, CH340'a bağlı) test baytları gönderir; USB takılıyken
    // bilgisayarda COM portundan okunur. Panel hattına gitmez.
    static const uint8_t pat[] = {0x03, 0x5A, 0xA2, 0x55, 0xAA, 0x00, 0xFF, 0x0F, 0xF0};
    if (cmd == "u0test") {
      Serial.end();
      Serial.begin(g_baud, g_cfg, SERIAL_TX_ONLY);   // donanım UART0
      Serial.pins(1, 3);                             // swap'ı kaldır -> TX = GPIO1
      delay(20);
      Serial.write(pat, sizeof pat);
      Serial.flush();
      delay(20);
    } else {
      SoftwareSerial t;                                // yeni nesne: end/begin döngüsü yok
      t.begin(g_baud, swCfg(), -1, 1, false);
      t.enableIntTx(false);
      delay(20);
      t.write(pat, sizeof pat);
      delay(20);
      t.end();
    }
    applyUart();
    monWrite("test baytlari GPIO1'den gonderildi\r\n");
  } else if (cmd == "txod") {
    arg.toLowerCase();
    g_txOpenDrain = (arg == "on");
    applyUart();
    monWrite(g_txOpenDrain ? "TX acik drenaj: ACIK\r\n" : "TX acik drenaj: KAPALI (push-pull)\r\n");
  } else if (cmd == "txpin") {
    arg.toLowerCase();
    int8_t g = -1;
    for (const auto& p : TX_PINS) if (arg == p.name) g = p.gpio;
    if (g < 0) { monWrite("txpin d4|d5|d6|d1|d2\r\n"); return; }
    if (g_txGpio != 2 && g_txGpio != g) { if (g_swActive) { swTx.end(); g_swActive = false; } pinMode(g_txGpio, INPUT); }
    g_txGpio = g;
    applyUart();
    snprintf(out, sizeof out, "TX pini = %s (GPIO%d)\r\n", txPinName(), g_txGpio);
    monWrite(out);
  } else if (cmd == "wx") {
    if (pnlState != PNL_OFF) { monWrite("once: poll off\r\n"); return; }
    if (!g_txEnabled) { monWrite("once: tx on\r\n"); return; }
    if (!arg.length()) { monWrite("ornek: wx R 00 04 F8 20\r\n"); return; }
    char c = (char)toupper(arg[0]);
    // Yazma/kurma komutlari (I W U K A D S C) bilerek engelli; gerekirse "hex" ile elle gonderilir
    if (!strchr("ZORHP", c)) { monWrite("wx sadece: Z O R H P\r\n"); return; }
    char tmp[256];
    strlcpy(tmp, arg.c_str() + 1, sizeof tmp);
    uint8_t m[72];
    size_t n = 2;
    m[1] = (uint8_t)c;
    for (char* t = strtok(tmp, " ,"); t && n < sizeof m - 1; t = strtok(nullptr, " ,"))
      m[n++] = (uint8_t)strtol(t, nullptr, 16);
    m[0] = (uint8_t)(n + 1);
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s += m[i];
    m[n++] = s ^ 0xFF;
    sendToPanel(m, n);
    monWrite("gonderildi\r\n");
  } else {
    monWrite("bilinmeyen komut, help yaz\r\n");
  }
}

static void pollMonitor() {
  static String lineBuf;
  static uint8_t iacSkip = 0;  // telnet müzakere baytlarını (FF xx yy) atla
  while (mon && mon.connected() && mon.available()) {
    uint8_t c = mon.read();
    if (iacSkip) { iacSkip--; continue; }
    if (c == 0xFF) { iacSkip = 2; continue; }
    if (c == '\n' || c == '\r') {
      if (lineBuf.length()) { String l = lineBuf; lineBuf = ""; handleCommand(l); }
      continue;
    }
    if (c >= 0x20 && c < 0x7F && lineBuf.length() < 200) lineBuf += (char)c;
  }
}

// ---------------- AĞ ----------------
static void acceptClients() {
  if (monServer.hasClient()) {
    WiFiClient c = monServer.accept();
    if (mon) mon.stop();
    mon = c;
    mon.setNoDelay(true);
    printBanner();
    ringDump(mon);
  }
  if (bridgeServer.hasClient()) {
    WiFiClient c = bridgeServer.accept();
    if (bridge) bridge.stop();
    bridge = c;
    bridge.setNoDelay(true);
    monWrite("-- kopru istemcisi baglandi --\r\n");
  }
}

// ---------------- ANA ----------------
void setup() {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, HIGH);  // söner (ters mantık)

  applyUart();
  cfgLoad();

  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 80 && WiFi.status() != WL_CONNECTED; i++) delay(250);

  configTime(3 * 3600, 0, "pool.ntp.org", "time.google.com");  // Türkiye UTC+3

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.begin();

  monServer.begin();
  bridgeServer.begin();
}

void loop() {
  ArduinoOTA.handle();
  acceptClients();
  pollMonitor();

  // Panel -> ESP
  uint8_t tmp[256];
  size_t n = 0;
  if (g_swActive && g_rxSw) {
    while (swTx.available() && n < sizeof tmp) tmp[n++] = (uint8_t)swTx.read();
  } else {
    while (Serial.available() && n < sizeof tmp) tmp[n++] = (uint8_t)Serial.read();
  }
  if (n) {
    g_rxTotal += n;
    if (g_quietActive) quietArm(QUIET_RX_TAIL_MS);  // cevap geliyor: son bayttan kısa süre sonra kapat
    if (pnlState != PNL_OFF || (int32_t)(millis() - pnlCloseUntil) < 0) {
      for (size_t i = 0; i < n; i++) pnlFeed(tmp[i]);   // kendi oturumumuz: monitöre yazma, Wintex'e sızdırma
    } else {
      for (size_t i = 0; i < n; i++) frameAdd(rxF, tmp[i], "RX");
      bridgeSend(tmp, n);
    }
    digitalWrite(ACT_LED, LOW);
    ledOffAt = millis() + 40;
  }

  pnlService();
  tgService();

  // Köprü istemcisi (ör. Wintex) -> Panel
  if (bridge && bridge.connected()) {
    int avail = bridge.available();
    while (avail > 0) {
      uint8_t buf[128];
      int got = bridge.read(buf, avail > (int)sizeof buf ? sizeof buf : avail);
      if (got <= 0) break;
      sendToPanel(buf, got);
      avail -= got;
    }
  }

  // Sessizlik olunca çerçeveyi kapat
  uint32_t now = millis();
  if (rxF.n && now - rxF.lastMs >= g_gapMs) emitFrame(rxF, "RX");
  if (txF.n && now - txF.lastMs >= g_gapMs) emitFrame(txF, "TX");

  if (g_quietActive && (int32_t)(millis() - g_quietUntil) >= 0) quietFlush();

  if (ledOffAt && (int32_t)(now - ledOffAt) >= 0) {
    digitalWrite(ACT_LED, HIGH);
    ledOffAt = 0;
  }
  yield();
}
