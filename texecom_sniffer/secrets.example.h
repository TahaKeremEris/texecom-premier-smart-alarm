#pragma once
// secrets.h olarak kopyala ve doldur.
static const char* WIFI_SSID = "WIFI_ADI";
static const char* WIFI_PASS = "WIFI_SIFRESI";
static const char* OTA_PASS  = "degistir-beni";

// Panel girisi: UDL rakamlari ham bayt olarak. Ornek 1234:
static const uint8_t PANEL_UDL[] = {1, 2, 3, 4};

// Telegram: @BotFather'in verdigi token ve kendi sohbet kimligin. Bossa bot kapali kalir.
static const char* TG_TOKEN   = "";
static const char* TG_CHAT_ID = "";

// Bota erisebilecek telefonlar (son 10 hane). Telegram numara paylasimiyla dogrulanir.
static const char* const TG_PHONES[] = {"5XXXXXXXXX"};

// Bildirimlerde gorunen isimler. TG_PHONE_NAMES, TG_PHONES ile ayni sirada.
static const char* TG_OWNER_NAME = "Ad Soyad";
static const char* const TG_PHONE_NAMES[] = {"Ad Soyad"};

// Telegram'dan alarm kapatma PIN'i. Alarm kodundan farkli olmali, bossa kapatma kapali kalir.
static const char* TG_DISARM_PIN = "";
