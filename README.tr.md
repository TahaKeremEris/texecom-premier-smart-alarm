# Texecom Premier 412 Akıllı Alarm (ESP8266 + Telegram + Wi-Fi üzerinden Wintex)

**[English](README.md)**

Klasik, internetsiz bir **Texecom Premier 412** hırsız alarmını **150 TL'lik bir NodeMCU (ESP8266)** ile akıllı hale getirir. Telegram'da canlı durum, bildirimler, uzaktan kurma ve PIN'le kapatma sağlar. Bulut, sunucu ya da aylık ücret yok, panel eskisi gibi çalışmaya devam eder.

ESP8266, panelin **Com1** portundan Texecom'un Wintex programının kullandığı Wintex/UDL protokolüyle konuşur. Wintex'i de Wi-Fi üzerinden köprüler, panel kutusunu açmadan bilgisayardan programlayabilirsin.

> ⚠️ Hobi projesidir, Texecom ile ilgisi yoktur. Bir güvenlik cihazıyla konuşuyor: dikkatli test et, alarmın normal çalışmasını aklında tut, sorumluluk sende.

## Özellikler

**Telegram botu (doğrudan ESP8266'nın üzerinde çalışır)**
- Düğmeli, tek mesajda güncellenen canlı ana ekran. Açtıktan sonra 3 dakika kendini yeniler.
- Kurulu, kapalı, çıkış ve giriş süresi, hazır durumu, aktif zonlar, akü ve sistem voltajı.
- Bildirimler:
  - Kurma ve kapatma, **kimin yaptığıyla** birlikte
  - Giriş süresi, siren/alarm, kuruluyken hareket
  - Bağlantı koptu/geldi, düşük voltaj
- **Uzaktan kurma:** Onay ve 30 saniyelik çıkış geri sayımı var (⚡ Hemen kur / ❌ İptal). Yalnızca tüm zonlar sakinken kurar.
- **Uzaktan kapatma:** Ekranda PIN tuş takımı var. Yanlış PIN herkese bildirilir, 3 yanlışta 15 dakika kilitlenir.
- Sayfalar: zonlar (son hareket, bugünkü sayı), olay geçmişi, çubuk grafikli günlük istatistik, dünün özeti.
- EEPROM'da saklanan bildirim ayarları, gece "alarmı kurmadın" hatırlatması ve sabah 09:00 özeti.
- **Aile erişimi:** En fazla 5 hesap. Yeni kişi **"📱 Numaramı paylaş"**'a basar, Telegram numaranın o hesaba ait olduğunu doğrular, bot da numara listedeyse kabul eder.
- Listede olmayan hesaplar hiçbir bilgi alamaz.
- Sertifika doğrulamalı TLS (Go Daddy kökü `telegram_root.h` içinde).

**Panel bağlantısı**
- Kendi UDL oturumu: giriş, saniyede ~4 okuma, oturumu canlı tutma. Panele yalnızca `Z`, `R`, `H` gönderir. `A`/`D` sadece Telegram'dan kurup kapatınca gider.
- **Wi-Fi üzerinden Wintex:** "Connect Via Network" ile port `10001`. Wintex bağlıyken ESP kendi oturumunu duraklatır, çıkınca devam eder.
- Port `23`'te hata ayıklama monitörü: protokol çözücü, ham hex araçları ve aşağıdaki iki hatayı bulan dahili **lojik analizör** (`scope`).

## Donanım

| Parça | Not |
|---|---|
| NodeMCU V3 (ESP8266, CH340) | Her ESP8266 olur, pin adları NodeMCU'ya göre |
| MP1584EN düşürücü | Com1 12V → ~4.9V (VIN), takmadan önce ayarla |
| 4 kanal BSS138 lojik dönüştürücü | Tek kanal kullanılıyor (panel TX 5V → ESP 3.3V) |
| 4 dişi-dişi jumper | Com1'den kutuya |
| 3D baskı kutu (isteğe bağlı) | [`enclosure/`](enclosure/) (OpenSCAD + STL) |

### Kablolama

Com1 (JP10), 5 pozisyonlu ve 4 pinli bir Molex. Önden bakınca soldan sağa sıra 5, 4, 3, (boşluk), 1:

| Com1 pin | Sinyal | Nereye |
|---|---|---|
| 1 | +12V | MP1584 IN+ |
| 3 | GND | MP1584 IN− |
| 4 | Panel RX (panelin pull-up'ıyla boşta ~4.8V) | NodeMCU **D5** (GPIO14), **doğrudan** |
| 5 | Panel TX (5V) | Dönüştürücü HV1 → LV1 → NodeMCU **D7** (GPIO13) |

Kutu içinde:

- MP1584 OUT+ → NodeMCU VIN ve dönüştürücü HV.
- MP1584 OUT− → NodeMCU G ve dönüştürücünün iki GND'si.
- NodeMCU 3V3 → dönüştürücü LV.

D4, D8, RX ve TX boş kalır.

- **Panelin elektriğini kesme:** çoğu kurulumda dış siren öter. Canlı bağla, önce GND.
- **USB ve VIN'i aynı anda bağlama.**
- GPIO3 (kartın RX pini) kullanılamaz, çünkü karttaki CH340 hattı 1'de tutuyor.

## İki gün süren iki tuzak (önce bunu oku)

1. **Hat 19200 8N2, iki yönde de.** 1 stop bitle panel her oturum isteğine `03 0F ED` (kapat) der. Bunu "temiz cevap" sanmak kolay.
2. **ESP'nin RX pinindeki dahili pull-up kapalı olmalı.** Çekirdeğin UART swap'ı (`pinMode(13, FUNCTION_4)`) da EspSoftwareSerial da onu açıyor. Açıkken her cevabın başı bozuluyor, sadece sonu geliyordu. Firmware artık D7'de `enableRxGPIOPullUp(false)` ile EspSoftwareSerial kullanıyor. Ölçülen başarı bu değişiklikle 0/20'den 19/20 geçerli cevaba çıktı.

`scope` komutu, kesmeler kapalıyken TX'i kendisi sürüp D7'yi örnekliyor. İki hata da bununla bulundu.

## Kurulum

1. **Panel (Wintex ile bir kez):** Com1 = PC-Com olsun, UDL şifresini not et. Uzaktan kontrol için Partition 1'de *Uzaktan Kurulum* ve *Uzaktan Çözme*'yi işaretle.
2. **Sırlar:** [`texecom_sniffer/secrets.example.h`](texecom_sniffer/secrets.example.h) dosyasını `secrets.h` olarak kopyala ve şunları doldur. `secrets.h` git'e girmez.
   - Wi-Fi, OTA şifresi
   - Ham rakamlarla UDL: `1234` → `{1,2,3,4}`
   - @BotFather'dan bot token'ı ve chat id
   - İzinli numaralar ve isimler
   - Kapatma PIN'i, alarm kodundan farklı olsun
3. **Derle** (esp8266 core 3.1.x, EspSoftwareSerial çekirdekle geliyor):
   ```bash
   arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --output-dir build texecom_sniffer
   arduino-cli upload -p COM3 --fqbn esp8266:esp8266:nodemcuv2 --input-dir build texecom_sniffer
   # sonra Wi-Fi'den:
   arduino-cli upload -p <ESP_IP> -l network --fqbn esp8266:esp8266:nodemcuv2 -F password=<OTA_PASS> --input-dir build texecom_sniffer
   ```
4. **Zon adları:** `.ino` dosyasında `ZONE_NAMES`.
5. **Wi-Fi'den Wintex:** Panel Details → *Connect Via Network*, adres ESP'nin IP'si, port `10001`.

## Protokol

Çerçeve `[uzunluk][komut][veri…][checksum]`. Checksum, önceki baytların toplamının 0xFF ile XOR'u.

| Ne | Komut |
|---|---|
| Giriş | boş `Z` → seri no, sonra `Z` + UDL (ham rakamlar) → ürün dizesi ("Premier 412 VF.04") |
| Zonlar (8 × 1 bayt, `01` = aktif) | `R 0x0004F0 08` |
| Alan bloğu | `R 0x000D6B 13`: +7 giriş `01` / çıkış `10`, +8 kurulu, +9 hazır, +6 siren/flaş, +0x12 kurma hatası |
| Voltaj | `R 0x000DCE 04`: ham × 18/256 V, tek ölçüme dayanıyor |
| Alan 1 kur / kapat | `A 01` / `D 01`, ACK `06`. Uzaktan kurmada **çıkış süresi yok**, geri sayımı bot yapıyor. |

Ayrıntılı notlar: [`docs/PROTOCOL.tr.md`](docs/PROTOCOL.tr.md). Üzerine kurulduğu çalışmalar:

- [shuckc/pialarm](https://github.com/shuckc/pialarm): Premier Elite Wintex kayıtları.
- [RoganDawes ESPHome Wintex bileşeni](https://github.com/RoganDawes/esphome/tree/wintex_v2/esphome/components/wintex): Premier 832.

## Güvenlik

- **Port 23 (monitör) ve 10001 (Wintex köprüsü) şifresiz.** Wi-Fi'ndaki herkes bağlanabilir. Modemden dışarı açma, ESP'yi güvendiğin bir ağda ya da VLAN'da tut.
- UDL üzerinden kapatma kullanıcı kodu istemiyor, asıl koruma Telegram PIN'i ve kilit. Ailedeki tüm hesaplarda Telegram'ın iki adımlı doğrulamasını aç.
- Listede olmayan Telegram hesaplarını bot yok sayar.

## Sınırlar

Bilinen eksikler ve henüz doğrulanmayanlar:

- Tek bir Premier 412 (V15.04) üzerinde denendi. 816/832/Elite'te adresler farklı olabilir.
- Hırsız, panik, yangın ve sabotaj bitleri gerçek alarmla doğrulanmadı, sadece siren baytı doğrulandı. Elektrik kesintisi henüz çözülmedi.
- Geçmiş ve sayaçlar RAM'de, yeniden başlayınca sıfırlanır. Wi-Fi ya da ESP giderse bot sana haber veremez. Dış izleme servisi planlanıyor.
- Klavyeden kurma "Klavyeden" görünür, çünkü kullanıcı panelin kaydından henüz okunmuyor.

## Lisans

[MIT](LICENSE)
