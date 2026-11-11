# Wintex (UDL) protokolü: bilinenler

Kaynaklar:
- shuckc/pialarm `protocol/wintex-protocol.md` (Premier Elite 24 kayıtları)
- RoganDawes/esphome `wintex_v2` dalı, `esphome/components/wintex/` (Premier 832, non-Elite)
- HA forumu "Integrating Texecom Premier Alarm Panels via ESPHome using Wintex protocol" (816 Plus deneyimi)

Premier 412/816/832 (non-Elite) panelleri Crestron/Simple protokolünü desteklemiyor. Yol: Wintex.

## Premier 412 kurulum kılavuzundan (INS159-13)
- Com1 Device Type: 0 = PC-Com (varsayılan), 1 = RadioPlus, 2 = Inovonics, 3 = ComIP, 4 = WebWayOne/Emizon,
  5 = Chiron. Crestron yok.
- "Disable Online Printing" Off (varsayılan) → **Com1 gerçek zamanlı olay verisi (yazıcı çıktısı) gönderir.**
  Pasif dinlemede ASCII olay satırları görülebilir; Wintex'e gerek kalmadan arm/disarm/alarm olayları
  buradan okunabilir. Baud/format bilinmiyor, ilk kayıtta `baud`/`cfg` ile dene.
- UDL Passcode en fazla 8 karakter.
- Aux 12V çıkışı F2 (900 mA PTC) ile korunuyor, 1.0 A nominal.
- Kullanıcının kartı: PCB D5084-02.06, etiket "412-LS1 V15.04".

## Çerçeve
```
[uzunluk][komut][veri ...][checksum]
uzunluk  = kendisi + checksum dahil toplam bayt
checksum = (önceki tüm baytların toplamı & 0xFF) XOR 0xFF
```
Örnek: `03 5A A2` → 0x03+0x5A = 0x5D, 0x5D ^ 0xFF = 0xA2.

## Komutlar
| Yön | Bayt | Anlam | Cevap |
|---|---|---|---|
| istemci → panel | `Z` 5A | oturum / UDL ile giriş, boş veriyle heartbeat | `Z` |
| istemci → panel | `O` 4F | konfig oku: `adr(3) n` | `I` + `adr(3) n veri` |
| istemci → panel | `R` 52 | volatile (anlık durum) oku: `adr(3) n` | `W` + `adr(3) n veri` |
| istemci → panel | `I` 49 | konfig YAZ | `06` ACK |
| istemci → panel | `W` 57 | volatile YAZ (bypass, kurma vb.) | `06` ACK |
| istemci → panel | `K` 4B | sanal keypad tuşu | `06` |
| istemci → panel | `H` 48 | oturumu kapat | |
| istemci → panel | `P` 50 | ping (pialarm'da 10 sn'de bir) | `P` |
| istemci → panel | `A` `D` `S` `C` `U` | kur / çöz / kısmi kur / reset / commit | |
| panel → istemci | `06` | ACK | |
| panel → istemci | `0F` | kapatma | |

Adresler 3 bayt, büyük uç (big-endian).

## Fiziksel katman (412, 2026-11-08 ölçümü)
19200 8N2, iki yönde. Panel cevabı isteğin bitişinden ~4–5 ms sonra başlıyor (bazen ~25 ms). 8N1 istek → `03 0F ED` (kapat).

## Giriş (login)
412 (2026-11-08 kaydı): bare Z (`03 5A A2`) → `0B 5A` + 8 bayt seri no + checksum → istemci `Z` + UDL 4 bayt **ham rakam**
→ panel `Z` + ürün dizesi "Premier 412 VF.04". Ardından Wintex `O` ile okuyor: 0x000650, 0x001FE5, 0x0004F2 (n=1).
Oturum açıkken heartbeat: boş Z her 10 sn, panel her seferinde 11 baytlık seri no cevabını veriyor.
Elite 24 kaydı: `03 5A A2` → panel `Z 05 + seri no (BCD)` → istemci `Z` + UDL → panel `Z` + 16 karakter ürün dizesi.
- Elite'te UDL ASCII gidiyor (`31 32 ...`).
- **816 Plus'ta ASCII başarısız oldu, ham rakamlar çalıştı**: UDL `8888` → `08 08 08 08`.
  412 büyük olasılıkla 816 gibi. Rakam 0'ın nasıl kodlandığı bilinmiyor (00 mı 0A mı) → Wintex kaydından teyit et.
- Başarılı girişte cevap verisi tam 16 yazdırılabilir karakter (ürün/sürüm dizesi).

## Premier 832 adresleri (RoganDawes) — 412'de FARKLI olabilir
| Ne | Okuma | Adres | Uzunluk |
|---|---|---|---|
| Zone durumları, zone başına 1 bayt | `R` | 0x0004F8 | 0x20 (32 zone) |
| Alan (area) durumları | `R` | 0x000D85 | 0x13 |
| Sistem / akü voltajı | `R` | 0x000DC7 | 4 (2+2 bayt) |
| Adres tablosu? (denenmiş, çözülmemiş) | `O` | 0x001FE6 | 0x18 |

Zone baytı bitleri: `01` aktif (hareket/kapı açık), `02` tamper, `08` test, `10` alarm, `20` bypass, `40` otomatik bypass.
Bypass yazma: zone baytına `W` ile `A0` (bypass) / `80` (kaldır).

Alan bloğu içinde ofsetler (alan maskesi ile): 0x00 panik, 0x01 hırsız, 0x06 siren/flaş, 0x07 giriş/çıkış,
0x08 kurulu, 0x09 hazır/bypass, 0x0E reset/onay gerekli, 0x0F doğrulanmış alarm, 0x12 kurma hatası/tam kurulu.

## Premier 412 adresleri (2026-11-08, Wintex Online Status kaydı)
| Ne | Okuma | Adres | n | Not |
|---|---|---|---|---|
| Zon durumu | `R` | 0x0004F0 | 8 | Zon başına 1 bayt: `00` kapalı (secure), `01` aktif. Sensörle birlikte değişti (zon 1, 2). |
| Zon durumu (ikiz) | `R` | 0x0004D0 | 8 | 0x4F0 ile aynı değerler, bazen ~1 sn geç. Farkı bilinmiyor. |
| Sistem + akü voltajı | `R` | 0x000DCE | 4 | `C1 00 C1 00`, Wintex 13.57 V gösterdi → ham × 18/256 (tek ölçüm). |
| Panel durumu? | `R` | 0x0000EC | 1 | `99` ↔ `9D` (bit 0x04) arada değişiyor, anlamı bilinmiyor. |
| Bilinmeyen | `R` | 0x0000E1, 0x0000E9, 0x000CB4, 0x000DCF | 1 | Aynı ekran okuyor, çözülmedi. |

Wintex bu 8 okumayı saniyede ~3–4 tur yaptı, 289 turun hepsi cevap aldı. Tamper/alarm/bypass bitleri 412'de henüz doğrulanmadı
(832 tablosu yukarıda).

### Alan (Partition) bloğu: `R` 0x000D6B, n=0x13 (832'deki 0x000D85'in karşılığı)
Bir kurma/çözme kaydından (18:30, Alan 1), Alan 1 = bit 0 / bit 4. Ofsetler 832 tablosuyla uyumlu:
| Ofset | Değer | Anlam |
|---|---|---|
| 0x07 | `10` | çıkış süresi sayıyor (18:30:04–18:30:23) |
| 0x07 | `01` | giriş süresi sayıyor (18:30:28) |
| 0x08 | `01` | **kurulu** (18:30:23–18:30:32) |
| 0x09 | `0F` / `0E` | bit 0 = hazır: zon aktifken `0E` |
| 0x0A | `FF` / `FE` | kurma anında kısa süre bit 0 düşüyor, anlamı bilinmiyor |
| 0x0B, 0x0D, 0x10, 0x11 | `10`/`11` | kurma/çözme sırasında değişti, anlamı bilinmiyor |

Kurma hatası kaydı (19:11, çıkış bitince zon 1 aktifti): ofset 0x06 = `11` (siren+flaş; kullanıcı 2 sn sonra kodu girince `00`, panel kendi
susturmadı), ofset 0x12 = `01` (kaldı), ofset 0x08 `00` kaldı (kurulmadı), 0x000CB4 = `03` ~4 sn. Çıkış süresince ofset 0x07 = `10`, 0x10 = `10`, 0x0A kısa `FE`.
Hırsız/panik bitleri (0x00, 0x01) gerçek alarm tetiklenmediği için 412'de doğrulanmadı; 832 tablosu geçerli varsayılıyor.

### Uzaktan kurma (2026-11-08 Wintex Remote Control kaydı)
Partitions sayfasında Enable Remote Arming açılınca Wintex'in Arm düğmesi aktif oldu. Arm = `04 41 01 B9` (`A` + alan maskesi 01), panel `03 06 F6` (ACK).
Komut anında alan hazır değildi (0x000D73 bloğunda ofset 1 = `0E`, zon 1 aktif). 2–3 sn içinde siren çaldı, kurulu baytı `00` kaldı (normal çıkış 20 sn).
2026-11-09 kaydı (panel hazırken): `A 01` sonrası ~0,2 sn'de kurulu baytı `01`, yani uzaktan kurmada çıkış süresi yok. Disarm = `04 44 01 B6` (`D 01`), panel `03 06 F6` (ACK), kurulu baytı `00`.

## 412 için bulunacaklar (Wintex kaydından)
1. Login biçimi ve panel kimlik dizesi.
2. Wintex "Diagnostics → Zones" ekranı açıkken tekrarlanan `R` okumaları → zone durum adresi (412: 12 zone, muhtemelen n=0x0C ya da 0x10).
3. Alan durum ekranı → alan adresi. Kur/çöz yaparken değişen baytlar.
4. Heartbeat düzeni (`Z` boş mu `P` mi) ve oturum zaman aşımı.

Hızlı deneme (Wintex'siz, monitörden; sadece okuma):
```
tx on
wx Z                    (veri yok: panel cevap veriyor mu? Elite'te seri no dönüyordu)
wx Z 08 08 08 08        (UDL'yi kendi şifrenle değiştir; olmazsa ASCII dene: wx Z 38 38 38 38)
wx R 00 04 F8 20        (832 zone adresi; 412'de anlamlı veri dönüyor mu bak)
wx H
tx off
```
