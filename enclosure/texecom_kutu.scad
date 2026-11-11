// Texecom köprü kutusu: NodeMCU V3 (LoLin) + MP1584EN + 4 kanal lojik seviye dönüştürücü
// Yazıcı: Bambu Lab A1 (destek gerekmez). Taban dik, kapak ters (üst yüzü tablaya) basılır.
//
// Çıktı almak için:  openscad -D 'part="base"' -o taban.stl texecom_kutu.scad
//                    openscad -D 'part="lid"'  -o kapak.stl texecom_kutu.scad
// part: "assembly" (önizleme) | "base" | "lid"
part = "assembly";

/* [Kutu] */
wall     = 1.2;   // yan duvar (0.4 nozul ile 3 çizgi; hızlı/ekonomik baskı)
floor_t  = 1.4;   // taban kalınlığı
lid_t    = 1.4;   // kapak kalınlığı
in_x     = 112;   // iç uzunluk
in_y     = 70;    // iç genişlik
in_h     = 34;    // iç yükseklik (taban üstünden duvar kenarına)
corner_r = 3;     // dış köşe yuvarlatma
fit      = 0.25;  // kapak dudağı ile duvar arası boşluk (kenar başına)
lip_t    = 1.2;   // kapak dudağı kalınlığı
lip_h    = 3.0;   // kapak dudağı yüksekliği

/* [Modül ölçüleri: kumpasla ölçüp düzelt] */
node_l     = 58.0;  // NodeMCU V3 PCB uzunluğu (ölçüldü)
node_w     = 31.5;  // NodeMCU V3 PCB genişliği
pcb_t      = 1.6;
node_top_h = 5.0;   // NodeMCU üstündeki en yüksek parça (USB, ESP-12, buton)
mp_l = 22.5;  mp_w = 17.5;   // MP1584EN (ölçüldü)
ls_l = 15.0;  ls_w = 13.5;   // lojik seviye dönüştürücü (ölçüldü)
clr  = 0.4;                  // modül çevresi boşluk

/* [Yükseklikler] */
post_h       = 24;  // NodeMCU PCB altının taban üstünden yüksekliği (alttan takılan DuPont için)
small_post_h = 24;  // küçük modül kaidesi; modülün pinleri YUKARI bakıyorsa 6 yap
pin_margin   = 3.6; // kaide kenarından pin sırasına mesafe (DuPont yuvası 2.54 mm)

/* [Yerleşim: iç köşe (sol-ön) = 0,0] */
node_x0 = 4;          // NodeMCU USB ucu sol duvara bakar
node_yc = in_y / 2;
mp_c    = [88, 51];   // MP1584EN merkezi
ls_c    = [88, 17];   // lojik dönüştürücü merkezi

/* [Delikler] */
usb_hole     = true;  usb_w = 12;  usb_h = 8;  usb_zoff = 1.5;
notch_y      = [in_y / 2];  // sağ duvardaki kablo oyuğu (Com1 demeti); kutu panele bu yüzü bakacak şekilde asılır
notch_w      = 10;  notch_d = 12;
floor_hole   = false; floor_hole_d = 12;  floor_hole_pos = [in_x - 12, in_y / 2];
vents        = true;  vent_w = 2.2;  vent_len = 20;  vent_pitch = 5;
tie_anchors  = true;       // kablo bağı (cırt) köprüleri

/* [Montaj] */
mount_ears = true;  ear_len = 12;  ear_w = 12;  ear_t = 3;  ear_hole_d = 4.4;
lid_screws = true;  post_s = 7;  pilot_d = 2.5;  pilot_depth = 14;   // M3 sac vidası
screw_d = 3.4;  cb_d = 6.4;  cb_depth = 1.2;
press_pins = true;  press_in = 2.5;  press_d = 3.5;  press_gap = 0.3;

// ---------------------------------------------------------------------------
ox = in_x + 2 * wall;
oy = in_y + 2 * wall;
bh = floor_t + in_h;
pcb_z = floor_t + post_h;          // NodeMCU PCB alt yüzü
eps = 0.01;
$fn = 32;

module rrect(x, y, h, r) {
  translate([r, r, 0]) linear_extrude(h) offset(r = r) square([x - 2 * r, y - 2 * r]);
}

module slot(len, w, h) {  // Y yönünde uzun, merkezli yuvarlak uçlu yarık
  hull() for (s = [-1, 1]) translate([0, s * (len - w) / 2, 0]) cylinder(d = w, h = h, $fn = 24);
}

function corner_posts() = [[0, 0], [in_x - post_s, 0], [0, in_y - post_s], [in_x - post_s, in_y - post_s]];

// ---------------- TABAN ----------------
module ear_shape() {
  hull() {
    translate([-1, -ear_w / 2, 0]) cube([1, ear_w, ear_t]);
    translate([ear_len - ear_w / 2, 0, 0]) cylinder(d = ear_w, h = ear_t, $fn = 48);
  }
}

module holder_corner() {  // PCB köşesi orijinde, PCB +x +y yönünde
  hold_t = 1.6; ledge = 3; lip = 1.5;
  top = pcb_z + pcb_t + lip;
  translate([-hold_t, -hold_t, 0]) cube([hold_t + ledge, hold_t + ledge, pcb_z]);   // oturma basamağı
  translate([-hold_t, -hold_t, 0]) cube([ledge + 2 * hold_t, hold_t, top]);        // x boyunca duvar
  translate([-hold_t, -hold_t, 0]) cube([hold_t, ledge + 2 * hold_t, top]);        // y boyunca duvar
}

module node_holders() {
  x0 = node_x0 - clr;  y0 = node_yc - node_w / 2 - clr;
  L = node_l + 2 * clr;  W = node_w + 2 * clr;
  for (cx = [0, 1], cy = [0, 1])
    translate([x0 + cx * L, y0 + cy * W, 0])
      scale([cx == 0 ? 1 : -1, cy == 0 ? 1 : -1, 1]) holder_corner();
}

module pedestal(c, l, w) {
  pl = l - 2 * pin_margin;  pw = w - 2 * pin_margin;
  translate([c[0] - pl / 2, c[1] - pw / 2, 0]) cube([pl, pw, floor_t + small_post_h]);
  translate([c[0] - pl / 2 - 2, c[1] - pw / 2 - 2, 0]) cube([pl + 4, pw + 4, floor_t + 2]);  // ayak
}

module tie_anchor() {  // cırt Y yönünde köprünün altından geçer
  l = 10; w = 5; h = 4.5; gap = 4.5; tun = 2.2;
  difference() {
    translate([-l / 2, -w / 2, 0]) cube([l, w, floor_t + h]);
    translate([-gap / 2, -w / 2 - 1, floor_t]) cube([gap, w + 2, tun]);
  }
}

module base() {
  difference() {
    union() {
      difference() {
        rrect(ox, oy, bh, corner_r);
        translate([wall, wall, floor_t]) cube([in_x, in_y, in_h + 1]);
      }
      if (mount_ears) {
        translate([ox, oy / 2, 0]) ear_shape();
        translate([0, oy / 2, 0]) mirror([1, 0, 0]) ear_shape();
      }
    }
    if (mount_ears)
      for (x = [ox + ear_len - ear_w / 2, -(ear_len - ear_w / 2)])
        translate([x, oy / 2, -1]) cylinder(d = ear_hole_d, h = ear_t + 2);
    if (vents)  // taban havalandırması (NodeMCU altı ve orta bant)
      for (x = [10 : vent_pitch : in_x - 26])
        translate([wall + x, wall + in_y / 2, -1]) slot(vent_len, vent_w, floor_t + 2);
    if (usb_hole)
      translate([-1, wall + node_yc - usb_w / 2, pcb_z + pcb_t + usb_zoff - usb_h / 2])
        cube([wall + 2, usb_w, usb_h]);
    for (yc = notch_y)  // sağ duvarda U kablo oyukları, kapak üstten kapatır
      translate([ox - wall - 1, wall + yc, bh - notch_d]) {
        translate([0, -notch_w / 2, notch_w / 2]) cube([wall + 2, notch_w, notch_d]);
        translate([0, 0, notch_w / 2]) rotate([0, 90, 0]) cylinder(d = notch_w, h = wall + 2, $fn = 40);
      }
    if (floor_hole)
      translate([wall + floor_hole_pos[0], wall + floor_hole_pos[1], -1])
        cylinder(d = floor_hole_d, h = floor_t + 2, $fn = 48);
  }
  translate([wall, wall, 0]) {
    node_holders();
    pedestal(mp_c, mp_l, mp_w);
    pedestal(ls_c, ls_l, ls_w);
    if (tie_anchors) for (yc = notch_y) translate([in_x - 9, yc, 0]) tie_anchor();
    if (lid_screws)
      for (p = corner_posts()) translate([p[0], p[1], 0]) difference() {
        cube([post_s, post_s, bh]);
        translate([post_s / 2, post_s / 2, bh - pilot_depth]) cylinder(d = pilot_d, h = pilot_depth + 1, $fn = 24);
      }
  }
}

// ---------------- KAPAK (takılı konumda modellenir) ----------------
module lid_assembled() {
  pin_bottom = pcb_z + pcb_t + press_gap;
  difference() {
    union() {
      translate([0, 0, bh]) rrect(ox, oy, lid_t, corner_r);
      translate([wall + fit, wall + fit, bh - lip_h]) difference() {
        cube([in_x - 2 * fit, in_y - 2 * fit, lip_h + eps]);
        translate([lip_t, lip_t, -1]) cube([in_x - 2 * fit - 2 * lip_t, in_y - 2 * fit - 2 * lip_t, lip_h + 2]);
      }
      if (press_pins)  // sadece anten ucu köşeleri: USB ucundaki butonlara basmasın
        for (s = [-1, 1])
          translate([wall + node_x0 + node_l - press_in, wall + node_yc + s * (node_w / 2 - press_in), pin_bottom])
            cylinder(d = press_d, h = bh - pin_bottom + eps);
    }
    if (lid_screws) for (p = corner_posts()) {
      translate([wall + p[0] - 1, wall + p[1] - 1, bh - lip_h - 1]) cube([post_s + 2, post_s + 2, lip_h + 1]);
      translate([wall + p[0] + post_s / 2, wall + p[1] + post_s / 2, bh - 1]) cylinder(d = screw_d, h = lid_t + 2);
      translate([wall + p[0] + post_s / 2, wall + p[1] + post_s / 2, bh + lid_t - cb_depth]) cylinder(d = cb_d, h = cb_depth + 1);
    }
    if (usb_hole)
      translate([wall - 1, wall + node_yc - usb_w / 2 - 1, bh - lip_h - 1]) cube([fit + lip_t + 2, usb_w + 2, lip_h + 1]);
    if (vents)
      for (x = [10 : vent_pitch : in_x - 10])
        translate([wall + x, wall + in_y / 2, bh - 1]) slot(vent_len, vent_w, lid_t + 2);
  }
}

module lid_print() { translate([0, oy, bh + lid_t]) rotate([180, 0, 0]) lid_assembled(); }

// ---------------- ÖNİZLEME İÇİN TEMSİLİ MODÜLLER ----------------
module dupont_row(n, x0, y, ztop) {  // ztop = PCB alt yüzü; header (2.5) + DuPont yuvası (14)
  for (i = [0 : n - 1]) translate([x0 + i * 2.54 - 1.27, y - 1.27, ztop - 16.5]) {
    color("black") translate([0, 0, 14]) cube([2.54, 2.54, 2.5]);
    color("gold") cube([2.54, 2.54, 14]);
  }
}

module dummies() {
  translate([wall, wall, 0]) {
    // NodeMCU V3
    nx = node_x0; ny = node_yc - node_w / 2;
    color("royalblue") translate([nx, ny, pcb_z]) cube([node_l, node_w, pcb_t]);
    color("silver") translate([nx + node_l - 24, node_yc - 8, pcb_z + pcb_t]) cube([24, 16, 3]);  // ESP-12
    color("silver") translate([nx - 1, node_yc - 4, pcb_z + pcb_t]) cube([6, 8, 3]);              // micro USB
    px0 = nx + (node_l - 14 * 2.54) / 2;
    for (s = [-1, 1]) dupont_row(15, px0, node_yc + s * 13.97, pcb_z);
    // MP1584EN
    color("darkblue") translate([mp_c[0] - mp_l / 2, mp_c[1] - mp_w / 2, floor_t + small_post_h]) cube([mp_l, mp_w, 1.2]);
    color("dimgray") translate([mp_c[0] - 3, mp_c[1] - 3, floor_t + small_post_h + 1.2]) cube([6, 6, 4]);
    for (sx = [-1, 1], sy = [-1, 1])
      dupont_row(1, mp_c[0] + sx * (mp_l / 2 - 1.5), mp_c[1] + sy * (mp_w / 2 - 1.5), floor_t + small_post_h);
    // Lojik seviye dönüştürücü
    color("red") translate([ls_c[0] - ls_l / 2, ls_c[1] - ls_w / 2, floor_t + small_post_h]) cube([ls_l, ls_w, 1.2]);
    for (s = [-1, 1]) dupont_row(6, ls_c[0] - 2.5 * 2.54, ls_c[1] + s * (ls_w / 2 - 1.27), floor_t + small_post_h);
  }
}

if (part == "base") base();
else if (part == "lid") lid_print();
else if (part == "open") { color("lightgray") base(); dummies(); }
else if (part == "guide" || part == "guide_iso") {  // küçük modüllerin kaidelerini vurgular
  color("lightgray") base();
  translate([wall, wall, 0]) {
    for (m = [[mp_c, mp_l, mp_w, "royalblue"], [ls_c, ls_l, ls_w, "red"]]) {
      pl = m[1] - 2 * pin_margin;  pw = m[2] - 2 * pin_margin;
      color(m[3]) translate([m[0][0] - pl / 2 - 0.2, m[0][1] - pw / 2 - 0.2, 0]) cube([pl + 0.4, pw + 0.4, floor_t + small_post_h + 0.2]);
      color(m[3], 0.3) translate([m[0][0] - m[1] / 2, m[0][1] - m[2] / 2, floor_t + small_post_h + 0.3]) cube([m[1], m[2], 0.4]);
    }
    if (part == "guide") color("black") translate([0, 0, bh + 1]) linear_extrude(0.4) {
      translate([mp_c[0], mp_c[1] + mp_w / 2 + 2.5]) text("MP1584", size = 4.5, halign = "center");
      translate([ls_c[0], ls_c[1] - ls_w / 2 - 7]) text("LOJIK", size = 4.5, halign = "center");
      translate([node_x0 + node_l / 2, node_yc - 2]) text("NodeMCU", size = 5, halign = "center");
    }
  }
}
else {
  color("lightgray") base();
  dummies();
  color("orange") translate([0, 0, 45]) lid_assembled();
}
