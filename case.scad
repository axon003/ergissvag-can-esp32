// case.scad - cutie ESP32 DevKit V1 + 2 module aux (fara capac)
// @author Gabriel Diaconu (ERGISS Media)
// @version 1.2
// @changes
//   v1.0 2026-05-23 - creat initial
//   v1.1 2026-05-23 - adaugat modul 2 (16mm) cu 2 standoff
//   v1.2 2026-05-23 - module dispuse pe lungime (X), nu stivuite pe Y; standoff pe Y ambele

// --- parametri ESP32 DevKit V1 (DOIT 38-pin) ---
esp_pcb_l   = 55;     // lungime PCB (axa X)
esp_pcb_w   = 28;     // latime PCB (axa Y)
esp_hole_dx = 48;     // distanta intre gauri pe X
esp_hole_dy = 23;     // distanta intre gauri pe Y
esp_usb_w   = 10;     // latime cutout microUSB
esp_usb_h   = 6;      // inaltime cutout microUSB

// --- modul 1 (27 x 21.5 mm) ---
mod_l       = 27;     // lungime modul 1 (axa X)
mod_w       = 21.5;   // latime modul 1 (axa Y)
mod_gap     = 10;     // distanta intre ESP si modul 1 (axa X)

// --- modul 2 (16 x 16 mm) ---
mod2_l      = 16;     // lungime modul 2 (axa X)
mod2_w      = 16;     // latime modul 2 (axa Y)
mod2_gap    = 10;     // distanta intre modul 1 si modul 2 (axa X)
mod2_standoff_dx = 10; // separare intre cele 2 standoff modul 2 (axa X)

// --- standoffs ---
standoff_h  = 5;      // inaltime PCB peste podea
standoff_od = 5;      // diametru exterior
standoff_id = 2.2;    // diametru interior (M2.5 self-tap)

// --- cutie ---
wall        = 2;      // grosime perete
floor_h     = 2;      // grosime podea
margin      = 2;      // spatiu liber intre PCB si perete
box_h       = 28;     // inaltime totala perete

// --- derivate ---
inner_l = esp_pcb_l + mod_gap + mod_l + mod2_gap + mod2_l + 2*margin - 20;  // -20: standoff modul 2 mutate la stanga
inner_w = max(esp_pcb_w, mod_w, mod2_w) + 2*margin;
outer_l = inner_l + 2*wall;
outer_w = inner_w + 2*wall;

// pozitii (origine = coltul interior stanga-jos, peste podea)
esp_x0   = margin;
esp_y0   = (inner_w - esp_pcb_w) / 2;
mod_x0   = esp_x0 + esp_pcb_l + mod_gap;
mod_y0   = (inner_w - mod_w) / 2;
mod2_x0  = mod_x0 + mod_l + mod2_gap;
mod2_y0  = (inner_w - mod2_w) / 2;

$fn = 48;

// ===== CUTIE =====
difference() {
    translate([-wall, -wall, 0])
        cube([outer_l, outer_w, box_h]);

    translate([0, 0, floor_h])
        cube([inner_l, inner_w, box_h]);

    // cutout microUSB pe peretele stang
    usb_z_center = floor_h + standoff_h + 1.5;
    translate([-wall - 1, esp_y0 + esp_pcb_w/2 - esp_usb_w/2, usb_z_center - esp_usb_h/2])
        cube([wall + 2, esp_usb_w, esp_usb_h]);

    // cutout 5x5mm pentru fire pe peretele drept (latura scurta), in dreapta jos
    translate([inner_l - 1, margin, floor_h])
        cube([wall + 2, 5, 5]);
}

// ===== STANDOFFS ESP32 (4 colturi) =====
esp_cx = esp_x0 + esp_pcb_l/2;
esp_cy = esp_y0 + esp_pcb_w/2;
for (sx = [-1, 1], sy = [-1, 1])
    translate([esp_cx + sx*esp_hole_dx/2, esp_cy + sy*esp_hole_dy/2, floor_h])
        standoff();

// ===== STANDOFFS MODUL 1 (2 buc: sus la Y-ul ESP, jos cu dify=21.5 difx=-3.5) =====
mod_cx = mod_x0 + mod_l/2;
mod_cy = mod_y0 + mod_w/2;
mod1_top_x = mod_cx - 3;
mod1_top_y = esp_cy + esp_hole_dy/2;     // acelasi Y cu standoff-urile sus ale ESP
mod1_bot_x = mod1_top_x - 3.5;
mod1_bot_y = mod1_top_y - 21.5;
translate([mod1_top_x, mod1_top_y, floor_h]) standoff();
translate([mod1_bot_x, mod1_bot_y, floor_h]) standoff();

// ===== STANDOFFS MODUL 2 (2 buc, ambele sus aliniate la Y-ul ESP, separare pe X) =====
mod2_cx = mod2_x0 + mod2_l/2;
mod2_cy = mod2_y0 + mod2_w/2;
mod2_top_y = esp_cy + esp_hole_dy/2;
for (sx = [-1, 1])
    translate([mod2_cx + sx*mod2_standoff_dx/2 - 20, mod2_top_y, floor_h])
        standoff();

module standoff() {
    difference() {
        cylinder(h = standoff_h, d = standoff_od);
        translate([0, 0, 1])
            cylinder(h = standoff_h, d = standoff_id);
    }
}
