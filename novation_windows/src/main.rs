/*
 * launchpad-qlc: Novation Launchpad Mini MK3 -> QLC+ OSC bridge
 *
 * Pages (defined in config.toml, top-row CC 91-98):
 *   - Normal:  individual buttons -> QLC+ OSC addresses
 *   - Custom:  all 64 pads freely assignable; long-press opens on-pad colour picker
 *   - XY:      8x8 grid -> pan/tilt OSC for moving heads
 *   - Fader:   columns = 8-step dimmer faders
 *   - Main:    triggers; cols 2-3 = 16-way fader-A selector; col 4 = live fader;
 *              col 6 = fader-B selector; col 7 = flash fader (16 OSC addresses)
 *   - RGBW:    4 channels (R,G,B,W) x 2 cols wide = full 8-step faders
 *   - Nudge XY: nudge rings + master / 4 fixture pan-tilt profiles (right-side CCs)
 *
 *              Right-side buttons (CC 89 top -> CC 19 bottom):
 *                CC 89 = profile 0 = MASTER  (sends to all fixtures)
 *                CC 79 = profile 1 = fixture 1
 *                ...
 *                CC 29 = profile 6 = fixture 6
 *                CC 19 = unused
 *
 *              Master profile OVERRIDES all fixtures: whatever you set on
 *              master is sent to every fixture immediately.
 *              Switching to a fixture profile loads a copy of the master
 *              values as a starting point.
 *              Switching back to master only re-renders; no OSC is sent.
 *
 * QLC+ setup:
 *   Inputs/Outputs -> OSC -> enable input on port 7700 (or config.toml value)
 */

use std::{
    collections::HashMap,
    net::UdpSocket,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use log::{debug, error, info};
use midir::{MidiInput, MidiOutput, MidiOutputConnection};
use rosc::{OscMessage, OscPacket, OscType, encoder};
use serde::{Deserialize, Serialize};

/* ────────────────────────────────────────────────────────────────────────────
 * Config
 * ──────────────────────────────────────────────────────────────────────────── */

#[derive(Debug, Deserialize, Clone)]
struct Config {
    osc: OscConfig,
    midi: MidiConfig,
    pages: Vec<PageConfig>,
}

#[derive(Debug, Deserialize, Clone)]
struct OscConfig {
    host: String,
    port: u16,
}

#[derive(Debug, Deserialize, Clone)]
struct MidiConfig {
    device_name: String,
    device_out_name: String,
}

#[derive(Debug, Deserialize, Clone)]
struct PageConfig {
    name: String,
    selector_color: u8,
    #[serde(default)]
    buttons: Vec<ButtonConfig>,
    #[serde(default)]
    xy_mode: bool,
    #[serde(default)]
    fader_mode: bool,
    #[serde(default)]
    custom_mode: bool,
    #[serde(default)]
    rgbw_mode: bool,
    #[serde(default)]
    nudge_xy_mode: bool,
    #[serde(default)]
    flash_fader_mode: bool,
    #[serde(default)]
    main_mode: bool,
    xy_config: Option<XyConfig>,
    nudge_xy_config: Option<NudgeXyConfig>,
    fader_config: Option<FaderConfig>,
    rgbw_config: Option<RgbwConfig>,
}

#[derive(Debug, Deserialize, Clone)]
struct ButtonConfig {
    note: u8,
    label: String,
    color: u8,
    osc_address: String,
    osc_value: f32,
}

#[derive(Debug, Deserialize, Clone)]
struct XyConfig {
    pan_address: String,
    tilt_address: String,
    color_inactive: u8,
    color_active: u8,
}

#[derive(Debug, Deserialize, Clone)]
struct NudgeXyConfig {
    pan_addresses: Vec<String>,
    tilt_addresses: Vec<String>,
}

#[derive(Debug, Deserialize, Clone)]
struct FaderConfig {
    addresses: Vec<String>,
    flash_addresses: Vec<String>,
    color_on: u8,
    color_off: u8,
}

/*
 * One OSC address per fixture (6 fixtures) for each of the 4 channels.
 */
#[derive(Debug, Deserialize, Clone)]
struct RgbwConfig {
    r_addresses: Vec<String>,
    g_addresses: Vec<String>,
    b_addresses: Vec<String>,
    w_addresses: Vec<String>,
}

/* ────────────────────────────────────────────────────────────────────────────
 * RGBW layout constants
 *
 * Grid cols:  0-1 = Red, 2-3 = Green, 4-5 = Blue, 6-7 = White
 * Each pair of cols acts as one fader (2 pads wide, 8 steps tall).
 *
 * Right-side CCs (bottom->top): 19, 29, 39, 49, 59, 69, 79, 89
 *   index 0 (CC 19) = unused
 *   index 1 (CC 29) = profile 6 = fixture 6
 *   index 2 (CC 39) = profile 5 = fixture 5
 *   index 3 (CC 49) = profile 4 = fixture 4
 *   index 4 (CC 59) = profile 3 = fixture 3
 *   index 5 (CC 69) = profile 2 = fixture 2
 *   index 6 (CC 79) = profile 1 = fixture 1
 *   index 7 (CC 89) = profile 0 = MASTER
 * ──────────────────────────────────────────────────────────────────────────── */

const RIGHT_CC: [u8; 8] = [19, 29, 39, 49, 59, 69, 79, 89];

const RGBW_NUM_PROFILES: usize = 7; /* 0 = master, 1-6 = fixtures */

const CH_R: usize = 0;
const CH_G: usize = 1;
const CH_B: usize = 2;
const CH_W: usize = 3;

const RGBW_COLOR_R_ON:  u8 = 5;
const RGBW_COLOR_R_OFF: u8 = 0;
const RGBW_COLOR_G_ON:  u8 = 21;
const RGBW_COLOR_G_OFF: u8 = 0;
const RGBW_COLOR_B_ON:  u8 = 45;
const RGBW_COLOR_B_OFF: u8 = 0;
const RGBW_COLOR_W_ON:  u8 = 3;
const RGBW_COLOR_W_OFF: u8 = 0;

const PROFILE_COLOR_MASTER:  u8 = 3; /* white  */
const PROFILE_COLOR_FIXTURE: u8 = 45; /* blue   */
const PROFILE_COLOR_ACTIVE:  u8 = 13; /* yellow */
const PROFILE_COLOR_UNUSED:  u8 = 0;

/* Nudge XY: profile 0 = master, profiles 1–4 = individual heads */
const NUDGE_XY_NUM_PROFILES: usize = 5;
const NUDGE_XY_DEFAULT: f32 = 0.5;

/* ────────────────────────────────────────────────────────────────────────────
 * SysEx / MIDI helpers
 * ──────────────────────────────────────────────────────────────────────────── */

const SYSEX_PROGRAMMER_MODE: &[u8] = &[
    0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x0E, 0x01, 0xF7,
];

fn sysex_set_leds_bulk(pairs: &[(u8, u8)]) -> Vec<u8> {
    let mut msg = vec![0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x03];
    for (note, color) in pairs {
        msg.push(0x00);
        msg.push(*note);
        msg.push(*color);
    }
    msg.push(0xF7);
    msg
}

fn sysex_clear_all() -> Vec<u8> {
    let pairs: Vec<(u8, u8)> = (0u8..=99).map(|n| (n, 0)).collect();
    sysex_set_leds_bulk(&pairs)
}

const PAGE_SELECT_CC: [u8; 8] = [91, 92, 93, 94, 95, 96, 97, 98];

fn grid_note(col: u8, row: u8) -> u8 {
    (row + 1) * 10 + (col + 1)
}

fn note_to_grid(note: u8) -> Option<(u8, u8)> {
    if note < 11 || note > 88 { return None; }
    let row = (note / 10).saturating_sub(1);
    let col = (note % 10).saturating_sub(1);
    if col > 7 || row > 7 { None } else { Some((col, row)) }
}

fn grid_to_pad_index(col: u8, row: u8) -> u8 {
    row * 8 + col + 1
}

/*
 * Nudge XY mode: determine direction and magnitude from grid position.
 * Returns (pan_nudge, tilt_nudge) as f32 increments.
 * Black (1)    = no function
 * Blue (45)    = unprecise (coarse) nudges ±0.25
 * Magenta (53) = medium nudges ±0.1
 * White (72)   = fine nudges ±0.05
 *
 * Each click sends the nudge immediately; click multiple times to nudge multiple steps.
 */
fn nudge_from_position(col: u8, row: u8, state: &mut BridgeState) -> Option<(f32, f32)> {
    const UNPRECISE: f32 = 0.25; /* blue */
    const MEDIUM: f32 = 0.05;     /* magenta */
    const FINE: f32 = 0.005;      /* white */

    let mut pan_nudge = 0.0;
    let mut tilt_nudge = 0.0;
    let mut magnitude = 0.0;
    info!("Calculating nudge for position col={}, row={}", col, row);
    /* Determine nudge direction and magnitude based on position */
    match (col, row) {
        /* Row 7 (top) - vertical up nudges */
        (2, 7) | (3, 7) | (4, 7) | (5, 7) => {
            tilt_nudge = -UNPRECISE; /* blue = unprecise */
        }
        /* Row 6 - up nudges, magenta = medium */
        (2, 6) | (3, 6) | (4, 6) | (5, 6) => {
            tilt_nudge = -MEDIUM;
        }
        (3, 5) | (4, 5) => {
            tilt_nudge = -FINE; /* up */
        }
        /* Rows 5,2 - mixed nudges (up/down at edges, horizontal on sides) 
        (0, 5) | (7, 5) | (0, 2) | (7, 2) => {
            /* Corners: blue = unprecise */
            if row == 5 {
                tilt_nudge = if col == 0 { -UNPRECISE } else { -UNPRECISE }; /* up */
            } else {
                tilt_nudge = if col == 0 { UNPRECISE } else { UNPRECISE }; /* down */
            }
        } */
        /*(1, 5) | (6, 5) | (1, 2) | (6, 2) => {
            /* magenta = medium */
            if row == 5 {
                tilt_nudge = -MEDIUM; /* up */
            } else {
                tilt_nudge = MEDIUM; /* down */
            }
        }*/
        /*(2, 5) | (5, 5) | (2, 2) | (5, 2) => {
            /* white = fine */
            if row == 5 {
                tilt_nudge = -FINE; /* up */
            } else {
                tilt_nudge = FINE; /* down */
            }
        }*/
        /* Rows 4,3 - left/right nudges */
        (0,5) | (0, 4) | (0, 3) | (0,2) | (7,5) | (7, 4) | (7, 3) | (7,2) => {
            /* blue = unprecise */
            pan_nudge = if col == 0 { -UNPRECISE } else { UNPRECISE };
        }
        (1, 5) | (6, 5) | (1, 2) | (6, 2) | (1, 4) | (1, 3) | (6, 4) | (6, 3) => {
            /* magenta = medium */
            pan_nudge = if col == 1 { -MEDIUM } else { MEDIUM };
        }
        (2, 4) | (2, 3) | (5, 4) | (5, 3) => {
            /* white = fine */
            pan_nudge = if col == 2 { -FINE } else { FINE };
        }
        /* Row 1 - down nudges, magenta = medium */
        (2, 1) | (3, 1) | (4, 1) | (5, 1) => {
            tilt_nudge = MEDIUM;
        }
        /* Row 0 (bottom) - down nudges, blue = unprecise */
        (2, 0) | (3, 0) | (4, 0) | (5, 0) => {
            tilt_nudge = UNPRECISE;
        }
        (3, 2) | (4, 2) => {
            tilt_nudge = FINE; /* up */
        }
        _ => {
        info!("Handling center button press for position col={}, row={}", col, row);
        if let Err(e) = state.send_osc(&format!("lp/mov/{}",col*10+row), 1.0) {
            error!("OSC: {}", e);
        }
        }
        /* Black buttons and center: no function */
    }

    if pan_nudge != 0.0 || tilt_nudge != 0.0 {
        Some((pan_nudge, tilt_nudge))
    } else {
        None
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 * Colour picker helpers
 * ──────────────────────────────────────────────────────────────────────────── */

fn picker_color_for_cell(col: u8, row: u8) -> u8 {
    /* Colour palette layout on 8x8 grid:
     * Top-left (col 0, row 7) = colour 0 (black/off)
     * Rest = colours 1-64 (left to right, top to bottom) */
    if col == 0 && row == 7 {
        0 /* black */
    } else {
        (7 - row) * 8 + col /* colours 1-64 */
    }
}

fn picker_cell_for_color(color: u8) -> Option<(u8, u8)> {
    if color == 0 {
        return Some((0, 7)); /* black is at top-left */
    }
    if color > 64 { return None; }
    let idx = color - 1;
    Some((idx % 8, 7 - (idx / 8)))
}

const LONG_PRESS_MS: u64 = 500;

const MAIN_NUM_FADERS: usize = 16;

/* Main-page layout (0-indexed cols/rows, row 0 = bottom) */
const MAIN_FADER_LEFT: u8 = 4;
const MAIN_SELECTOR_RIGHT: u8 = 6;
const MAIN_FLASH_FADER: u8 = 7;

/* Fader-A selector: cols 2–3 × 8 rows = 16 slots */
const MAIN_SELECTOR_A_COLS: [u8; 2] = [2, 3];

const MAIN_SELECTOR_ACTIVE: u8 = 72;   /* white  */
const MAIN_SELECTOR_INACTIVE: u8 = 45; /* blue   */

fn main_selector_a_slot(col: u8, row: u8) -> Option<usize> {
    if !MAIN_SELECTOR_A_COLS.contains(&col) {
        return None;
    }
    let base = if col == 2 { 0usize } else { 8 };
    Some(base + row as usize)
}

fn main_is_selector_a(col: u8, row: u8) -> bool {
    main_selector_a_slot(col, row).is_some()
}

fn main_is_selector_b(col: u8) -> bool {
    col == MAIN_SELECTOR_RIGHT
}

/* Trigger pads: cols 0–1 (full height), col 5 rows 0–5 */
fn main_is_button_cell(col: u8, row: u8) -> bool {
    if col <= 1 {
        return true;
    }
    col == 5 && row <= 5
}

fn main_is_fader_a(col: u8, _row: u8) -> bool {
    col == MAIN_FADER_LEFT
}

fn main_is_flash_fader_b(col: u8) -> bool {
    col == MAIN_FLASH_FADER
}

fn main_a_level_to_osc(level: u8) -> f32 {
    if level == 0 { 0.0 } else { (level - 1) as f32 / 7.0 }
}

fn main_b_level_to_osc(level: u8) -> f32 {
    if level == 0 { 0.0 } else { (level - 1) as f32 / 6.0 }
}

/* ────────────────────────────────────────────────────────────────────────────
 * RGBW state
 * ──────────────────────────────────────────────────────────────────────────── */

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RgbwProfile {
    /* levels[channel] = 0..=8  (0 = off, 8 = full) */
    levels: [u8; 4],
}

impl Default for RgbwProfile {
    fn default() -> Self { Self { levels: [0; 4] } }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RgbwState {
    /* profile 0 = master, profiles 1-6 = individual fixtures */
    profiles: [RgbwProfile; RGBW_NUM_PROFILES],
    /* currently selected profile (0 = master) */
    active_profile: usize,
}

impl Default for RgbwState {
    fn default() -> Self {
        Self {
            profiles: std::array::from_fn(|_| RgbwProfile::default()),
            active_profile: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct NudgeXyProfile {
    pan: f32,
    tilt: f32,
}

impl Default for NudgeXyProfile {
    fn default() -> Self {
        Self { pan: NUDGE_XY_DEFAULT, tilt: NUDGE_XY_DEFAULT }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct NudgeXyState {
    profiles: [NudgeXyProfile; NUDGE_XY_NUM_PROFILES],
    active_profile: usize,
}

impl Default for NudgeXyState {
    fn default() -> Self {
        Self {
            profiles: std::array::from_fn(|_| NudgeXyProfile::default()),
            active_profile: 0,
        }
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 * Overlay
 * ──────────────────────────────────────────────────────────────────────────── */

enum Overlay {
    None,
    ColorPicker { page_idx: usize, note: u8 },
    /* Flash/fader colour picker: page_idx and col (0-7) identify the fader */
    FlashFaderColorPicker { page_idx: usize, col: u8 },
}

/* ────────────────────────────────────────────────────────────────────────────
 * Bridge state
 * ──────────────────────────────────────────────────────────────────────────── */

struct BridgeState {
    config: Config,
    current_page: usize,
    osc_socket: UdpSocket,
    osc_target: String,
    fader_levels: Vec<u8>,
    xy_selected: Option<(u8, u8)>,
    overlay: Overlay,
    custom_colors: HashMap<(usize, u8), u8>,
    /* Per-fader colour overrides: key = (page_idx, col), value = palette colour */
    fader_colors: HashMap<(usize, u8), u8>,
    press_times: HashMap<u8, Instant>,
    colors_path: String,
    rgbw_states: HashMap<usize, RgbwState>,
    rgbw_path: String,
    nudge_xy_states: HashMap<usize, NudgeXyState>,
    nudge_xy_path: String,
    /* Track if CC 19 (colour picker modifier) is held down */
    color_picker_modifier_active: bool,
    /* Track which flash button columns are currently pressed (for rendering) */
    flash_buttons_pressed: Vec<bool>,
    /* Main page: OSC slot selection per column (independent) */
    main_fader_a: usize,
    main_fader_b: usize,
    /* Main page: column fader heights — independent of slot selection */
    main_level_a: u8,
    main_level_b: u8,
}

/* ── Persistence ────────────────────────────────────────────────────────────── */

type ColorsFile = HashMap<String, u8>;

fn colors_to_file(map: &HashMap<(usize, u8), u8>) -> ColorsFile {
    map.iter()
        .map(|((page, note), &color)| (format!("{}:{}", page, note), color))
        .collect()
}

fn colors_from_file(file: ColorsFile) -> HashMap<(usize, u8), u8> {
    file.into_iter()
        .filter_map(|(key, color)| {
            let mut parts = key.splitn(2, ':');
            let page: usize = parts.next()?.parse().ok()?;
            let note: u8    = parts.next()?.parse().ok()?;
            Some(((page, note), color))
        })
        .collect()
}

impl BridgeState {
    fn new(config: Config, colors_path: String, rgbw_path: String, nudge_xy_path: String) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("bind UDP")?;
        let target = format!("{}:{}", config.osc.host, config.osc.port);

        let custom_colors = std::fs::read_to_string(&colors_path)
            .ok()
            .and_then(|s| serde_json::from_str::<ColorsFile>(&s).ok())
            .map(colors_from_file)
            .unwrap_or_default();

        /* derive fader_colors path from colors_path */
        let fader_colors_path = colors_path.replace("custom_colors.json", "fader_colors.json");
        let fader_colors: HashMap<(usize, u8), u8> = std::fs::read_to_string(&fader_colors_path)
            .ok()
            .and_then(|s| serde_json::from_str::<ColorsFile>(&s).ok())
            .map(colors_from_file)
            .unwrap_or_default();

        let rgbw_states: HashMap<usize, RgbwState> = std::fs::read_to_string(&rgbw_path)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();

        let nudge_xy_states: HashMap<usize, NudgeXyState> = std::fs::read_to_string(&nudge_xy_path)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();

        info!(
            "Loaded {} custom colour(s), {} fader colour(s), {} RGBW page state(s), {} nudge XY state(s)",
            custom_colors.len(), fader_colors.len(), rgbw_states.len(), nudge_xy_states.len()
        );

        Ok(Self {
            fader_levels: vec![0u8; 8],
            xy_selected: None,
            overlay: Overlay::None,
            custom_colors,
            fader_colors,
            press_times: HashMap::new(),
            current_page: 0,
            osc_socket: socket,
            osc_target: target,
            colors_path,
            rgbw_states,
            rgbw_path,
            nudge_xy_states,
            nudge_xy_path,
            color_picker_modifier_active: false,
            flash_buttons_pressed: vec![false; 8],
            main_fader_a: 0,
            main_fader_b: 0,
            main_level_a: 0,
            main_level_b: 0,
            config,
        })
    }

    fn send_osc(&self, address: &str, value: f32) -> Result<()> {
        let msg = OscPacket::Message(OscMessage {
            addr: address.to_string(),
            args: vec![OscType::Float(value)],
        });
        let bytes = encoder::encode(&msg).context("encode OSC")?;
        self.osc_socket.send_to(&bytes, &self.osc_target).context("send OSC")?;
        debug!("OSC -> {} = {:.3}", address, value);
        Ok(())
    }

    fn save_colors(&self) {
        match serde_json::to_string_pretty(&colors_to_file(&self.custom_colors)) {
            Ok(json) => { let _ = std::fs::write(&self.colors_path, json); }
            Err(e)   => error!("Serialise colors: {}", e),
        }
    }

    fn save_fader_colors(&self) {
        let fader_colors_path = self.colors_path.replace("custom_colors.json", "fader_colors.json");
        match serde_json::to_string_pretty(&colors_to_file(&self.fader_colors)) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&fader_colors_path, json) {
                    error!("Save fader colours: {}", e);
                }
            }
            Err(e) => error!("Serialise fader colours: {}", e),
        }
    }

    fn get_fader_color(&self, page_idx: usize, col: u8) -> u8 {
        if let Some(&c) = self.fader_colors.get(&(page_idx, col)) { return c; }
        /* fall back to config default */
        self.config.pages[page_idx].fader_config.as_ref()
            .map(|f| f.color_on).unwrap_or(72)
    }

    fn save_rgbw(&self) {
        match serde_json::to_string_pretty(&self.rgbw_states) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&self.rgbw_path, json) {
                    error!("Save RGBW state: {}", e);
                }
            }
            Err(e) => error!("Serialise RGBW: {}", e),
        }
    }

    fn custom_button_color(&self, page_idx: usize, note: u8) -> u8 {
        if let Some(&c) = self.custom_colors.get(&(page_idx, note)) { return c; }
        let page = &self.config.pages[page_idx];
        if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
            return btn.color;
        }
        if page.main_mode {
            return self.main_default_button_color(page_idx, note);
        }
        21
    }

    fn custom_osc_address(&self, page_idx: usize, note: u8) -> String {
        let page = &self.config.pages[page_idx];
        if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
            return btn.osc_address.clone();
        }
        if self.config.pages[page_idx].main_mode {
            return format!("/lp/main/{}", note);
        }
        note_to_grid(note)
            .map(|(col, row)| format!("/lp/c{}", grid_to_pad_index(col, row)))
            .unwrap_or_else(|| "/lp/custom/unknown".to_string())
    }

    fn main_default_button_color(&self, _page_idx: usize, note: u8) -> u8 {
        if let Some((col, row)) = note_to_grid(note) {
            if main_is_selector_a(col, row) || main_is_selector_b(col) {
                return MAIN_SELECTOR_INACTIVE;
            }
        }
        21 /* dim green */
    }

    fn ensure_rgbw_state(&mut self, page_idx: usize) {
        self.rgbw_states.entry(page_idx).or_default();
    }

    fn save_nudge_xy(&self) {
        match serde_json::to_string_pretty(&self.nudge_xy_states) {
            Ok(json) => {
                if let Err(e) = std::fs::write(&self.nudge_xy_path, json) {
                    error!("Save nudge XY: {}", e);
                }
            }
            Err(e) => error!("Serialise nudge XY: {}", e),
        }
    }

    fn ensure_nudge_xy_state(&mut self, page_idx: usize) {
        self.nudge_xy_states.entry(page_idx).or_default();
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 * Rendering
 * ──────────────────────────────────────────────────────────────────────────── */

fn render_page(state: &BridgeState, out: &mut MidiOutputConnection) {
    let mut pairs: Vec<(u8, u8)> = Vec::with_capacity(200);

    for n in 0u8..=99 { pairs.push((n, 0)); }

    /* colour picker overlay */
    if let Overlay::ColorPicker { page_idx, note } = &state.overlay {
        for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
            pairs.push((cc, if i == state.current_page { 5 } else { 0 }));
        }
        for row in 0u8..8 {
            for col in 0u8..8 {
                pairs.push((grid_note(col, row), picker_color_for_cell(col, row)));
            }
        }
        if let Some((pc, pr)) = picker_cell_for_color(state.custom_button_color(*page_idx, *note)) {
            pairs.push((grid_note(pc, pr), 72)); /* highlight current colour */
        }
        pairs.push((19, 13)); /* CC 19: yellow in colour picker */
        let _ = out.send(&sysex_set_leds_bulk(&pairs));
        return;
    }

    /* flash/fader colour picker overlay */
    if let Overlay::FlashFaderColorPicker { page_idx, col } = &state.overlay {
        for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
            pairs.push((cc, if i == state.current_page { 5 } else { 0 }));
        }
        for row in 0u8..8 {
            for c in 0u8..8 {
                pairs.push((grid_note(c, row), picker_color_for_cell(c, row)));
            }
        }
        let current_color = state.get_fader_color(*page_idx, *col);
        if let Some((pc, pr)) = picker_cell_for_color(current_color) {
            pairs.push((grid_note(pc, pr), 72)); /* highlight current colour */
        }
        pairs.push((19, 13)); /* CC 19: yellow in colour picker */
        let _ = out.send(&sysex_set_leds_bulk(&pairs));
        return;
    }

    /* page selector top row */
    for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
        let color = if i < state.config.pages.len() {
            if i == state.current_page { 72 } else { state.config.pages[i].selector_color }
        } else { 0 };
        pairs.push((cc, color));
    }

    let page = &state.config.pages[state.current_page];

    if page.rgbw_mode {
        render_rgbw(state, state.current_page, &mut pairs);
    } else if page.flash_fader_mode {
        if let Some(_fader) = &page.fader_config {
            for col in 0u8..8 {
                let level = *state.fader_levels.get(col as usize).unwrap_or(&0);
                let color_on  = state.get_fader_color(state.current_page, col);
                let is_flash_pressed = state.flash_buttons_pressed.get(col as usize).copied().unwrap_or(false);
                for row in 0u8..8 {
                    let color = if row == 0 {
                        /* bottom row: flash button
                         * - when pressed: show fader colour
                         * - when not pressed: show colour 1 (dim red) */
                        if is_flash_pressed { color_on } else { 1 }
                    } else {
                        /* rows 1-7: fader steps - light up in fader colour, black when off */
                        if level >= row { color_on } else { 0 }
                    };
                    pairs.push((grid_note(col, row), color));
                }
            }
        }
    } else if page.main_mode {
        render_main(state, &mut pairs);
    } else if page.custom_mode {
        for row in 0u8..8 {
            for col in 0u8..8 {
                let note = grid_note(col, row);
                pairs.push((note, state.custom_button_color(state.current_page, note)));
            }
        }
    } else if page.nudge_xy_mode {
        render_nudge_xy_grid(state, &mut pairs);
        if page.nudge_xy_config.is_some() {
            render_nudge_xy_profiles(state, state.current_page, &mut pairs);
        }
    } else if page.xy_mode {
        if let Some(xy) = &page.xy_config {
            for row in 0u8..8 {
                for col in 0u8..8 {
                    let color = if state.xy_selected == Some((col, row)) {
                        xy.color_active
                    } else { xy.color_inactive };
                    pairs.push((grid_note(col, row), color));
                }
            }
        }
    } else if page.fader_mode {
        if let Some(fader) = &page.fader_config {
            for col in 0u8..8 {
                let level = *state.fader_levels.get(col as usize).unwrap_or(&0);
                for row in 0u8..8 {
                    let color = if level >= row + 1 { fader.color_on } else { fader.color_off };
                    pairs.push((grid_note(col, row), color));
                }
            }
        }
    } else if page.flash_fader_mode {
        if let Some(fader) = &page.fader_config {
            for col in 0u8..8 {
                let level = *state.fader_levels.get(col as usize).unwrap_or(&0);
                for row in 0u8..8 {
                    let color = if row == 0 {
                        /* bottom row: flash button - always lit in a distinct colour */
                        fader.color_on
                    } else {
                        /* rows 1-7: fader steps (row 1 = step 1, row 7 = step 7) */
                        if level >= row { fader.color_on } else { fader.color_off }
                    };
                    pairs.push((grid_note(col, row), color));
                }
            }
        }
    } else {
        for btn in &page.buttons {
            pairs.push((btn.note, btn.color));
        }
    }

    /* CC 19 (colour picker modifier button) - rendered on all pages */
    let cc19_color = if state.color_picker_modifier_active { 13 } else { 1 }; /* yellow when active, dim red when inactive */
    pairs.push((19, cc19_color));

    if let Err(e) = out.send(&sysex_set_leds_bulk(&pairs)) {
        error!("LED SysEx: {}", e);
    }
}

fn render_main(state: &BridgeState, pairs: &mut Vec<(u8, u8)>) {
    let page_idx = state.current_page;
    let page     = &state.config.pages[page_idx];
    let fader    = match &page.fader_config {
        Some(f) => f,
        None => return,
    };

    for row in 0u8..8 {
        for col in 0u8..8 {
            let note = grid_note(col, row);

            if main_is_button_cell(col, row) {
                pairs.push((note, state.custom_button_color(page_idx, note)));
                continue;
            }

            if let Some(slot) = main_selector_a_slot(col, row) {
                let color = if slot == state.main_fader_a {
                    state.custom_colors.get(&(page_idx, note)).copied()
                        .unwrap_or(MAIN_SELECTOR_ACTIVE)
                } else {
                    state.custom_colors.get(&(page_idx, note)).copied()
                        .unwrap_or(MAIN_SELECTOR_INACTIVE)
                };
                pairs.push((note, color));
                continue;
            }

            if main_is_selector_b(col) {
                let slot  = row as usize;
                let color = if slot == state.main_fader_b {
                    state.custom_colors.get(&(page_idx, note)).copied()
                        .unwrap_or(MAIN_SELECTOR_ACTIVE)
                } else {
                    state.custom_colors.get(&(page_idx, note)).copied()
                        .unwrap_or(MAIN_SELECTOR_INACTIVE)
                };
                pairs.push((note, color));
                continue;
            }

            if main_is_fader_a(col, row) {
                let slot      = state.main_fader_a;
                let level     = state.main_level_a;
                let color_on  = state.get_fader_color(page_idx, slot as u8);
                let color_off = fader.color_off;
                let step      = row + 1;
                let color     = if level >= step { color_on } else { color_off };
                pairs.push((note, color));
                continue;
            }

            if main_is_flash_fader_b(col) {
                let slot               = state.main_fader_b;
                let level              = state.main_level_b;
                let color_on           = state.get_fader_color(page_idx, slot as u8);
                let is_flash_pressed   = state.flash_buttons_pressed[7];
                let color = if row == 0 {
                    if is_flash_pressed { color_on } else { 1 }
                } else if level >= row {
                    color_on
                } else {
                    fader.color_off
                };
                pairs.push((note, color));
            }
        }
    }
}

fn render_nudge_xy_grid(state: &BridgeState, pairs: &mut Vec<(u8, u8)>) {
    let page = &state.config.pages[state.current_page];
    let inactive = page.xy_config.as_ref().map(|xy| xy.color_inactive).unwrap_or(1);
    for row in 0u8..8 {
        for col in 0u8..8 {
            let color = match (col, row) {
                (0, 7) | (1, 7) | (6, 7) | (7, 7) => 1,
                (2, 7) | (3, 7) | (4, 7) | (5, 7) => 45,
                (0, 6) | (1, 6) | (6, 6) | (7, 6) => 1,
                (2, 6) | (3, 6) | (4, 6) | (5, 6) => 53,
                (0, 5) | (7, 5) => 45,
                (1, 5) | (6, 5) => 53,
                (2, 5) | (5, 5) => 1,
                (3, 5) | (4, 5) => 72,
                (0, 4) | (7, 4) => 45,
                (1, 4) | (6, 4) => 53,
                (2, 4) | (5, 4) => 72,
                (3, 4) | (4, 4) => 1,
                (0, 3) | (7, 3) => 45,
                (1, 3) | (6, 3) => 53,
                (2, 3) | (5, 3) => 72,
                (3, 3) | (4, 3) => 1,
                (0, 2) | (7, 2) => 45,
                (1, 2) | (6, 2) => 53,
                (2, 2) | (5, 2) => 1,
                (3, 2) | (4, 2) => 72,
                (0, 1) | (1, 1) | (6, 1) | (7, 1) => 1,
                (2, 1) | (3, 1) | (4, 1) | (5, 1) => 53,
                (0, 0) | (1, 0) | (6, 0) | (7, 0) => 1,
                (2, 0) | (3, 0) | (4, 0) | (5, 0) => 45,
                _ => inactive,
            };
            pairs.push((grid_note(col, row), color));
        }
    }
}

fn render_nudge_xy_profiles(state: &BridgeState, page_idx: usize, pairs: &mut Vec<(u8, u8)>) {
    let rs = &state.nudge_xy_states[&page_idx];
    for i in 0usize..8 {
        let color = if i <= 2 {
            PROFILE_COLOR_UNUSED
        } else {
            let prof_idx = 7 - i; /* 7->0 master, 6->1, 5->2, 4->3, 3->4 */
            if prof_idx >= NUDGE_XY_NUM_PROFILES {
                PROFILE_COLOR_UNUSED
            } else if prof_idx == rs.active_profile {
                PROFILE_COLOR_ACTIVE
            } else if prof_idx == 0 {
                PROFILE_COLOR_MASTER
            } else {
                PROFILE_COLOR_FIXTURE
            }
        };
        pairs.push((RIGHT_CC[i], color));
    }
}

fn render_rgbw(state: &BridgeState, page_idx: usize, pairs: &mut Vec<(u8, u8)>) {
    let rs      = &state.rgbw_states[&page_idx];
    let profile = &rs.profiles[rs.active_profile];

    /* grid: 4 channel faders, each 2 cols wide */
    for row in 0u8..8 {
        for col in 0u8..8 {
            let channel = (col / 2) as usize;
            let lit     = profile.levels[channel] >= row + 1;
            let color = match (channel, lit) {
                (CH_R, true)  => RGBW_COLOR_R_ON,
                (CH_R, false) => RGBW_COLOR_R_OFF,
                (CH_G, true)  => RGBW_COLOR_G_ON,
                (CH_G, false) => RGBW_COLOR_G_OFF,
                (CH_B, true)  => RGBW_COLOR_B_ON,
                (CH_B, false) => RGBW_COLOR_B_OFF,
                (CH_W, true)  => RGBW_COLOR_W_ON,
                _             => RGBW_COLOR_W_OFF,
            };
            pairs.push((grid_note(col, row), color));
        }
    }

    /* right-side buttons: profile select */
    for i in 0usize..8 {
        let color = if i == 0 {
            PROFILE_COLOR_UNUSED
        } else {
            let prof_idx = 7 - i; /* index 7 -> profile 0 (master), index 1 -> profile 6 */
            if prof_idx == rs.active_profile  { PROFILE_COLOR_ACTIVE  }
            else if prof_idx == 0             { PROFILE_COLOR_MASTER  }
            else                              { PROFILE_COLOR_FIXTURE }
        };
        pairs.push((RIGHT_CC[i], color));
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 * MIDI handling
 * ──────────────────────────────────────────────────────────────────────────── */

fn handle_midi(message: &[u8], state: &mut BridgeState, out: &mut MidiOutputConnection) {
    if message.len() < 3 { return; }
    let status   = message[0];
    let data1    = message[1];
    let data2    = message[2];
    let msg_type = status & 0xF0;

    match msg_type {
        /* CC: top-row page select + right-side RGBW profile buttons */
        0xB0 => {
            if data2 == 0 {
                /* CC released: CC 19 is the colour picker modifier */
                if data1 == 19 {
                    state.color_picker_modifier_active = false;
                }
                return;
            }

            /* CC 19 (bottom-right): toggle colour picker modifier */
            if data1 == 19 {
                state.color_picker_modifier_active = !state.color_picker_modifier_active;
                info!("Colour picker modifier: {}", if state.color_picker_modifier_active { "ON" } else { "OFF" });
                render_page(state, out);
                return;
            }

            if let Some(page_idx) = PAGE_SELECT_CC.iter().position(|&cc| cc == data1) {
                /* close any open colour picker */
                if matches!(state.overlay, Overlay::ColorPicker { .. } | Overlay::FlashFaderColorPicker { .. }) {
                    state.overlay = Overlay::None;
                    render_page(state, out);
                    return;
                }
                if page_idx < state.config.pages.len() {
                    info!("Page -> {} ({})", page_idx, state.config.pages[page_idx].name);
                    state.current_page = page_idx;
                    state.fader_levels = vec![0u8; 8];
                    state.main_fader_a = 0;
                    state.main_fader_b = 0;
                    state.main_level_a = 0;
                    state.main_level_b = 0;
                    state.xy_selected  = None;
                    state.press_times.clear();
                    /* nudge values persist across pages; remove this if you want to reset them */
                    if state.config.pages[page_idx].rgbw_mode {
                        state.ensure_rgbw_state(page_idx);
                    }
                    if state.config.pages[page_idx].nudge_xy_mode {
                        state.ensure_nudge_xy_state(page_idx);
                    }
                    render_page(state, out);
                }
                return;
            }

            let page_now = &state.config.pages[state.current_page];
            if let Some(right_idx) = RIGHT_CC.iter().position(|&cc| cc == data1) {
                if page_now.rgbw_mode {
                    handle_rgbw_profile(right_idx, state, out);
                } else if page_now.nudge_xy_mode && page_now.nudge_xy_config.is_some() {
                    handle_nudge_xy_profile(right_idx, state, out);
                }
            }
        }

        /* Note On */
        0x90 if data2 > 0 => {
            let note = data1;
            state.press_times.insert(note, Instant::now());
            
            /* ── Colour picker is open: detect colour selection ────────────────── */
            if let Overlay::ColorPicker { page_idx, note: target_note } = state.overlay {
                if let Some((col, row)) = note_to_grid(note) {
                    let chosen = picker_color_for_cell(col, row);
                    info!("Colour picker: note {} -> colour {}", target_note, chosen);
                    state.custom_colors.insert((page_idx, target_note), chosen);
                    state.save_colors();
                }
                state.overlay = Overlay::None;
                render_page(state, out);
                return;
            }

            if let Overlay::FlashFaderColorPicker { page_idx, col: target_col } = state.overlay {
                if let Some((col, row)) = note_to_grid(note) {
                    let chosen = picker_color_for_cell(col, row);
                    info!("Fader colour picker: col {} -> colour {}", target_col, chosen);
                    state.fader_colors.insert((page_idx, target_col), chosen);
                    state.save_fader_colors();
                }
                state.overlay = Overlay::None;
                render_page(state, out);
                return;
            }

            /* ignore presses while colour picker is open (shouldn't reach here) */
            if matches!(state.overlay, Overlay::ColorPicker { .. } | Overlay::FlashFaderColorPicker { .. }) {
                return;
            }

            /* ── Modifier active: open colour picker on next Note On ──────────── */
            if state.color_picker_modifier_active {
                let page = &state.config.pages[state.current_page];
                if page.custom_mode || page.main_mode {
                    if page.main_mode {
                        if let Some((col, row)) = note_to_grid(note) {
                            if main_is_fader_a(col, row) {
                                state.overlay = Overlay::FlashFaderColorPicker {
                                    page_idx: state.current_page,
                                    col: state.main_fader_a as u8,
                                };
                                render_page(state, out);
                                return;
                            }
                            if main_is_flash_fader_b(col) && row > 0 {
                                state.overlay = Overlay::FlashFaderColorPicker {
                                    page_idx: state.current_page,
                                    col: state.main_fader_b as u8,
                                };
                                render_page(state, out);
                                return;
                            }
                        }
                    }
                    state.overlay = Overlay::ColorPicker {
                        page_idx: state.current_page,
                        note,
                    };
                    render_page(state, out);
                    return;
                } else if page.flash_fader_mode {
                    if let Some((col, row)) = note_to_grid(note) {
                        if row > 0 {
                            /* fader row (not flash button) */
                            state.overlay = Overlay::FlashFaderColorPicker {
                                page_idx: state.current_page,
                                col,
                            };
                            render_page(state, out);
                            return;
                        }
                    }
                }
                return;
            }

            let page = &state.config.pages[state.current_page];
            if      page.rgbw_mode        { handle_rgbw_grid(note, state, out);    }
            else if page.nudge_xy_mode    { handle_nudge_xy(note, state, out);     }
            else if page.xy_mode          { handle_xy(note, state, out);           }
            else if page.fader_mode       { handle_fader(note, state, out);        }
            else if page.flash_fader_mode {
                if let Some((_col, row)) = note_to_grid(note) {
                    /* row 0 flash: Note On = release on Launchpad MK3 */
                    handle_flash_fader(note, row != 0, state, out);
                }
            }
            else if page.main_mode        { handle_main_grid(note, state, out);      }
            else if !page.custom_mode     { handle_button(note, state);            }
        }

        /* Note Off */
        0x80 | 0x90 => {
            if msg_type == 0x90 && data2 != 0 { return; }
            let note    = data1;
            let elapsed = state.press_times.remove(&note)
                .map(|t| t.elapsed()).unwrap_or(Duration::ZERO);

            let page = &state.config.pages[state.current_page];

            /* While colour picker is open: ignore Note Offs completely.
             * The picker will be closed when a colour is selected (in Note On above). */
            if matches!(state.overlay, Overlay::ColorPicker { .. } | Overlay::FlashFaderColorPicker { .. }) {
                return;
            }

            /* flash_fader: Note Off on bottom row = flash press on Launchpad MK3 */
            if page.flash_fader_mode {
                if let Some((_col, row)) = note_to_grid(note) {
                    if row == 0 {
                        handle_flash_fader(note, true, state, out);
                        return;
                    }
                }
            }

            if page.main_mode {
                if let Some((col, row)) = note_to_grid(note) {
                    if main_is_flash_fader_b(col) && row == 0 {
                        handle_main_flash_fader(note, true, state, out);
                        return;
                    }
                    if main_is_button_cell(col, row) {
                        handle_custom_button(note, state);
                    }
                }
                return;
            }

            if page.custom_mode {
                handle_custom_button(note, state);
            }
        }

        _ => {}
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 * RGBW handlers
 * ──────────────────────────────────────────────────────────────────────────── */

/*
 * Grid pad tapped on an RGBW page.
 * Master profile (0): sends new value to ALL fixture addresses.
 * Fixture profile (1-6): sends only to that fixture's address.
 */
fn handle_rgbw_grid(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };
    let channel    = (col / 2) as usize; /* 0=R, 1=G, 2=B, 3=W */
    let step       = row + 1;            /* 1-8 */

    let page_idx = state.current_page;
    state.ensure_rgbw_state(page_idx);

    let rs = state.rgbw_states.get_mut(&page_idx).unwrap();
    let current = rs.profiles[rs.active_profile].levels[channel];
    /* tap same step again -> turn off; otherwise set to that step */
    rs.profiles[rs.active_profile].levels[channel] = if current == step { 0 } else { step };
    let new_level      = rs.profiles[rs.active_profile].levels[channel];
    let active_profile = rs.active_profile;

    let value = new_level as f32 / 8.0; /* 0.0 - 1.0, no dimmer multiplier */

    let page = state.config.pages[page_idx].clone();
    if let Some(rgbw) = &page.rgbw_config {
        let addresses: Vec<String> = if active_profile == 0 {
            /* master -> all fixtures */
            match channel {
                CH_R => rgbw.r_addresses.clone(),
                CH_G => rgbw.g_addresses.clone(),
                CH_B => rgbw.b_addresses.clone(),
                _    => rgbw.w_addresses.clone(),
            }
        } else {
            /* fixture profile -> that fixture only */
            let fix  = active_profile - 1;
            let addr = match channel {
                CH_R => rgbw.r_addresses.get(fix),
                CH_G => rgbw.g_addresses.get(fix),
                CH_B => rgbw.b_addresses.get(fix),
                _    => rgbw.w_addresses.get(fix),
            };
            addr.cloned().into_iter().collect()
        };

        info!(
            "RGBW profile={} ch={} step={} value={:.2} -> {:?}",
            active_profile, ["R","G","B","W"][channel], new_level, value, addresses
        );
        for addr in &addresses {
            if let Err(e) = state.send_osc(addr, value) {
                error!("OSC: {}", e);
            }
        }
    }

    state.save_rgbw();
    render_page(state, out);
}

/*
 * Right-side button tapped on an RGBW page.
 * right_idx = position in RIGHT_CC array (0=bottom CC19, 7=top CC89).
 *
 * Switching TO a fixture profile: copy current master levels into that
 * fixture profile so the faders start from where master left off.
 * Switching TO master: just re-render, no OSC sent.
 */
fn handle_rgbw_profile(right_idx: usize, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    if right_idx == 0 { return; } /* CC19 unused */

    /* index 7 -> profile 0 (master), index 6 -> profile 1, ..., index 1 -> profile 6 */
    let new_profile = 7 - right_idx;

    let page_idx = state.current_page;
    state.ensure_rgbw_state(page_idx);

    let rs = state.rgbw_states.get_mut(&page_idx).unwrap();

    if new_profile == rs.active_profile { return; } /* already on this profile, nothing to do */

    if new_profile != 0 {
        /* switching to a fixture profile: seed it with master's current values */
        let master_levels = rs.profiles[0].levels;
        rs.profiles[new_profile].levels = master_levels;
        info!(
            "RGBW -> fixture profile {} (seeded from master: {:?})",
            new_profile, master_levels
        );
    } else {
        /* switching back to master: just show it, no OSC */
        info!("RGBW -> master profile (display only, no OSC)");
    }

    rs.active_profile = new_profile;

    state.save_rgbw();
    render_page(state, out);
}

/* ────────────────────────────────────────────────────────────────────────────
 * Other page handlers
 * ──────────────────────────────────────────────────────────────────────────── */

fn handle_button(note: u8, state: &mut BridgeState) {
    let page = &state.config.pages[state.current_page];
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Button: {} -> {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC: {}", e);
        }
    }
}

fn handle_custom_button(note: u8, state: &mut BridgeState) {
    let addr  = state.custom_osc_address(state.current_page, note);
    let value = state.config.pages[state.current_page].buttons.iter()
        .find(|b| b.note == note).map(|b| b.osc_value).unwrap_or(1.0);
    info!("Custom note {} -> {} = {}", note, addr, value);
    if let Err(e) = state.send_osc(&addr, value) {
        error!("OSC: {}", e);
    }
}

fn handle_xy(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };
    let pan  = col as f32 / 7.0;
    let tilt = (7 - row) as f32 / 7.0;
    state.xy_selected = Some((col, row));
    let page = state.config.pages[state.current_page].clone();
    if let Some(xy) = &page.xy_config {
        info!("XY col={} row={} pan={:.2} tilt={:.2}", col, row, pan, tilt);
        if let Err(e) = state.send_osc(&xy.pan_address.clone(), pan)   { error!("{}", e); }
        if let Err(e) = state.send_osc(&xy.tilt_address.clone(), tilt) { error!("{}", e); }
    }
    render_page(state, out);
}

fn nudge_xy_send_osc(
    state: &BridgeState,
    cfg: &NudgeXyConfig,
    profile: usize,
    pan: f32,
    tilt: f32,
) {
    let num = cfg.pan_addresses.len().min(cfg.tilt_addresses.len());
    if num == 0 { return; }

    let targets: Vec<usize> = if profile == 0 {
        (0..num).collect()
    } else {
        let fix = profile - 1;
        if fix < num { vec![fix] } else { return; }
    };

    for i in targets {
        if let Some(addr) = cfg.pan_addresses.get(i) {
            if let Err(e) = state.send_osc(addr, pan) { error!("{}", e); }
        }
        if let Some(addr) = cfg.tilt_addresses.get(i) {
            if let Err(e) = state.send_osc(addr, tilt) { error!("{}", e); }
        }
    }
}

fn handle_nudge_xy_profile(right_idx: usize, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    if right_idx <= 2 { return; } /* CC 39/29/19 unused */

    let new_profile = 7 - right_idx;
    if new_profile >= NUDGE_XY_NUM_PROFILES { return; }

    let page_idx = state.current_page;
    state.ensure_nudge_xy_state(page_idx);

    let rs = state.nudge_xy_states.get_mut(&page_idx).unwrap();
    if new_profile == rs.active_profile { return; }

    if new_profile != 0 {
        let master = rs.profiles[0];
        rs.profiles[new_profile] = master;
        info!(
            "Nudge XY -> head {} (seeded pan={:.3} tilt={:.3})",
            new_profile, master.pan, master.tilt
        );
    } else {
        info!("Nudge XY -> master profile (display only, no OSC)");
    }

    rs.active_profile = new_profile;
    state.save_nudge_xy();
    render_page(state, out);
}

fn handle_nudge_xy(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };
    if let Some((pan_delta, tilt_delta)) = nudge_from_position(col, row, state) {
        let page_idx = state.current_page;
        let page     = state.config.pages[page_idx].clone();

        if let Some(nxy) = &page.nudge_xy_config {
            state.ensure_nudge_xy_state(page_idx);
            let rs      = state.nudge_xy_states.get_mut(&page_idx).unwrap();
            let prof    = rs.active_profile;
            let current = rs.profiles[prof];
            let new_pan  = (current.pan + pan_delta).clamp(0.0, 1.0);
            let new_tilt = (current.tilt + tilt_delta).clamp(0.0, 1.0);
            rs.profiles[prof].pan  = new_pan;
            rs.profiles[prof].tilt = new_tilt;

            info!(
                "Nudge XY profile={} pan {:.3}->{:.3} tilt {:.3}->{:.3}",
                prof, current.pan, new_pan, current.tilt, new_tilt
            );

            nudge_xy_send_osc(state, nxy, prof, new_pan, new_tilt);
            state.save_nudge_xy();
        } else if let Some(xy) = &page.xy_config {
            /* Legacy single pan/tilt pair */
            state.ensure_nudge_xy_state(page_idx);
            let rs      = state.nudge_xy_states.get_mut(&page_idx).unwrap();
            let current = rs.profiles[0];
            let new_pan  = (current.pan + pan_delta).clamp(0.0, 1.0);
            let new_tilt = (current.tilt + tilt_delta).clamp(0.0, 1.0);
            rs.profiles[0].pan  = new_pan;
            rs.profiles[0].tilt = new_tilt;

            info!(
                "Nudge: pan {:.3}->{:.3} tilt {:.3}->{:.3}",
                current.pan, new_pan, current.tilt, new_tilt
            );

            if let Err(e) = state.send_osc(&xy.pan_address, new_pan)  { error!("{}", e); }
            if let Err(e) = state.send_osc(&xy.tilt_address, new_tilt) { error!("{}", e); }
            state.save_nudge_xy();
        }
    }
}

/*
 * Switch fader-A profile: column height stays put, old OSC channel gets 0.0,
 * new channel receives the current column brightness.
 */
fn main_switch_fader_a(
    state: &mut BridgeState,
    new_slot: usize,
    out: &mut MidiOutputConnection,
) {
    let old_slot = state.main_fader_a;
    if old_slot == new_slot {
        return;
    }

    let page = state.config.pages[state.current_page].clone();
    let fader = match &page.fader_config {
        Some(f) => f.clone(),
        None => return,
    };

    if let Some(addr) = fader.addresses.get(old_slot) {
        info!("Main: fader A off slot {} -> {}", old_slot + 1, addr);
        if let Err(e) = state.send_osc(addr, 0.0) {
            error!("{}", e);
        }
    }

    let level = state.main_level_a;
    if let Some(addr) = fader.addresses.get(new_slot) {
        let value = main_a_level_to_osc(level);
        info!(
            "Main: fader A slot {} -> {} = {:.2} (level {})",
            new_slot + 1, addr, value, level
        );
        if let Err(e) = state.send_osc(addr, value) {
            error!("{}", e);
        }
    }

    state.main_fader_a = new_slot;
}

fn main_switch_fader_b(
    state: &mut BridgeState,
    new_slot: usize,
    out: &mut MidiOutputConnection,
) {
    let old_slot = state.main_fader_b;
    if old_slot == new_slot {
        return;
    }

    let page = state.config.pages[state.current_page].clone();
    
    // Check for explicit flash addresses, otherwise fall back to the old array
    let addresses = if !page.fader_config.as_ref().unwrap().flash_addresses.is_empty() {
        page.fader_config.as_ref().unwrap().flash_addresses.clone()
    } else {
        info!("No flash addresses found, falling back to fader_config");
        page.fader_config.as_ref().map(|f| f.addresses.clone()).unwrap_or_default()
    };

    if let Some(addr) = addresses.get(old_slot) {
        info!("Main: fader B off slot {} -> {}", old_slot + 1, addr);
        if let Err(e) = state.send_osc(addr, 0.0) {
            error!("{}", e);
        }
    }

    /* Column level (main_level_b) stays put; OSC only sent on flash press */
    info!("Main: fader B -> slot {} (level {} retained)", new_slot + 1, state.main_level_b);
    state.main_fader_b = new_slot;
}

fn handle_main_grid(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };

    if main_is_button_cell(col, row) {
        return; /* OSC on Note Off */
    }

    let page_idx = state.current_page;
    let page     = state.config.pages[page_idx].clone();

    if let Some(slot) = main_selector_a_slot(col, row) {
        main_switch_fader_a(state, slot, out);
        render_page(state, out);
        return;
    }

    if main_is_selector_b(col) {
        let slot = row as usize;
        main_switch_fader_b(state, slot, out);
        render_page(state, out);
        return;
    }

    if main_is_flash_fader_b(col) {
        /* row 0 flash: Note On = release on Launchpad MK3 */
        handle_main_flash_fader(note, row != 0, state, out);
        return;
    }

    if main_is_fader_a(col, row) {
        let slot      = state.main_fader_a;
        let step      = row + 1;
        let current   = state.main_level_a;
        let new_level = if current == step { 0 } else { step };
        state.main_level_a = new_level;
        let value = main_a_level_to_osc(new_level);
        if let Some(fader) = &page.fader_config {
            if let Some(addr) = fader.addresses.get(slot) {
                info!("Main fader A {} level={} -> {} = {:.2}", slot + 1, new_level, addr, value);
                if let Err(e) = state.send_osc(addr, value) { error!("{}", e); }
            }
        }
        render_page(state, out);
    }
}

/*
 * Main page col 7: flash fader for the col-6 selection.
 * Row 0 = momentary flash; rows 1–7 = 7-step level (OSC only on flash press/release).
 */
fn handle_main_flash_fader(
    note: u8,
    pressed: bool,
    state: &mut BridgeState,
    out: &mut MidiOutputConnection,
) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };
    if !main_is_flash_fader_b(col) { return; }

    let page = state.config.pages[state.current_page].clone();
    let slot = state.main_fader_b;

    // Direct Main B to flash_addresses if present, otherwise fall back to old behavior
    let addresses = if !page.fader_config.as_ref().unwrap().flash_addresses.is_empty() {
        page.fader_config.as_ref().unwrap().flash_addresses.clone()
    } else {
        info!("No flash addresses found, falling back to fader_config, {:?}", page.fader_config);
        page.fader_config.as_ref().map(|f| f.addresses.clone()).unwrap_or_default()
    };

    if row == 0 {
        state.flash_buttons_pressed[7] = pressed;
        let osc_value = if pressed {
            main_b_level_to_osc(state.main_level_b)
        } else {
            0.0
        };
        if let Some(addr) = addresses.get(slot) {
            info!("Main flash B {} -> {} = {:.2}", if pressed { "ON" } else { "OFF" }, addr, osc_value);
            if let Err(e) = state.send_osc(addr, osc_value) { error!("{}", e); }
        }
        render_page(state, out);
    } else if pressed {
        let fader_step = row;
        let current    = state.main_level_b;
        let new_level  = if current == fader_step { 0 } else { fader_step };
        state.main_level_b = new_level;
        info!("Main flash B level={} (awaiting flash, slot {})", new_level, slot + 1);
        render_page(state, out);
    }
}

fn handle_fader(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };
    let step      = row + 1;
    let current   = state.fader_levels.get(col as usize).copied().unwrap_or(0);
    let new_level = if current == step { 0 } else { step };
    state.fader_levels[col as usize] = new_level;
    /* step 1 = 0.0, step 8 = 1.0  ->  value = (step - 1) / 7.0
     * level 0 (off/toggled off) = 0.0 */
    let value = if new_level == 0 { 0.0 } else { (new_level - 1) as f32 / 7.0 };
    let page  = state.config.pages[state.current_page].clone();
    if let Some(fader) = &page.fader_config {
        if let Some(addr) = fader.addresses.get(col as usize) {
            info!("Fader col={} level={} -> {} = {:.2}", col, new_level, addr, value);
            if let Err(e) = state.send_osc(&addr.clone(), value) { error!("{}", e); }
        }
    }
    render_page(state, out);
}

/*
 * flash_fader_mode grid handler.
 * Row 0 (bottom) = flash button for this column.
 * Rows 1-7       = 7-step fader (step 1 = 0.0, step 7 = 1.0).
 * Flash button pressed: send fader's current value.
 * Flash button released: send 0.0.
 * Both use the same fader_config address.
 * Long-press (>=500ms) on a fader row: open colour picker for that fader.
 */
fn handle_flash_fader(
    note: u8,
    pressed: bool,
    state: &mut BridgeState,
    out: &mut MidiOutputConnection,
) {
    let (col, row) = match note_to_grid(note) { Some(v) => v, None => return };

    let page = state.config.pages[state.current_page].clone();
    let fader_cfg = match &page.fader_config { Some(f) => f.clone(), None => return };

    if row == 0 {
        /* bottom row: flash button - sends current fader value when pressed, 0.0 when released */
        /* track press state for rendering */
        if let Some(slot) = state.flash_buttons_pressed.get_mut(col as usize) {
            *slot = pressed;
        }

        let level = state.fader_levels.get(col as usize).copied().unwrap_or(0);
        let osc_value = if pressed {
            if level == 0 { 0.0 } else { (level - 1) as f32 / 6.0 }
        } else {
            0.0
        };
        if let Some(addr) = fader_cfg.addresses.get(col as usize) {
            info!("Flash col={} {} -> {} = {:.2}", col,
                if pressed { "ON" } else { "OFF" }, addr, osc_value);
            if let Err(e) = state.send_osc(addr, osc_value) { error!("{}", e); }
        }
        /* re-render so flash button colour updates */
        render_page(state, out);
    } else if pressed {
        /* fader rows: only act on press - set or toggle fader level, NO OSC sent */
        let fader_step = row; /* row 1 = step 1 ... row 7 = step 7 */
        let current    = state.fader_levels.get(col as usize).copied().unwrap_or(0);
        let new_level  = if current == fader_step { 0 } else { fader_step };
        state.fader_levels[col as usize] = new_level;
        info!("FlashFader col={} level={} (no OSC, awaiting flash)", col, new_level);
        render_page(state, out);
    }
}


/* ────────────────────────────────────────────────────────────────────────────
 * Main
 * ──────────────────────────────────────────────────────────────────────────── */

fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info"),
    ).init();

    let config_path = std::env::args().nth(1).unwrap_or_else(|| "config.toml".to_string());
    let config_str  = std::fs::read_to_string(&config_path)
        .with_context(|| format!("Cannot read config: {}", config_path))?;
    let config: Config = toml::from_str(&config_str).context("Parse config.toml")?;

    let base        = std::path::Path::new(&config_path).parent()
                        .unwrap_or_else(|| std::path::Path::new("."));
    let colors_path = base.join("custom_colors.json").to_string_lossy().into_owned();
    let rgbw_path     = base.join("rgbw_state.json").to_string_lossy().into_owned();
    let nudge_xy_path = base.join("nudge_xy_state.json").to_string_lossy().into_owned();

    info!("Config: {} pages | OSC {}:{}", config.pages.len(), config.osc.host, config.osc.port);

    let midi_in  = MidiInput::new("launchpad-qlc-in")?;
    let midi_out = MidiOutput::new("launchpad-qlc-out")?;

    let device_name = config.midi.device_name.to_lowercase();
    let in_port = midi_in.ports().into_iter()
        .find(|p| midi_in.port_name(p)
            .map(|n| n.to_lowercase().contains(&device_name)).unwrap_or(false))
        .with_context(|| format!(
            "No MIDI input matching '{}'. Available:\n  {}",
            config.midi.device_name,
            midi_in.ports().iter()
                .filter_map(|p| midi_in.port_name(p).ok())
                .collect::<Vec<_>>().join("\n  ")
        ))?;

    let device_out_name = config.midi.device_out_name.to_lowercase();
    let out_port = midi_out.ports().into_iter()
        .find(|p| midi_out.port_name(p)
            .map(|n| n.to_lowercase().contains(&device_out_name)).unwrap_or(false))
        .with_context(|| format!("No MIDI output matching '{}'", config.midi.device_out_name))?;

    info!("MIDI in:  {}", midi_in.port_name(&in_port).unwrap_or_default());
    info!("MIDI out: {}", midi_out.port_name(&out_port).unwrap_or_default());

    let mut midi_out_conn = midi_out.connect(&out_port, "launchpad-qlc-out")
        .map_err(|e| anyhow::anyhow!("MIDI out: {}", e))?;

    midi_out_conn.send(SYSEX_PROGRAMMER_MODE)?;
    std::thread::sleep(Duration::from_millis(50));
    midi_out_conn.send(&sysex_clear_all())?;
    std::thread::sleep(Duration::from_millis(20));

    let mut state = BridgeState::new(config, colors_path, rgbw_path, nudge_xy_path)?;

    /* pre-init persisted page state */
    let rgbw_indices: Vec<usize> = state.config.pages.iter().enumerate()
        .filter(|(_, p)| p.rgbw_mode).map(|(i, _)| i).collect();
    for idx in rgbw_indices { state.ensure_rgbw_state(idx); }
    let nudge_indices: Vec<usize> = state.config.pages.iter().enumerate()
        .filter(|(_, p)| p.nudge_xy_mode).map(|(i, _)| i).collect();
    for idx in nudge_indices { state.ensure_nudge_xy_state(idx); }

    let state = Arc::new(Mutex::new(state));

    {
        let mut s = state.lock().unwrap();
        render_page(&s, &mut midi_out_conn);
    }

    let midi_out_conn    = Arc::new(Mutex::new(midi_out_conn));
    let midi_out_conn_cb = Arc::clone(&midi_out_conn);
    let state_cb         = Arc::clone(&state);

    let _midi_in_conn = midi_in.connect(
        &in_port, "launchpad-qlc-in",
        move |_stamp, message, _| {
            let mut s   = state_cb.lock().unwrap();
            let mut out = midi_out_conn_cb.lock().unwrap();
            handle_midi(message, &mut s, &mut out);
        },
        (),
    ).map_err(|e| anyhow::anyhow!("MIDI in: {}", e))?;

    info!("Bridge running. Ctrl+C to quit.");
    info!("RGBW / Nudge XY: master profile -> all heads | fixture profiles seeded from master");

    loop { std::thread::sleep(Duration::from_secs(1)); }
}
