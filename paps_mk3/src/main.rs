//! launchpad-qlc v2: Novation Launchpad Mini MK3 → QLC+ OSC bridge
//!
//! ┌────────────────────────────────────────────────────────────────────────┐
//! │  PAGE LAYOUT  (top-row CC 91–98 switch pages)                         │
//! │                                                                        │
//! │  [91]RGB  [92]FLASH [93]MOVE [94]BEAT [95]XY   [96]FADE [97]── [98]──│
//! │                                                                        │
//! │  Right-column notes 19,29,39,49,59,69,79,89 are used as:             │
//! │    89 = on XY page: toggle FINE/COARSE adjustment mode                │
//! │    79 = on XY page: increase step size (fine only)                    │
//! │    69 = on XY page: decrease step size                                │
//! │    59 = on BEAT page: TAP TEMPO                                       │
//! │    49 = on BEAT page: next color palette                              │
//! │    39 = on BEAT page: previous color palette                          │
//! └────────────────────────────────────────────────────────────────────────┘
//!
//! Launchpad Mini MK3 grid note layout:
//!   81 82 83 84 85 86 87 88  │ 89  ← row 7 (top)
//!   71 72 73 74 75 76 77 78  │ 79
//!   61 62 63 64 65 66 67 68  │ 69
//!   51 52 53 54 55 56 57 58  │ 59
//!   41 42 43 44 45 46 47 48  │ 49
//!   31 32 33 34 35 36 37 38  │ 39
//!   21 22 23 24 25 26 27 28  │ 29
//!   11 12 13 14 15 16 17 18  │ 19  ← row 0 (bottom)
//!
//! QLC+ setup:
//!   Inputs/Outputs → OSC → enable input on port 7700
//!   All OSC values are f32 in [0.0, 1.0].

use std::{
    collections::HashMap,
    net::UdpSocket,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use log::{debug, error, info, warn};
use midir::{MidiInput, MidiOutput, MidiOutputConnection};
use rosc::{OscMessage, OscPacket, OscType, encoder};
use serde::Deserialize;

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// CC numbers for the top-row page selectors (left → right: 91–98).
const PAGE_SELECT_CC: [u8; 8] = [91, 92, 93, 94, 95, 96, 97, 98];

/// Note numbers for the right column (bottom → top: 19, 29, … 89).
const RIGHT_COL_NOTES: [u8; 8] = [19, 29, 39, 49, 59, 69, 79, 89];

/// SysEx: switch Launchpad Mini MK3 into Programmer mode.
const SYSEX_PROGRAMMER_MODE: &[u8] = &[
    0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x0E, 0x01, 0xF7,
];

/// Long-press threshold in milliseconds (used for colour picker).
const LONG_PRESS_MS: u64 = 500;

/// Default XY step in grid cells when in COARSE mode.
const XY_STEP_COARSE: u8 = 2;
/// Default XY step when in FINE mode (1 cell at a time).
const XY_STEP_FINE: u8 = 1;

// ─────────────────────────────────────────────────────────────────────────────
// Config structures
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Deserialize, Clone)]
struct Config {
    osc:   OscConfig,
    midi:  MidiConfig,
    pages: Vec<PageConfig>,
}

#[derive(Debug, Deserialize, Clone)]
struct OscConfig {
    host: String,
    port: u16,
}

#[derive(Debug, Deserialize, Clone)]
struct MidiConfig {
    device_name:     String,
    device_out_name: String,
}

#[derive(Debug, Deserialize, Clone)]
struct PageConfig {
    name:           String,
    selector_color: u8,
    #[serde(default)]
    buttons: Vec<ButtonConfig>,
    // ── mode flags ─────────────────────────────────────────────────────────
    #[serde(default)] pub rgb_mode:   bool,
    #[serde(default)] pub flash_mode: bool,
    #[serde(default)] pub move_mode:  bool,
    #[serde(default)] pub beat_mode:  bool,
    #[serde(default)] pub xy_mode:    bool,
    #[serde(default)] pub fader_mode: bool,
    #[serde(default)] pub custom_mode: bool,
    // ── mode configs ───────────────────────────────────────────────────────
    xy_config:    Option<XyConfig>,
    fader_config: Option<FaderConfig>,
    beat_config:  Option<BeatConfig>,
}

/// A single button mapping for normal / rgb / flash / move pages.
#[derive(Debug, Deserialize, Clone)]
struct ButtonConfig {
    /// Launchpad note number.
    note:        u8,
    /// Human-readable label (used for logging).
    #[serde(default)]
    label:       String,
    /// OSC address to send to QLC+.
    osc_address: String,
    /// OSC value sent on press (default 1.0).
    #[serde(default = "default_osc_value")]
    osc_value:   f32,
    /// OSC value sent on release for flash buttons (default 0.0).
    #[serde(default)]
    osc_value_off: f32,
    /// LED colour when the button is in its ON state.
    #[serde(default = "default_color_on")]
    color_on:    u8,
    /// LED colour when the button is in its OFF state.
    #[serde(default)]
    color_off:   u8,
}

fn default_osc_value() -> f32  { 1.0 }
fn default_color_on()  -> u8   { 72  } // white

/// XY (pan/tilt) page configuration.
#[derive(Debug, Deserialize, Clone)]
struct XyConfig {
    /// OSC address for pan  (0.0 = left,  1.0 = right).
    pan_address:    String,
    /// OSC address for tilt (0.0 = front, 1.0 = back).
    tilt_address:   String,
    color_active:   u8,
    color_inactive: u8,
}

/// Fader page — columns 0..3 = halogens, columns 4..7 = RGBs.
#[derive(Debug, Deserialize, Clone)]
struct FaderConfig {
    /// 8 OSC addresses, one per column.
    /// Columns 0-3 → halogen dimmers, columns 4-7 → RGB master dimmers.
    addresses:   Vec<String>,
    /// Colour used for lit (active) steps in the halogen columns (0-3).
    color_halogen_on:  u8,
    color_halogen_off: u8,
    /// Colour used for lit (active) steps in the RGB columns (4-7).
    color_rgb_on:  u8,
    color_rgb_off: u8,
}

/// Beat-sync chaser configuration.
#[derive(Debug, Deserialize, Clone)]
struct BeatConfig {
    /// OSC addresses of the RGB fixtures to cycle through.
    rgb_addresses: Vec<String>,
    /// Starting BPM (user can tap-tempo to override).
    #[serde(default = "default_bpm")]
    default_bpm: f32,
    /// Colour palettes: each inner Vec is a sequence of QLC+ palette colours.
    /// The beat thread cycles through the active palette's entries.
    palettes: Vec<Vec<u8>>,
}
fn default_bpm() -> f32 { 120.0 }

// ─────────────────────────────────────────────────────────────────────────────
// SysEx helpers
// ─────────────────────────────────────────────────────────────────────────────

fn sysex_clear_all() -> Vec<u8> {
    // Set all LEDs to colour 0 (off) via bulk LED SysEx
    let mut pairs: Vec<(u8, u8)> = Vec::new();
    // Grid notes 11–89 + right-col + top-row
    for row in 0u8..8 {
        for col in 0u8..9 {
            let note = (row + 1) * 10 + col + 1;
            pairs.push((note, 0));
        }
    }
    for &cc in &PAGE_SELECT_CC {
        pairs.push((cc, 0));
    }
    sysex_set_leds_bulk(&pairs)
}

/// Build a SysEx bulk LED set message for the Launchpad Mini MK3.
/// Each pair is (note_or_cc, palette_colour_0_127).
fn sysex_set_leds_bulk(pairs: &[(u8, u8)]) -> Vec<u8> {
    // SysEx format: F0 00 20 29 02 0D 03  [type note colour]... F7
    // type 0 = note, type 1 = cc
    let mut buf = vec![0xF0u8, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x03];
    for &(note, color) in pairs {
        // All grid pads and right-col use note (type 0).
        // Top-row buttons are CCs (type 1).
        let kind = if PAGE_SELECT_CC.contains(&note) { 1u8 } else { 0u8 };
        buf.push(kind);
        buf.push(note);
        buf.push(color & 0x7F);
    }
    buf.push(0xF7);
    buf
}

fn set_single_led(note: u8, color: u8, out: &mut MidiOutputConnection) {
    let pairs = vec![(note, color)];
    let sysex = sysex_set_leds_bulk(&pairs);
    if let Err(e) = out.send(&sysex) {
        error!("LED single set error: {}", e);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid coordinate helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Convert a grid note (11–88) to (col 0-7, row 0-7) where row 0 = bottom.
fn note_to_grid(note: u8) -> Option<(u8, u8)> {
    if note < 11 || note > 88 { return None; }
    let row = (note / 10).saturating_sub(1);
    let col = (note % 10).saturating_sub(1);
    if col > 7 || row > 7 { return None; }
    Some((col, row))
}

/// Convert (col, row) back to note number.
fn grid_note(col: u8, row: u8) -> u8 {
    (row + 1) * 10 + (col + 1)
}

// ─────────────────────────────────────────────────────────────────────────────
// Colour-picker helpers (used in custom_mode pages)
// ─────────────────────────────────────────────────────────────────────────────

fn picker_color_for_cell(col: u8, row: u8) -> u8 {
    let palette_idx = (7 - row) * 8 + col; // 0..63
    palette_idx + 1
}

fn picker_cell_for_color(color: u8) -> Option<(u8, u8)> {
    if color == 0 || color > 64 { return None; }
    let idx = color - 1;
    let row = 7 - (idx / 8);
    let col = idx % 8;
    Some((col, row))
}

// ─────────────────────────────────────────────────────────────────────────────
// Beat-sync shared state (lives in Arc<Mutex<BeatState>>)
// ─────────────────────────────────────────────────────────────────────────────

struct BeatState {
    /// Beats per minute.
    bpm: f32,
    /// Index into the active palette.
    step: usize,
    /// Which palette is active.
    palette_idx: usize,
    /// Tap-tempo tap timestamps.
    taps: Vec<Instant>,
    /// Whether the beat thread should be running.
    running: bool,
    /// OSC addresses of the RGB fixtures.
    rgb_addresses: Vec<String>,
    /// Available palettes.
    palettes: Vec<Vec<u8>>,
}

impl BeatState {
    fn new(cfg: &BeatConfig) -> Self {
        Self {
            bpm:          cfg.default_bpm,
            step:         0,
            palette_idx:  0,
            taps:         Vec::new(),
            running:      true,
            rgb_addresses: cfg.rgb_addresses.clone(),
            palettes:      cfg.palettes.clone(),
        }
    }

    /// Register a tap and recalculate BPM from the last 4 taps.
    fn tap(&mut self) {
        let now = Instant::now();
        // Discard taps older than 3 seconds
        self.taps.retain(|t| now.duration_since(*t).as_secs_f32() < 3.0);
        self.taps.push(now);
        if self.taps.len() >= 2 {
            let intervals: Vec<f32> = self.taps
                .windows(2)
                .map(|w| w[1].duration_since(w[0]).as_secs_f32())
                .collect();
            let avg = intervals.iter().sum::<f32>() / intervals.len() as f32;
            self.bpm = 60.0 / avg;
            info!("Tap tempo → {:.1} BPM", self.bpm);
        }
    }

    fn active_palette(&self) -> &[u8] {
        if self.palettes.is_empty() { return &[]; }
        &self.palettes[self.palette_idx % self.palettes.len()]
    }

    fn next_step_color(&mut self) -> u8 {
        let pal = self.active_palette();
        if pal.is_empty() { return 0; }
        let c = pal[self.step % pal.len()];
        self.step = (self.step + 1) % pal.len();
        c
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// XY fine/coarse mode
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
enum XyAdjustMode {
    Coarse,
    Fine,
}

// ─────────────────────────────────────────────────────────────────────────────
// Overlay (colour picker)
// ─────────────────────────────────────────────────────────────────────────────

enum Overlay {
    None,
    ColorPicker { page_idx: usize, note: u8 },
}

// ─────────────────────────────────────────────────────────────────────────────
// Main bridge state
// ─────────────────────────────────────────────────────────────────────────────

struct BridgeState {
    config:       Config,
    current_page: usize,
    osc_socket:   UdpSocket,
    osc_target:   String,

    // ── Fader page ──────────────────────────────────────────────────────────
    /// Per-column fader level (0 = off, 1–8 = step from bottom).
    fader_levels: Vec<u8>,

    // ── XY page ─────────────────────────────────────────────────────────────
    xy_selected:   Option<(u8, u8)>,
    xy_adjust_mode: XyAdjustMode,
    /// Fine-mode step size (1–4 cells), toggled with right-col buttons.
    xy_fine_step:  u8,

    // ── RGB toggles ─────────────────────────────────────────────────────────
    /// note → bool (true = ON).
    rgb_states: HashMap<u8, bool>,

    // ── Beat sync ───────────────────────────────────────────────────────────
    beat_state: Option<Arc<Mutex<BeatState>>>,

    // ── Colour picker overlay ───────────────────────────────────────────────
    overlay: Overlay,

    // ── Custom page runtime colours ─────────────────────────────────────────
    custom_colors: HashMap<(usize, u8), u8>,

    // ── Long-press tracking ─────────────────────────────────────────────────
    press_times: HashMap<u8, Instant>,
}

impl BridgeState {
    fn new(config: Config) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("bind UDP")?;
        let target = format!("{}:{}", config.osc.host, config.osc.port);
        Ok(Self {
            fader_levels:   vec![0u8; 8],
            xy_selected:    None,
            xy_adjust_mode: XyAdjustMode::Coarse,
            xy_fine_step:   XY_STEP_FINE,
            rgb_states:     HashMap::new(),
            beat_state:     None,
            overlay:        Overlay::None,
            custom_colors:  HashMap::new(),
            press_times:    HashMap::new(),
            current_page:   0,
            osc_socket:     socket,
            osc_target:     target,
            config,
        })
    }

    fn send_osc(&self, address: &str, value: f32) -> Result<()> {
        let msg = OscPacket::Message(OscMessage {
            addr: address.to_string(),
            args: vec![OscType::Float(value)],
        });
        let bytes = encoder::encode(&msg).context("encode OSC")?;
        self.osc_socket
            .send_to(&bytes, &self.osc_target)
            .context("send OSC")?;
        debug!("OSC → {} = {}", address, value);
        Ok(())
    }

    fn custom_button_color(&self, page_idx: usize, note: u8) -> u8 {
        if let Some(&c) = self.custom_colors.get(&(page_idx, note)) {
            return c;
        }
        self.config.pages[page_idx]
            .buttons
            .iter()
            .find(|b| b.note == note)
            .map(|b| b.color_on)
            .unwrap_or(0)
    }

    fn custom_osc_address(&self, page_idx: usize, note: u8) -> String {
        self.config.pages[page_idx]
            .buttons
            .iter()
            .find(|b| b.note == note)
            .map(|b| b.osc_address.clone())
            .unwrap_or_else(|| {
                let idx = note_to_grid(note)
                    .map(|(c, r)| r * 8 + c + 1)
                    .unwrap_or(0);
                format!("/qlc/custom/{}", idx)
            })
    }

    /// Initialise (or re-initialise) the beat-sync state for the current page.
    fn init_beat(&mut self) {
        let page = &self.config.pages[self.current_page];
        if let Some(cfg) = &page.beat_config {
            let bs = Arc::new(Mutex::new(BeatState::new(cfg)));
            self.beat_state = Some(bs);
        }
    }

    /// Stop the beat thread by setting running = false.
    fn stop_beat(&mut self) {
        if let Some(bs) = &self.beat_state {
            bs.lock().unwrap().running = false;
        }
        self.beat_state = None;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Beat-sync background thread
// ─────────────────────────────────────────────────────────────────────────────

/// Spawns the beat-sync chaser thread.
/// It reads BPM from `beat_state`, sends an OSC colour command every beat,
/// cycling through the active palette.  Colours are sent as full-on (1.0)
/// to the current fixture and off (0.0) to all others → one-at-a-time chase.
fn spawn_beat_thread(
    beat_state: Arc<Mutex<BeatState>>,
    osc_socket: UdpSocket,
    osc_target: String,
) {
    std::thread::spawn(move || {
        info!("Beat thread started");
        loop {
            // Read current BPM and check if still running
            let (bpm, running, addresses, color) = {
                let mut bs = beat_state.lock().unwrap();
                if !bs.running { break; }
                let bpm  = bs.bpm;
                let addrs = bs.rgb_addresses.clone();
                let color = bs.next_step_color();
                (bpm, bs.running, addrs, color)
            };
            if !running { break; }

            // Send OSC: current address full on, rest off
            // We encode colour as a float 0.0–1.0 mapped from palette index.
            // QLC+ needs a real dimmer value; colour choice is done via a
            // separate scene per palette entry.  We send the palette colour
            // index normalised: color / 127.0.
            let value = color as f32 / 127.0;
            let n = addresses.len();
            for (i, addr) in addresses.iter().enumerate() {
                let v = if i == 0 { value } else { 0.0 };
                // Build OSC manually to avoid locking state again
                let msg = OscPacket::Message(OscMessage {
                    addr: addr.clone(),
                    args: vec![OscType::Float(v)],
                });
                if let Ok(bytes) = encoder::encode(&msg) {
                    let _ = osc_socket.send_to(&bytes, &osc_target);
                }
            }
            drop(n); // suppress warning

            // Sleep for one beat duration
            let beat_ms = (60_000.0 / bpm) as u64;
            std::thread::sleep(Duration::from_millis(beat_ms));
        }
        info!("Beat thread stopped");
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// LED rendering
// ─────────────────────────────────────────────────────────────────────────────

fn render_page(state: &BridgeState, out: &mut MidiOutputConnection) {
    // Handle colour-picker overlay first
    if let Overlay::ColorPicker { .. } = &state.overlay {
        render_color_picker(out);
        return;
    }

    let mut pairs: Vec<(u8, u8)> = Vec::new();

    // ── Top-row selector LEDs ────────────────────────────────────────────────
    for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
        let color = if i < state.config.pages.len() {
            if i == state.current_page { 72 } // white = active
            else { state.config.pages[i].selector_color }
        } else { 0 };
        pairs.push((cc, color));
    }

    let page = &state.config.pages[state.current_page];

    // ── Right column context indicators ─────────────────────────────────────
    if page.xy_mode {
        // 89 = mode indicator: green=coarse, yellow=fine
        let mode_color = match state.xy_adjust_mode {
            XyAdjustMode::Coarse => 21,  // green
            XyAdjustMode::Fine   => 13,  // yellow
        };
        pairs.push((89, mode_color));
        // 79 = increase fine step (blue when fine mode)
        let step_color = if state.xy_adjust_mode == XyAdjustMode::Fine { 45 } else { 0 };
        pairs.push((79, step_color));
        // 69 = decrease fine step
        pairs.push((69, step_color));
        // 59/49/39/29/19 = off
        for &n in &[59u8, 49, 39, 29, 19] { pairs.push((n, 0)); }
    } else if page.beat_mode {
        // 59 = tap tempo (red)
        pairs.push((59, 5));
        // 49 = next palette (cyan)
        pairs.push((49, 57));
        // 39 = prev palette (purple)
        pairs.push((39, 53));
        for &n in &[89u8, 79, 69, 29, 19] { pairs.push((n, 0)); }
    } else {
        // All right-column off on other pages
        for &n in &RIGHT_COL_NOTES { pairs.push((n, 0)); }
    }

    // ── Main 8×8 grid ────────────────────────────────────────────────────────
    if page.custom_mode {
        for row in 0u8..8 {
            for col in 0u8..8 {
                let note  = grid_note(col, row);
                let color = state.custom_button_color(state.current_page, note);
                pairs.push((note, color));
            }
        }
    } else if page.rgb_mode {
        for btn in &page.buttons {
            let on    = *state.rgb_states.get(&btn.note).unwrap_or(&false);
            let color = if on { btn.color_on } else { btn.color_off };
            pairs.push((btn.note, color));
        }
    } else if page.flash_mode || page.move_mode {
        // Flash and Move pages: buttons shown with their ON colour.
        // Flash colour is sent full-on; actual flash dimming via QLC+ scene.
        for btn in &page.buttons {
            pairs.push((btn.note, btn.color_on));
        }
    } else if page.beat_mode {
        // Beat page: show rhythm grid (static visual pattern).
        // Columns = fixture index, rows = beat subdivisions.
        // Light the bottom row as a "ready" indicator.
        for col in 0u8..8 {
            for row in 0u8..8 {
                let note  = grid_note(col, row);
                let color = if row == 0 { 96 } else { 0 }; // orange bottom row
                pairs.push((note, color));
            }
        }
    } else if page.xy_mode {
        if let Some(xy) = &page.xy_config {
            for row in 0u8..8 {
                for col in 0u8..8 {
                    let note  = grid_note(col, row);
                    let color = if state.xy_selected == Some((col, row)) {
                        xy.color_active
                    } else {
                        xy.color_inactive
                    };
                    pairs.push((note, color));
                }
            }
        }
    } else if page.fader_mode {
        if let Some(fader) = &page.fader_config {
            for col in 0u8..8 {
                let level = *state.fader_levels.get(col as usize).unwrap_or(&0);
                // Columns 0-3 = halogens, 4-7 = RGBs
                let (c_on, c_off) = if col < 4 {
                    (fader.color_halogen_on, fader.color_halogen_off)
                } else {
                    (fader.color_rgb_on, fader.color_rgb_off)
                };
                for row in 0u8..8 {
                    let note  = grid_note(col, row);
                    let step  = row + 1;
                    let color = if level >= step { c_on } else { c_off };
                    pairs.push((note, color));
                }
            }
        }
    } else {
        // Generic normal button page
        for btn in &page.buttons {
            pairs.push((btn.note, btn.color_on));
        }
    }

    let sysex = sysex_set_leds_bulk(&pairs);
    if let Err(e) = out.send(&sysex) {
        error!("Failed to send LED SysEx: {}", e);
    }
}

fn render_color_picker(out: &mut MidiOutputConnection) {
    let mut pairs: Vec<(u8, u8)> = Vec::new();
    for row in 0u8..8 {
        for col in 0u8..8 {
            let note  = grid_note(col, row);
            let color = picker_color_for_cell(col, row);
            pairs.push((note, color));
        }
    }
    let sysex = sysex_set_leds_bulk(&pairs);
    if let Err(e) = out.send(&sysex) {
        error!("Failed to send colour picker SysEx: {}", e);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MIDI event dispatcher
// ─────────────────────────────────────────────────────────────────────────────

fn handle_midi(
    message: &[u8],
    state:   &mut BridgeState,
    out:     &mut MidiOutputConnection,
) {
    if message.len() < 3 { return; }

    let status   = message[0];
    let data1    = message[1];
    let data2    = message[2];
    let msg_type = status & 0xF0;

    match msg_type {
        // ── CC messages: top-row page selectors ─────────────────────────────
        0xB0 => {
            if data2 > 0 {
                if let Some(page_idx) = PAGE_SELECT_CC.iter().position(|&cc| cc == data1) {
                    // Cancel colour picker if open
                    if matches!(state.overlay, Overlay::ColorPicker { .. }) {
                        state.overlay = Overlay::None;
                        render_page(state, out);
                        return;
                    }

                    if page_idx < state.config.pages.len() {
                        // Stop any running beat thread
                        state.stop_beat();

                        info!("Page → {} ({})", page_idx, state.config.pages[page_idx].name);
                        state.current_page  = page_idx;
                        state.fader_levels  = vec![0u8; 8];
                        state.xy_selected   = None;
                        state.rgb_states.clear();

                        // Start beat thread if entering beat page
                        let is_beat = state.config.pages[page_idx].beat_mode;
                        if is_beat {
                            state.init_beat();
                            if let Some(bs) = state.beat_state.clone() {
                                let socket = state.osc_socket.try_clone()
                                    .expect("clone UDP socket");
                                let target = state.osc_target.clone();
                                spawn_beat_thread(bs, socket, target);
                            }
                        }

                        render_page(state, out);
                    }
                }
            }
        }

        // ── Note On ─────────────────────────────────────────────────────────
        0x90 if data2 > 0 => {
            state.press_times.insert(data1, Instant::now());
            handle_note_on(data1, state, out);
        }

        // ── Note Off (or Note On with velocity 0) ───────────────────────────
        0x80 | 0x90 => {
            let elapsed = state.press_times
                .remove(&data1)
                .map(|t| t.elapsed())
                .unwrap_or(Duration::ZERO);
            handle_note_off(data1, elapsed, state, out);
        }

        _ => {}
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Note-On handler
// ─────────────────────────────────────────────────────────────────────────────

fn handle_note_on(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let page = state.config.pages[state.current_page].clone();

    // ── Colour picker overlay ─────────────────────────────────────────────
    if let Overlay::ColorPicker { page_idx, note: target_note } = &state.overlay {
        let (page_idx, target_note) = (*page_idx, *target_note);
        if let Some((col, row)) = note_to_grid(note) {
            let chosen = picker_color_for_cell(col, row);
            state.custom_colors.insert((page_idx, target_note), chosen);
            info!("Colour picker: note {} → colour {}", target_note, chosen);
        }
        state.overlay = Overlay::None;
        render_page(state, out);
        return;
    }

    // ── Right-column special buttons ──────────────────────────────────────
    if RIGHT_COL_NOTES.contains(&note) {
        handle_right_col(note, state, out);
        return;
    }

    // ── Dispatch by page mode ─────────────────────────────────────────────
    if page.rgb_mode {
        handle_rgb_on(note, state, out);
    } else if page.flash_mode {
        handle_flash_on(note, state, out);
    } else if page.move_mode {
        handle_move(note, state);
    } else if page.beat_mode {
        // Beat page: grid pads not used for triggers (right-col handles it)
    } else if page.xy_mode {
        handle_xy(note, state, out);
    } else if page.fader_mode {
        handle_fader(note, state, out);
    } else if page.custom_mode {
        // Start timing for long-press
        // (Already inserted into press_times above; nothing else yet)
    } else {
        handle_button(note, state);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Note-Off handler
// ─────────────────────────────────────────────────────────────────────────────

fn handle_note_off(
    note:    u8,
    elapsed: Duration,
    state:   &mut BridgeState,
    out:     &mut MidiOutputConnection,
) {
    let page = state.config.pages[state.current_page].clone();

    // Right column: no special off handling
    if RIGHT_COL_NOTES.contains(&note) { return; }

    if page.flash_mode {
        handle_flash_off(note, state, out);
    } else if page.custom_mode {
        if elapsed.as_millis() >= LONG_PRESS_MS as u128 {
            // Long press → open colour picker
            info!("Long press on note {} → colour picker", note);
            state.overlay = Overlay::ColorPicker {
                page_idx: state.current_page,
                note,
            };
            render_page(state, out);
        } else {
            handle_custom_button(note, state);
        }
    }
    // RGB, Move, XY, Fader: no off action needed
}

// ─────────────────────────────────────────────────────────────────────────────
// Right-column button handler
// ─────────────────────────────────────────────────────────────────────────────

fn handle_right_col(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let page = &state.config.pages[state.current_page];

    if page.xy_mode {
        match note {
            89 => {
                // Toggle FINE / COARSE mode
                state.xy_adjust_mode = match state.xy_adjust_mode {
                    XyAdjustMode::Coarse => XyAdjustMode::Fine,
                    XyAdjustMode::Fine   => XyAdjustMode::Coarse,
                };
                info!("XY adjust mode → {:?}", state.xy_adjust_mode);
                render_page(state, out);
            }
            79 => {
                // Increase fine step (capped at 4)
                if state.xy_adjust_mode == XyAdjustMode::Fine && state.xy_fine_step < 4 {
                    state.xy_fine_step += 1;
                    info!("XY fine step → {}", state.xy_fine_step);
                }
            }
            69 => {
                // Decrease fine step (min 1)
                if state.xy_adjust_mode == XyAdjustMode::Fine && state.xy_fine_step > 1 {
                    state.xy_fine_step -= 1;
                    info!("XY fine step → {}", state.xy_fine_step);
                }
            }
            _ => {}
        }
        return;
    }

    if page.beat_mode {
        match note {
            59 => {
                // Tap tempo
                if let Some(bs) = &state.beat_state {
                    bs.lock().unwrap().tap();
                }
            }
            49 => {
                // Next palette
                if let Some(bs) = &state.beat_state {
                    let mut b = bs.lock().unwrap();
                    if !b.palettes.is_empty() {
                        b.palette_idx = (b.palette_idx + 1) % b.palettes.len();
                        info!("Beat palette → {}", b.palette_idx);
                    }
                }
            }
            39 => {
                // Previous palette
                if let Some(bs) = &state.beat_state {
                    let mut b = bs.lock().unwrap();
                    if !b.palettes.is_empty() {
                        b.palette_idx = if b.palette_idx == 0 {
                            b.palettes.len() - 1
                        } else {
                            b.palette_idx - 1
                        };
                        info!("Beat palette → {}", b.palette_idx);
                    }
                }
            }
            _ => {}
        }
        return;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Mode handlers
// ─────────────────────────────────────────────────────────────────────────────

/// Generic button (normal page): fire OSC once on press.
fn handle_button(note: u8, state: &mut BridgeState) {
    let page = &state.config.pages[state.current_page];
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Button: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC send error: {}", e);
        }
    }
}

/// Custom page (long-press colour picker): short press fires OSC.
fn handle_custom_button(note: u8, state: &mut BridgeState) {
    let addr  = state.custom_osc_address(state.current_page, note);
    let value = {
        let page = &state.config.pages[state.current_page];
        page.buttons
            .iter()
            .find(|b| b.note == note)
            .map(|b| b.osc_value)
            .unwrap_or(1.0)
    };
    info!("Custom button note {} → {} = {}", note, addr, value);
    if let Err(e) = state.send_osc(&addr, value) {
        error!("OSC send error: {}", e);
    }
}

/// RGB toggle: press toggles ON/OFF and sends the appropriate OSC value.
fn handle_rgb_on(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let page = state.config.pages[state.current_page].clone();
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        let current = *state.rgb_states.get(&note).unwrap_or(&false);
        let next    = !current;
        state.rgb_states.insert(note, next);

        let (osc_val, led_color) = if next {
            (btn.osc_value, btn.color_on)
        } else {
            (btn.osc_value_off, btn.color_off)
        };

        info!(
            "RGB toggle note {} → {} = {} ({})",
            note, btn.osc_address, osc_val,
            if next { "ON" } else { "OFF" }
        );
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), osc_val) {
            error!("OSC RGB error: {}", e);
        }
        set_single_led(note, led_color, out);
    }
}

/// Flash: send ON value on press.
fn handle_flash_on(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let page = state.config.pages[state.current_page].clone();
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Flash ON: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC flash ON error: {}", e);
        }
        // Bright white while held
        set_single_led(note, 72, out);
    }
}

/// Flash: send OFF value on release.
fn handle_flash_off(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let page = state.config.pages[state.current_page].clone();
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Flash OFF: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value_off);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value_off) {
            error!("OSC flash OFF error: {}", e);
        }
        // Restore resting colour
        set_single_led(note, btn.color_on, out);
    }
}

/// Move: trigger a named scene / EFX in QLC+.
fn handle_move(note: u8, state: &mut BridgeState) {
    let page = state.config.pages[state.current_page].clone();
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Move trigger: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC move error: {}", e);
        }
    }
}

/// XY: tap a grid cell to set pan/tilt.
/// Also respects FINE/COARSE mode for arrow-based nudging (see handle_right_col).
fn handle_xy(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let (col, row) = match note_to_grid(note) {
        Some(v) => v,
        None    => return,
    };

    // In FINE mode the step size can be fractional; we still snap to the
    // tapped cell, but the right-col buttons will nudge by xy_fine_step cells.
    let pan  = col as f32 / 7.0;
    let tilt = (7 - row) as f32 / 7.0;

    state.xy_selected = Some((col, row));

    let page = state.config.pages[state.current_page].clone();
    if let Some(xy) = &page.xy_config {
        info!(
            "XY tap: col={} row={} → pan={:.3} tilt={:.3} (mode={:?})",
            col, row, pan, tilt, state.xy_adjust_mode
        );
        if let Err(e) = state.send_osc(&xy.pan_address.clone(), pan) {
            error!("OSC pan error: {}", e);
        }
        if let Err(e) = state.send_osc(&xy.tilt_address.clone(), tilt) {
            error!("OSC tilt error: {}", e);
        }
    }

    render_page(state, out);
}

/// Fader: tap a cell to set that column's level.
/// Columns 0-3 = halogens, 4-7 = RGBs — same grid, different colours.
fn handle_fader(
    note:  u8,
    state: &mut BridgeState,
    out:   &mut MidiOutputConnection,
) {
    let (col, row) = match note_to_grid(note) {
        Some(v) => v,
        None    => return,
    };

    let step    = row + 1;              // 1 = bottom cell, 8 = top cell
    let current = state.fader_levels.get(col as usize).copied().unwrap_or(0);
    // Tapping the current top cell acts as a toggle (set to 0 = off).
    let new_level = if current == step { 0 } else { step };
    state.fader_levels[col as usize] = new_level;

    let value = new_level as f32 / 8.0;

    let page = state.config.pages[state.current_page].clone();
    if let Some(fader) = &page.fader_config {
        if let Some(addr) = fader.addresses.get(col as usize) {
            let kind = if col < 4 { "halogen" } else { "RGB" };
            info!(
                "Fader {} col={} level={} → {} = {:.2}",
                kind, col, new_level, addr, value
            );
            if let Err(e) = state.send_osc(&addr.clone(), value) {
                error!("OSC fader error: {}", e);
            }
        }
    }

    render_page(state, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info"),
    )
    .init();

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config.toml".to_string());
    let config_str = std::fs::read_to_string(&config_path)
        .with_context(|| format!("Cannot read config file: {}", config_path))?;
    let config: Config = toml::from_str(&config_str).context("Parse config.toml")?;

    info!("Config loaded: {} pages defined", config.pages.len());
    info!("OSC target: {}:{}", config.osc.host, config.osc.port);

    // ── Find MIDI ports ──────────────────────────────────────────────────────
    let midi_in  = MidiInput::new("launchpad-qlc-in")?;
    let midi_out = MidiOutput::new("launchpad-qlc-out")?;

    let device_name = config.midi.device_name.to_lowercase();
    let in_port = midi_in
        .ports()
        .into_iter()
        .find(|p| {
            midi_in
                .port_name(p)
                .map(|n| n.to_lowercase().contains(&device_name))
                .unwrap_or(false)
        })
        .with_context(|| {
            format!(
                "No MIDI input port found matching '{}'.\nAvailable ports:\n  {}",
                config.midi.device_name,
                midi_in
                    .ports()
                    .iter()
                    .filter_map(|p| midi_in.port_name(p).ok())
                    .collect::<Vec<_>>()
                    .join("\n  ")
            )
        })?;

    let device_out_name = config.midi.device_out_name.to_lowercase();
    let out_port = midi_out
        .ports()
        .into_iter()
        .find(|p| {
            midi_out
                .port_name(p)
                .map(|n| n.to_lowercase().contains(&device_out_name))
                .unwrap_or(false)
        })
        .with_context(|| {
            format!(
                "No MIDI output port found matching '{}'",
                config.midi.device_out_name
            )
        })?;

    info!("MIDI in:  {}", midi_in.port_name(&in_port).unwrap_or_default());
    info!("MIDI out: {}", midi_out.port_name(&out_port).unwrap_or_default());

    let mut midi_out_conn = midi_out
        .connect(&out_port, "launchpad-qlc-out")
        .map_err(|e| anyhow::anyhow!("MIDI out connect: {}", e))?;

    midi_out_conn.send(SYSEX_PROGRAMMER_MODE)?;
    std::thread::sleep(Duration::from_millis(50));
    midi_out_conn.send(&sysex_clear_all())?;
    std::thread::sleep(Duration::from_millis(20));

    let state = BridgeState::new(config)?;
    let state = Arc::new(Mutex::new(state));

    // Initial render
    {
        let s = state.lock().unwrap();
        render_page(&s, &mut midi_out_conn);
    }

    let midi_out_conn    = Arc::new(Mutex::new(midi_out_conn));
    let midi_out_conn_cb = Arc::clone(&midi_out_conn);
    let state_cb         = Arc::clone(&state);

    let _midi_in_conn = midi_in
        .connect(
            &in_port,
            "launchpad-qlc-in",
            move |_stamp, message, _| {
                let mut s   = state_cb.lock().unwrap();
                let mut out = midi_out_conn_cb.lock().unwrap();
                handle_midi(message, &mut s, &mut out);
            },
            (),
        )
        .map_err(|e| anyhow::anyhow!("MIDI in connect: {}", e))?;

    info!("Bridge running. Press Ctrl+C to quit.");
    info!("Pages: [91]RGB [92]FLASH [93]MOVE [94]BEAT [95]XY [96]FADER");
    info!(
        "Right-col on XY page: [89]=fine/coarse toggle  [79]=step+ [69]=step-"
    );
    info!("Right-col on BEAT page: [59]=tap tempo  [49]=next palette  [39]=prev palette");
    info!(
        "Custom pages: short press = OSC trigger | hold {}ms = colour picker",
        LONG_PRESS_MS
    );

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
