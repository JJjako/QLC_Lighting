/// launchpad-qlc: Novation Launchpad Mini MK3 → QLC+ OSC bridge
///
/// Pages are defined in config.toml. The top-row CC buttons (91–98) switch pages.
/// Each page can be:
///   - Normal:      individual buttons mapped to QLC+ OSC addresses
///   - Custom mode: all 64 pads are freely assignable; long-press opens an on-pad
///                  colour picker so you can change each button's LED colour live
///   - XY mode:     the 8×8 grid acts as a pan/tilt pad for moving heads
///   - Fader mode:  each column is an 8-step dimmer fader
///
/// QLC+ setup:
///   Inputs/Outputs → OSC → enable input on port 7700 (or whatever you set in config.toml)

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
// Config structures
// ─────────────────────────────────────────────────────────────────────────────

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
    /// Set true to make all 64 pads freely customisable with long-press colour picker
    #[serde(default)]
    custom_mode: bool,
    xy_config: Option<XyConfig>,
    fader_config: Option<FaderConfig>,
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
struct FaderConfig {
    addresses: Vec<String>,
    color_on: u8,
    color_off: u8,
}

// ─────────────────────────────────────────────────────────────────────────────
// Launchpad Mini MK3 SysEx / note helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Set the Launchpad into "programmer mode" so we control all LEDs directly.
const SYSEX_PROGRAMMER_MODE: &[u8] = &[
    0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x0E, 0x01, 0xF7,
];

/// Set multiple LEDs at once (more efficient).
fn sysex_set_leds_bulk(pairs: &[(u8, u8)]) -> Vec<u8> {
    let mut msg = vec![0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x03];
    for (note, color) in pairs {
        msg.push(0x00); // palette type
        msg.push(*note);
        msg.push(*color);
    }
    msg.push(0xF7);
    msg
}

/// Clear all LEDs.
fn sysex_clear_all() -> Vec<u8> {
    let pairs: Vec<(u8, u8)> = (0u8..=99).map(|n| (n, 0)).collect();
    sysex_set_leds_bulk(&pairs)
}

/// Top-row CC note numbers for page selection (CC 91–98).
const PAGE_SELECT_CC: [u8; 8] = [91, 92, 93, 94, 95, 96, 97, 98];

/// Convert grid position (col 0-7, row 0-7) to Launchpad note number.
/// Row 0 = bottom, row 7 = top.
fn grid_note(col: u8, row: u8) -> u8 {
    (row + 1) * 10 + (col + 1)
}

/// Decode a note number back to (col, row). Returns None if out of grid range.
fn note_to_grid(note: u8) -> Option<(u8, u8)> {
    if note < 11 || note > 88 {
        return None;
    }
    let row = (note / 10).saturating_sub(1);
    let col = (note % 10).saturating_sub(1);
    if col > 7 || row > 7 { None } else { Some((col, row)) }
}

/// Map a grid position to a 1-based pad index (col-major, left-to-right, bottom-to-top).
fn grid_to_pad_index(col: u8, row: u8) -> u8 {
    row * 8 + col + 1 // 1..=64
}

// ─────────────────────────────────────────────────────────────────────────────
// Colour-picker palette layout
//
// We display palette colours 1–64 across the 8×8 grid.
// The top-left pad (row 7, col 0) = palette colour 1,
// increasing left-to-right then top-to-bottom, so:
//   row 7 cols 0-7 → colours  1- 8
//   row 6 cols 0-7 → colours  9-16
//   …
//   row 0 cols 0-7 → colours 57-64
// ─────────────────────────────────────────────────────────────────────────────

fn picker_color_for_cell(col: u8, row: u8) -> u8 {
    // row 7 = top row in our coordinate system
    let palette_idx = (7 - row) * 8 + col; // 0..63
    palette_idx + 1 // palette colour 1..64
}

fn picker_cell_for_color(color: u8) -> Option<(u8, u8)> {
    if color == 0 || color > 64 {
        return None;
    }
    let idx = color - 1; // 0..63
    let row = 7 - (idx / 8);
    let col = idx % 8;
    Some((col, row))
}

// ─────────────────────────────────────────────────────────────────────────────
// Long-press threshold
// ─────────────────────────────────────────────────────────────────────────────

const LONG_PRESS_MS: u64 = 500;

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────

/// Which overlay (if any) is currently active.
enum Overlay {
    None,
    /// Colour picker is open for note `note` on page `page_idx`.
    ColorPicker { page_idx: usize, note: u8 },
}

struct BridgeState {
    config: Config,
    current_page: usize,
    osc_socket: UdpSocket,
    osc_target: String,
    /// Fader levels per column (0 = off, 1–8 = step from bottom).
    fader_levels: Vec<u8>,
    /// XY selected cell.
    xy_selected: Option<(u8, u8)>,
    /// Overlay state (colour picker).
    overlay: Overlay,
    /// Per-page, per-note runtime colour overrides for custom pages.
    /// Key: (page_idx, note), Value: palette colour 0-127.
    custom_colors: HashMap<(usize, u8), u8>,
    /// Timestamps of Note On events, for long-press detection.
    press_times: HashMap<u8, Instant>,
}

impl BridgeState {
    fn new(config: Config) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("bind UDP")?;
        let target = format!("{}:{}", config.osc.host, config.osc.port);
        Ok(Self {
            fader_levels: vec![0u8; 8],
            xy_selected: None,
            overlay: Overlay::None,
            custom_colors: HashMap::new(),
            press_times: HashMap::new(),
            current_page: 0,
            osc_socket: socket,
            osc_target: target,
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

    /// Effective colour for a custom-page button (runtime override → config default → 72).
    fn custom_button_color(&self, page_idx: usize, note: u8) -> u8 {
        if let Some(&c) = self.custom_colors.get(&(page_idx, note)) {
            return c;
        }
        // Fall back to config color if this note happens to be listed
        let page = &self.config.pages[page_idx];
        page.buttons
            .iter()
            .find(|b| b.note == note)
            .map(|b| b.color)
            .unwrap_or(0) // default: off
    }

    /// OSC address for a custom-page pad (auto-generated from pad index).
    fn custom_osc_address(&self, page_idx: usize, note: u8) -> String {
        // Check if the config has an explicit entry for this note
        let page = &self.config.pages[page_idx];
        if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
            return btn.osc_address.clone();
        }
        // Auto-generate: /qlc/custom/<pad_index>
        if let Some((col, row)) = note_to_grid(note) {
            let idx = grid_to_pad_index(col, row);
            format!("/qlc/custom/{}", idx)
        } else {
            format!("/qlc/custom/unknown")
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LED rendering
// ─────────────────────────────────────────────────────────────────────────────

fn render_page(state: &BridgeState, out: &mut MidiOutputConnection) {
    let mut pairs: Vec<(u8, u8)> = Vec::with_capacity(128);

    // Clear everything
    for n in 0u8..=99 {
        pairs.push((n, 0));
    }

    // ── Colour picker overlay ────────────────────────────────────────────────
    if let Overlay::ColorPicker { page_idx, note } = &state.overlay {
        // Top row: dim all page selectors except show a "cancel" hint on the active one
        for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
            pairs.push((cc, if i == state.current_page { 5 } else { 0 })); // dim red = cancel hint
        }

        // Grid: show 64-colour palette
        for row in 0u8..8 {
            for col in 0u8..8 {
                let n = grid_note(col, row);
                let c = picker_color_for_cell(col, row);
                pairs.push((n, c));
            }
        }

        // Highlight the currently assigned colour in the palette
        let current_color = state.custom_button_color(*page_idx, *note);
        if let Some((pc, pr)) = picker_cell_for_color(current_color) {
            // Overwrite with a bright blink indicator — we flash by using colour 72 (white)
            // as a border; the cell itself keeps its natural palette colour so the user
            // can still see which one is selected. We mark it by replacing the 4 neighbours
            // with white. Simple approach: just set the cell to white.
            pairs.push((grid_note(pc, pr), 72));
        }

        let sysex = sysex_set_leds_bulk(&pairs);
        if let Err(e) = out.send(&sysex) {
            error!("Failed to send LED SysEx: {}", e);
        }
        return;
    }

    // ── Normal page rendering ────────────────────────────────────────────────

    // Page selector buttons (top row)
    for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
        let color = if i < state.config.pages.len() {
            if i == state.current_page { 72 } else { state.config.pages[i].selector_color }
        } else {
            0
        };
        pairs.push((cc, color));
    }

    let page = &state.config.pages[state.current_page];

    if page.custom_mode {
        // Light all 64 grid pads with their assigned colours
        for row in 0u8..8 {
            for col in 0u8..8 {
                let note = grid_note(col, row);
                let color = state.custom_button_color(state.current_page, note);
                pairs.push((note, color));
            }
        }
    } else if page.xy_mode {
        if let Some(xy) = &page.xy_config {
            for row in 0u8..8 {
                for col in 0u8..8 {
                    let note = grid_note(col, row);
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
                for row in 0u8..8 {
                    let note = grid_note(col, row);
                    let step = row + 1;
                    let color = if level >= step { fader.color_on } else { fader.color_off };
                    pairs.push((note, color));
                }
            }
        }
    } else {
        // Normal button page
        for btn in &page.buttons {
            pairs.push((btn.note, btn.color));
        }
    }

    let sysex = sysex_set_leds_bulk(&pairs);
    if let Err(e) = out.send(&sysex) {
        error!("Failed to send LED SysEx: {}", e);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MIDI message handling
// ─────────────────────────────────────────────────────────────────────────────

fn handle_midi(
    message: &[u8],
    state: &mut BridgeState,
    out: &mut MidiOutputConnection,
) {
    if message.len() < 3 {
        return;
    }

    let status = message[0];
    let data1  = message[1];
    let data2  = message[2];
    let msg_type = status & 0xF0;

    match msg_type {
        // ── CC: top-row page selectors (also acts as "close picker" when picker open) ──
        0xB0 => {
            if data2 > 0 {
                if let Some(page_idx) = PAGE_SELECT_CC.iter().position(|&cc| cc == data1) {
                    // If picker is open, pressing any top-row button cancels it
                    if matches!(state.overlay, Overlay::ColorPicker { .. }) {
                        info!("Color picker cancelled");
                        state.overlay = Overlay::None;
                        render_page(state, out);
                        return;
                    }

                    if page_idx < state.config.pages.len() {
                        info!(
                            "Page → {} ({})",
                            page_idx, state.config.pages[page_idx].name
                        );
                        state.current_page = page_idx;
                        state.fader_levels  = vec![0u8; 8];
                        state.xy_selected   = None;
                        state.press_times.clear();
                        render_page(state, out);
                    }
                }
            }
        }

        // ── Note On: record press time ────────────────────────────────────────
        0x90 if data2 > 0 => {
            let note = data1;
            state.press_times.insert(note, Instant::now());

            let page = &state.config.pages[state.current_page];

            // In colour-picker overlay, Note On is ignored (we act on Note Off)
            if matches!(state.overlay, Overlay::ColorPicker { .. }) {
                return;
            }

            // For non-custom pages, handle immediately on press
            if page.xy_mode {
                handle_xy(note, state, out);
            } else if page.fader_mode {
                handle_fader(note, state, out);
            } else if !page.custom_mode {
                handle_button(note, state);
            }
            // custom_mode: wait for Note Off to decide short vs long
        }

        // ── Note Off (or Note On with vel=0): long-press detection for custom pages ──
        0x80 | 0x90 => {
            // 0x80 = explicit Note Off; 0x90 with vel=0 is also Note Off
            if msg_type == 0x90 && data2 != 0 {
                return; // already handled above
            }
            let note = data1;
            let elapsed = state
                .press_times
                .remove(&note)
                .map(|t| t.elapsed())
                .unwrap_or(Duration::ZERO);

            let page = &state.config.pages[state.current_page];

            // ── Colour picker is open: this Note Off picks a colour ───────────
            if let Overlay::ColorPicker { page_idx, note: target_note } = state.overlay {
                if let Some((col, row)) = note_to_grid(note) {
                    let chosen_color = picker_color_for_cell(col, row);
                    info!(
                        "Color picker: note {} → colour {}",
                        target_note, chosen_color
                    );
                    state.custom_colors.insert((page_idx, target_note), chosen_color);
                }
                state.overlay = Overlay::None;
                render_page(state, out);
                return;
            }

            // ── Custom mode: short press fires OSC, long press opens picker ───
            if page.custom_mode {
                if elapsed >= Duration::from_millis(LONG_PRESS_MS) {
                    // Long press → open colour picker
                    info!(
                        "Long press on note {} ({} ms) → opening colour picker",
                        note,
                        elapsed.as_millis()
                    );
                    state.overlay = Overlay::ColorPicker {
                        page_idx: state.current_page,
                        note,
                    };
                    render_page(state, out);
                } else {
                    // Short press → fire OSC
                    handle_custom_button(note, state);
                }
            }
        }

        _ => {}
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Button handlers
// ─────────────────────────────────────────────────────────────────────────────

fn handle_button(note: u8, state: &mut BridgeState) {
    let page = &state.config.pages[state.current_page];
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Button: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC send error: {}", e);
        }
    }
}

fn handle_custom_button(note: u8, state: &mut BridgeState) {
    let addr = state.custom_osc_address(state.current_page, note);
    // Check if there's an explicit osc_value in config, otherwise send 1.0
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

fn handle_xy(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) {
        Some(v) => v,
        None => return,
    };

    let pan  = col as f32 / 7.0;
    let tilt = (7 - row) as f32 / 7.0;

    state.xy_selected = Some((col, row));

    let page = state.config.pages[state.current_page].clone();
    if let Some(xy) = &page.xy_config {
        info!("XY: col={} row={} → pan={:.2} tilt={:.2}", col, row, pan, tilt);
        if let Err(e) = state.send_osc(&xy.pan_address.clone(), pan) {
            error!("OSC pan error: {}", e);
        }
        if let Err(e) = state.send_osc(&xy.tilt_address.clone(), tilt) {
            error!("OSC tilt error: {}", e);
        }
    }

    render_page(state, out);
}

fn handle_fader(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    let (col, row) = match note_to_grid(note) {
        Some(v) => v,
        None => return,
    };

    let step    = row + 1;
    let current = state.fader_levels.get(col as usize).copied().unwrap_or(0);
    let new_level = if current == step { 0 } else { step };
    state.fader_levels[col as usize] = new_level;

    let value = new_level as f32 / 8.0;

    let page = state.config.pages[state.current_page].clone();
    if let Some(fader) = &page.fader_config {
        if let Some(addr) = fader.addresses.get(col as usize) {
            info!("Fader col={} level={} → {} = {:.2}", col, new_level, addr, value);
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

    {
        let mut s = state.lock().unwrap();
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
    info!(
        "Top row buttons switch pages. Current page: 0 = {}",
        state.lock().unwrap().config.pages[0].name
    );
    info!(
        "Custom pages: short press = OSC trigger | hold {}ms = colour picker",
        LONG_PRESS_MS
    );

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}