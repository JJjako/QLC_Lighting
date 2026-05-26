/// launchpad-qlc: Novation Launchpad Mini MK3 → QLC+ OSC bridge
///
/// Pages are defined in config.toml. The top-row CC buttons (91–98) switch pages.
/// Each page can be:
///   - Normal: individual buttons mapped to QLC+ OSC addresses
///   - XY mode: the 8×8 grid acts as a pan/tilt pad for moving heads
///   - Fader mode: each column is an 8-step dimmer fader
///
/// QLC+ setup:
///   Inputs/Outputs → OSC → enable input on port 7700 (or whatever you set in config.toml)

use std::{
    net::UdpSocket,
    sync::{Arc, Mutex},
    time::Duration,
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

/// Set a single pad LED by note number with a palette colour (velocity).
fn sysex_set_led(note: u8, color: u8) -> Vec<u8> {
    // SysEx: Lighting type 0x0A = palette colour
    vec![
        0xF0, 0x00, 0x20, 0x29, 0x02, 0x0D, 0x03,
        0x00,  // lighting type: palette
        note,
        color,
        0xF7,
    ]
}

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
    // Set every possible note 0–99 to colour 0
    let pairs: Vec<(u8, u8)> = (0u8..=99).map(|n| (n, 0)).collect();
    sysex_set_leds_bulk(&pairs)
}

/// Top-row CC note numbers for page selection (CC 91–98 = notes in the top row).
/// On the MK3 in programmer mode these come in as CC messages on channel 1.
const PAGE_SELECT_CC: [u8; 8] = [91, 92, 93, 94, 95, 96, 97, 98];

/// Convert grid position (col 0-7, row 0-7) to Launchpad note number.
/// Row 0 = bottom, row 7 = top.
fn grid_note(col: u8, row: u8) -> u8 {
    // Bottom-left = 11, top-right = 88
    (row + 1) * 10 + (col + 1)
}

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────

struct BridgeState {
    config: Config,
    current_page: usize,
    osc_socket: UdpSocket,
    osc_target: String,
    /// For fader pages: track the active step per column (0 = off, 1–8 = step from bottom)
    fader_levels: Vec<u8>,
    /// For xy page: track last selected cell
    xy_selected: Option<(u8, u8)>, // (col, row)
}

impl BridgeState {
    fn new(config: Config) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("bind UDP")?;
        let target = format!("{}:{}", config.osc.host, config.osc.port);
        Ok(Self {
            fader_levels: vec![0u8; 8],
            xy_selected: None,
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
}

// ─────────────────────────────────────────────────────────────────────────────
// LED rendering
// ─────────────────────────────────────────────────────────────────────────────

fn render_page(state: &BridgeState, out: &mut MidiOutputConnection) {
    let mut pairs: Vec<(u8, u8)> = Vec::with_capacity(128);

    // Clear everything first
    for n in 0u8..=99 {
        pairs.push((n, 0));
    }

    // Light page selector buttons (top row CCs)
    for (i, &cc) in PAGE_SELECT_CC.iter().enumerate() {
        let color = if i < state.config.pages.len() {
            if i == state.current_page {
                // Bright white for active page
                72
            } else {
                state.config.pages[i].selector_color
            }
        } else {
            0
        };
        pairs.push((cc, color));
    }

    let page = &state.config.pages[state.current_page];

    if page.xy_mode {
        if let Some(xy) = &page.xy_config {
            // Light whole grid with inactive colour, active cell with active colour
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
                    // row 0 = bottom step 1, row 7 = top step 8
                    let step = row + 1;
                    let color = if level >= step {
                        fader.color_on
                    } else {
                        fader.color_off
                    };
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
    let data1 = message[1];
    let data2 = message[2];

    let msg_type = status & 0xF0;
    // channel = status & 0x0F  (not needed but left for reference)

    match msg_type {
        // ── CC: top-row page selectors ────────────────────────────────────
        0xB0 => {
            if data2 > 0 {
                // button pressed (data2 = velocity / value)
                if let Some(page_idx) = PAGE_SELECT_CC.iter().position(|&cc| cc == data1) {
                    if page_idx < state.config.pages.len() {
                        info!(
                            "Page → {} ({})",
                            page_idx, state.config.pages[page_idx].name
                        );
                        state.current_page = page_idx;
                        // Reset per-page transient state
                        state.fader_levels = vec![0u8; 8];
                        state.xy_selected = None;
                        render_page(state, out);
                    }
                }
            }
        }

        // ── Note On: grid buttons ─────────────────────────────────────────
        0x90 if data2 > 0 => {
            let note = data1;
            let page = state.config.pages[state.current_page].clone();

            if page.xy_mode {
                handle_xy(note, state, out);
            } else if page.fader_mode {
                handle_fader(note, state, out);
            } else {
                handle_button(note, state);
            }
        }

        _ => {}
    }
}

fn handle_button(note: u8, state: &mut BridgeState) {
    let page = &state.config.pages[state.current_page];
    if let Some(btn) = page.buttons.iter().find(|b| b.note == note) {
        info!("Button: {} → {} = {}", btn.label, btn.osc_address, btn.osc_value);
        if let Err(e) = state.send_osc(&btn.osc_address.clone(), btn.osc_value) {
            error!("OSC send error: {}", e);
        }
    }
}

fn handle_xy(note: u8, state: &mut BridgeState, out: &mut MidiOutputConnection) {
    // Decode note back to (col, row)
    if note < 11 || note > 88 {
        return;
    }
    let row = (note / 10).saturating_sub(1);
    let col = (note % 10).saturating_sub(1);
    if col > 7 || row > 7 {
        return;
    }

    // pan: col/7.0 (left→right = 0.0→1.0)
    // tilt: (7-row)/7.0 (top=0.0, bottom=1.0)
    let pan = col as f32 / 7.0;
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
    if note < 11 || note > 88 {
        return;
    }
    let row = (note / 10).saturating_sub(1); // 0 = bottom row
    let col = (note % 10).saturating_sub(1);
    if col > 7 || row > 7 {
        return;
    }

    // Pressing a step sets the fader to that level (toggle off if already at that step)
    let step = row + 1; // 1–8
    let current = state.fader_levels.get(col as usize).copied().unwrap_or(0);
    let new_level = if current == step { 0 } else { step };
    state.fader_levels[col as usize] = new_level;

    let value = new_level as f32 / 8.0; // 0.0 – 1.0

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

    // Load config
    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config.toml".to_string());
    let config_str = std::fs::read_to_string(&config_path)
        .with_context(|| format!("Cannot read config file: {}", config_path))?;
    let config: Config = toml::from_str(&config_str).context("Parse config.toml")?;

    info!("Config loaded: {} pages defined", config.pages.len());
    info!("OSC target: {}:{}", config.osc.host, config.osc.port);

    // ── Find MIDI ports ──────────────────────────────────────────────────────
    let midi_in = MidiInput::new("launchpad-qlc-in")?;
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
                "No MIDI input port found matching '{}'.\nAvailable ports:\n{}",
                config.midi.device_name,
                {
                    let ports = midi_in.ports();
                    ports
                        .iter()
                        .filter_map(|p| midi_in.port_name(p).ok())
                        .collect::<Vec<_>>()
                        .join("\n  ")
                }
            )
        })?;

    let out_port = midi_out
        .ports()
        .into_iter()
        .find(|p| {
            midi_out
                .port_name(p)
                .map(|n| n.to_lowercase().contains(&device_name))
                .unwrap_or(false)
        })
        .with_context(|| {
            format!(
                "No MIDI output port found matching '{}'",
                config.midi.device_name
            )
        })?;

    info!(
        "MIDI in:  {}",
        midi_in.port_name(&in_port).unwrap_or_default()
    );
    info!(
        "MIDI out: {}",
        midi_out.port_name(&out_port).unwrap_or_default()
    );

    // ── Open output first, then input ────────────────────────────────────────
    let mut midi_out_conn = midi_out
        .connect(&out_port, "launchpad-qlc-out")
        .map_err(|e| anyhow::anyhow!("MIDI out connect: {}", e))?;

    // Enter programmer mode
    midi_out_conn.send(SYSEX_PROGRAMMER_MODE)?;
    std::thread::sleep(Duration::from_millis(50));

    // Clear all LEDs
    midi_out_conn.send(&sysex_clear_all())?;
    std::thread::sleep(Duration::from_millis(20));

    // Build initial state
    let state = BridgeState::new(config)?;
    let state = Arc::new(Mutex::new(state));

    // Initial render
    {
        let mut s = state.lock().unwrap();
        render_page(&s, &mut midi_out_conn);
    }

    // Wrap midi_out_conn in Arc<Mutex<>> so the MIDI callback can use it
    let midi_out_conn = Arc::new(Mutex::new(midi_out_conn));
    let midi_out_conn_cb = Arc::clone(&midi_out_conn);
    let state_cb = Arc::clone(&state);

    // ── Open MIDI input ───────────────────────────────────────────────────────
    let _midi_in_conn = midi_in
        .connect(
            &in_port,
            "launchpad-qlc-in",
            move |_stamp, message, _| {
                let mut s = state_cb.lock().unwrap();
                let mut out = midi_out_conn_cb.lock().unwrap();
                handle_midi(message, &mut s, &mut out);
            },
            (),
        )
        .map_err(|e| anyhow::anyhow!("MIDI in connect: {}", e))?;

    info!("Bridge running. Press Ctrl+C to quit.");
    info!("Top row buttons switch pages. Current page: 0 = {}", {
        let s = state.lock().unwrap();
        s.config.pages[0].name.clone()
    });

    // Keep the main thread alive
    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}