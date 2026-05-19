use launchy::mini_mk3::Canvas;
use launchy::{Canvas as _, CanvasMessage, Color, MsgPollingWrapper as _, Pad};
use dict::Dict;
use rosc::{OscMessage, OscPacket, OscType, encoder};
use core::time;
use std::{net::UdpSocket, thread::sleep, time::{Duration, SystemTime}};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (mut canvas, poller) = Canvas::guess_polling()?;
    let mut scene:u32  = 0;
    let mut normal_canvas = vec![
        Color::RED,Color::new(1.0,0.5,0.0),Color::new(1.0, 1.0, 0.0),Color::new(0.0,1.0,0.0),Color::CYAN,Color::new(0.0,0.5,1.0),Color::BLUE,Color::WHITE,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLUE,Color::BLACK,
        Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLACK,Color::BLUE,Color::BLUE,Color::CYAN
    ];
    for x in 0..8{
        for y in 1..9{
            canvas[Pad{x: x, y: y}] = normal_canvas[(x+y*8-8) as  usize];
        }
    }
    canvas.flush();
    for msg in poller.iter() {
        match msg {
            CanvasMessage::Press { x, y } => {
                let pad = Pad { x: x as i32, y: y as i32 };
                if y == 0 || x == 8{
                    println!("Szene {}", x); 
                    //scene = x;
                }
                else {
                    send_dmx(1, (x+y*8+scene*100), 1.0);
                
                }

                canvas[pad] = Color::WHITE;
                canvas.flush()?;
            }
            CanvasMessage::Release { x, y } => {
                if y == 0 || x == 8{
                    canvas[Pad { x: x as i32, y: y as i32 }] = Color::BLACK;
                }
                else {
                    canvas[Pad { x: x as i32, y: y as i32 }] = normal_canvas[(x+y*8-8) as  usize];
                
                }
                
                canvas.flush()?;
            }
        }
    }

    Ok(())
}
fn send_dmx(universe: u8, channel: u32, value: f32) {
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();

    // QLC+ path: /universe-1/dmx/channel-1
    let addr = format!("/{}/dmx/{}", universe - 1, channel - 1);

    let packet = OscPacket::Message(OscMessage {
        addr,
        args: vec![OscType::Float(value)],  // 0.0 to 1.0
    });

    let buf = encoder::encode(&packet).unwrap();
    socket.send_to(&buf, "127.0.0.1:7700").unwrap();
}
/*use rosc::{OscMessage, OscPacket, OscType, encoder};
use core::time;
use std::{net::UdpSocket, thread::sleep, time::{Duration, SystemTime}};



fn main() {
    loop {
        
      // Universe 1, channel 3, 75% brightness
    sleep(Duration::from_secs(2));}
}*/
/* 
use rosc::{decoder, OscPacket, OscType};
use std::net::UdpSocket;

fn main() {
    let socket = UdpSocket::bind("0.0.0.0:9000").unwrap();
    let mut buf = [0u8; 1024];

    loop {
        let (size, _addr) = socket.recv_from(&mut buf).unwrap();
        match decoder::decode_udp(&buf[..size]) {
            Ok((_, OscPacket::Message(msg))) => {
                println!("Address: {}", msg.addr);
                for arg in &msg.args {
                    match arg {
                        OscType::Float(f) => println!("  value: {}", f),
                        OscType::Int(i)   => println!("  value: {}", i),
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}*/
