use launchy::mini_mk3::Canvas;
use launchy::{Canvas as _, CanvasMessage, Color, MsgPollingWrapper as _, Pad};
use dict::Dict;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (mut canvas, poller) = Canvas::guess_polling()?;

    let mut x_mut = 2;
    let mut y_mut = 3;
    while (x_mut < 8){
        for i in 1..=8{
            canvas[Pad{x: x_mut, y: i}] = Color::CYAN;
        }
        x_mut += 3;
    }
    while (y_mut < 8){
        for i in 0..=7{
            canvas[Pad{x: i, y: y_mut}] = Color::CYAN;
        }
        y_mut += 3;
    }
    canvas.flush()?;

    for msg in poller.iter() {
        match msg {
            CanvasMessage::Press { x, y } => {
                let pad = Pad { x: x as i32, y: y as i32 };
                println!("Pressed: ({x}, {y})");

                match (x, y) {
                    //(0, 0) => { std::process::Command::new("firefox").spawn().ok(); }
                    //(1, 0) => { std::process::Command::new("kitty").spawn().ok(); }
                    _ => {}
                }

                canvas[pad] = Color::WHITE;
                canvas.flush()?;
            }
            CanvasMessage::Release { x, y } => {
                canvas[Pad { x: x as i32, y: y as i32 }] = Color::BLACK;
                canvas.flush()?;
            }
        }
    }

    Ok(())
}