extern crate image;
extern crate structopt;

use std::num::ParseIntError;
use std::path::PathBuf;
use structopt::StructOpt;


#[derive(Debug, StructOpt)]
#[structopt(name = "colorscope", about = "Color area calculator.")]
struct Opt {
    /// File path to the image.
    #[structopt(short, long, parse(from_os_str))]
    file_path: PathBuf,

    /// Target color in RGB. (e.g. 255,0,0 for red)
    #[structopt(short = "c", long = "color", parse(try_from_str = parse_color))]
    color: (u8, u8, u8),

    /// Color tolerance. (default: 10)
    #[structopt(short, long, default_value = "10")]
    tolerance: u8,
}


fn parse_color(s: &str) -> Result<(u8, u8, u8), ParseIntError> {
    let parts: Vec<&str> = s.split(',').collect();
    let r = parts[0].parse::<u8>()?;
    let g = parts[1].parse::<u8>()?;
    let b = parts[2].parse::<u8>()?;
    Ok((r, g, b))
}


fn calculate_color_area(image_path: PathBuf, target_color: (u8, u8, u8), tolerance: u8) -> f32 {

    let img = image::open(image_path).unwrap().to_rgb8();
    
    let (width, height) = img.dimensions();

    let mut target_pixels: u32 = 0;

    // scan all pixels and count target pixels
    for x in 0..width {
        for y in 0..height {
            let pixel = img.get_pixel(x, y);
            let pixel_color = (pixel[0], pixel[1], pixel[2]);
            if color_in_range(pixel_color, target_color, tolerance) {
                target_pixels += 1;
            }
        }
    }
    
    let total_pixels = (width * height) as u32;
    let ratio = target_pixels as f32 / total_pixels as f32;

    ratio
}


fn color_in_range(color: (u8, u8, u8), target: (u8, u8, u8), tolerance: u8) -> bool {
    let lower_bound = (
        target.0.saturating_sub(tolerance),
        target.1.saturating_sub(tolerance),
        target.2.saturating_sub(tolerance),
    );
    let upper_bound = (
        target.0.saturating_add(tolerance),
        target.1.saturating_add(tolerance),
        target.2.saturating_add(tolerance),
    );

    color.0 >= lower_bound.0 && color.0 <= upper_bound.0
        && color.1 >= lower_bound.1 && color.1 <= upper_bound.1
        && color.2 >= lower_bound.2 && color.2 <= upper_bound.2
}

fn main() {
    let opt = Opt::from_args();
    let ratio = calculate_color_area(
        opt.file_path,
        opt.color,
        opt.tolerance);

    // println!("Ratio of target color: {:.2}%", ratio * 100.0);
    println!("{}", ratio * 100.0);
}
