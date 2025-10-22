#[cfg(feature = "mimalloc")]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

#[cfg(feature = "jemalloc")]
#[global_allocator]
static GLOBAL: tikv_jemallocator::Jemalloc = tikv_jemallocator::Jemalloc;
use chrono::NaiveDateTime;
use eframe::egui::{self, Align2, Color32, Frame, Painter, Pos2, Rect, Stroke, TextStyle, Vec2};
use eframe::egui::{Context, TextureHandle};
use gphoto2;
use std::process::Command;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{self, Receiver},
    Arc,
};
use std::time::{Duration, Instant};

const PROG_NAME: &str = "bookThing";
const PROG_VERSION: &str = "v1.0.0";

const PREVIEW_FPS: f32 = 24.0;
const UI_FPS: f32 = 60.0;
const PREVIEW_SCALE: PreviewScale = PreviewScale::Full;

const PREVIEW_DT: Duration = Duration::from_micros((1_000_000.0 / PREVIEW_FPS) as u64);
const UI_DT: Duration = Duration::from_micros((1_000_000.0 / UI_FPS) as u64);
const LOG_REBUILD_DT: Duration = Duration::from_millis(120);

pub enum PhotoResult {
    Success,
    NoCamera,
    WriteFail,
    Unknown,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PhotoType {
    TIFF,
    ARW,
    JPEG,
    CR2,
    PNG,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PreviewScale {
    Full,
    Half,
    Quarter,
}

impl PreviewScale {
    pub fn label(self) -> &'static str {
        match self {
            PreviewScale::Full => "Full",
            PreviewScale::Half => "1/2",
            PreviewScale::Quarter => "1/4",
        }
    }
}

impl PhotoType {
    pub fn extension(self) -> &'static str {
        match self {
            PhotoType::TIFF => "TIFF",
            PhotoType::ARW => "ARW",
            PhotoType::JPEG => "JPG",
            PhotoType::CR2 => "CR2",
            PhotoType::PNG => "PNG",
        }
    }
    pub fn label(self) -> &'static str {
        self.extension()
    }
}
impl std::fmt::Display for PhotoType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

pub struct Page {
    idx: u64,
    file_path: String,
}

pub struct Book {
    common_name: String,
    publication_date: String,
    publication_place: String,
    authors: String,
    publisher: String,
    edition: String,
    isbn: String,
    language_primary: String,
    language: String,
    pages: Vec<Page>,
    captured_at: NaiveDateTime,
}

pub struct Capture {
    iso: String,                   // /main/imgsettings/iso
    f_number: String,              // /main/capturesettings/f-number
    color_temperature: String,     // /main/imgsettings/colortemperature
    white_balance: String,         // /main/imgsettings/whitebalance
    exposure_compensation: String, // /main/capturesettings/exposurecompensation
    shutter_speed: String,         // /main/capturesettings/shutterspeed
}

pub struct App {
    texture: Option<TextureHandle>,
    frame_idx: u64,
    last_tick: Instant,
    busy: bool,
    preview: bool,
    camera: String,
    port: String,
    session_logs: Vec<String>,
    output_path: String,
    scanning: bool,
    current_book: Book,
    capture_settings: Capture,
    peek_idx: u64,
    preview_bytes: Option<Vec<u8>>,
    preview_rx: Option<Receiver<Vec<u8>>>,
    preview_thread: Option<std::thread::JoinHandle<()>>,
    preview_run: Option<Arc<AtomicBool>>,
    preview_pause: Option<Arc<AtomicBool>>,
    output_type: PhotoType,
    cap_rx: Option<Receiver<Capture>>,
    preview_scale: PreviewScale,
    log_cache: String,
    log_cached_count: usize,
    log_last_rebuild: Instant,
}

fn draw_box(p: &Painter, r: Rect, stroke: Stroke, title: &str) {
    p.rect_stroke(r, 0.0, stroke, egui::StrokeKind::Middle);
    p.text(
        Pos2::new(r.left() + 10.0, r.top() + 10.0),
        Align2::LEFT_TOP,
        title,
        TextStyle::Heading.resolve(&egui::Style::default()),
        Color32::LIGHT_GRAY,
    );
}

fn enforce_16x9(ctx: &egui::Context) {
    let win_rect = ctx.input(|i| i.screen_rect());
    let size = win_rect.size();
    if size.y <= 0.0 {
        return;
    }
    let current = size.x / size.y;
    let target = 16.0 / 9.0;
    if (current - target).abs() > 0.01 {
        let new_h = size.x / target;
        ctx.send_viewport_cmd(egui::viewport::ViewportCommand::InnerSize(Vec2::new(
            size.x, new_h,
        )));
    }
}

fn write_dynimage_to_vec(
    img: &image::DynamicImage,
    fmt: image::ImageFormat,
) -> Result<Vec<u8>, String> {
    use std::io::Cursor;
    let mut cursor = Cursor::new(Vec::new());
    img.write_to(&mut cursor, fmt).map_err(|e| e.to_string())?;
    Ok(cursor.into_inner())
}

fn sniff_container(bytes: &[u8]) -> &'static str {
    if bytes.len() >= 3 && bytes[0] == 0xFF && bytes[1] == 0xD8 && bytes[2] == 0xFF {
        return "jpeg";
    }
    if bytes.starts_with(b"\x89PNG\r\n\x1A\n") {
        return "png";
    }
    if bytes.len() >= 4 && (&bytes[0..4] == b"II*\0" || &bytes[0..4] == b"MM\0*") {
        if bytes.len() >= 12 && &bytes[8..12] == b"CR\x02\0" {
            return "cr2";
        }
        if bytes.len() >= 2048 && bytes[..2048].windows(4).any(|w| w == b"SONY") {
            return "arw";
        }
        return "tiff";
    }
    "unknown"
}

fn find_embedded_jpeg_in(input: &[u8]) -> Option<Vec<u8>> {
    use image::{self, ImageFormat};

    if input.len() >= 3 && input[0] == 0xFF && input[1] == 0xD8 && input[2] == 0xFF {
        return Some(input.to_vec());
    }

    let scan_len = input.len().min(16 * 1024 * 1024);
    let mut i = 0usize;
    let mut best: Option<Vec<u8>> = None;

    while i + 1 < scan_len {
        let slice = &input[i..scan_len];
        let Some(s_rel) = slice.windows(2).position(|w| w == [0xFF, 0xD8]) else {
            break;
        };
        let s = i + s_rel;

        let rest = &input[s + 2..scan_len];
        let Some(e_rel) = rest.windows(2).position(|w| w == [0xFF, 0xD9]) else {
            break;
        };
        let e = s + 2 + e_rel + 2;

        let cand = &input[s..e];
        if image::load_from_memory_with_format(cand, ImageFormat::Jpeg).is_ok() {
            if best.as_ref().map_or(true, |b| cand.len() > b.len()) {
                best = Some(cand.to_vec());
            }
        }
        i = e;
    }
    best
}

fn decode_best(input: &[u8]) -> Option<image::DynamicImage> {
    use image::{self, ImageFormat};

    if let Ok(r) = image::ImageReader::new(std::io::Cursor::new(input)).with_guessed_format() {
        if let Ok(img) = r.decode() {
            return Some(img);
        }
    }
    if let Some(jpg) = find_embedded_jpeg_in(input) {
        image::load_from_memory_with_format(&jpg, ImageFormat::Jpeg).ok()
    } else {
        None
    }
}

fn convert_for_export_bytes(input: &[u8], target: PhotoType) -> Result<(Vec<u8>, String), String> {
    use image::{self};

    let src_kind = sniff_container(input).to_string();

    match target {
        PhotoType::JPEG => {
            if src_kind == "jpeg" {
                return Ok((
                    input.to_vec(),
                    format!("jpeg→jpeg (copy, metadata preserved)"),
                ));
            }
            if matches!(src_kind.as_str(), "cr2" | "arw" | "tiff") {
                if let Some(jpg) = find_embedded_jpeg_in(input) {
                    return Ok((jpg, format!("{}→jpeg (embedded preview copy)", src_kind)));
                }
            }
            let img = decode_best(input).ok_or_else(|| "decode failed".to_string())?;
            let mut out = Vec::new();
            let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, 92);
            enc.encode_image(&img).map_err(|e| e.to_string())?;
            Ok((
                out,
                format!("{}→jpeg (re-encode, metadata may be lost)", src_kind),
            ))
        }

        PhotoType::PNG => {
            let img = decode_best(input).ok_or_else(|| "decode failed".to_string())?;
            let out = write_dynimage_to_vec(&img, image::ImageFormat::Png)?;
            Ok((out, format!("{}→png (re-encode)", src_kind)))
        }

        PhotoType::TIFF => {
            let img = decode_best(input).ok_or_else(|| "decode failed".to_string())?;
            let out = write_dynimage_to_vec(&img, image::ImageFormat::Tiff)?;
            Ok((out, format!("{}→tiff (re-encode)", src_kind)))
        }

        PhotoType::ARW => {
            if src_kind == "arw" {
                Ok((input.to_vec(), "arw→arw (pass-through)".into()))
            } else {
                let img = decode_best(input).ok_or_else(|| "decode failed".to_string())?;
                let out = write_dynimage_to_vec(&img, image::ImageFormat::Tiff)?;
                Ok((
                    out,
                    format!("{}→arw not supported; exported tiff instead", src_kind),
                ))
            }
        }
        PhotoType::CR2 => {
            if src_kind == "cr2" {
                Ok((input.to_vec(), "cr2→cr2 (pass-through)".into()))
            } else {
                let img = decode_best(input).ok_or_else(|| "decode failed".to_string())?;
                let out = write_dynimage_to_vec(&img, image::ImageFormat::Tiff)?;
                Ok((
                    out,
                    format!("{}→cr2 not supported; exported tiff instead", src_kind),
                ))
            }
        }
    }
}

fn parse_current_value(s: &str) -> Option<String> {
    for line in s.lines() {
        if let Some(rest) = line.strip_prefix("Current:") {
            return Some(rest.trim().to_string());
        }
    }
    None
}

fn read_cfg_cli(camera: &str, port: &str, key: &str) -> Option<String> {
    let out = std::process::Command::new("gphoto2")
        .args(&["--camera", camera, "--port", port, "--get-config", key])
        .output()
        .ok()?;

    if !out.status.success() {
        return None;
    }

    let s = String::from_utf8_lossy(&out.stdout);
    parse_current_value(&s)
}

fn capture_from_cli(camera: &str, port: &str) -> Capture {
    let mut c = Capture::new();

    c.iso = read_cfg_cli(camera, port, "iso").unwrap_or(c.iso);
    c.f_number = read_cfg_cli(camera, port, "f-number").unwrap_or(c.f_number);
    c.color_temperature =
        read_cfg_cli(camera, port, "colortemperature").unwrap_or(c.color_temperature);
    c.white_balance = read_cfg_cli(camera, port, "whitebalance").unwrap_or(c.white_balance);
    c.exposure_compensation =
        read_cfg_cli(camera, port, "exposurecompensation").unwrap_or(c.exposure_compensation);

    c.shutter_speed = read_cfg_cli(camera, port, "shutterspeed")
        .or_else(|| read_cfg_cli(camera, port, "shutter_speed"))
        .unwrap_or(c.shutter_speed);

    c
}

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1920.0, 1080.0])
            .with_resizable(true)
            .with_icon(egui::viewport::IconData::default()),
        ..Default::default()
    };
    eframe::run_native(
        PROG_NAME,
        options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}

impl Capture {
    pub fn new() -> Self {
        Self {
            iso: "N/A".into(),                   // /main/imgsettings/iso
            f_number: "N/A".into(),              // /main/capturesettings/f-number
            color_temperature: "N/A".into(),     // /main/imgsettings/colortemperature
            white_balance: "N/A".into(),         // /main/imgsettings/whitebalance
            exposure_compensation: "N/A".into(), // /main/capturesettings/exposurecompensation
            shutter_speed: "N/A".into(),         // /main/capturesettings/shutter_speed
        }
    }
}

impl Page {
    pub fn new(idx: u64, output_path: String, ext: &str) -> Self {
        let filename = format!("{}.{}", idx, ext);
        let full = std::path::Path::new(&output_path).join(filename);
        Self {
            idx,
            file_path: full.to_string_lossy().into_owned(),
        }
    }
}

impl Book {
    pub fn new() -> Self {
        let now_local = chrono::Local::now();
        let naive_now: NaiveDateTime = now_local.naive_local();
        Self {
            common_name: "".into(),
            captured_at: naive_now,
            publication_place: "".into(),
            authors: "".into(),
            publisher: "".into(),
            edition: "".into(),
            isbn: "".into(),
            language_primary: "".into(),
            language: "".into(),
            pages: Vec::new(),
            publication_date: "".into(),
        }
    }
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut session_logs: Vec<String> = Vec::new();
        session_logs.push(format!("{} {}\n\nGeneral purpose frontend for gphoto2 to track and handle taking & organizing bulk sets of images\n\nDeveloped by: Raeid-U <raeidus15@gmail.com>\nTutorial found at: https://github.com/Raeid-U/bookThing\n", PROG_NAME, PROG_VERSION).into());

        let mut outdir = std::env::temp_dir();
        outdir.push(PROG_NAME);
        outdir.push("output");
        let _ = std::fs::create_dir_all(&outdir);

        let mut output_path = outdir.to_string_lossy().to_string();
        if !output_path.ends_with(std::path::MAIN_SEPARATOR) {
            output_path.push(std::path::MAIN_SEPARATOR);
        }

        Self {
            texture: None,
            frame_idx: 0,
            last_tick: Instant::now(),
            busy: false,
            preview: false,
            camera: " ".into(),
            port: " ".into(),
            session_logs,
            output_path,
            current_book: Book::new(),
            scanning: false,
            capture_settings: Capture::new(),
            peek_idx: 0,
            preview_bytes: None,
            preview_rx: None,
            preview_thread: None,
            preview_run: None,
            preview_pause: None,
            output_type: PhotoType::ARW,
            cap_rx: None,
            preview_scale: PREVIEW_SCALE,
            log_cache: String::new(),
            log_cached_count: 0,
            log_last_rebuild: Instant::now(),
        }
    }

    fn maybe_rebuild_log_cache(&mut self) {
        if self.session_logs.len() == self.log_cached_count
            || self.log_last_rebuild.elapsed() < LOG_REBUILD_DT
        {
            return;
        }

        self.log_cache.clear();
        let approx: usize = self.session_logs.iter().map(|s| s.len() + 1).sum();
        if self.log_cache.capacity() < approx {
            self.log_cache.reserve(approx - self.log_cache.capacity());
        }
        for line in &self.session_logs {
            self.log_cache.push_str(line);
            if !line.ends_with('\n') {
                self.log_cache.push('\n');
            }
        }

        self.log_cached_count = self.session_logs.len();
        self.log_last_rebuild = Instant::now();
    }

    fn merge_capture_settings(&mut self, inc: Capture) {
        fn set_if(val: String, dst: &mut String) {
            if val != "N/A" && !val.is_empty() {
                *dst = val;
            }
        }
        set_if(inc.iso, &mut self.capture_settings.iso);
        set_if(inc.f_number, &mut self.capture_settings.f_number);
        set_if(
            inc.color_temperature,
            &mut self.capture_settings.color_temperature,
        );
        set_if(inc.white_balance, &mut self.capture_settings.white_balance);
        set_if(
            inc.exposure_compensation,
            &mut self.capture_settings.exposure_compensation,
        );
        set_if(inc.shutter_speed, &mut self.capture_settings.shutter_speed);
    }

    fn retake_page(&mut self, idx: u64) {
        let page = Page::new(idx, self.output_path.clone(), self.output_type.extension());

        self.session_logs.push(format!(
            "[Retake]\n> Retaking page #{} at {}",
            idx, page.file_path
        ));

        match self.take_photo(&page) {
            PhotoResult::Success => {
                self.session_logs
                    .push(format!("[Retake]\n> Replaced page #{} successfully.", idx));
            }
            PhotoResult::NoCamera => {
                self.session_logs
                    .push("[Retake]\n> No camera detected.".into());
            }
            PhotoResult::WriteFail => {
                self.session_logs
                    .push("[Retake]\n> Could not write new image to disk.".into());
            }
            PhotoResult::Unknown => {
                self.session_logs
                    .push("[Retake]\n> Capture failed for an unknown reason.".into());
            }
        }
    }

    fn delete_page(&mut self, idx: u64) {
        use std::{fs, path::Path};

        if self.current_book.pages.is_empty() {
            self.session_logs
                .push("[Remove]\n> No pages to remove.".into());
            return;
        }

        let Some(pos) = self.current_book.pages.iter().position(|p| p.idx == idx) else {
            self.session_logs
                .push(format!("[Remove]\n> Page #{} not found.", idx));
            return;
        };

        let removed = self.current_book.pages.remove(pos);
        if Path::new(&removed.file_path).exists() {
            match fs::remove_file(&removed.file_path) {
                Ok(_) => self
                    .session_logs
                    .push(format!("[Remove]\n> Deleted {}", removed.file_path)),
                Err(e) => self.session_logs.push(format!(
                    "[Remove]\n> Failed to delete {}: {}",
                    removed.file_path, e
                )),
            }
        } else {
            self.session_logs.push(format!(
                "[Remove]\n> File not found (already missing?): {}",
                removed.file_path
            ));
        }

        for i in pos..self.current_book.pages.len() {
            let new_idx = (i as u64) + 1;

            let old_path = std::mem::take(&mut self.current_book.pages[i].file_path);
            let new_path = format!(
                "{}{}.{}",
                self.output_path,
                new_idx,
                self.output_type.extension()
            );

            let _ = fs::remove_file(&new_path);

            match fs::rename(&old_path, &new_path) {
                Ok(_) => {
                    self.session_logs
                        .push(format!("[Remove]\n> Renamed {} -> {}", old_path, new_path));
                }
                Err(e1) => {
                    let copied = fs::copy(&old_path, &new_path);
                    match copied {
                        Ok(_) => {
                            let _ = fs::remove_file(&old_path);
                            self.session_logs.push(format!(
                                "[Remove]\n> Copied+Removed (rename failed: {}): {} -> {}",
                                e1, old_path, new_path
                            ));
                        }
                        Err(e2) => {
                            self.session_logs.push(format!(
                                "[Remove]\n> Failed to move {} -> {} (rename: {}, copy: {})",
                                old_path, new_path, e1, e2
                            ));
                        }
                    }
                }
            }

            self.current_book.pages[i].idx = new_idx;
            self.current_book.pages[i].file_path = new_path;
        }

        let new_len = self.current_book.pages.len() as u64;
        if self.peek_idx > new_len {
            self.peek_idx = 0;
        }

        self.session_logs.push(format!(
            "[Remove]\n> Removed page #{}. Now {} pages remain.",
            idx, new_len
        ));
    }

    fn export_book(&mut self) {
        use chrono::Local;
        use serde::Serialize;
        use serde_json;
        use std::path::PathBuf;
        use zip::{write::FileOptions, ZipWriter};

        self.session_logs
            .push("[Export]\n> Starting export (background)".into());

        fn sanitize_for_fs(s: &str) -> String {
            let s = s.trim();
            if s.is_empty() {
                return "untitled".into();
            }
            let mut out = String::with_capacity(s.len());
            let mut prev_us = false;
            for ch in s.chars() {
                let ok = ch.is_ascii_alphanumeric() || ch == '.' || ch == '_' || ch == '-';
                if ok {
                    out.push(ch);
                    prev_us = false;
                } else if !prev_us {
                    out.push('_');
                    prev_us = true;
                }
            }
            let out = out.trim_matches('_');
            if out.is_empty() {
                "untitled".into()
            } else {
                out.into()
            }
        }
        fn split_and_sanitize(list: &str) -> Vec<String> {
            list.split(',')
                .map(|x| x.trim())
                .filter(|x| !x.is_empty())
                .map(sanitize_for_fs)
                .collect()
        }

        #[derive(Serialize)]
        struct ManifestPage {
            idx: u64,
            file_path: String,
        }
        #[derive(Serialize)]
        struct ManifestCapture {
            iso: String,
            f_number: String,
            color_temperature: String,
            white_balance: String,
            exposure_compensation: String,
            shutter_speed: String,
        }
        #[derive(Serialize)]
        struct ManifestBook {
            common_name: String,
            publication_date: String,
            publication_place: String,
            authors: Vec<String>,
            publisher: String,
            edition: String,
            isbn: String,
            language_primary: String,
            language: Vec<String>,
            captured_at: String,
            capture_settings: ManifestCapture,
            pages: Vec<ManifestPage>,
            export_target: String,
        }

        // do not remove:
        //  snapshot data needed for background thread
        let target_ext = self.output_type.extension().to_string();
        let export_target_label = self.output_type.to_string();

        let pages_snapshot: Vec<(u64, String)> = self
            .current_book
            .pages
            .iter()
            .map(|p| (p.idx, p.file_path.clone()))
            .collect();

        let sanitized_name = sanitize_for_fs(&self.current_book.common_name);
        let ts = Local::now().format("%Y%m%d-%H%M%S").to_string();

        let manifest = ManifestBook {
            common_name: sanitized_name.clone(),
            publication_date: sanitize_for_fs(&self.current_book.publication_date),
            publication_place: sanitize_for_fs(&self.current_book.publication_place),
            authors: split_and_sanitize(&self.current_book.authors),
            publisher: sanitize_for_fs(&self.current_book.publisher),
            edition: sanitize_for_fs(&self.current_book.edition),
            isbn: sanitize_for_fs(&self.current_book.isbn),
            language_primary: sanitize_for_fs(&self.current_book.language_primary),
            language: split_and_sanitize(&self.current_book.language),
            captured_at: self
                .current_book
                .captured_at
                .format("%Y-%m-%d %H:%M:%S")
                .to_string(),
            capture_settings: ManifestCapture {
                iso: self.capture_settings.iso.clone(),
                f_number: self.capture_settings.f_number.clone(),
                color_temperature: self.capture_settings.color_temperature.clone(),
                white_balance: self.capture_settings.white_balance.clone(),
                exposure_compensation: self.capture_settings.exposure_compensation.clone(),
                shutter_speed: self.capture_settings.shutter_speed.clone(),
            },
            pages: pages_snapshot
                .iter()
                .map(|(idx, _)| ManifestPage {
                    idx: *idx,
                    file_path: format!("{}.{}", idx, target_ext),
                })
                .collect(),
            export_target: export_target_label.clone(),
        };

        let manifest_bytes = match serde_json::to_vec_pretty(&manifest) {
            Ok(v) => v,
            Err(e) => {
                self.session_logs
                    .push(format!("[Export]\n> Failed to serialize manifest: {e}"));
                return;
            }
        };

        // important:
        //  thread-safe owned copies
        let staging_dir: PathBuf =
            std::env::temp_dir().join(format!("export_{}_{}", ts, sanitized_name));
        let downloads_dir = dirs_next::download_dir()
            .or_else(|| dirs_next::home_dir().map(|h| h.join("Downloads")))
            .unwrap_or_else(|| std::env::temp_dir());
        let zip_name = format!("{}-{}.zip", ts, sanitized_name);
        let zip_path = downloads_dir.join(&zip_name);

        let output_type = self.output_type;

        self.session_logs.push(format!(
            "[Export]\n> Export started in background.\n> Target: {}\n> Zip: {}",
            export_target_label,
            zip_path.display()
        ));

        std::thread::spawn(move || {
            use std::{fs, io::Write};

            let mut log_lines: Vec<String> = vec![
                format!(
                    "{} export started at {}",
                    PROG_NAME,
                    chrono::Local::now().format("%Y-%m-%d %H:%M:%S")
                ),
                format!("Target: {}", export_target_label),
                format!("Pages: {}", pages_snapshot.len()),
            ];

            if let Err(e) = fs::create_dir_all(&staging_dir) {
                eprintln!(
                    "[Export] Failed to create staging dir {}: {}",
                    staging_dir.display(),
                    e
                );
                return;
            }

            let manifest_path = staging_dir.join("manifest.json");
            if let Err(e) = fs::write(&manifest_path, &manifest_bytes) {
                eprintln!("[Export] Failed to write manifest: {}", e);
                let _ = fs::remove_dir_all(&staging_dir);
                return;
            }

            let mut converted = 0usize;
            let mut failed = 0usize;
            for (idx, src_path) in &pages_snapshot {
                let src_bytes = match fs::read(src_path) {
                    Ok(b) => b,
                    Err(e) => {
                        log_lines.push(format!("page {}: read failed: {}", idx, e));
                        failed += 1;
                        continue;
                    }
                };

                match convert_for_export_bytes(&src_bytes, output_type) {
                    Ok((out_bytes, note)) => {
                        let dest = staging_dir.join(format!("{}.{}", idx, &target_ext));
                        match fs::write(&dest, &out_bytes) {
                            Ok(_) => {
                                converted += 1;
                                log_lines.push(format!("page {}: {}", idx, note));
                            }
                            Err(e) => {
                                failed += 1;
                                log_lines.push(format!("page {}: write failed: {}", idx, e));
                            }
                        }
                    }
                    Err(e) => {
                        failed += 1;
                        log_lines.push(format!("page {}: convert failed: {}", idx, e));
                    }
                }
            }

            let log_path = staging_dir.join("export_log.txt");
            let _ = fs::write(&log_path, log_lines.join("\n"));

            match fs::File::create(&zip_path) {
                Ok(file) => {
                    let mut zip = ZipWriter::new(file);
                    let opts: FileOptions = FileOptions::default()
                        .compression_method(zip::CompressionMethod::Stored)
                        .unix_permissions(0o644);

                    if let Ok(bytes) = std::fs::read(&manifest_path) {
                        let _ = zip.start_file("manifest.json", opts.clone());
                        let _ = zip.write_all(&bytes);
                    }

                    if let Ok(bytes) = std::fs::read(&log_path) {
                        let _ = zip.start_file("export_log.txt", opts.clone());
                        let _ = zip.write_all(&bytes);
                    }

                    for (idx, _) in &pages_snapshot {
                        let filename = format!("{}.{}", idx, &target_ext);
                        let fpath = staging_dir.join(&filename);
                        if fpath.exists() {
                            if let Ok(bytes) = std::fs::read(&fpath) {
                                let _ = zip.start_file(&filename, opts.clone());
                                let _ = zip.write_all(&bytes);
                            }
                        }
                    }

                    let _ = zip.finish();
                }
                Err(e) => {
                    eprintln!(
                        "[Export] Could not create zip at {}: {}",
                        zip_path.display(),
                        e
                    );
                    let _ = fs::remove_dir_all(&staging_dir);
                    return;
                }
            }

            let _ = fs::remove_dir_all(&staging_dir);

            eprintln!(
                "[Export] Background export complete: {} (ok: {}, failed: {})",
                zip_path.display(),
                converted,
                failed
            );
        });
    }

    fn destroy_book(&mut self) {
        use std::{fs, path::Path};

        self.busy = true;
        let outdir = Path::new(&self.output_path);

        self.session_logs.push(format!(
            "[Reset]\n> Starting cleanup of {}",
            outdir.display()
        ));

        let mut removed = 0usize;
        let mut failed = 0usize;

        if outdir.exists() {
            match fs::read_dir(outdir) {
                Ok(entries) => {
                    for entry in entries {
                        match entry {
                            Ok(ent) => {
                                let p = ent.path();
                                let ty = match fs::symlink_metadata(&p) {
                                    Ok(m) => m.file_type(),
                                    Err(e) => {
                                        failed += 1;
                                        self.session_logs.push(format!(
                                            "[Reset]\n> Failed to stat {}: {e}",
                                            p.display()
                                        ));
                                        continue;
                                    }
                                };

                                let res = if ty.is_dir() {
                                    fs::remove_dir_all(&p)
                                } else {
                                    fs::remove_file(&p)
                                };

                                match res {
                                    Ok(_) => removed += 1,
                                    Err(e) => {
                                        failed += 1;
                                        self.session_logs.push(format!(
                                            "[Reset]\n> Failed to remove {}: {e}",
                                            p.display()
                                        ));
                                    }
                                }
                            }
                            Err(e) => {
                                failed += 1;
                                self.session_logs.push(format!(
                                    "[Reset]\n> Failed to read an entry in {}: {e}",
                                    outdir.display()
                                ));
                            }
                        }
                    }
                }
                Err(e) => {
                    self.session_logs.push(format!(
                        "[Reset]\n> Could not list {}: {e}",
                        outdir.display()
                    ));
                }
            }
        } else if let Err(e) = fs::create_dir_all(outdir) {
            self.session_logs.push(format!(
                "[Reset]\n> Output directory did not exist and could not be created ({}): {e}",
                outdir.display()
            ));
        }

        self.current_book = Book::new();

        self.session_logs.push(format!(
            "[Reset]\n> Book fields cleared and output cleaned (removed {}, {} failures).",
            removed, failed
        ));

        self.busy = false;
    }

    fn take_photo(&mut self, page: &Page) -> PhotoResult {
        use std::{fs, path::Path, process::Command};

        self.busy = true;

        self.pause_preview_thread();

        std::thread::sleep(Duration::from_millis(300));

        std::thread::sleep(Duration::from_millis(700));

        let dest_path = Path::new(&page.file_path);
        if let Some(parent) = dest_path.parent() {
            if let Err(e) = fs::create_dir_all(parent) {
                self.session_logs.push(format!(
                    "[Photo]\n> Failed to create parent dirs for {}: {e}",
                    dest_path.display()
                ));
                self.resume_preview_thread();
                self.busy = false;
                return PhotoResult::WriteFail;
            }
        }

        self.session_logs.push(format!(
            "[Photo]\n> Capturing to {} using camera='{}' port='{}'",
            dest_path.display(),
            self.camera,
            self.port
        ));

        let output = Command::new("gphoto2")
            .args(&[
                "--camera",
                &self.camera,
                "--port",
                &self.port,
                "--capture-image-and-download",
                "--force-overwrite",
                "--filename",
                &format!("{}", dest_path.display()),
            ])
            .output();

        std::thread::sleep(Duration::from_millis(150));

        let result = match output {
            Ok(out) if out.status.success() => {
                self.session_logs.push(format!(
                    "[Photo]\n> Downloaded image to {}",
                    dest_path.display()
                ));
                PhotoResult::Success
            }
            Ok(out) => {
                let stderr = String::from_utf8_lossy(&out.stderr).to_string();
                let stderr_l = stderr.to_lowercase();
                self.session_logs.push(format!(
                    "[Photo]\n> gphoto2 failed (exit {:?}).\n --- \n stderr:\n{}\n --- \n",
                    out.status.code(),
                    stderr.trim()
                ));

                if stderr_l.contains("no camera")
                    || stderr_l.contains("no device")
                    || stderr_l.contains("not found")
                {
                    PhotoResult::NoCamera
                } else if stderr_l.contains("permission")
                    || stderr_l.contains("read-only")
                    || stderr_l.contains("no such file")
                    || stderr_l.contains("is a directory")
                {
                    PhotoResult::WriteFail
                } else {
                    PhotoResult::Unknown
                }
            }
            Err(e) => {
                self.session_logs
                    .push(format!("[Photo]\n> Failed to spawn gphoto2: {e}"));
                PhotoResult::Unknown
            }
        };

        self.resume_preview_thread();
        self.busy = false;
        result
    }

    fn start_preview_thread(&mut self) {
        if self.preview_thread.is_some() {
            return;
        }
        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(2);
        let (cap_tx, cap_rx) = mpsc::sync_channel::<Capture>(1);

        let run = Arc::new(AtomicBool::new(true));
        let run_flag = run.clone();

        let pause = Arc::new(AtomicBool::new(false));
        let pause_flag = pause.clone();

        let camera = self.camera.clone();
        let port = self.port.clone();

        let handle = std::thread::Builder::new()
            .name(format!("{}-preview", PROG_NAME).into())
            .spawn(move || {
                use std::io::Read;
                use std::process::{Command, Stdio};
                use std::time::{Duration, Instant};

                fn spawn_child(camera: &str, port: &str) -> std::io::Result<std::process::Child> {
                    Command::new("gphoto2")
                        .args(&[
                            "--camera",
                            camera,
                            "--port",
                            port,
                            "--capture-preview",
                            "--stdout",
                        ])
                        .stdout(Stdio::piped())
                        .stderr(Stdio::null())
                        .spawn()
                }

                let mut last_settings = Instant::now() - Duration::from_secs(10);
                let mut child: Option<std::process::Child> = None;
                let mut window: Vec<u8> = Vec::with_capacity(2 * 1024 * 1024);
                let mut buf = vec![0u8; 256 * 1024];
                let mut last_emit = Instant::now();

                loop {
                    if !run_flag.load(Ordering::Relaxed) {
                        if let Some(mut c) = child.take() {
                            let _ = c.kill();
                            let _ = c.wait();
                        }
                        break;
                    }

                    if pause_flag.load(Ordering::Relaxed) {
                        if let Some(mut c) = child.take() {
                            let _ = c.kill();
                            let _ = c.wait();
                        }
                        std::thread::sleep(PREVIEW_DT);
                        continue;
                    }

                    if child.is_none() {
                        match spawn_child(&camera, &port) {
                            Ok(c) => child = Some(c),
                            Err(_) => {
                                std::thread::sleep(Duration::from_millis(200));
                                continue;
                            }
                        }
                        window.clear();
                    }

                    if last_settings.elapsed() >= Duration::from_millis(1600) {
                        let cap = capture_from_cli(&camera, &port);
                        let _ = cap_tx.try_send(cap);
                        last_settings = Instant::now();
                    }

                    let need_sleep = if let Some(c) = child.as_mut() {
                        if let Some(out) = c.stdout.as_mut() {
                            match out.read(&mut buf) {
                                Ok(0) => {
                                    if let Some(mut c) = child.take() {
                                        let _ = c.kill();
                                        let _ = c.wait();
                                    }
                                    true
                                }
                                Ok(n) => {
                                    window.extend_from_slice(&buf[..n]);

                                    let mut emitted = false;
                                    loop {
                                        let soi = window.windows(2).position(|w| w == [0xFF, 0xD8]);
                                        let eoi = window.windows(2).position(|w| w == [0xFF, 0xD9]);
                                        match (soi, eoi) {
                                            (Some(s), Some(e)) if e > s => {
                                                let end = e + 2;
                                                let frame = window[s..end].to_vec();
                                                window.drain(..end);

                                                if last_emit.elapsed() >= PREVIEW_DT {
                                                    let _ = tx.try_send(frame);
                                                    last_emit = Instant::now();
                                                    emitted = true;
                                                }
                                            }
                                            _ => break,
                                        }
                                    }
                                    !emitted
                                }
                                Err(_) => {
                                    if let Some(mut c) = child.take() {
                                        let _ = c.kill();
                                        let _ = c.wait();
                                    }
                                    true
                                }
                            }
                        } else {
                            true
                        }
                    } else {
                        true
                    };

                    if need_sleep {
                        std::thread::sleep(PREVIEW_DT);
                    }
                }
            })
            .expect("failed to spawn preview thread");

        self.preview_rx = Some(rx);
        self.cap_rx = Some(cap_rx);
        self.preview_run = Some(run);
        self.preview_pause = Some(pause);
        self.preview_thread = Some(handle);
        self.session_logs
            .push("[Preview]\n> Spawned streaming preview thread.".into());
    }

    fn stop_preview_thread(&mut self) {
        if let Some(flag) = &self.preview_run {
            flag.store(false, Ordering::Relaxed);
        }
        if let Some(h) = self.preview_thread.take() {
            let _ = h.join();
        }
        self.preview_run = None;
        self.preview_rx = None;
        self.preview_pause = None;
        self.session_logs
            .push("[Preview]\n> Stopped preview thread.".into());
    }

    fn pause_preview_thread(&mut self) {
        if let Some(pause) = &self.preview_pause {
            pause.store(true, Ordering::Relaxed);
            if let Some(rx) = &self.preview_rx {
                while rx.try_recv().is_ok() {}
            }
            self.session_logs
                .push("[Preview]\n> Paused preview.".into());
        }
    }

    fn resume_preview_thread(&mut self) {
        if let Some(pause) = &self.preview_pause {
            pause.store(false, Ordering::Relaxed);
            self.session_logs
                .push("[Preview]\n> Resumed preview.".into());
        }
    }

    fn load_jpeg_color_image_from_memory(&self, jpg: &[u8]) -> Option<eframe::egui::ColorImage> {
        self.load_jpeg_color_image_from_memory_scaled(jpg, self.preview_scale)
    }

    fn load_jpeg_color_image_from_memory_scaled(
        &self,
        jpg: &[u8],
        scale: PreviewScale,
    ) -> Option<eframe::egui::ColorImage> {
        use image::{self, imageops::FilterType, GenericImageView, ImageFormat};
        let mut img = image::load_from_memory_with_format(jpg, ImageFormat::Jpeg).ok()?;

        if !matches!(scale, PreviewScale::Full) {
            let (w, h) = img.dimensions();
            let (tw, th) = match scale {
                PreviewScale::Full => (w, h),
                PreviewScale::Half => (w / 2, h / 2),
                PreviewScale::Quarter => (w / 4, h / 4),
            };
            if tw.max(th) > 0 {
                img = img.resize_exact(tw.max(1), th.max(1), FilterType::Triangle);
            }
        }

        let rgba = img.to_rgba8();
        let size = [rgba.width() as usize, rgba.height() as usize];
        Some(eframe::egui::ColorImage::from_rgba_unmultiplied(
            size,
            rgba.as_flat_samples().as_slice(),
        ))
    }

    #[cfg(feature = "turbojpeg")]
    fn load_jpeg_color_image_from_memory_scaled(
        &self,
        jpg: &[u8],
        scale: PreviewScale,
    ) -> Option<eframe::egui::ColorImage> {
        use turbojpeg::{Decompressor, PixelFormat, ScalingFactor, Subsamp};
        let mut dec = Decompressor::new().ok()?;
        let header = dec.read_header(jpg).ok()?;
        let scale_factor = match scale {
            PreviewScale::Full => ScalingFactor::new(1, 1).unwrap(),
            PreviewScale::Half => ScalingFactor::new(1, 2).unwrap(),
            PreviewScale::Quarter => ScalingFactor::new(1, 4).unwrap(),
        };
        let (w, h) = header.scaled_size(scale_factor);
        let mut out = vec![0u8; (w * h * 4) as usize];
        dec.decompress(jpg, &mut out, w as usize, PixelFormat::RGBA)
            .ok()?;
        Some(eframe::egui::ColorImage::from_rgba_unmultiplied(
            [w as usize, h as usize],
            &out,
        ))
    }

    fn find_embedded_jpeg(&self, input: &[u8]) -> Option<Vec<u8>> {
        use image::{self, ImageFormat};

        if input.len() >= 3 && input[0] == 0xFF && input[1] == 0xD8 && input[2] == 0xFF {
            return Some(input.to_vec());
        }

        let scan_len = input.len().min(16 * 1024 * 1024);
        let mut i = 0usize;
        let mut best: Option<Vec<u8>> = None;

        // evil image byte magic
        while i + 1 < scan_len {
            // find SOI
            let slice = &input[i..scan_len];
            let Some(s_rel) = slice.windows(2).position(|w| w == [0xFF, 0xD8]) else {
                break;
            };
            let s = i + s_rel;

            // find EOI after SOI
            let rest = &input[s + 2..scan_len];
            let Some(e_rel) = rest.windows(2).position(|w| w == [0xFF, 0xD9]) else {
                break;
            };
            let e = s + 2 + e_rel + 2; // include EOI

            let cand = &input[s..e];
            if image::load_from_memory_with_format(cand, ImageFormat::Jpeg).is_ok() {
                if best.as_ref().map_or(true, |b| cand.len() > b.len()) {
                    best = Some(cand.to_vec());
                }
            }
            i = e;
        }
        best
    }

    fn load_color_image_from_disk(&self, path: &str) -> Option<eframe::egui::ColorImage> {
        use image::{self, ImageFormat};

        let bytes = std::fs::read(path).ok()?;

        let img = match image::ImageReader::new(std::io::Cursor::new(&bytes))
            .with_guessed_format()
            .ok()?
            .decode()
        {
            Ok(img) => img,
            Err(_) => {
                let jpg = self.find_embedded_jpeg(&bytes)?;
                image::load_from_memory_with_format(&jpg, ImageFormat::Jpeg).ok()?
            }
        };

        let rgba = img.to_rgba8();
        let size = [rgba.width() as usize, rgba.height() as usize];
        Some(eframe::egui::ColorImage::from_rgba_unmultiplied(
            size,
            rgba.as_flat_samples().as_slice(),
        ))
    }

    fn refresh_texture(&mut self, ctx: &Context, image: Option<&str>) {
        let maybe_ci = if image.is_none() {
            self.preview_bytes
                .as_deref()
                .and_then(|b| self.load_jpeg_color_image_from_memory(b))
        } else {
            self.load_color_image_from_disk(image.unwrap())
        };

        if let Some(color_image) = maybe_ci {
            if let Some(tex) = &mut self.texture {
                tex.set(color_image, egui::TextureOptions::default());
            } else {
                self.texture = Some(ctx.load_texture("preview", color_image, Default::default()));
            }
            if image.is_none() {
                self.preview_bytes = None;
            }
            self.frame_idx = self.frame_idx.wrapping_add(1);
        }
    }

    fn camera_info(&mut self, force: bool) {
        if force {
            #[cfg(target_os = "windows")]
            {
                // No Windows Support lol
                panic!();
            }

            #[cfg(target_os = "macos")]
            {
                // Common daemons that block libgphoto2 devices
                let _ = Command::new("pkill").args(&["-9", "-f", "-i", "'ptpcamerad|PTPCamera|mscamerad|cameracaptured|appleh13camerad|VDCAssistant|AppleCameraAssistant'"]).status().expect("Unable to Kill Process");
            }
            #[cfg(target_os = "linux")]
            {
                use std::{process::Command, thread, time::Duration};

                // Common daemons that block libgphoto2 devices
                const NAMES: &[&str] = &[
                    "gvfsd-gphoto2",
                    "gvfs-gphoto2-volume-monitor",
                    "gvfsd-mtp",  // sometimes pokes PTP devices too
                    "kio_camera", // KDE issue
                    "kio-gphoto",
                    "kioexec",     // can hold FDs via KIO
                    "kdeconnectd", // rare, but can touch MTP
                    "shotwell",    // photo managers that auto-open cameras
                    "gnome-photos",
                    "digikam",
                ];

                // graceful stop
                for n in NAMES {
                    let _ = Command::new("pkill").args(["-TERM", "-x", n]).status();
                }

                thread::sleep(Duration::from_millis(300));

                // insurance
                for n in NAMES {
                    let _ = Command::new("pkill").args(["-KILL", "-x", n]).status();
                }
            }
        }

        let (camera, port): (String, String) = if let Ok(context) = gphoto2::Context::new() {
            if let Ok(camera) = context.autodetect_camera().wait() {
                if let Ok(port_info) = camera.port_info() {
                    (camera.abilities().model().into(), port_info.path().into())
                } else {
                    (" ".into(), " ".into())
                }
            } else {
                (" ".into(), " ".into())
            }
        } else {
            (" ".into(), " ".into())
        };

        if &camera == " " {
            self.session_logs
                .push("[Camera Detection]\n> Unable to find a camera or port.".into())
        }

        self.camera = camera;
        self.port = port;
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        if self.preview {
            if self.peek_idx == 0 {
                if self.preview_thread.is_none() {
                    self.start_preview_thread();
                }
                // drain 1: latest frames
                if let Some(rx) = &self.preview_rx {
                    let mut latest: Option<Vec<u8>> = None;
                    while let Ok(bytes) = rx.try_recv() {
                        latest = Some(bytes);
                    }
                    if let Some(bytes) = latest {
                        self.preview_bytes = Some(bytes);
                    }
                }
                // drain 2: capture data
                {
                    let mut incoming: Vec<Capture> = Vec::new();
                    if let Some(rx) = &self.cap_rx {
                        while let Ok(c) = rx.try_recv() {
                            incoming.push(c);
                        }
                    }
                    for c in incoming {
                        self.merge_capture_settings(c);
                    }
                }
            } else if self.preview_thread.is_some() {
                self.stop_preview_thread();
            }
        } else if self.preview_thread.is_some() {
            self.stop_preview_thread();
        }

        // ui throttle
        if self.preview {
            if self.last_tick.elapsed() >= UI_DT {
                if self.peek_idx == 0 {
                    self.refresh_texture(ctx, None);
                } else {
                    let page_path_owned = {
                        if let Some(p) = self
                            .current_book
                            .pages
                            .iter()
                            .find(|p| p.idx == self.peek_idx)
                        {
                            p.file_path.clone()
                        } else {
                            format!(
                                "{}{}.{}",
                                self.output_path,
                                self.peek_idx,
                                self.output_type.extension()
                            )
                        }
                    };
                    self.refresh_texture(ctx, Some(&page_path_owned));
                }
                self.last_tick = Instant::now();
            }
            ctx.request_repaint_after(UI_DT);
        }

        enforce_16x9(ctx);

        self.maybe_rebuild_log_cache();

        egui::CentralPanel::default()
            .frame(Frame::NONE)
            .show(ctx, |ui| {
                let base_w = 1920.0;
                let base_h = 1080.0;
                let screen = ui.max_rect();
                let kx = screen.width() / base_w;
                let ky = screen.height() / base_h;
                let k = kx.min(ky);
                let pad = (base_w / 64.0) * k;
                let gutter = pad;
                let content = screen.shrink2(Vec2::splat(pad));
                let left_w = 1220.0 * k;
                let right_w = 610.0 * k;
                let top_h = 660.0 * k;
                let bot_h = 330.0 * k;
                let left_origin = content.min;

                let r_preview = Rect::from_min_size(left_origin, Vec2::new(left_w, top_h));
                let inner_preview =
                    Rect::from_center_size(r_preview.center(), Vec2::new(1152.0 * k, 648.0 * k));
                let y_bottom = r_preview.bottom() + gutter;
                let r_logs = Rect::from_min_size(
                    Pos2::new(left_origin.x, y_bottom),
                    Vec2::new(695.0 * k, bot_h),
                );
                let r_camset = Rect::from_min_size(
                    Pos2::new(r_logs.right() + gutter, y_bottom),
                    Vec2::new(495.0 * k, bot_h),
                );
                let r_sidebar = Rect::from_min_size(
                    Pos2::new(left_origin.x + left_w + gutter, left_origin.y),
                    Vec2::new(right_w, 1020.0 * k),
                );

                let p = ui.painter();
                let stroke = Stroke::new(2.0, Color32::from_gray(80));
                draw_box(&p, r_preview, stroke, "");
                draw_box(&p, r_logs, stroke, "");
                draw_box(&p, r_camset, stroke, "");
                draw_box(&p, r_sidebar, stroke, "");

                p.rect_stroke(
                    inner_preview,
                    0.0,
                    Stroke::new(0.0, Color32::GRAY),
                    egui::StrokeKind::Middle,
                );

                // preview
                ui.allocate_ui_at_rect(inner_preview, |ui| {
                    ui.centered_and_justified(|ui| {
                        if self.preview {
                            if let Some(tex) = &self.texture {
                                ui.add(
                                    egui::widgets::Image::new(tex)
                                        .fit_to_exact_size(inner_preview.size()),
                                );
                            } else {
                                ui.label("Waiting for preview");
                            }
                        } else {
                            ui.label("Preview Stopped");
                        }
                    });
                });

                // controls/logs
                ui.allocate_ui_at_rect(r_logs, |ui| {
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.separator();
                        if ui.button("Clear Logs").clicked() {
                            self.session_logs.clear();
                            self.log_cache.clear();
                            self.log_cached_count = 0;
                            self.log_last_rebuild = Instant::now();
                        }
                        ui.separator();
                    });
                    ui.separator();
                    egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                        egui::CollapsingHeader::new("Logs")
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.style_mut().override_font_id = Some(egui::FontId::monospace(12.0));
                                // One label, one layout pass:
                                ui.add(egui::Label::new(&self.log_cache).wrap());
                            });
                    });
                });

                // settings, capture config, & camera toggle
                ui.allocate_ui_at_rect(r_camset, |ui| {
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.separator();
                        if ui.button("Toggle Preview").clicked() {
                            self.peek_idx = 0;
                            if !self.preview {
                                self.camera_info(false);
                            }

                            if self.camera != " " {
                                self.capture_settings = Capture::new();
                                self.preview = !self.preview;

                                if self.preview {
                                    self.start_preview_thread();
                                } else {
                                    self.stop_preview_thread();
                                    std::thread::sleep(Duration::from_millis(150));
                                    self.camera_info(false);
                                }
                            } else {
                                self.session_logs
                                    .push("[Preview Toggle]\n> Unable to start preview.".into())
                            }
                        }
                        ui.separator();

                        if self.preview == true {
                            if self.peek_idx == 0 {
                                ui.label(format!("Live Feed · 1920x1080 · ~{} fps", PREVIEW_FPS));
                            } else {
                                ui.label(format!("Viewing Scan #{}", self.peek_idx));
                            }
                        }
                    });
                    ui.separator();

                    egui::ScrollArea::vertical().id_salt("Settings").show(ui, |ui| {
                        egui::CollapsingHeader::new("Capture Settings")
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.label(format!(
                                    "Camera: {}",
                                    self.camera
                                ));
                                ui.label(format!(
                                    "Port: {}",
                                    self.port
                                ));
                                ui.separator();
                                ui.label(format!(
                                    "Shutter Speed: {}",
                                    self.capture_settings.shutter_speed
                                ));
                                ui.label(format!("ISO: {}", self.capture_settings.iso));
                                ui.label(format!(
                                    "f-Number (iris): {}",
                                    self.capture_settings.f_number
                                ));
                                ui.label(format!(
                                    "White Balance: {}",
                                    self.capture_settings.white_balance
                                ));
                                ui.label(format!(
                                    "Color Temp: {} ºK",
                                    self.capture_settings.color_temperature
                                ));
                                ui.label(format!(
                                    "Exposure Compensation: {}",
                                    self.capture_settings.exposure_compensation
                                ));
                            });

                        egui::CollapsingHeader::new("More Settings")
                            .default_open(false)
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("Use with Caution:");
                                    ui.style_mut().visuals.widgets.hovered.weak_bg_fill = Color32::RED;

                                    let is_preview_unpaused = self
                                        .preview_run
                                        .as_ref()
                                        .map(|run| run.load(Ordering::Relaxed))
                                        .unwrap_or(false)
                                        &&
                                        self
                                            .preview_pause
                                            .as_ref()
                                            .map(|p| !p.load(Ordering::Relaxed))
                                            .unwrap_or(false);

                                    let btn = ui.add_enabled(!is_preview_unpaused, egui::Button::new("Force Detect Camera"));
                                    if btn.clicked() {
                                            self.camera_info(true);
                                            if self.camera != " " {
                                                self.session_logs.push(
                                                    format!(
                                                        "[Auto-Detection Report]\n> Camera: {}\n    Port: {}",
                                                        self.camera, self.port
                                                    )
                                                    .into(),
                                                );
                                            }
                                        }
                                    ui.style_mut().visuals.widgets.hovered.weak_bg_fill =
                                        Color32::DARK_GRAY;
                                });
                                ui.label("* Note the above kills any existing camera processes & refreshes drivers.\n* May cause unintended behaviour if other processes are running on the same machine.\n* Use only as a last resort & with no other camera-related processes live.\n");
                                ui.horizontal(|ui| {
                                    ui.label("Preview quality:");
                                    egui::ComboBox::from_id_salt("preview_scale_combo")
                                        .selected_text(self.preview_scale.label())
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.preview_scale, PreviewScale::Full,    PreviewScale::Full.label());
                                            ui.selectable_value(&mut self.preview_scale, PreviewScale::Half,    PreviewScale::Half.label());
                                            ui.selectable_value(&mut self.preview_scale, PreviewScale::Quarter, PreviewScale::Quarter.label());
                                        });
                                });
                                ui.label(" ");
                            });
                    });
                });

                ui.allocate_ui_at_rect(r_sidebar, |ui| {
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.separator();
                        ui.heading(format!("{} - {}", PROG_NAME, PROG_VERSION));
                        ui.separator();
                        if ui.button("Exit Program").clicked() {
                            self.destroy_book();
                            self.scanning = false;
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.separator();
                        if self.scanning {
                            if ui.button("Export Book as ZIP").clicked() {
                                self.session_logs
                                    .push("[Export Service]\n> Beginning Export process".into());
                                self.export_book();
                                self.session_logs.push("[Export Service]\n> Finished Export process".into());
                            }
                            ui.separator();
                            ui.label(format!("Type: {}", self.output_type));
                            ui.separator();
                            ui.style_mut().visuals.widgets.hovered.weak_bg_fill = Color32::RED;
                            if ui.button("Trash Book").clicked() {
                                self.peek_idx = 0;
                                self.destroy_book();
                                self.scanning = false;
                            }
                            ui.style_mut().visuals.widgets.hovered.weak_bg_fill =
                                Color32::DARK_GRAY;
                        } else {
                            if ui.button("New Book").clicked() {
                                self.destroy_book();
                                self.scanning = true;
                            }
                            ui.separator();
                            ui.label("File Type:");
                            egui::ComboBox::from_id_salt("output_type_combo")
                                        .selected_text(self.output_type.to_string())
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.output_type, PhotoType::ARW,  "ARW");
                                            ui.selectable_value(&mut self.output_type, PhotoType::CR2,  "CR2");
                                            ui.selectable_value(&mut self.output_type, PhotoType::TIFF, "TIFF");
                                            ui.selectable_value(&mut self.output_type, PhotoType::JPEG, "JPEG");
                                            ui.selectable_value(&mut self.output_type, PhotoType::PNG,  "PNG");
                                        });
                            ui.separator();
                        }
                    });

                    ui.separator();
                    ui.vertical(|ui| {
                        if self.scanning {
                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Book Common Name");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.common_name,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Edition");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.edition,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Publisher");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.publisher,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Publication Date");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.publication_date,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Publication Location");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.publication_place,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("ISBN");
                                        ui.text_edit_singleline(&mut self.current_book.isbn)
                                            .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label("Primary Language");
                                        ui.text_edit_singleline(
                                            &mut self.current_book.language_primary,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label(
                                            "All Languages (Comma delineated for multiple)",
                                        );
                                        ui.text_edit_singleline(
                                            &mut self.current_book.language,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.vertical(|ui| {
                                ui.horizontal(|ui| {
                                    ui.separator();
                                    ui.vertical(|ui| {
                                        ui.label(
                                            "All Authors (Comma delineated for multiple)",
                                        );
                                        ui.text_edit_singleline(
                                            &mut self.current_book.authors,
                                        )
                                        .highlight();
                                    });
                                });
                            });

                            ui.separator();
                            ui.horizontal(|ui| {
                                ui.separator();
                                if ui.button("Take Photo").clicked() {
                                    let new_page = Page::new(
                                        (self.current_book.pages.len() + 1) as u64,
                                        self.output_path.clone(),
                                        self.output_type.extension(),
                                    );

                                    match self.take_photo(&new_page) {
                                        PhotoResult::Success => {
                                            self.current_book.pages.push(new_page);
                                        }
                                        PhotoResult::NoCamera => {
                                            self.session_logs.push("No camera detected. Check USB, power, and MTP/PTP mode.".into());
                                        }
                                        PhotoResult::WriteFail => {
                                            self.session_logs.push("Could not write photo to disk (permissions/path).".into());
                                        }
                                        PhotoResult::Unknown => {
                                            self.session_logs.push("Capture failed for an unknown reason.".into());
                                        }
                                    }
                                }
                                ui.separator();
                            });
                            ui.separator();
                            ui.horizontal(|ui| {
                                ui.separator();
                                ui.label(format!(
                                    "Next Page Index: {}",
                                    self.current_book.pages.len() + 1
                                ));
                                ui.separator();
                                ui.label(format!(
                                    "Already Scanned {} pages",
                                    self.current_book.pages.len()
                                ));
                                ui.separator();
                            });

                            ui.separator();

                            egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                                ui.collapsing("Captured Pages (Click on Name to Preview)", |ui| {
                                    ui.vertical(|ui| {
                                        ui.selectable_value(&mut self.peek_idx, 0, "View Live Preview");

                                        let mut want_retake: Option<u64> = None;
                                        let mut want_remove: Option<u64> = None;

                                        for page in &self.current_book.pages {
                                            ui.horizontal(|ui| {
                                                ui.selectable_value(
                                                    &mut self.peek_idx,
                                                    page.idx,
                                                    format!("Scan Image #{}", page.idx),
                                                );

                                                if ui.button("Retake").clicked() {
                                                    want_retake = Some(page.idx);
                                                }
                                                if ui.button("Remove").clicked() {
                                                    want_remove = Some(page.idx);
                                                }
                                            });
                                        }

                                        if let Some(idx) = want_retake {
                                            self.retake_page(idx);
                                        }
                                        if let Some(idx) = want_remove {
                                            self.delete_page(idx);
                                        }
                                    });
                                });
                            });
                        }
                    });
                });
            });
    }
}
