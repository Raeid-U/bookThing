# bookThing

High-throughput camera capture for public archival projects.
A desktop GUI built with egui/eframe that drives a `gphoto2` compatible camera to digitize books, documents, and records. bookThing captures pages with live preview, supports bulk review and corrections, and exports a portable ZIP that includes page images in a requested format and a structured manifest.

## Highlights

* Live preview and live video stream from the camera using `libgphoto2`
* Multithreaded design

  * Dedicated preview thread for smooth video and low UI latency
  * Background capture-settings polling without blocking preview
  * Background export worker for large batch conversion and zipping
* Bulk image processing at export time with on-the-fly format conversion
* Large data handling

  * Streams frames into a ring buffer to avoid stalls
  * Stages page files and builds the final archive out of process
* Robust USB connection handling for Unix-like systems

  * Force detect action that stops OS services known to reserve cameras
  * Pause and resume of the preview thread around critical camera operations
* Live review and curation

  * Retake, remove, and renumber pages
  * Immediate preview of any captured page

## Long Form Demo video

https://github.com/user-attachments/assets/7015a1dc-a446-48b0-a50f-3e7b9d03ef35

## What bookThing produces

Exports a ZIP in the user Downloads folder:

```
<YYYYMMDD-HHMMSS>-<common_name>.zip
├── manifest.json
├── export_log.txt
├── 1.<EXT>
├── 2.<EXT>
└── ...
```

* `manifest.json` contains bibliographic fields, capture timestamp, current capture settings, and the ordered list of pages with normalized file names.
* `export_log.txt` records per page conversion notes and any failures.

## Supported platforms

* macOS and Linux.
* Windows is not supported

  * (neither wsl passthrough or Windows compile target)

## Cameras and formats

* Any camera that works with libgphoto2 and supports preview and remote capture over USB.
* Export targets selectable in the UI: ARW, CR2, TIFF, JPEG, PNG.
* Source file type detection is automatic. RAW containers are detected by signature checks and light heuristics.

### Official support

* Sony Alpha series. The Alpha A6100 and A7 III are the primary development cameras and are fully supported.
* Canon EOS support is currently in testing.
* Other libgphoto2-compatible bodies may work but are **not** guaranteed.

### Export conversion behavior

* When the target format matches the source RAW (ARW to ARW or CR2 to CR2) bookThing passes the file through without re-encoding.
* For JPEG target

  * If the source is already JPEG bookThing copies bytes to preserve metadata.
  * If the source is RAW bookThing prefers the embedded JPEG preview when present.
  * Otherwise it decodes and re-encodes as JPEG.
* For PNG and TIFF targets bookThing decodes and re-encodes from the best available image representation.
* When the requested target is a different proprietary RAW (for example CR2 target from ARW source) bookThing exports a TIFF instead and records the substitution in `export_log.txt`.

## Architecture

* **Main UI thread**
  egui draws the interface, drains new preview frames, applies user actions, and shows logs.
* **Preview thread**
  Calls `gphoto2 --capture-preview --stdout` to get a continuous preview stream and sends frames over a bounded channel. This thread also performs periodic capture-settings polling using `gphoto2 --get-config` and sends updates separately so the UI can reflect ISO, f-number, white balance, and related values without touching the camera from the UI thread.
* **Export worker**
  Spawns a background thread that

  1. converts each page to the selected target format,
  2. writes `manifest.json` and `export_log.txt`, and
  3. builds the ZIP with all artifacts.
     The UI stays responsive during long exports.

Critical camera operations such as taking a photo pause the preview thread, wait briefly to release the driver, perform the capture and download, then resume preview.

## Features

* Camera autodetect using libgphoto2
* Force detect action for macOS and Linux that terminates known OS services that keep the camera device busy
* Live preview with periodic capture-settings polling
* One click capture that writes to a stable working directory
* Page list with Live Preview, Retake, Remove, and direct viewing of any page
* Structured export to a ZIP with a normalized manifest
* Comma separated input for Authors and Languages is parsed into arrays in the manifest
* File name and text field sanitization for portability

## Manifest schema

Example:

```json
{
  "common_name": "my_book",
  "publication_date": "2025",
  "publication_place": "City",
  "authors": ["Author_One", "Author_Two"],
  "publisher": "Publisher",
  "edition": "1",
  "isbn": "9780123456789",
  "language_primary": "en",
  "language": ["en", "fr"],
  "captured_at": "2025-01-10 14:22:05",
  "capture_settings": {
    "iso": "400",
    "f_number": "f/4.0",
    "color_temperature": "0K",
    "white_balance": "Auto",
    "exposure_compensation": "0",
    "shutter_speed": "1/60"
  },
  "pages": [
    { "idx": 1, "file_path": "1.TIFF" },
    { "idx": 2, "file_path": "2.TIFF" }
  ],
  "export_target": "TIFF"
}
```

Notes:

* Authors and language lists are derived from comma separated inputs and normalized to file system safe tokens.
* `captured_at` is set when a new book is created.
* `capture_settings` reflect the latest values observed during the session.

## System requirements

* Unix-like OS with libgphoto2 and the `gphoto2` CLI in `PATH`
* Rust toolchain
* Minimum hardware target is 2 cores and 4 threads
* USB connection to a camera that supports PTP or similar remote control

## Install

### macOS

```bash
xcode-select --install   # if not already installed
brew install gphoto2 pkg-config
rustup update stable
git clone https://github.com/Raeid-U/bookThing
cd bookThing
cargo build --release
```

### Debian or Ubuntu

```bash
sudo apt-get update
sudo apt-get install -y \
  gphoto2 libgphoto2-dev pkg-config build-essential \
  clang libclang-dev llvm-dev cmake \
  libudev-dev \
  libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev \
  libxkbcommon-dev libwayland-dev
rustup update stable
git clone https://github.com/Raeid-U/bookThing
cd bookThing
cargo build --release
```

### Run

```bash
# ran from the command line via:
cargo run --release
# or
./target/release/bookThing
```


## Usage

1. Connect the camera over USB and set it to PC Remote or PTP mode.
2. Click Force Detect Camera on macOS or Linux if the camera is busy. Then click Toggle Preview.
3. Choose a File Type for the working files and export target.
4. Enter bibliographic metadata.
5. Click Take Photo per page. Use Retake or Remove for corrections.
6. Click Export Book as ZIP to build the archive in Downloads. The export runs in the background and writes a log with conversion details.

## Working directories

* Working images
  `<tmp>/bookThing/output/` with 1-based page indices and the currently selected extension
* Export staging
  `<tmp>/export_<YYYYMMDD-HHMMSS>_<common_name>/`
* Final ZIP
  `~/Downloads/<YYYYMMDD-HHMMSS>-<common_name>.zip`

`<tmp>` is the OS temporary directory.

## Performance tips

* Keep preview running only when needed. Stopping preview releases the camera and reduces CPU load.
* Leave the app in release mode for scanning sessions. Debug builds may exhibit reduced frame rate.
* Use a USB cable and port known to be stable at sustained transfer rates.

## Resource usage (typical)

Measured with a release build:

* Idle with preview off: ~180–190 MB RSS
* Live preview on: ~190 MB RSS
* Exporting ~20 ARW files: peaks at ~300 MB RSS, then returns to ~180–190 MB when complete

Exports are streamed per page and memory returns to baseline after buffers are dropped. Peak memory will scale with the largest decoded frame and the chosen export format (TIFF and PNG briefly allocate more during re-encode).

## Minimum and recommended hardware

### Minimum (tested)

* CPU: 2 cores, 4 threads
* RAM: 4 GB
* Storage: 10+ GB free SSD or fast HDD
* USB: Stable USB 2.0 or better

* Notes: On a Raspberry Pi 4B (4 GB) the UI is usable with minor ghosting; live preview typically 4–8 fps.

### Recommended

* CPU: Apple M-series or modern 4+ core CPU
* RAM: 8–16 GB
* Storage: SSD with sustained write throughput (NVMe preferred)
* USB: USB 3.x port and quality cable

* Notes: On a MacBook Air M1 (16 GB) the UI is smooth and live preview is ~16 fps.

### Throughput guidance

* Export runs in a background thread. More cores reduce end-to-end export time but preview and capture remain responsive on 2C/4T systems.
* Peak RAM during export scales with decoded image size and target format. TIFF and PNG have larger transient allocations than JPEG or pass-through RAW.
* Plan storage for RAW workflows: a 24 MB page × 500 pages is ~12 GB before derivatives.

## Troubleshooting camera access

* If the camera is detected but preview fails
  Use Force Detect Camera which terminates common services that reserve PTP devices. Then toggle preview again.
* If capture hangs or fails intermittently
  Stop preview, wait a moment, then try capture again. If the camera remains busy, power cycle the camera and reconnect USB. Restarting the program refreshes libgphoto2 state.

## Known problems

* CR2 image output with Sony Cameras A6X00 series is slow and may hang indefinitely. A program restart and camera reconnection is required.
* OS specific processes may reserve the camera over USB. To free on macOS or Linux use the Force Detect Camera button in Global Settings, then start Preview to lock the device in the preview thread.
* Stale drivers may cause performance issues. Restart libgphoto2 and gphoto2 by restarting the program and reconnecting the USB camera.
* Serial port support is limited for older cameras. The workflow targets camera over USB.
* The UI may stall in development builds. Use an optimized release build for scanning sessions.

## Security and rights

Ensure you have the right to digitize and distribute each work. The manifest includes operator supplied metadata and capture settings. Respect all applicable laws and collection policies.

## License

bookThing is licensed under the `MIT License`. See `LICENSE` in the repository.
