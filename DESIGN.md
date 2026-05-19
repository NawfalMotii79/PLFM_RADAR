Radar System GUI — Visual Design Description
Overall Appearance

A full-screen, dark-themed tactical radar interface that looks like military/aerospace ground station software. The entire window is pitch black (#1a1a1a background) with emerald green (#10b981) as the dominant accent color. Every single corner in the UI is perfectly sharp — zero rounded corners anywhere. There are no shadows, no cards, no floating elements. The entire layout is a dense grid of panels butted directly against each other, separated only by 1-pixel dark gray (#2a2a2a) borders. There is zero whitespace or padding between major sections — panels tile edge-to-edge like a cockpit instrument cluster. The overall density is very high — this is a professional tool, not a consumer app.

Two fonts are used throughout: Rajdhani (a geometric semi-condensed sans-serif) for UI labels and headings, and JetBrains Mono (monospaced) for all data, numbers, coordinates, and technical readouts. Almost all text is very small (9px–12px) and rendered in UPPERCASE with wide letter-spacing, giving it a military stencil feel. The text color hierarchy is: bright white (#f5f5f5) for active/important text, medium gray (#a1a1aa) for labels and secondary info, and green/yellow/red for status-coded values.
Top: Tab Navigation Bar

A thin horizontal bar (~36px tall) spanning the full width, sitting at the very top. Background is the same dark panel color. Five tab labels sit in a row, left-aligned with small horizontal padding: MAIN VIEW, MAP VIEW, DIAGNOSTICS, SETTINGS, RAW INPUTS. Each label is 11px, uppercase, widely letter-spaced, in gray. The currently active tab has white text and a 2-pixel-tall bright green underline directly below it, plus a very subtle lighter background tint. The whole bar has a 1px border on its bottom edge.
Tab 1: MAIN VIEW
Control Panel (top ribbon, ~40px tall)

A horizontal toolbar stretching full width, dark panel background, 1px border on bottom. Contents flow left to right in a single row:

    Gray label "STM32" (11px, uppercase) followed by a small dark dropdown/combobox showing device names like "STM32 CDC (COM3)". The dropdown has a 1px border, square corners, dark background, monospaced text.
    A thin 1px-wide vertical gray line (16px tall) as a separator.
    Gray label "FT601" followed by another identical dropdown showing "FT601 USB 3.0 \0".
    Another vertical separator.
    A small square checkbox (14×14px, accent green when checked) with label "BURST MODE" in 11px gray uppercase.
    A small gray button labeled "REFRESH" with a tiny refresh/rotate icon (12px), uppercase, square corners, muted gray background.
    A large flexible gap pushing everything after it to the right.
    A status readout: a tiny diamond shape (◆, ~8px, bright green, gently pulsing) followed by monospaced green text reading something like RUNNING · PKTS: 12,345 · PITCH: +2.1°. When idle, it just says READY in dim gray with a static gray diamond.
    Another vertical separator.
    A prominent action button (~110px wide): when inactive, it has a solid bright green (#10b981) background with black text reading "Start Radar" with a small antenna/radio icon. When active, it flips to solid red (#f87171) background with white text reading "Stop Radar" with a small square stop icon. Square corners, no border radius.

GPS / Pitch Ribbon (second ribbon, ~32px tall)

Another full-width horizontal bar directly below the control panel, separated by a 1px border. Same dark panel background. Contents:

    A small satellite icon (14px) colored green if GPS is locked, yellow if not.
    Text "GPS" in 10px monospaced uppercase with extra letter-spacing, gray color.
    A tiny rectangular badge: bright green background with black text "LOCK" (9px) — or yellow background with "NO FIX".
    Vertical separator.
    A row of coordinate readouts in monospace: LAT (10px gray label) 37.774929 (12px white value), a gray dot separator ·, LON -122.419416, ·, ALT 42.3m. When radar is off, this area just shows "Waiting for GPS…" in gray monospace.
    Vertical separator.
    Pitch section: gray label "PITCH" (10px uppercase), then the value +2.1° in 14px monospace colored green (if <5°), yellow (if 5–12°), or red (if >12°). Next to it, a tiny horizontal bar (64px wide, 6px tall) with a dark green-tinted background track. A colored fill extends from the center outward proportional to the pitch angle. A tiny 1px center tick mark divides it. After the bar, a tiny status word: "LEVEL", "TILT", or "WARN" in 9px gray. If pitch exceeds 12°, a small yellow/red warning triangle icon appears.
    Flexible spacer.
    Right-aligned timestamp: T 14:23:05 in 10px gray monospace.

Main Content Area (fills remaining height, split horizontally)

The space below the two ribbons is divided into two panels side by side with a 1px vertical border between them.

Left panel (Range-Doppler Map, ~67% width):

Has its own thin header bar at the top: left side shows "Range-Doppler Map" in 11px gray uppercase with wide letter-spacing. Right side shows "10 GHz · PRF1=1000 Hz" in 10px gray monospace (only when active), a 1px vertical separator, a small square indicator (8×8px, green and pulsing when active, gray when standby), and text "ACTIVE" or "STANDBY" in 10px gray monospace.

Below the header, the entire area is a rendered canvas:

    When active: A full-color heatmap using the "jet" colormap (blue → cyan → green → yellow → red). The background noise floor appears as dark blue speckle. Three bright target blobs glow in warm colors (yellow/red) at different positions, slowly drifting. Over this, a semi-transparent green grid overlay divides the space into ~6 columns and ~5 rows with thin green lines at 25% opacity. Axis labels in white monospace: "Range (m) →" at bottom-left, "Velocity (m/s)" rotated vertically on the left edge. Tick labels along the bottom ("10 km", "20 km"...) and left side ("-25 m/s", "0 m/s", "+25 m/s"...) in faded white. Around each target blob, a green rectangular bracket with corner tick marks (like a targeting reticle). Next to each bracket, a label: "TGT-1" in 11px green monospace, with "125m +12.4 m/s" below it in 10px faded white.
    When standby: The canvas is solid dark (#1a1a1a) with a faint green grid (lines every 80px, ~20% opacity). Faint green crosshair lines through the center (horizontal and vertical, full span, ~15% opacity). Centered text in 14px green monospace at ~45% opacity: "RANGE-DOPPLER MAP — STANDBY", with "Start radar to begin acquisition" in 11px white at 25% opacity below it.

Right panel (Targets Table, fixed ~320px width):

Has its own thin header bar: left shows "Detected Targets" in 10px gray monospace uppercase. Right side shows a small green badge "CFAR 3/5" (green background, black text, 10px) and "5 TGT" in gray monospace.

Below is a compact data table filling the available height. Column headers are tiny (9px), gray, uppercase, right-aligned, on a sticky dark background. The columns are very narrow and tightly packed: CFAR (centered diamond symbols), TRK (green track IDs like "1003"), RNG (range numbers), VEL (velocity with +/- signs, colored yellow/red for high speeds), AZ (gray), EL (gray), CEL° (corrected elevation), SNR (colored: green ≥25dB, yellow ≥15dB, red <15dB), CHP (gray chirp counter). Each row is only 20px tall with 11px monospaced text. Rows have faint bottom borders. Hovering a row gives it a very faint green tint. Clicking selects it with a brighter green tint.

When empty, the table shows a centered dark square outline containing a pulsing green square, with "SCANNING..." or "STANDBY" below it.

When a row is selected, a detail panel appears below the table: a dark slightly-green-tinted area showing "TRK 1003 · Detail" with a CFAR badge, then a 2-column grid of label-value pairs (Range, Velocity, Azimuth, etc.) in 10px monospace — labels gray, values green.

At the very bottom of the right panel, a thin footer bar shows: "AVG 234m" | "MAX SNR 28.4dB" (colored) | "PKT 12,345" — all in 10px gray monospace, separated by the panel width.
Tab 2: MAP VIEW
Main Area (full-height canvas with overlays)

The entire tab is dominated by a large canvas with a dark background. Layered on it:

    Grid pattern: Very faint green lines forming a grid — minor lines every 25px at ~6% opacity, major lines every 100px at ~16% opacity. Gives a subtle graph-paper look.

    Crosshairs: A horizontal and vertical line passing through the exact center, spanning the full width/height, green at 10% opacity.

    Coverage rings: Three concentric squares (not circles) centered in the view. Inner square (~250px), medium square (~500px, fainter), outer square (~800px, dashed, very faint). All green outlines at decreasing opacity.

    Rotating sweep: A bright green line extending from the center point to the right edge (and beyond, ~6000px long), continuously rotating clockwise completing one full revolution every 5 seconds. The line has a gradient: bright green (95% opacity) at the center fading to transparent at the tip. Behind the line, trailing about 70°, is a faint green conic gradient "glow" that looks like a radar sweep trail — brightest just behind the line, fading to transparent. The line casts a subtle green glow/shadow (box-shadow effect). The sweep line and trail are part of a single rotating container so they stay perfectly synchronized.

    Target markers: Three diamond-shaped markers at different offsets from center. Each diamond is 16×16px, filled solid with a color based on its SNR: green (≥25dB), yellow (15–25dB), or red (<15dB). Each has a pulsing outer ring (28×28px, same color, animating like a ping/ripple). Adjacent to each diamond is a label box: opaque near-black background (82% opacity) with a 1px colored border matching the target, containing two lines — "TGT-1" in 11px colored bold text, and "28.4 dB" in 10px light gray.

    Radar origin: At the exact center, a 20×20px square with a 2px green border and dark fill, containing a small navigation/compass arrow icon (10px, green). Surrounding it, a pulsing 28×28px green square outline that gently fades in and out.

    Corner overlays (floating on top of the canvas, anchored to corners):
        Top-left: A small info badge with 1px borders on its bottom and right edges, dark panel background. Contains: a globe icon (14px, green), "MAP VIEW" in 11px monospace uppercase, a 1px vertical separator, a tiny pulsing green square (6px), and "LIVE" in 10px gray monospace.
        Bottom-left: Scale reference — an 80px-wide horizontal green line with "500 m" label in 10px gray monospace.
        Bottom-right: Coordinate readout — "LAT" (gray) "37.774900" (green) and "LON" (gray) "-122.419400" (green) in 11px monospace.

Stats Bar (thin bar below the canvas)

Four equal-width cells spanning full width, separated by 1px vertical borders. Each cell has:

    Top: a 9px gray uppercase widely-spaced label (e.g., "Radar Position", "Targets Detected", "Coverage Radius", "Mode")
    Bottom: a 12px monospace value (e.g., "37.7749, -122.4194", "3 active" in green, "500 m", "SURVEILLANCE")

Controls Row (bottom bar)

A thin bar with:

    "Open in Browser" button: green text, small external-link icon, square corners, transparent bg with green hover tint
    "Refresh" button: gray text, refresh icon, similar style
    Buttons separated by 1px right borders
    Right side: a tiny green square (6px) + "STANDBY" in 10px gray monospace

Tab 3: DIAGNOSTICS
Metric Strip (top, 4 equal columns)

Four metric cards in a horizontal row, each separated by 1px vertical borders. Each card (~80px tall) has dark panel background and contains:

    Top row: Left side has an icon (20px, colored) + a gray uppercase label (10px). Right side has the value in 15px monospace, colored (green if healthy, yellow/red if threshold exceeded).
    Bottom: A full-width progress bar, only 4px tall. Dark green-tinted track with a colored fill proportional to the value. Smooth animated transitions.

The four metrics:

    CPU icon + "CPU" → "67.3%" (green when <80, yellow when >80)
    Hard drive icon + "MEMORY" → "45.2%" (same thresholds)
    Activity/chart icon + "DATA RATE" → "823 KB/s" (always green)
    WiFi icon + "LINK" → "Connected" (green) or "Disconnected" (red)

System Log (middle, fills remaining space)

Header bar: "System Log" (11px gray uppercase) on the left, "{count} entries" (10px gray monospace) on the right. 1px border bottom.

Below: a scrollable area with a monospaced log output. Each entry is a single row containing:

    Timestamp: "14:23:05" in 11px gray monospace (fixed width, left-aligned)
    Level tag: "[INFO]" in green, "[WARNING]" in yellow, or "[ERROR]" in red — 11px, fixed ~80px width
    Message: the log text in 11px white at 80% opacity, left-aligned, filling remaining width
    Each row separated by very faint bottom borders

The log auto-scrolls to the newest entry. New lines appear every ~2 seconds.
Hardware Status Ribbon (bottom, thin single row)

A compact bottom bar:

    Far left cell: "HW STATUS" in 10px gray uppercase, separated by a right border
    Four device status cells, each containing: a tiny colored square (8px), the device name ("STM32", "FT601", "GPS", "ANTENNA") in 10px gray uppercase monospace, and the status ("ONLINE" in green, "STANDBY" in yellow) — each cell separated by 1px borders
    Far right: a pulsing green dot (6px) + "SYSTEM NOMINAL" in 10px gray monospace

Tab 4: SETTINGS
Header Bar (~52px)

Left side: "Radar System Configuration" in 13px uppercase white, with subtitle "STM32 · FT601 · Chirp · Detection Parameters" in 11px gray below it. Right side: Two buttons — "Reset Defaults" (gray bg, secondary style, rotate icon) and "Apply Settings" (bright green bg, black text, save/checkmark icon). Both 11px uppercase, square corners.
Settings Body (scrollable, 2-column layout)

The settings area is split into two equal columns by a 1px vertical border.

Section headers: Full-width bars within each column, slightly lighter background tint, text in 10px green uppercase with very wide letter-spacing. Examples: "System Configuration", "Chirp Parameters", "Frequency Range", "Pulse Repetition Frequency", "Detection Parameters", "Logging".

Setting rows: Each row is a horizontal strip (~40px tall) with dark panel background, 1px bottom border:

    A gray label (12px, left side, ~200px wide): e.g., "System Frequency", "CFAR Threshold"
    An input field (monospaced 12px text, dark background, 1px gray border, no rounded corners, green focus ring). For dropdowns, it's a similarly-styled select/combobox.
    A unit label (11px gray monospace, ~48px): e.g., "Hz", "dB", "m", "s"

External Services Section (full width below the 2-column grid)

Single setting row: "Google Maps API Key" with a password-masked input spanning the full width.
Warning Notice (bottom)

A full-width bar with a yellow triangle warning icon on the left and explanatory text in 11px gray about applying settings during standby mode.
Tab 5: RAW INPUTS
Controls Bar (top ribbon)

Left to right:

    Capture button: Bright green background with black text "Capture" + play icon when paused. Yellow/amber background with "Pause" + pause icon when capturing. 11px uppercase, square corners.
    Clear button: 1px bordered, gray text, trash icon, transparent bg. 11px uppercase.
    1px vertical separator.
    Filter group: Four buttons joined together in a segmented control with 1px outer border and 1px internal dividers: "ALL", "RANGE", "DOPPLER", "DETECT" — each 10px uppercase. The active filter has a green-tinted background and green text; others are gray.
    Legend: Three tiny colored squares (6px) with labels in 9px gray: green "RANGE", yellow "DOPPLER", blue "DETECTION".
    Flexible spacer.
    Stats (right-aligned): PKTS (gray) 42 (green), RATE 12/s, DATA 8.3 KB — all 10px monospace. If there are CRC errors: a small red warning triangle + ERR 2 in red. If capturing: a pulsing green dot + LIVE in green.

Main Area (3-panel layout filling remaining height)

Left panel: FT601 Hex Dump (flexible width, ~60%)

Header bar: small green radio/antenna icon + "FT601 · USB 3.0 · Packet Stream" in 10px gray uppercase. Right: "OFFSET · HEX (16B/ROW) · ASCII" in 9px gray.

Scrollable area containing packet blocks stacked vertically. Each packet block:

    Banner row: Very dark semi-transparent background strip. Contains: a green/yellow/blue chevron-right icon (▸), the packet type name ("RANGE" in green, "DOPPLER" in yellow, "DETECTION" in blue), "PKT #0042" in gray, "FRM 002A" in gray, "45B" in gray, "CRC:OK" in green (or "CRC:ERR" in red), and right-aligned stream offset "@00012Ah" in gray. All 10px monospace.

    Hex data rows: Below each banner, one or more rows of hex data (16 bytes per row). Each row has three sections:
        Offset (left, 52px): 6-digit hex address in gray monospace, e.g., "00012A"
        Hex bytes (middle): 16 bytes displayed as 2-char hex values separated by spaces, with an extra-wide gap after byte 8 (like standard hex editors). The SYNC bytes (first two: A5 A5) and TYPE byte (5th byte) are highlighted in the packet's type color (green/yellow/blue). Other bytes are white at 75% opacity.
        ASCII (right, ~120px): The ASCII representation of those 16 bytes. Non-printable characters shown as middle dots "·". Gray text, 10px, tightly spaced.

    Each packet block has a very subtle background tint matching its type color at ~5% opacity. The entire block is clickable; when selected, the background brightens to a green tint (~10% opacity).

Empty state: a centered bordered box with "No packets captured" and "Press Capture to start recording FT601 data" in 10px gray.

Right column (fixed 380px wide), split into two stacked panels:

Top: STM32 UART Terminal (flexible height, ~50%)

Header: "STM32 · UART Stream" in 10px gray uppercase. Right: "115200 8N1" in 9px gray + pulsing green dot when capturing.

Scrollable terminal with lines of text. Each line:

    Timestamp: "14:23:05" (10px gray, fixed width)
    Type badge: "[NMEA]" in green, "[ACK]" in blue, "[STATUS]" in yellow, "[ERROR]" in red — 10px, 60px wide
    Raw data: The actual serial output (NMEA sentences like "$GPGGA,142305.00,3746.4947,N,...", ACK responses, status telemetry, error messages) in 10px white at 75% opacity, wrapping if needed.

Lines separated by very faint borders. Auto-scrolls to bottom.

Bottom: Packet Inspector (fixed 240px height)

Header: "Packet Inspector" in 10px gray uppercase. Right (when a packet is selected): "PKT #0042 · RANGE" in the packet's type color.

When no packet is selected: centered gray text "← Click a packet row to inspect".

When selected, shows decoded fields as a vertical list of label-value rows:

    SYNC: "0xA5 0xA5 → 0xA5A5" in green
    FRAME_ID: "0x002A → 42" in green
    TYPE: "0x01 → RANGE" in the type color
    LENGTH: "0x0020 → 32B payload" in green
    PAYLOAD: "32 bytes (see preview below)" in green
    CRC: "0x4F → ✓ PASS" in green or "✗ FAIL" in red

Below that, separated by a faint border: "Payload Preview (first 6 words)" title in 9px gray uppercase. Then rows: "W0 0x1A2B 6699 669.9m" showing word index, raw hex, decimal value, and interpreted value (meters for range packets, m/s for doppler packets). Values in green, labels in gray.

Footer: timestamp left, stream offset right, both in 9px gray.
Animation Details

    The radar sweep on the Map View rotates smoothly and continuously (360° every 5 seconds)
    Target outer rings pulse (grow/fade) like sonar pings
    Status indicator diamonds pulse gently (opacity fading in/out every ~1s)
    The Range-Doppler heatmap continuously re-renders with slight target drift and noise variation
    All progress bars animate smoothly when values change (700ms transition)
    The hex dump and UART terminal auto-scroll to the bottom as new data arrives
    System log auto-scrolls similarly

