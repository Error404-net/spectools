# Hak5 Pineapple Pager Spectools Payload Specification

## Project Goal

Develop a native graphical payload for the Hak5 Pineapple Pager that recreates the classic Spectools experience using a Wi-Spy DBx spectrum analyzer.

The payload should provide a polished, standalone application that runs entirely on the Pager without requiring SSH or a desktop computer. The interface should be rendered directly to the LCD using the Linux framebuffer (`/dev/fb0`) and be fully navigable using the Pager's hardware buttons.

The objective is to recreate the classic Spectools visualization modes—including Waterfall, Spectral, Topographical, and Planar views—in a portable form factor while integrating cleanly with the Pager's existing payload framework and Loot system.

---

# Application Startup

When the payload is launched:

1. Initialize the framebuffer (`/dev/fb0`).
2. Detect the display resolution and pixel format dynamically.
3. Clear the screen.
4. Display a centered **Loading...** screen.
5. During loading:

   * Verify that a supported Wi-Spy DBx is connected.
   * Verify that `spectool_raw` is installed and executable.
   * Verify that communication with the device succeeds.
6. If any check fails:

   * Display a graphical error screen.
   * Explain what failed.
   * Allow the user to return to the Pager.
7. If all checks succeed:

   * Initialize a new scan session.
   * Create the Loot session directory.
   * Continue to the main menu.

---

# Main Menu

The application should present a graphical menu rendered entirely through `/dev/fb0`.

Menu entries should include:

* Waterfall View
* Spectral View
* Topographical View
* Planar View
* Device Information
* Settings
* Exit

The currently selected item should always be highlighted.

Navigation should feel similar to the Pager's native UI.

---

# Frequency Selection

Before launching a visualization, allow the user to choose:

* 2.4 GHz
* 5 GHz

The chosen band should configure the appropriate Spectools parameters before the visualization begins.

The architecture should make it easy to add additional frequency ranges in the future.

---

# Visualization Modes

The application should recreate the classic Spectools graphical modes as closely as practical while optimizing them for the Pager's display.

## Waterfall

A continuously scrolling waterfall showing signal intensity over time.

## Spectral

A live FFT/spectrum graph showing current energy levels.

## Topographical

A persistence-based visualization showing historical activity.

## Planar

A channel occupancy or planar visualization similar to the original Spectools application.

Each visualization should use the same rendering engine and share common navigation behavior.

Switching between views should not require restarting the scan.

---

# Framebuffer Rendering

Render directly to:

`/dev/fb0`

Do not depend on terminal output or the existing Pager UI.

Requirements:

* Detect framebuffer geometry dynamically.
* Detect pixel format dynamically.
* Avoid hardcoded display dimensions.
* Update only changed screen regions whenever practical.
* Separate rendering from acquisition and UI logic.

Design the renderer so future screens can easily be added.

---

# Hardware Controls

Use the existing Linux input subsystem:

`/dev/input/event0`

Reuse the proven `evtest`-based architecture already used by other Pager payloads.

Use the existing optimized event parsing implementation including:

* `$KEYCKTMP_FILE`
* `$BTN_EVT_FILE`
* `$DPAD_EVT_FILE`
* `$DPAD_PENDING_FILE`
* `$SCREENSHOT_EVT_FILE`

Continue using:

* `evtest`
* single-pass `awk`
* `fflush()`
* existing long-press detection

Do not replace this architecture.

---

# Button Mapping

## Main Menu

**Up**

Move selection upward.

**Down**

Move selection downward.

**OK (short press)**

Open the highlighted menu item.

**Back**

Return to the previous screen.

Exit the payload from the main menu.

---

# During Live Visualization

## Left

Switch to the previous visualization mode.

Example:

Planar ← Topographical ← Spectral ← Waterfall

## Right

Switch to the next visualization mode.

Example:

Waterfall → Spectral → Topographical → Planar

Changing views should not restart the Wi-Spy capture.

## Up / Down

Adjust display-specific parameters such as:

* Vertical scaling
* FFT averaging
* Dynamic range
* Persistence
* Color palette

These controls should be easy to extend later.

## OK

Short press:

Pause or resume rendering while continuing to collect spectrum data.

Display a translucent "Paused" overlay.

## Back

Return immediately to the previous menu.

Terminate child processes cleanly.

Do not leave orphaned Spectools processes running.

---

# Screenshot Functionality

The payload should support capturing screenshots of the active visualization.

## Screenshot Trigger

Use the existing long-press detection framework.

Button behavior:

* Short OK press → Select / Pause / Resume
* Long OK press (default: approximately 2 seconds) → Capture screenshot

The hold duration should be configurable through the payload settings.

---

# Screenshot Capture

When a screenshot is requested:

1. Capture the contents of `/dev/fb0`.
2. Save the image into the current Loot session.
3. Resume rendering immediately.

Filename example:

`screenshot_YYYYMMDD_HHMMSS.bmp`

PNG may also be supported if practical.

---

# Screenshot Feedback

After successfully saving a screenshot:

* Flash the display for approximately 100–200 ms.
* Play a short confirmation tone using the Pager buzzer (if available).
* Display a temporary overlay:

```
Screenshot Saved

session_YYYYMMDD_HHMMSS_fb
```

The overlay should disappear automatically after approximately one second.

The capture process should not interrupt the live spectrum stream.

---

# Loot Integration

Reuse the existing SpecPine Loot system.

Use:

```
/root/loot/specpine/
```

or

```
/tmp/specpine/
```

depending on the existing `noloot` configuration.

Continue using the current session directory format:

```
session_YYYYMMDD_HHMMSS_<mode>/
```

Do not create a new directory layout.

---

# Session Artifacts

Continue generating compatible artifacts including:

* `meta.json`
* `events.jsonl`
* `gps.txt`
* Spectools metadata
* Exported summaries
* Screenshots
* Future capture exports

Maintain compatibility with existing cleanup and session management logic.

---

# Device Detection

Create a dedicated device detection layer that verifies:

* Wi-Spy DBx present
* USB communication functioning
* `spectool_raw` installed
* Spectools executable
* Device responding correctly

Present graphical error screens instead of returning to a shell if validation fails.

---

# Architecture

Separate the project into reusable modules.

Suggested organization:

* Input Manager
* Framebuffer Renderer
* Menu System
* Spectools Interface
* Device Detection
* Session Manager
* Loot Manager
* Screenshot Manager
* Audio/Tone Manager
* Visualization Engine

Avoid tightly coupling rendering with Spectools communication.

---

# Performance Requirements

The Pineapple Pager has limited CPU and memory resources.

Optimize accordingly.

Requirements:

* Minimize process creation.
* Continue using the optimized single-pass `awk` event parser.
* Avoid unnecessary framebuffer redraws.
* Update only modified screen regions whenever practical.
* Keep navigation responsive while processing live spectrum data.
* Avoid blocking UI updates during rendering.
* Cleanly terminate all child processes when exiting.

---

# User Experience

The finished payload should feel like a native application rather than a shell script.

The user experience should be:

1. Launch payload.
2. Loading screen.
3. Device detection.
4. Main menu.
5. Select frequency.
6. Start live visualization.
7. Switch instantly between:

   * Waterfall
   * Spectral
   * Topographical
   * Planar
8. Pause/resume using the OK button.
9. Hold the OK button for approximately two seconds to capture a screenshot, accompanied by a brief screen flash and confirmation tone, with the image saved to the current Loot session.
10. Return to the menu or exit cleanly.

The overall appearance should be optimized for the Pineapple Pager's display, hardware controls, and embedded Linux environment. Theme and styling should resemble 90's warez scene hacker vibes.
