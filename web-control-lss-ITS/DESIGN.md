---
name: Oceanic Telemetry & Autonomous Surface Control
colors:
  surface: '#faf8ff'
  surface-dim: '#d2d9f4'
  surface-bright: '#faf8ff'
  surface-container-lowest: '#ffffff'
  surface-container-low: '#f2f3ff'
  surface-container: '#eaedff'
  surface-container-high: '#e2e7ff'
  surface-container-highest: '#dae2fd'
  on-surface: '#131b2e'
  on-surface-variant: '#3f4850'
  inverse-surface: '#283044'
  inverse-on-surface: '#eef0ff'
  outline: '#707881'
  outline-variant: '#bfc7d2'
  surface-tint: '#006398'
  primary: '#006194'
  on-primary: '#ffffff'
  primary-container: '#007bb9'
  on-primary-container: '#fdfcff'
  inverse-primary: '#93ccff'
  secondary: '#006399'
  on-secondary: '#ffffff'
  secondary-container: '#7bc2ff'
  on-secondary-container: '#004f7b'
  tertiary: '#006947'
  on-tertiary: '#ffffff'
  tertiary-container: '#00855b'
  on-tertiary-container: '#f5fff6'
  error: '#ba1a1a'
  on-error: '#ffffff'
  error-container: '#ffdad6'
  on-error-container: '#93000a'
  primary-fixed: '#cce5ff'
  primary-fixed-dim: '#93ccff'
  on-primary-fixed: '#001d31'
  on-primary-fixed-variant: '#004b73'
  secondary-fixed: '#cde5ff'
  secondary-fixed-dim: '#94ccff'
  on-secondary-fixed: '#001d32'
  on-secondary-fixed-variant: '#004b74'
  tertiary-fixed: '#6ffbbe'
  tertiary-fixed-dim: '#4edea3'
  on-tertiary-fixed: '#002113'
  on-tertiary-fixed-variant: '#005236'
  background: '#faf8ff'
  on-background: '#131b2e'
  surface-variant: '#dae2fd'
typography:
  display-lg:
    fontFamily: Plus Jakarta Sans
    fontSize: 3.5rem
    fontWeight: '700'
    lineHeight: 4.25rem
    letterSpacing: -0.03em
  display-sm:
    fontFamily: Plus Jakarta Sans
    fontSize: 2.25rem
    fontWeight: '700'
    lineHeight: 2.75rem
    letterSpacing: -0.02em
  headline-lg:
    fontFamily: Plus Jakarta Sans
    fontSize: 2rem
    fontWeight: '600'
    lineHeight: 2.5rem
    letterSpacing: -0.02em
  headline-lg-mobile:
    fontFamily: Plus Jakarta Sans
    fontSize: 1.5rem
    fontWeight: '600'
    lineHeight: 2rem
    letterSpacing: -0.01em
  headline-md:
    fontFamily: Plus Jakarta Sans
    fontSize: 1.25rem
    fontWeight: '600'
    lineHeight: 1.75rem
    letterSpacing: -0.01em
  headline-sm:
    fontFamily: Plus Jakarta Sans
    fontSize: 1.125rem
    fontWeight: '600'
    lineHeight: 1.5rem
    letterSpacing: '0'
  body-lg:
    fontFamily: Plus Jakarta Sans
    fontSize: 1rem
    fontWeight: '400'
    lineHeight: 1.5rem
    letterSpacing: '0'
  body-md:
    fontFamily: Plus Jakarta Sans
    fontSize: 0.875rem
    fontWeight: '400'
    lineHeight: 1.375rem
    letterSpacing: '0'
  body-sm:
    fontFamily: Plus Jakarta Sans
    fontSize: 0.75rem
    fontWeight: '400'
    lineHeight: 1.125rem
    letterSpacing: 0.01em
  label-numeric:
    fontFamily: JetBrains Mono
    fontSize: 0.875rem
    fontWeight: '500'
    lineHeight: 1.25rem
    letterSpacing: -0.01em
  label-code:
    fontFamily: JetBrains Mono
    fontSize: 0.75rem
    fontWeight: '400'
    lineHeight: 1rem
    letterSpacing: 0.02em
  label-tag:
    fontFamily: Plus Jakarta Sans
    fontSize: 0.6875rem
    fontWeight: '700'
    lineHeight: 0.875rem
    letterSpacing: 0.05em
rounded:
  sm: 0.25rem
  DEFAULT: 0.5rem
  md: 0.75rem
  lg: 1rem
  xl: 1.5rem
  full: 9999px
spacing:
  gutter: 1rem
  gutter-md: 1.25rem
  gutter-lg: 1.5rem
  margin: 1rem
  margin-md: 1.5rem
  margin-lg: 2rem
  space-xs: 0.25rem
  space-sm: 0.5rem
  space-md: 0.75rem
  space-lg: 1.25rem
  space-xl: 2rem
---

## Brand & Style

The design system is crafted for high-stakes oceanographic research, autonomous marine navigation, and real-time remote scientific telemetry. The visual aesthetic fuses the clinical precision and expansive whitespace of Apple Human Interface Guidelines with the structured spatial hierarchy and tokenized consistency of Google Material 3.

### Personality & Emotional Tenor
- **Scientific Precision:** Calm, authoritative, and noise-free. Critical hydrographic measurements, bathymetric scanning, and satellite link matrices are parsed instantly without cognitive friction.
- **Marine Modernism:** Evoking deep ocean clarity and clean aerospace instrumentation rather than industrial grime.
- **Trust & Resilience:** High-clarity feedback loops instill complete operational confidence under fluctuating offshore conditions.

### Design Style Archetype
- **Clean Oceanic Minimalist:** Pure optical whites, hyper-subtle cool slate backgrounds, crisp low-contrast hairline borders, and translucent glass-surface navigation over mission maps.

## Colors

The palette establishes an ultra-clean, daylight-readable maritime control room environment. Contrast ratios strictly exceed WCAG 2.1 AAA for mission-critical telemetry readouts.

### Palette Architecture
- **Primary (`#0284C7` / Ocean Blue):** Represents active steering vectors, primary waypoints, interactive actions, and main system states.
- **Secondary (`#0369A1` / Deep Nautical Cyan):** Applied to active navigation rails, focused node selectors, and deep-water acoustic sensor graphs.
- **Tertiary (`#10B981` / Oceanic Emerald):** Reserved for nominal hardware health, reliable GNSS-RTK locks, stable propulsion, and clean battery health.
- **Neutral Core (`#0F172A` / Slate Navy):** Provides high-precision contrast for titles, primary values, and mission logs, paired with `#334155` for contextual secondary labels.
- **Backgrounds & Canvases:** Base operational canvas sits at `#F8FAFC`, card container surfaces at `#FFFFFF`, and secondary sub-panels at `#F1F5F9`.
- **System Diagnostics:** Warning and anomaly states use `#F59E0B` (Amber Warning) and `#EF4444` (Critical Loss of Link / Hull Breach Alert).

## Typography

Typography balances Scandinavian editorial clarity with technical avionics instrumentation.

- **Primary Interface (`Plus Jakarta Sans`):** Delivers clean geometry, crisp apertures, and balanced human touches. Used for headlines, interface labels, navigational commands, and standard descriptive copy.
- **Telemetry & Metric Mono (`JetBrains Mono`):** Dedicated to coordinates (lat/long), pitch/roll/yaw gyroscopic vectors, battery voltages, ping latency, and raw hydrophone signal rates to ensure tabular numeral stability without optical jitter during live streaming.

## Layout & Spacing

The layout is structured around an adaptive 12-column density grid designed for continuous multi-window monitoring.

### Spatial Paradigm
- **Docked Mission Viewport:** The geospatial map and live sonobuoy feed maintain central visual dominance.
- **Peripheral Telemetry Rails:** Dual modular sidebars (Left: Autopilot & Propulsion; Right: Oceanographic Scientific Payloads & CTD Sensors) maintain a fixed 380px footprint on extra-large displays and stack gracefully into collapsible drawer-sheets on tablet screens.
- **Rhythm:** An 8pt base grid with a 4pt baseline subdivision ensures alignment across numeric gauge cards, chart widgets, and data bars.

## Elevation & Depth

Visual hierarchy prioritizes ultra-subtle ambient depth and hairline boundary articulation to prevent visual fatigue during prolonged shifts.

### Elevation Architecture
- **Level 0 (Canvas Base):** Flat `#F8FAFC` surface with zero shadow.
- **Level 1 (Card & Module Layer):** Pure `#FFFFFF` surface accompanied by a dual ambient shadow:
  - `box-shadow: 0 4px 20px -2px rgba(15, 23, 42, 0.04), 0 2px 6px -1px rgba(15, 23, 42, 0.02);`
  - Border: 1px solid `#E2E8F0`.
- **Level 2 (Active Gauges & Floating Flight HUDs):**
  - `box-shadow: 0 12px 32px -4px rgba(15, 23, 42, 0.06), 0 4px 12px -2px rgba(15, 23, 42, 0.03);`
  - Border: 1px solid `#CBD5E1`.
- **Level 3 (Modal Overrides & Emergency E-Stop Confirmations):**
  - `box-shadow: 0 24px 48px -12px rgba(15, 23, 42, 0.12);`
  - Backdrop Blur: `backdrop-filter: blur(16px); background: rgba(255, 255, 255, 0.85);`

## Shapes

The design system incorporates refined, soft curves aligned with modern OS design paradigms (Apple macOS Sonoma and Google Material 3 rounded surfaces).

- **Standard Cards & Dash Panels:** `rounded-2xl` (16px / 1rem) for an organic, friendly scientific apparatus enclosure.
- **Nested Gauges & Sub-modules:** `rounded-xl` (12px / 0.75rem).
- **Interactive Controls (Inputs, Buttons, Dropdowns):** `rounded-xl` (10px - 12px).
- **Status Indicators & Telemetry Pills:** Full circular pill radius (`rounded-full` / 9999px) to preserve distinct visual separation between actionable panels and non-actionable status signals.

## Components

### Buttons
- **Primary Action (Engage Autopilot / Waypoint Add):** `#0284C7` solid fill, white text, 0 1px 2px rgba(0,0,0,0.05). Hover shifts to `#0369A1` with an inner micro-glow.
- **Secondary / Utility (Calibrate Sonar, Zero Gyro):** `#FFFFFF` surface with `#E2E8F0` border and `#0F172A` text. Hover adds subtle `#F1F5F9` background transition.
- **Critical Abort / E-Stop:** Elevated `#EF4444` solid with white bold text and pulsing ping indicator for hardware override states.

### Status Chips & Pills
- **Nominal Telemetry Pill:** Background `#ECFDF5`, text `#065F46`, paired with an animated pulse dot in `#10B981`.
- **Advisory Pill (High Wave Drift):** Background `#FFFBEB`, text `#92400E`, with `#F59E0B` icon.
- **Off-grid/Offline Pill:** Background `#F1F5F9`, text `#64748B`.

### Telemetry Cards
- Constructed with `#FFFFFF` background, `rounded-2xl` contour, and standard Level 1 ambient drop shadow.
- Header row includes metadata title in uppercase `label-tag` styling, secondary unit measurement, and an inline sparkline graph.
- Data payload renders primary numerical indicators using `label-numeric` with fixed width tabular figures to prevent layout shifts during live data streams.

### Form Inputs & Mission Steppers
- Input fields leverage a clean `#F8FAFC` base that transitions to `#FFFFFF` on active focus, framed by a 1.5px `#0284C7` ring and 0 0 0 3px rgba(2, 132, 199, 0.12).
- Stepper controls and slider rails incorporate clear ticks for thruster throttle (0–100%) and rudder deflection angles (-45° to +45°).

### Hydrographic Map & HUD Overlays
- Floating map widget cards employ frosted glass (`rgba(255, 255, 255, 0.88)` with `blur(12px)`), bounded by `1px solid rgba(226, 232, 240, 0.8)`.
- Compass rose and crosshair targeting employ high-contrast vector paths rendered in `#0369A1` and `#0284C7`.