# LCD Display & Telemetry

## Update Pipeline

The LCD updates at 20 Hz (every 3 ticks) rather than the guidance rate of 60 Hz. This reduces computational overhead while maintaining smooth visual output:

```mermaid
flowchart TD
    TICK["Active Guidance Tick"] --> CHECK{"lcdMain != null &&\n_ticks % 3 == 0?"}
    CHECK -- "No" --> SKIP["Skip LCD update\n(save performance)"]
    CHECK -- "Yes" --> CALC["Calculate telemetry:\ndistanceToTarget\ncurrentSpeed\ntimeToTargetSec = dist / speed\nprogress = (startDist - dist) / startDist"]

    CALC --> SETUP["LCD Configuration:\nContentType = TEXT_AND_IMAGE\nFont = Monospace\nFontSize = 0.8\nAlignment = LEFT\nPadding = 2"]

    SETUP --> MODE{"timeToTargetSec\n<= 2.5?"}
    MODE -- "Yes" --> IMPACT["Impact Warning Mode\nFlashing animation"]
    MODE -- "No" --> NORMAL["Normal Telemetry Mode\nFull data display"]

    IMPACT --> WRITE["lcd.WriteText(display)"]
    NORMAL --> WRITE

    style NORMAL fill:#2d5a2d
    style IMPACT fill:#8b0000,color:#fff
    style SKIP fill:#5a2d2d
```

**Source:** `Program.cs` — lines 596-599 (tick check), 956-1033 (DisplayOnLCD)

---

## Display Layout

The LCD output uses monospace text to create a structured telemetry display:

```mermaid
flowchart TD
    subgraph Header ["Header"]
        H1["====================================="]
        H2["   NYINAH CORP MISSILE CAM"]
        H3["====================================="]
    end

    subgraph Normal ["Normal Telemetry (ETA > 2.5s)"]
        SPEED["Speed:     {speed} m/s"]
        DIST["Distance:  {distance} m"]
        ETA["ETA:       {eta} sec  (or '---' if > 999)"]
        TGT["Target:    {name}  (or 'Seeking...')"]
        BAR["Progress:  [=====>-----------]"]
        PCT["           {percent}%"]
        STATUS["Status:    TRACKING / FINAL APPROACH / TERMINAL PHASE"]
    end

    subgraph Impact ["Impact Warning (ETA <= 2.5s)"]
        FLASH["Alternating every 10 ticks:"]
        F1["!!! IMPACT IMMINENT !!!\n    {eta} SECONDS\n!!! IMPACT IMMINENT !!!"]
        F2[">>> {eta} SECONDS <<<"]
    end

    subgraph Footer ["Footer"]
        FOOT["====================================="]
    end

    Header --> Normal
    Header --> Impact
    Normal --> Footer
    Impact --> Footer

    style Impact fill:#8b0000,color:#fff
    style Normal fill:#2d5a2d
```

### Display Fields

| Field | Source | Format | Condition |
|-------|--------|--------|-----------|
| Speed | `currentSpeed` | `F0` (integer m/s) | Always |
| Distance | `distanceToTarget` | `F0` (integer m) | Always |
| ETA | `distanceToTarget / currentSpeed` | `F1` (1 decimal s) | < 999s |
| Target | `detectedEntity.Name` | String | Radar has target |
| Progress | `(startDist - dist) / startDist` | 25-char bar | Always |
| Status | Distance thresholds | String | See below |

### Status Thresholds

| Distance | Status Label |
|----------|-------------|
| > 500 m | `TRACKING` |
| 100-500 m | `FINAL APPROACH` |
| < 100 m | `TERMINAL PHASE` |

**Source:** `Program.cs` — `DisplayOnLCD()`, lines 956-1033

---

## Impact Warning Animation

When time-to-impact drops below 2.5 seconds, the display switches to a flashing warning mode:

```mermaid
flowchart TD
    ETA["timeToTargetSec <= 2.5\n&& > 0.1"] --> FRAME["frame = (_ticks / 10) % 2"]

    FRAME --> F0{"frame == 0?"}
    F0 -- "Yes" --> WARN["!!! IMPACT IMMINENT !!!\n    {eta} SECONDS\n!!! IMPACT IMMINENT !!!"]
    F0 -- "No" --> COUNT["\n>>> {eta} SECONDS <<<\n"]

    WARN --> DISPLAY["lcd.WriteText()"]
    COUNT --> DISPLAY

    style WARN fill:#8b0000,color:#fff
    style COUNT fill:#5a4a2d
```

> The flash rate is 10 ticks per frame (~167ms per state at 60Hz), creating a rapid 3 Hz blink effect. The 0.1s lower bound prevents the warning from displaying during the detonation frame.

**Source:** `Program.cs` — lines 977-992
