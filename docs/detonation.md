# Detonation & Warhead Management

## Detonation Decision Tree

Three independent paths can trigger warhead detonation. They are checked in priority order each tick during active guidance:

```mermaid
flowchart TD
    ENTRY["Main Loop\n(tick 100+)"] --> MANUAL{"argument ==\n'detonate'?"}

    MANUAL -- "Yes" --> DET["DetonateWarheads()"]
    MANUAL -- "No" --> PROX{"distanceToTarget\n< 500m?"}

    PROX -- "No" --> DIST{"distanceToTarget\n<= 8.0m?"}
    PROX -- "Yes" --> RAY["PerformRaycastCheck()"]
    RAY --> HIT{"Raycast hit\nwithin 3m?"}
    HIT -- "Yes" --> DET
    HIT -- "No" --> DIST

    DIST -- "Yes" --> DET
    DIST -- "No" --> CONTINUE["Continue guidance"]

    DET --> FOREACH["foreach warhead:\nwarhead.Detonate()"]

    style DET fill:#8b0000,color:#fff
    style FOREACH fill:#8b0000,color:#fff
    style CONTINUE fill:#2d5a2d
```

> Manual detonation via the `"detonate"` argument is checked at the very start of `Main()`, before any phase logic. This ensures it works regardless of the missile's current state.

**Source:** `Program.cs` — lines 152-155 (manual), 441-451 (proximity + distance)

---

## Proximity Raycast — PerformRaycastCheck()

The proximity fuse uses a forward-facing camera to detect nearby obstacles:

```mermaid
flowchart TD
    START["PerformRaycastCheck()"] --> BOMB{"armtype == 'bomb'?"}
    BOMB -- "Yes" --> FALSE1["return false\n(bombs have no proximity fuse)"]

    BOMB -- "No" --> FIND["camera = GetBlockWithName('ProxCam')"]
    FIND --> NULL{"camera != null?"}
    NULL -- "No" --> FALSE2["return false"]
    NULL -- "Yes" --> SCAN{"camera.CanScan(3.0)?"}
    SCAN -- "No" --> FALSE3["return false\n(camera recharging)"]
    SCAN -- "Yes" --> CAST["hitInfo = camera.Raycast(3.0)"]
    CAST --> HITP{"hitInfo.HitPosition\n.HasValue?"}
    HITP -- "Yes" --> TRUE["return true\n→ DetonateWarheads()"]
    HITP -- "No" --> FALSE4["return false"]

    style TRUE fill:#8b0000,color:#fff
    style FALSE1 fill:#5a2d2d
    style FALSE2 fill:#5a2d2d
    style FALSE3 fill:#5a2d2d
    style FALSE4 fill:#5a2d2d
```

> The 3.0m raycast range is very short — essentially a contact fuse. The camera must be named "ProxCam" and oriented forward on the missile. Space Engineers cameras have a recharge time between raycasts, so `CanScan()` is checked first.

**Source:** `Program.cs` — `PerformRaycastCheck()`, lines 725-741

---

## Warhead Detonation Sequence

```mermaid
sequenceDiagram
    participant Main as Main Loop
    participant Check as Detonation Check
    participant DW as DetonateWarheads()
    participant GTS as GridTerminalSystem
    participant WH as Warheads

    Main->>Check: distanceToTarget < 500m
    Check->>Check: PerformRaycastCheck()
    alt Raycast hit within 3m
        Check->>DW: Trigger detonation
    else Distance <= 8.0m
        Check->>DW: Trigger detonation
    else Manual "detonate" arg
        Main->>DW: Trigger detonation
    end

    DW->>DW: _warheads.Count == 0?
    alt No cached warheads
        DW->>GTS: GetBlocksOfType(_warheads)
        GTS-->>DW: Warhead list
    end

    alt No warheads found
        DW->>Main: Echo "[ERROR] No warheads found!"
    else Warheads available
        loop foreach warhead
            DW->>WH: warhead.Detonate()
        end
    end
```

> Warheads are cached after the first `GetBlocksOfType` call (during tick 5-10 initialization). The redundant check in `DetonateWarheads()` is a safety net in case initialization was incomplete.

**Source:** `Program.cs` — `DetonateWarheads()`, lines 940-955

---

## Detonation Conditions by Distance

```mermaid
flowchart LR
    subgraph Always ["Any Distance"]
        MANUAL["Manual Detonation\narg == 'detonate'\nPriority: Immediate"]
    end

    subgraph Medium ["< 500m"]
        PROX["Proximity Fuse Active\nProxCam raycast\nRange: 3.0m\nRecharge-dependent"]
    end

    subgraph Close ["<= 8m"]
        CONTACT["Distance Detonation\nGPS distance check\nGuaranteed trigger"]
    end

    Always --> Medium --> Close

    style MANUAL fill:#5a4a2d
    style PROX fill:#2d4a5a
    style CONTACT fill:#8b0000,color:#fff
```

### Detonation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `DETONATION_DISTANCE` | 8.0 m | Distance-based detonation trigger |
| Proximity raycast range | 3.0 m | Camera raycast distance |
| Proximity activation range | 500 m | Distance at which raycast checks begin |
| Manual trigger | `"detonate"` | Programmable block argument string |

> The 500m activation threshold for proximity checks prevents unnecessary raycasts during the cruise phase, saving camera charge for when it matters.

**Source:** `Program.cs` — lines 78-79 (constants), 441-451 (check logic), 725-741 (raycast)
