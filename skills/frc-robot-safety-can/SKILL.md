---
name: frc-robot-safety-can
description: Enforces motor current limits, brake/coast configurations, soft limits, and CAN bandwidth optimization for FRC mechanisms.
---

# FRC Robot Safety & Hardware Protection

## Context
FRC robots handle high-power brushless motors (Kraken X60, Falcon 500, NEO, Vortex). Improper software configuration can strip gears, snap polycord, or burn out motors in seconds.

## Key Review Checks
1. **Current Limiting (Critical):**
   - Every motor controller configuration **must** explicitly configure vendor-appropriate current limits:
     - **CTRE (TalonFX / Kraken / Falcon / TalonSRX):** Configure stator current limits (thermal & torque motor protection) and/or supply current limits (PDH/PDP breaker and brownout protection) via `CurrentLimitsConfigs`.
     - **REV (Spark MAX / Spark Flex / NEO / Vortex):** Configure smart current limits (e.g., `setSmartCurrentLimit()` or `SparkBaseConfig.smartCurrentLimit()`) and optional secondary current limits.
   - Ensure limits match the mechanism's physical load (e.g., 30A–40A+ for drivetrains, 20A–30A for intakes/indexers).
2. **Mechanism Protection:**
   - Check that articulated mechanisms (arms, elevators, wrists) define software soft limits or limit switch configurations to prevent mechanical hard-stops.
   - Verify neutral mode configuration: Elevators and arms should generally be set to `Brake` mode to avoid unpowered dropping.
3. **CAN Bus Utilization:**
   - Flag excessive high-frequency status frame updates on devices where telemetric feedback is unnecessary.
   - Ensure motor inversion (`setInverted(true/false)`) is applied in configuration rather than by negating output values haphazardly throughout the code.

## Feedback Tone & Style
- Emphasize safety and asset protection without sounding accusatory.
- Frame issues around competitive readiness: *"Configuring current limits now prevents blown breakers during rapid cycling at competition!"*
