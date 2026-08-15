---
name: frc-telemetry-units
description: Enforces WPILib Units library usage, structured logging practices, and clean SmartDashboard/AdvantageKit conventions.
---

# FRC Telemetry, Logging & Units Safety

## Context
Modern FRC projects prioritize structured telemetry (WPILib `DataLog`, AdvantageKit, or SmartDashboard/NetworkTables) and compile-time dimensional analysis via the WPILib Units library.

## Key Review Checks
1. **Dimensional Analysis:**
   - Encourage the use of WPILib Java Units (`Volts.of(...)`, `MetersPerSecond.of(...)`, `Degrees.of(...)`) instead of raw `double` literals to prevent degree/radian or inch/meter conversion bugs.
2. **Telemetry Overhead:**
   - Avoid raw string concatenation inside `SmartDashboard.putData()` or `periodic()`.
   - Ensure critical values (battery voltage, motor current, subsystem state, pose estimation) are logged consistently for post-match drive team debriefs.
3. **Magic Numbers:**
   - Ensure PID constants, gear ratios, wheel diameters, and CAN IDs are isolated inside dedicated `Constants.java` classes or subsystem config records.

## Feedback Tone & Style
- Praise clean constants organization.
- Explain dimensional bugs using relatable examples (e.g., *"Mixing radians and degrees is the most common reason swerve modules spin in circles!"*).
