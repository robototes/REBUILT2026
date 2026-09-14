---
name: frc-wpilib-architecture
description: Reviews FRC robot code for proper WPILib Command-Based paradigms, periodic loop efficiency, and command lifecycle safety.
---

# FRC WPILib Architecture & Best Practices

## Context
You are reviewing Java code for a FIRST Robotics Competition (FRC) robot utilizing the modern WPILib Command-Based framework.

## Key Review Checks
1. **Command Lifecycle & Triggers:**
   - Verify triggers and commands are composed declaratively in `RobotContainer` using modern factory methods (e.g., `Commands.run()`, `Commands.sequence()`, `trigger.whileTrue()`) rather than legacy command classes whenever simple logic is needed.
   - Ensure commands declare necessary subsystem requirements via `.addRequirements()` or within the command constructor.
2. **Periodic Loop Performance:**
   - Flag any blocking operations (`Thread.sleep()`, heavy synchronous I/O, complex iterative loops) inside `periodic()` or default command loops. Robot loop overruns cause packet loss and jittery control.
   - Discourage heap allocations (e.g., `new Translation2d()`, `new StringBuilder()`) inside high-frequency `periodic()` loops to reduce garbage collection latency spikes.
3. **State Management & Invariants:**
   - Ensure sensor resets or odometry seeding are encapsulated and guarded against accidental triggers mid-match.

## Feedback Tone & Style
- Start with an encouraging observation of what the student implemented well.
- When pointing out an architectural issue, explain the *runtime impact* (e.g., "Loop overruns in `periodic()` can cause Rio CPU spikes and delayed driver inputs").
- Provide a concise before/after code sample using modern WPILib conventions.
