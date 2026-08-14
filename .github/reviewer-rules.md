# Code Reviewer Instructions & Domain Knowledge

## Model & Infrastructure Rules
- We use the `gemini-3.6-flash` model endpoint across our CI/CD workflows. It is valid and working; do not suggest changing model versions.

## FRC Robotics Best Practices
- **WPILib Command-Based:** Verify subsystems require commands properly (`addRequirements(this)`).
- **CAN Bus Safety:** Ensure CAN IDs and motor controller configurations avoid blocking calls in periodic loops.
- **Null Safety:** Check that sensors and encoders initialized in constructor are null-checked before `.get()`.
- **Educational Tone:** Explain *why* something is an issue rather than just giving a diff.
