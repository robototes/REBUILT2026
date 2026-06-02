# VisionSim Layout

SimGUI layout for testing the PhotonVision → Limelight vision simulation pipeline.

## What's in here

| File | Purpose |
|------|---------|
| `simgui.json` | SimGUI window layout with Field2d widgets configured to show ground truth pose, estimated pose, per-camera vision estimates, and whether AprilTags are visible or not  |
| `swerve_module.png` | Icon used on the Field2d to represent individual swerve modules |
| `tag-blue.png` | Icon for AprilTag markers on the field visible to cameras |
| `tag-green.png` | Icon for other AprilTag markers |

## How to use

Copy everything in this folder to the project root and run the simulator:

The Field2d widget will show:
- **EstimatedRobot** — where the robot *thinks* it is (odometry + vision correction)
- **GroundTruthRobot** — where the robot *actually* is in the physics sim
- **Vision\_\<camera\>\_MT1 / MT2** — individual vision pose estimates per camera and MegaTag type
