/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.sim.visionproducers;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Vision simulation using PhotonVision. */
@SuppressWarnings("PMD.TooManyStaticImports")
public class VisionSim implements VisionSimInterface {
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_photonEstimator;

  @SuppressWarnings("unused")
  private Matrix<N3, N1> m_curStdDevs;

  // Simulation
  private PhotonCameraSim m_cameraSim;
  private VisionSystemSim m_visionSystemSim;

  // Limelight NetworkTables publisher
  private final LimelightTablePublisher m_limelightPublisher;

  /** Constructor. */
  public VisionSim() {

    // This is good sample code for PhotonVision usage in-general, but we spin this up ONLY for
    // simulation.  You'll need a separate implementation for real robot vision processing.
    if (!Robot.isSimulation()) {
      throw new IllegalStateException("VisionSim should only be instantiated in simulation");
    }

    m_camera = new PhotonCamera(kCameraName);
    m_photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    m_limelightPublisher = new LimelightTablePublisher("limelight");

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      m_visionSystemSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to simulated field.
      m_visionSystemSim.addAprilTags(kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(
          kCameraResWidth, kCameraResHeight, Rotation2d.fromDegrees(kCameraFOVDegrees));
      cameraProp.setCalibError(kCalibErrorAvg, kCalibErrorStdDev);
      cameraProp.setFPS(kCameraFPS);
      cameraProp.setAvgLatencyMs(kAvgLatencyMs);
      cameraProp.setLatencyStdDevMs(kLatencyStdDevMs);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible targets.
      m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
      // Set realistic detection range limits
      m_cameraSim.setMinTargetAreaPixels(kMinTargetAreaPixels);
      m_cameraSim.setMaxSightRange(kMaxSightRangeMeters);
      // Add the simulated camera to view the targets on this simulated field.
      m_visionSystemSim.addCamera(m_cameraSim, kRobotToCam);

      m_cameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public void periodic() {
    // We only do pose estimation if someone is subscribed
    generatePoseEstimate();
  }

  private void generatePoseEstimate() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : m_camera.getAllUnreadResults()) {
      visionEst = m_photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = m_photonEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEst, result.getTargets());

      // Publish to Limelight NetworkTables for LimelightOdometry to consume
      LimelightData data = PhotonToLimelightConverter.convertPipelineResult(result, kRobotToCam);
      double totalLatencyMs = data.pipelineLatencyMs + data.captureLatencyMs;
      PhotonToLimelightConverter.convertBotpose(
          visionEst.map(est -> est.estimatedPose).orElse(null),
          result.getTargets(),
          kRobotToCam,
          totalLatencyMs,
          data);
      m_limelightPublisher.publish(data);
    }
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      m_curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }

        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        m_curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        m_curStdDevs = estStdDevs;
      }
    }
  }

  // ----- Simulation

  @Override
  public void simulationPeriodic(Pose2d robotSimPose) {
    m_visionSystemSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  @Override
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      m_visionSystemSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  @Override
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) {
      throw new IllegalStateException("getSimDebugField should only be called in simulation");
    }
    return m_visionSystemSim.getDebugField();
  }
}
