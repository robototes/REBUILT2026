package robotutils.groundtruthsim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import robotutils.pub.utils.AllianceCalc;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;


class TestGroundTruthSim {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Controllable chassis speeds returned by the speeds supplier. */
    private ChassisSpeeds m_speeds;

    /** Controllable estimated pose returned by the pose supplier. */
    private Pose2d m_estimatedPose;

    /** Captures poses passed to drivetrain resetPose. */
    private Consumer<Pose2d> m_mockDrivetrainResetPose;

    /** Captures poses passed to the poseResetConsumer (for cycle/auto resets). */
    private Consumer<Pose2d> m_mockPoseResetConsumer;

    private MockedStatic<RobotBase> m_mockedRobot;
    private MockedStatic<Utils> m_mockedUtils;

    /** Controllable clock for Utils.getCurrentTimeSeconds(). */
    private double m_currentTime;

    @BeforeEach
    void setUp() {
        m_mockedRobot = mockStatic(RobotBase.class);
        m_mockedRobot.when(RobotBase::isSimulation).thenReturn(true);

        m_currentTime = 0.0;
        m_mockedUtils = mockStatic(Utils.class);
        m_mockedUtils.when(Utils::getCurrentTimeSeconds).thenAnswer(inv -> m_currentTime);

        m_speeds = new ChassisSpeeds(0, 0, 0);
        m_estimatedPose = new Pose2d();

        m_mockDrivetrainResetPose = mockConsumer();
        m_mockPoseResetConsumer = mockConsumer();
    }

    @AfterEach
    void tearDown() {
        m_mockedUtils.close();
        m_mockedRobot.close();
    }

    // ── Factory helpers ──────────────────────────────────────────────

    @SuppressWarnings("unchecked")
    private static <T> Consumer<T> mockConsumer() {
        return mock(Consumer.class);
    }

    private SwerveDriveState buildDriveState() {
        SwerveDriveState state = new SwerveDriveState();
        state.Speeds = m_speeds;
        state.Pose = m_estimatedPose;
        return state;
    }

    /** Creates a GroundTruthSim with default randomness. */
    private GroundTruthSim createSim() {
        return new GroundTruthSim(
            this::buildDriveState,
            m_mockDrivetrainResetPose,
            m_mockPoseResetConsumer,
            Optional.empty());
    }

    // ── Constructor tests ────────────────────────────────────────────

    @Test
    void constructor_throwsOutsideSimulation() {
        m_mockedRobot.when(RobotBase::isSimulation).thenReturn(false);

        assertThrows(IllegalStateException.class, () -> createSim());
    }

    @Test
    void constructor_succeedsInSimulation() {
        assertDoesNotThrow(() -> createSim());
    }

    // ── updateGroundTruthPose tests ──────────────────────────────────

    @Test
    void updateGroundTruthPose_straightLineForward() {
        GroundTruthSim sim = createSim();

        // Drive 2 m/s in robot-X for 0.5s
        m_speeds = new ChassisSpeeds(2.0, 0, 0);
        m_currentTime = 0.5;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(1.0, gt.getX(), 1e-6, "Should travel 1m in X");
        assertEquals(0.0, gt.getY(), 1e-6, "Should not move in Y");
        assertEquals(0.0, gt.getRotation().getDegrees(), 1e-6, "Heading unchanged");
    }

    @Test
    void updateGroundTruthPose_straightLineSideways() {
        GroundTruthSim sim = createSim();

        // Drive 1 m/s in robot-Y (strafe left) for 1s
        m_speeds = new ChassisSpeeds(0, 1.0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6, "No forward motion");
        assertEquals(1.0, gt.getY(), 1e-6, "Should strafe 1m in Y");
    }

    @Test
    void updateGroundTruthPose_rotatedHeading_movesFwdInFieldY() {
        GroundTruthSim sim = createSim();

        // Start at 90° heading (facing field +Y)
        sim.resetGroundTruthPoseForSim(
            new Pose2d(0, 0, Rotation2d.fromDegrees(90)));

        // Drive 1 m/s in robot-X — should map to field +Y
        m_speeds = new ChassisSpeeds(1.0, 0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6, "Robot-X maps to near-zero field-X at 90°");
        assertEquals(1.0, gt.getY(), 1e-6, "Robot-X maps to field-Y at 90°");
    }

    @Test
    void updateGroundTruthPose_rotationIntegration() {
        GroundTruthSim sim = createSim();

        // Rotate at 1 rad/s for 1s, no translation
        m_speeds = new ChassisSpeeds(0, 0, 1.0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6);
        assertEquals(0.0, gt.getY(), 1e-6);
        assertEquals(Math.toDegrees(1.0), gt.getRotation().getDegrees(), 1e-6,
            "Heading should increase by 1 radian");
    }

    @Test
    void updateGroundTruthPose_accumulates() {
        GroundTruthSim sim = createSim();

        // First timestep: 1 m/s for 1s
        m_speeds = new ChassisSpeeds(1.0, 0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        // Second timestep: 1 m/s for another 1s
        m_currentTime = 2.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(2.0, gt.getX(), 1e-6, "Pose should accumulate over timesteps");
    }

    @Test
    void updateGroundTruthPose_zeroDeltaTime_noPoseChange() {
        GroundTruthSim sim = createSim();

        // Same timestamp as construction — deltaTime = 0
        m_speeds = new ChassisSpeeds(100, 100, 100);
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6);
        assertEquals(0.0, gt.getY(), 1e-6);
    }

    @Test
    void updateGroundTruthPose_diagonalMotion() {
        GroundTruthSim sim = createSim();

        // Drive diagonally: 1 m/s forward, 1 m/s left for 1s
        m_speeds = new ChassisSpeeds(1.0, 1.0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(1.0, gt.getX(), 1e-6);
        assertEquals(1.0, gt.getY(), 1e-6);
    }

    @Test
    void updateGroundTruthPose_negativeVelocity() {
        GroundTruthSim sim = createSim();

        // Drive backwards at 1 m/s for 1s
        m_speeds = new ChassisSpeeds(-1.0, 0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(-1.0, gt.getX(), 1e-6, "Negative velocity should move in -X");
        assertEquals(0.0, gt.getY(), 1e-6);
    }

    @Test
    void updateGroundTruthPose_heading180_reversesFieldDirection() {
        GroundTruthSim sim = createSim();

        // Start at 180° heading (facing field -X)
        sim.resetGroundTruthPoseForSim(
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

        m_speeds = new ChassisSpeeds(1.0, 0, 0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(-1.0, gt.getX(), 1e-6, "At 180° robot-forward maps to field -X");
        assertEquals(0.0, gt.getY(), 1e-2, "sin(π) ≈ 0");
    }

    // ── resetGroundTruthPoseForSim tests ─────────────────────────────

    @Test
    void resetGroundTruthPoseForSim_setsToGivenPose() {
        GroundTruthSim sim = createSim();

        Pose2d targetPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
        sim.resetGroundTruthPoseForSim(targetPose);

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(5.0, gt.getX(), 1e-6);
        assertEquals(3.0, gt.getY(), 1e-6);
        assertEquals(45.0, gt.getRotation().getDegrees(), 1e-6);
    }

    @Test
    void resetGroundTruthPoseForSim_resetsDistanceCounters() {
        GroundTruthSim sim = createSim();

        // Drive to accumulate distance
        m_speeds = new ChassisSpeeds(1.0, 0, 1.0);
        m_currentTime = 1.0;
        sim.updateGroundTruthPose();

        // Reset
        sim.resetGroundTruthPoseForSim(new Pose2d());

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6, "Pose X should be reset");
        assertEquals(0.0, gt.getY(), 1e-6, "Pose Y should be reset");
    }

    // ── resetAllPosesToSelectedAutoPos tests ─────────────────────────

    @Test
    void resetAllPoses_blueAlliance_noFlip() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);

            sim.resetAllPosesToSelectedAutoPos(bluePose, true);

            verify(m_mockPoseResetConsumer).accept(bluePose);
            // flipFieldPose should NOT have been called
            mockedAlliance.verify(
                () -> AllianceCalc.flipFieldPose(any()), never());
        }
    }

    @Test
    void resetAllPoses_redAlliance_flips() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        Pose2d flippedPose = new Pose2d(14, 5, Rotation2d.fromDegrees(150));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(true);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(bluePose))
                .thenReturn(flippedPose);

            sim.resetAllPosesToSelectedAutoPos(bluePose, true);

            verify(m_mockPoseResetConsumer).accept(flippedPose);
        }
    }

    @Test
    void resetAllPoses_wrongSide_invertedFlip() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        Pose2d flippedPose = new Pose2d(14, 5, Rotation2d.fromDegrees(150));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            // Blue alliance + useCorrectTeamSide=false → should flip (wrong side)
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(bluePose))
                .thenReturn(flippedPose);

            sim.resetAllPosesToSelectedAutoPos(bluePose, false);

            // We're blue but asking for wrong side, so it flips
            verify(m_mockPoseResetConsumer).accept(flippedPose);
        }
    }

    @Test
    void resetAllPoses_redAndWrongSide_noFlip() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            // Red alliance + useCorrectTeamSide=false → should NOT flip
            // (wrong side for red = blue side = no flip needed)
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(true);

            sim.resetAllPosesToSelectedAutoPos(bluePose, false);

            verify(m_mockPoseResetConsumer).accept(bluePose);
            mockedAlliance.verify(
                () -> AllianceCalc.flipFieldPose(any()), never());
        }
    }

    // ── cycleResetPosition tests ─────────────────────────────────────

    @Test
    void cycleResetPosition_fullCycle_fourCalls() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(any()))
                .thenAnswer(inv -> {
                    Pose2d p = inv.getArgument(0);
                    return new Pose2d(-p.getX(), -p.getY(), p.getRotation());
                });

            // All 4 calls at the same time (no timeout)
            for (int i = 0; i < 4; i++) {
                sim.cycleResetPosition(bluePose);
            }

            ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
            verify(m_mockPoseResetConsumer, times(4)).accept(captor.capture());

            List<Pose2d> poses = captor.getAllValues();

            // State 0: auto pose, correct side (blue=no flip) → bluePose
            assertEquals(bluePose.getX(), poses.get(0).getX(), 1e-6, "State 0 X");

            // State 1: auto pose, wrong side (blue=flip) → flipped bluePose
            assertEquals(-bluePose.getX(), poses.get(1).getX(), 1e-6, "State 1 X (flipped)");

            // State 2: origin, correct side → (0,0)
            assertEquals(0.0, poses.get(2).getX(), 1e-6, "State 2 X (origin)");

            // State 3: origin, wrong side → flipped origin
            assertEquals(0.0, poses.get(3).getX(), 1e-6, "State 3 X (flipped origin)");
        }
    }

    @Test
    void cycleResetPosition_wrapsAroundAfterFour() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(any()))
                .thenAnswer(inv -> inv.getArgument(0));

            // Call 5 times — 5th should wrap back to state 0
            for (int i = 0; i < 5; i++) {
                sim.cycleResetPosition(bluePose);
            }

            verify(m_mockPoseResetConsumer, times(5)).accept(any());
        }
    }

    @Test
    void cycleResetPosition_timeoutResetsCycleToZero() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(any()))
                .thenAnswer(inv -> inv.getArgument(0));

            // Call at t=0 → state 0 (advances to 1)
            sim.cycleResetPosition(bluePose);

            // Advance past CYCLE_TIMEOUT_SECONDS (2.0s) → resets to state 0
            m_currentTime = 3.0;
            sim.cycleResetPosition(bluePose);

            // Both calls executed state 0 → blue pose (correct side, no flip)
            ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
            verify(m_mockPoseResetConsumer, times(2)).accept(captor.capture());

            assertEquals(bluePose.getX(), captor.getAllValues().get(0).getX(), 1e-6);
            assertEquals(bluePose.getX(), captor.getAllValues().get(1).getX(), 1e-6);
        }
    }

    @Test
    void cycleResetPosition_noTimeoutWhenCalledWithinWindow() {
        GroundTruthSim sim = createSim();
        Pose2d bluePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));

        try (MockedStatic<AllianceCalc> mockedAlliance = mockStatic(AllianceCalc.class)) {
            mockedAlliance.when(AllianceCalc::isRedAlliance).thenReturn(false);
            mockedAlliance.when(() -> AllianceCalc.flipFieldPose(any()))
                .thenAnswer(inv -> {
                    Pose2d p = inv.getArgument(0);
                    return new Pose2d(-p.getX(), -p.getY(), p.getRotation());
                });

            // Call at t=0 → state 0
            sim.cycleResetPosition(bluePose);

            // Call at t=1.5 (within timeout) → state 1
            m_currentTime = 1.5;
            sim.cycleResetPosition(bluePose);

            ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
            verify(m_mockPoseResetConsumer, times(2)).accept(captor.capture());

            // First call: state 0 (correct side) → unflipped
            assertEquals(bluePose.getX(), captor.getAllValues().get(0).getX(), 1e-6);

            // Second call: state 1 (wrong side) → flipped
            assertEquals(-bluePose.getX(), captor.getAllValues().get(1).getX(), 1e-6);
        }
    }

    // ── injectDriftToPoseEstimate tests ─────────────────────────────

    @Test
    void injectDriftToPoseEstimate_callsResetPoseOnDrivetrain() {
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(5, 5, new Rotation2d());
        sim.injectDriftToPoseEstimate(1.0, 0.0, 10.0);

        verify(m_mockDrivetrainResetPose).accept(any(Pose2d.class));
    }

    @Test
    void injectDriftToPoseEstimate_translationDistanceMatchesOffset() {
        GroundTruthSim sim = createSim();

        Pose2d startPose = new Pose2d(5, 5, new Rotation2d());
        m_estimatedPose = startPose;

        sim.injectDriftToPoseEstimate(2.0, 0.0, 0.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        Pose2d drifted = captor.getValue();
        double distance = startPose.getTranslation().getDistance(drifted.getTranslation());
        assertEquals(2.0, distance, 1e-6,
            "Translation drift distance should equal the offset parameter");
    }

    @Test
    void injectDriftToPoseEstimate_robotHeadingZero_xDriftAlongFieldX() {
        // Robot facing field +X (heading=0): local x offset → field +X
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
        sim.injectDriftToPoseEstimate(1.5, 0.0, 0.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        Pose2d drifted = captor.getValue();
        assertEquals(3.0 + 1.5, drifted.getX(), 1e-6, "Heading=0, x-drift → +field X");
        assertEquals(4.0, drifted.getY(), 1e-6, "No field Y change");
    }

    @Test
    void injectDriftToPoseEstimate_robotHeadingNinety_xDriftAlongFieldY() {
        // Robot facing field +Y (heading=90°): local x offset → field +Y
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(3, 4, Rotation2d.fromDegrees(90));
        sim.injectDriftToPoseEstimate(1.0, 0.0, 0.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        Pose2d drifted = captor.getValue();
        assertEquals(3.0, drifted.getX(), 1e-6, "Heading=90°, x-drift → no field X change");
        assertEquals(5.0, drifted.getY(), 1e-6, "Heading=90°, x-drift → +field Y");
    }

    @Test
    void injectDriftToPoseEstimate_negativeRotation() {
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        sim.injectDriftToPoseEstimate(0.0, 0.0, -15.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        assertEquals(-15.0, captor.getValue().getRotation().getDegrees(), 1e-6,
            "Negative rotation offset applied directly");
    }

    @Test
    void injectDriftToPoseEstimate_positiveRotation() {
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        sim.injectDriftToPoseEstimate(0.0, 0.0, 20.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        assertEquals(20.0, captor.getValue().getRotation().getDegrees(), 1e-6,
            "Positive rotation offset applied directly");
    }

    @Test
    void injectDriftToPoseEstimate_doesNotChangeGroundTruth() {
        GroundTruthSim sim = createSim();

        Pose2d groundTruthBefore = sim.getGroundTruthPose();
        m_estimatedPose = new Pose2d(5, 5, new Rotation2d());

        sim.injectDriftToPoseEstimate(3.0, 0.0, 30.0);

        Pose2d groundTruthAfter = sim.getGroundTruthPose();
        assertEquals(groundTruthBefore.getX(), groundTruthAfter.getX(), 1e-6,
            "Ground truth X should not change");
        assertEquals(groundTruthBefore.getY(), groundTruthAfter.getY(), 1e-6,
            "Ground truth Y should not change");
    }

    @Test
    void injectDriftToPoseEstimate_combinedTranslationAndRotation() {
        // Robot at heading=10°, local x=2, rotation=+15° → field offsets via transformBy
        GroundTruthSim sim = createSim();

        m_estimatedPose = new Pose2d(1, 2, Rotation2d.fromDegrees(10));
        sim.injectDriftToPoseEstimate(2.0, 0.0, 15.0);

        ArgumentCaptor<Pose2d> captor = ArgumentCaptor.forClass(Pose2d.class);
        verify(m_mockDrivetrainResetPose).accept(captor.capture());

        Pose2d drifted = captor.getValue();
        // local x=2 along heading=10° → field dx=2*cos(10°), dy=2*sin(10°)
        assertEquals(1 + 2 * Math.cos(Math.toRadians(10)), drifted.getX(), 1e-6);
        assertEquals(2 + 2 * Math.sin(Math.toRadians(10)), drifted.getY(), 1e-6);
        // rotation = 10 + 15 = 25
        assertEquals(25.0, drifted.getRotation().getDegrees(), 1e-6);
    }

    // ── injectDriftToGroundTruth tests ──────────────────────────────

    @Test
    void injectDriftToGroundTruth_doesNotCallDrivetrainResetPose() {
        GroundTruthSim sim = createSim();

        sim.injectDriftToGroundTruth(1.0, 0.0, 10.0);

        verify(m_mockDrivetrainResetPose, never()).accept(any(Pose2d.class));
    }

    @Test
    void injectDriftToGroundTruth_translationDistanceMatchesOffset() {
        GroundTruthSim sim = createSim();

        Pose2d before = sim.getGroundTruthPose();
        sim.injectDriftToGroundTruth(2.0, 0.0, 0.0);

        double distance = before.getTranslation().getDistance(sim.getGroundTruthPose().getTranslation());
        assertEquals(2.0, distance, 1e-6,
            "Ground truth translation drift distance should equal the offset parameter");
    }

    @Test
    void injectDriftToGroundTruth_headingZero_xDriftAlongFieldX() {
        GroundTruthSim sim = createSim();
        sim.resetGroundTruthPoseForSim(new Pose2d(3, 4, Rotation2d.fromDegrees(0)));

        sim.injectDriftToGroundTruth(1.5, 0.0, 0.0);

        Pose2d after = sim.getGroundTruthPose();
        assertEquals(3.0 + 1.5, after.getX(), 1e-6, "Heading=0, x-drift → +field X");
        assertEquals(4.0, after.getY(), 1e-6, "No field Y change");
    }

    @Test
    void injectDriftToGroundTruth_headingNinety_xDriftAlongFieldY() {
        GroundTruthSim sim = createSim();
        sim.resetGroundTruthPoseForSim(new Pose2d(3, 4, Rotation2d.fromDegrees(90)));

        sim.injectDriftToGroundTruth(1.0, 0.0, 0.0);

        Pose2d after = sim.getGroundTruthPose();
        assertEquals(3.0, after.getX(), 1e-6, "Heading=90°, x-drift → no field X change");
        assertEquals(5.0, after.getY(), 1e-6, "Heading=90°, x-drift → +field Y");
    }

    @Test
    void injectDriftToGroundTruth_negativeRotation() {
        GroundTruthSim sim = createSim();
        sim.resetGroundTruthPoseForSim(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        sim.injectDriftToGroundTruth(0.0, 0.0, -15.0);

        assertEquals(-15.0, sim.getGroundTruthPose().getRotation().getDegrees(), 1e-6,
            "Negative rotation offset applied to ground truth");
    }

    @Test
    void injectDriftToGroundTruth_positiveRotation() {
        GroundTruthSim sim = createSim();
        sim.resetGroundTruthPoseForSim(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        sim.injectDriftToGroundTruth(0.0, 0.0, 20.0);

        assertEquals(20.0, sim.getGroundTruthPose().getRotation().getDegrees(), 1e-6,
            "Positive rotation offset applied to ground truth");
    }

    @Test
    void injectDriftToGroundTruth_doesNotChangePoseEstimate() {
        GroundTruthSim sim = createSim();
        m_estimatedPose = new Pose2d(5, 5, new Rotation2d());

        sim.injectDriftToGroundTruth(3.0, 0.0, 30.0);

        // The pose estimator (drivetrain) must never be touched
        verify(m_mockDrivetrainResetPose, never()).accept(any(Pose2d.class));
    }

    @Test
    void injectDriftToGroundTruth_combinedTranslationAndRotation() {
        GroundTruthSim sim = createSim();
        sim.resetGroundTruthPoseForSim(new Pose2d(1, 2, Rotation2d.fromDegrees(10)));

        sim.injectDriftToGroundTruth(2.0, 0.0, 15.0);

        Pose2d after = sim.getGroundTruthPose();
        // local x=2 along heading=10° → field dx=2*cos(10°), dy=2*sin(10°)
        assertEquals(1 + 2 * Math.cos(Math.toRadians(10)), after.getX(), 1e-6);
        assertEquals(2 + 2 * Math.sin(Math.toRadians(10)), after.getY(), 1e-6);
        // rotation = 10 + 15 = 25
        assertEquals(25.0, after.getRotation().getDegrees(), 1e-6);
    }

    // ── simulationPeriodic tests ─────────────────────────────────────

    @SuppressWarnings("VariableDeclarationUsageDistance")
    @Test
    void simulationPeriodic_publishesCurrentGroundTruth() {
        GroundTruthSim sim = createSim();

        m_speeds = new ChassisSpeeds(1.0, 0, 0);
        m_estimatedPose = new Pose2d();
        m_currentTime = 0.5;

        // Ground truth integration happens in the high-frequency callback.
        sim.updateGroundTruthPose();

        // simulationPeriodic() should publish the already-integrated pose.
        assertDoesNotThrow(() -> sim.simulationPeriodic());

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.5, gt.getX(), 1e-6, "Ground truth should advance");
    }

    // ── getGroundTruthPose tests ─────────────────────────────────────

    @Test
    void getGroundTruthPose_initiallyOrigin() {
        GroundTruthSim sim = createSim();

        Pose2d gt = sim.getGroundTruthPose();
        assertEquals(0.0, gt.getX(), 1e-6);
        assertEquals(0.0, gt.getY(), 1e-6);
        assertEquals(0.0, gt.getRotation().getDegrees(), 1e-6);
    }
}
