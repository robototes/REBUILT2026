package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.Telemetry;
import frc.robot.subsystems.Launcher.HoodSim;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.NtTunableDouble;
import frc.robot.util.ShotCalculator;
import frc.robot.util.TurretSubsystemSim;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.*;

public class TurretSubsystem extends SubsystemBase {

        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private double triggerTolerance = 5;
        @Getter private double unwrapTolerance = 10;

        /* Turret config settings */
        @Getter private final double zeroVoltage = 0.7;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double supplyCurrentLimit = 5;
        @Getter private final double statorCurrentLimit = 10;
        @Getter private final double positionKp = 1000;
        @Getter private final double positionKd = 175;
        @Getter private final double positionKv = 0.15;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        @Getter @Setter private double sensorToMechanismRatio = 22.4;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 22.4;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 1;

        @Getter @Setter private double maxRotations = 1;
        @Getter @Setter private double minRotations = -1;

        @Getter @Setter private boolean turretZeroed = false;


        TalonFX motor = new TalonFX(Hardware.TURRET_MOTOR_ID);
        private TurretSim sim;
        private DoublePublisher positionPub;
        private DoublePublisher goalPub;

        private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);
        private final MotionMagicVoltage request = new MotionMagicVoltage(0);

        private final CommandSwerveDrivetrain drive;


    private void configureMotors() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator configurator = motor.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // create PID gains
    config.Slot0.kP = positionKp;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = positionKd;
    config.Slot0.kA = positionKa;
    config.Slot0.kV = positionKv;
    config.Slot0.kS = positionKs;
    config.Slot0.kG = positionKg;

    config.MotionMagic.MotionMagicAcceleration = mmAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = mmCruiseVelocity;
    config.MotionMagic.MotionMagicJerk = mmJerk;

    config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
    config.Feedback.RotorToSensorRatio = rotorToSensorRatio;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMaxRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMinRotations();

    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configurator.apply(config);
  }
  public void initNT(){
    var nt = NetworkTableInstance.getDefault();
    positionPub = nt.getDoubleTopic("/turret/position").publish();
    positionPub.set(0);
    goalPub = nt.getDoubleTopic("/turret/goal").publish();
    goalPub.set(request.Position);


  }



    public TurretSubsystem(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        motor = new TalonFX(Hardware.TURRET_MOTOR_ID);
        configureMotors();
        initNT();

        if (RobotBase.isSimulation()) {
      sim = new TurretSim(motor, this);
    }
    }

  public Command voltageControl(Supplier<Voltage> voltageSupplier) {
    return runEnd(
            () -> {
              // hood.setVoltage(voltageSupplier.get().in(Units.Volts));
              motor.setControl(voltageRequest.withOutput(voltageSupplier.get()));
            },
            () -> {
              motor.stopMotor();
            })
        .withName("Voltage Control");
  }
  public void zero() {
    motor.setPosition(0);
    turretZeroed = true;
  }

  public Command zeroCommand() {
    return runOnce(this::zero).withName("Zeroing Turret");
  }
    public Command autoZeroCommand() {
        return Commands.parallel(voltageControl(() -> Volts.of(zeroVoltage)))
        .until(() -> motor.getStatorCurrent().getValueAsDouble() >= (statorCurrentLimit - 1))
        .andThen(zeroCommand())
        .withTimeout(3)
        .withName("Automatic Zero turret");
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    // Choose the best equivalent in degrees that lies inside the configured soft-limits.
    // If no equivalent exists in the soft-limit window (soft window < 360°), clamp to nearest endpoint.
    private double wrapDegreesToSoftLimits(double targetDegrees) {

        double minDeg = getMinRotations() * 360.0;
        double maxDeg = getMaxRotations() * 360.0;
        double currentDeg = motor.getPosition().getValueAsDouble() * 360.0;

        // Solve for integer n such that minDeg <= targetDegrees + 360*n <= maxDeg
        int nMin = (int) Math.ceil((minDeg - targetDegrees) / 360.0);
        int nMax = (int) Math.floor((maxDeg - targetDegrees) / 360.0);

        if (nMin <= nMax) {
            // At least one equivalent fits in soft limits.
            int nClosest = (int) Math.round((currentDeg - targetDegrees) / 360.0);
            int n = Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
            return targetDegrees + n * 360.0;
        } else {
            // No equivalent fits in soft limits -> clamp to nearest soft limit endpoint.
            double toMin = Math.abs(currentDeg - minDeg);
            double toMax = Math.abs(currentDeg - maxDeg);
            return (toMin < toMax) ? minDeg : maxDeg;
        }
    }

    public void aimFieldRelative(Rotation2d fieldAngle) {
        double robotHeadingDeg = drive.getState().Pose.getRotation().getDegrees();
        double turretDeg = fieldAngle.getDegrees() - robotHeadingDeg;
        final double wrappedTurretDeg = wrapDegreesToSoftLimits(turretDeg);

        motor.setControl(request.withPosition(Units.degreesToRotations(wrappedTurretDeg)));

    }

    public Command trackTargetCommand() {
        return run(() -> {
            var params = ShotCalculator.getInstance(drive).getParameters();
            aimFieldRelative(params.turretAngle());
        });
    }

    /** Holds the position of the Turret. */
    public Command runHoldTurret() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Turret.holdPosition");
                addRequirements(TurretSubsystem.this);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void initialize() {
                holdPosition = motor.getPosition().getValueAsDouble();
                motor.stopMotor();
            }

            @Override
            public void execute() {
                if (Math.abs(motor.getVelocity().getValueAsDouble() * 60) > holdMaxSpeedRPM) {
                    motor.stopMotor();
                    holdPosition = motor.getPosition().getValueAsDouble();
                } else {
                    motor.setControl(request.withPosition(holdPosition));
                }
            }

            @Override
            public void end(boolean interrupted) {
                motor.stopMotor();
            }
        };
    }

    public Command moveToDegrees(DoubleSupplier degrees) {
        return run(() -> motor.setControl(request.withPosition(Units.degreesToRotations(wrapDegreesToSoftLimits(degrees.getAsDouble()))))).withName("runPoseDegrees");
    }

    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(((Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) % 360) + 360) % 360 - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

}
