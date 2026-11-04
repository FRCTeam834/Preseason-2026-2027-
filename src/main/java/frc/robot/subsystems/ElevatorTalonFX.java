package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
 
  // 5 motor turns = 1 drum turn = 0.279 meters travelled
  // For every 5 rotations of the motor, the drum, the spool that winds up the elevator’s rope or                
  // belt to life it, makes one full rotation
  private static final double METERS_PER_MOTOR_ROTATION =
      2.0 * Math.PI * Units.inchesToMeters(1.75) / 5.0;


  private static final double MAX_HEIGHT_METERS = 1.44;

  private final TalonFX elevatorMotor1;
  private final TalonFX elevatorMotor2;

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(0);

  private final DutyCycleOut speedRequest = new DutyCycleOut(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private double setpointHeight = 0.0;
  private boolean positionControlEnabled = false;

  public Elevator() {
    elevatorMotor1 = new TalonFX(10);
    elevatorMotor2 = new TalonFX(11);

    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Initial direction choice; verify that positive output raises the elevator.
    leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leaderConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leaderConfig.Voltage.PeakForwardVoltage = 12.0;
    leaderConfig.Voltage.PeakReverseVoltage = -12.0;

 .
    // Convert between rotations and meters in this subsystem.
    leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
    leaderConfig.Feedback.RotorToSensorRatio = 1.0;

    // Starting conversion of REV duty-cycle PID to voltage PID:
    // REV kP: duty cycle / meter
    // Phoenix kP: volts / motor rotation
    leaderConfig.Slot0.kP =
        0.83 * 12.0 * METERS_PER_MOTOR_ROTATION;
    leaderConfig.Slot0.kI = 0.0;
    leaderConfig.Slot0.kD = 0.0;

    // Assumes your original feedforward gains use volts and meter-based units.
    leaderConfig.Slot0.kS = 0.584;
    leaderConfig.Slot0.kG = 0.366;
    leaderConfig.Slot0.kV = 2.15 * METERS_PER_MOTOR_ROTATION;
    leaderConfig.Slot0.kA = 0.05 * METERS_PER_MOTOR_ROTATION;
    leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Phoenix uses rotations/second and rotations/second^2.
    leaderConfig.MotionMagic.MotionMagicCruiseVelocity =
        0.75 / METERS_PER_MOTOR_ROTATION;
    leaderConfig.MotionMagic.MotionMagicAcceleration =
        1.0 / METERS_PER_MOTOR_ROTATION;

    // Zero jerk selects a trapezoidal motion profile.
    leaderConfig.MotionMagic.MotionMagicJerk = 0.0;

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();

    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    followerConfig.Voltage.PeakForwardVoltage = 12.0;
    followerConfig.Voltage.PeakReverseVoltage = -12.0;

    elevatorMotor1.getConfigurator().apply(leaderConfig);
    elevatorMotor2.getConfigurator().apply(followerConfig);

    // Update both output signals because manual control uses duty cycle
    // while position/voltage control uses voltage.
    elevatorMotor1.getDutyCycle().setUpdateFrequency(50.0);
    elevatorMotor1.getMotorVoltage().setUpdateFrequency(50.0);

    // Equivalent intent to REV follow(elevatorMotor1, true).
    elevatorMotor2.setControl(
        new Follower(
            elevatorMotor1.getDeviceID(),
            MotorAlignmentValue.Opposed));

    // The elevator MUST start at its zero height.
    elevatorMotor1.setPosition(0.0);
  }

  public double getElevatorHeight() {
    return elevatorMotor1.getPosition().getValueAsDouble()
        * METERS_PER_MOTOR_ROTATION;
  }

  public double getElevatorVelocity() {
    return elevatorMotor1.getVelocity().getValueAsDouble()
        * METERS_PER_MOTOR_ROTATION;
  }

  public void setDesiredHeight(double heightMeters) {
    setpointHeight =
        MathUtil.clamp(heightMeters, 0.0, MAX_HEIGHT_METERS);

    elevatorMotor1.setControl(
        positionRequest.withPosition(
            setpointHeight / METERS_PER_MOTOR_ROTATION));

    positionControlEnabled = true;
  }

  public void setElevatorSpeed(double speed) {
    positionControlEnabled = false;

    elevatorMotor1.setControl(
        speedRequest.withOutput(MathUtil.clamp(speed, -1.0, 1.0)));
  }

  public void setElevatorVoltage(double voltage) {
    positionControlEnabled = false;

    elevatorMotor1.setControl(
        voltageRequest.withOutput(
            MathUtil.clamp(voltage, -12.0, 12.0)));
  }

  public void stop() {
    positionControlEnabled = false;
    elevatorMotor1.stopMotor();
  }

  public void zeroEncoder() {
    stop();
    elevatorMotor1.setPosition(0.0);
    setpointHeight = 0.0;
  }

  public boolean isAtSetpoint() {
    return positionControlEnabled
        && Math.abs(getElevatorHeight() - setpointHeight)
            < Units.inchesToMeters(1);
  }

  @Override
  public void periodic() {
  }
}

