package frc.robot.subsystems.elevator;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkFlex elevatorMotor1;
  private final SparkFlex elevatorMotor2;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController elevatorController;
  private double setpointHeight = 0.0;
  private boolean positionControlEnabled = false;

  public Elevator() {
    elevatorMotor1 = new SparkFlex(10, MotorType.kBrushless);
    elevatorMotor2 = new SparkFlex(11, MotorType.kBrushless);
    relativeEncoder = elevatorMotor1.getEncoder();
    elevatorController = elevatorMotor1.getClosedLoopController();

    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .inverted(true);

    leaderConfig.encoder
        .positionConversionFactor(2.0 * Math.PI * Units.inchesToMeters(1.75) / 5.0)
        .velocityConversionFactor(2.0 * Math.PI * Units.inchesToMeters(1.75) / 5.0 / 60.0);

    leaderConfig.closedLoop
        .feedbackSe nsor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.83, 0.0, 0.0)
        .outputRange(-1.0, 1.0);

    leaderConfig.closedLoop.feedForward
        .kS(0.584)
        .kG(0.366)
        .kV(2.15)
        .kA(0.05);

    leaderConfig.closedLoop.maxMotion
        .cruiseVelocity(0.75)
        .maxAcceleration(1.0)
        .allowedProfileError(0.01);

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .follow(elevatorMotor1, true);

    elevatorMotor1.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //The elevator MUST start at its zero height.
    relativeEncoder.setPosition(0.0);
  }

  public double getElevatorHeight() {
    return relativeEncoder.getPosition();
  }

  public double getElevatorVelocity() {
    return relativeEncoder.getVelocity();
  }

  public void setDesiredHeight(double heightMeters) {
    setpointHeight = MathUtil.clamp(heightMeters, 0.0, 1.44);
    elevatorController.setSetpoint(
        setpointHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    positionControlEnabled = true;
  }

  public void setElevatorSpeed(double speed) {
    positionControlEnabled = false;
    elevatorMotor1.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void setElevatorVoltage(double voltage) {
    positionControlEnabled = false;
    elevatorMotor1.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public void stop() {
    positionControlEnabled = false;
    elevatorMotor1.stopMotor();
  }

  public void zeroEncoder() {
    stop();
    relativeEncoder.setPosition(0.0);
    setpointHeight = 0.0;
  }

  public boolean isAtSetpoint() {
    return positionControlEnabled
        && Math.abs(getElevatorHeight() - setpointHeight) < Units.inchesToMeters(1);
  }

  @Override
  public void periodic() {
  }
}



