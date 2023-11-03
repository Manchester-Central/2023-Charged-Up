// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.chaos131.pid.PIDUpdate;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;
import frc.robot.util.DashboardNumber;

public abstract class SwerveModule {
  private Translation2d m_translation;
  private double m_simdistance;
  private SwerveModuleState m_targetState;
  private WPI_TalonFX m_angle;
  private WPI_TalonFX m_velocity;
  private double initialEncoder;
  private String m_name;
  private LinearFilter m_absoluteAngleDegreesRollingAverage = LinearFilter.movingAverage(100);
  private double m_absoluteAngleDegreesRollingAverageValue = 0;

  private static DashboardNumber VelocityRampRateDriver = new DashboardNumber("Swerve/VelocityRampRateDriver", 0.2, DebugConstants.EnableDriveDebug, (newValue) -> {});
  private static DashboardNumber VelocityRampRateAuto = new DashboardNumber("Swerve/VelocityRampRateAuto", 0.5, DebugConstants.EnableDriveDebug, (newValue) -> {});

  /** Creates a new SwerveModule. */
  public SwerveModule(String name, Translation2d translation, int canIdAngle, int canIdVelocity, boolean invertDirection) {
    m_name = name;
    m_translation = translation;
    m_simdistance = 0;
    m_targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    m_angle = new WPI_TalonFX(canIdAngle);
    m_velocity = new WPI_TalonFX(canIdVelocity);
    m_velocity.setInverted(invertDirection);
    m_angle.configFactoryDefault();
    m_velocity.configFactoryDefault();
    m_angle.configAllowableClosedloopError(0, degreesToEncoder(0.5));
    m_velocity.setNeutralMode(NeutralMode.Coast);
    m_angle.setNeutralMode(NeutralMode.Brake);
    m_velocity.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);
    m_velocity.configVelocityMeasurementWindow(32);
    m_angle.setInverted(TalonFXInvertType.Clockwise);

    m_velocity.configClosedloopRamp(VelocityRampRateDriver.get());
    m_angle.configClosedloopRamp(0);

    Robot.logManager.addNumber(getDSKey("Velocity_Temp_C"), DebugConstants.EnableDriveDebug, () -> m_velocity.getTemperature());
    Robot.logManager.addNumber(getDSKey("Angle_Temp_C"), DebugConstants.EnableDriveDebug, () -> m_angle.getTemperature());

  }
    
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
    
  }

  public void driverModeInit() {
    m_velocity.configClosedloopRamp(VelocityRampRateDriver.get());
  }

  public void driveToPositionInit() {
    m_velocity.configClosedloopRamp(VelocityRampRateAuto.get());
  }

  public abstract double getAbsoluteAngle();
  public abstract double getAngleEncoderRatio();
  public abstract double getVelocityEncoderRatio();
  public abstract double getWheelCircumference();
  public abstract double getRawAbsoluteAngle();

  public void setTarget(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getModuleState().angle);
    double targetAngle = closestTarget(getModuleState().angle.getDegrees(), state.angle.getDegrees());
    double convertedVelocity = meterPerSecondToVelocityUnit(state.speedMetersPerSecond);

    m_angle.set(TalonFXControlMode.Position, degreesToEncoder(targetAngle));
    m_velocity.set(TalonFXControlMode.Velocity, convertedVelocity);
    m_targetState = state;
  }

  public void stop() {
    m_angle.set(TalonFXControlMode.PercentOutput, 0);
    m_velocity.set(TalonFXControlMode.PercentOutput, 0);
    m_targetState = new SwerveModuleState(0, getModuleState().angle); 
  }

  public SwerveModuleState getModuleState() {
    if (Robot.isSimulation()) {
      return m_targetState;
    }
    double velocity = m_velocity.getSelectedSensorVelocity();
    double angle = m_angle.getSelectedSensorPosition();
    velocity = velocityUnitToMeterPerSecond(velocity);
    angle = encoderToDegrees(angle);
    return new SwerveModuleState(velocity, Rotation2d.fromDegrees(angle));
  }

  public Translation2d getTranslation() {
    return m_translation;
  }

  public SwerveModulePosition getPosition() {
    if (Robot.isSimulation()) {
      m_simdistance = m_simdistance + m_targetState.speedMetersPerSecond / Constants.UpdateFrequency_Hz;
      return new SwerveModulePosition(m_simdistance, m_targetState.angle);
    }
    double distance = m_velocity.getSelectedSensorPosition();
    double angle = m_angle.getSelectedSensorPosition();
    distance = encoderToDistanceMeters(distance);
    angle = encoderToDegrees(angle);
    return new SwerveModulePosition(distance, Rotation2d.fromDegrees(angle));
  }

  public void UpdateVelocityPIDConstants(PIDUpdate update) {
    m_velocity.config_kP(0, update.P);
    m_velocity.config_kI(0, update.I);
    m_velocity.config_kD(0, update.D);
    m_velocity.config_kF(0, update.F);
  }

  public void UpdateAnglePIDConstants(PIDUpdate update) {
    m_angle.config_kP(0, update.P);
    m_angle.config_kI(0, update.I);
    m_angle.config_kD(0, update.D);
  }

  protected String getDSKey(String field) {
    return "Swerve Module " + m_name + "/" + field;
  }

  private void updateDashboard() {
    if (DebugConstants.EnableDriveDebug) {
      SmartDashboard.putNumber(getDSKey("Angle"), getModuleState().angle.getDegrees());
      SmartDashboard.putNumber(getDSKey("Speed"), getModuleState().speedMetersPerSecond);
      SmartDashboard.putNumber(getDSKey("TargetSpeed"), m_targetState.speedMetersPerSecond);
      SmartDashboard.putNumber(getDSKey("Current_amps"), m_velocity.getStatorCurrent());
      // SmartDashboard.putNumber(getDSKey("VelocityEncoderPosition"), m_velocity.getSelectedSensorPosition());
      // SmartDashboard.putNumber(getDSKey("AngleEncoder"), m_angle.getSelectedSensorPosition());
      // SmartDashboard.putNumber(getDSKey("Position"), getPosition().distanceMeters);
      // SmartDashboard.putNumber(getDSKey("VelocityEncoderVelocity"), m_velocity.getSelectedSensorVelocity());
      SmartDashboard.putNumber(getDSKey("AbsoluteAngle"), getAbsoluteAngle());
      SmartDashboard.putNumber(getDSKey("AbsoluteAngleRollingAverage"), m_absoluteAngleDegreesRollingAverageValue);
      // SmartDashboard.putNumber(getDSKey("InitialEncoder"), initialEncoder);
      // SmartDashboard.putNumber(getDSKey("AbsoluteEncoder"), getRawAbsoluteAngle());
    }
  }

  public void periodic() {
    m_absoluteAngleDegreesRollingAverageValue = m_absoluteAngleDegreesRollingAverage.calculate(getAbsoluteAngle());
    updateDashboard();
  }

  public double encoderToDegrees(double counts) {
    counts = counts / 2048;
    counts = counts / getAngleEncoderRatio();
    return counts * 360;
  }

  public double degreesToEncoder(double angle) {
    angle = angle * 2048;
    angle = angle * getAngleEncoderRatio();
    return angle / 360;

  }

  public double encoderToDistanceMeters(double counts) {
    counts = counts / 2048;
    counts = counts / getVelocityEncoderRatio();
    return counts * getWheelCircumference();

  }

  public double distanceMetersToEncoders(double distance) {
    distance = distance * 2048;
    distance = distance * getVelocityEncoderRatio();
    return distance / getWheelCircumference();

  }

  public double velocityUnitToMeterPerSecond(double velocityUnit){
    return encoderToDistanceMeters(velocityUnit) * 10;
  }

  public double meterPerSecondToVelocityUnit(double metersPerSecond){
    return distanceMetersToEncoders(metersPerSecond) / 10;
  }

  public void recalibrate() {
    initialEncoder = degreesToEncoder(m_absoluteAngleDegreesRollingAverageValue);
    m_angle.setSelectedSensorPosition(initialEncoder);
  }

  /**
   * This function takes in the current angle read by the encoder and a target angle for the robot to move to.
   * The target angle will be between -PI and PI, but this function will scale it up so it is an equivalent
   * angle that is closer to the current encoder angle. It will return this "optimized" angle to avoid
   * the wheels overspinning. 
   * @param currentModuleAngle_deg - The current swerve module's angle in degrees
   * @param targetAngle_deg - The target angle in degrees
   * @return The swerve module target angle 
   */
  public static double closestTarget(double currentModuleAngle_deg, double targetAngle_deg) {
    Rotation2d currentModuleAngle = Rotation2d.fromDegrees(currentModuleAngle_deg);
    Rotation2d targetAngle = Rotation2d.fromDegrees(targetAngle_deg);
    Rotation2d angleDifference = currentModuleAngle.minus(targetAngle);
    return currentModuleAngle_deg - angleDifference.getDegrees();
 
  } 


}
//“If you don’t agree to focus, you’re going to the business team.” -Josh 2/21/2023