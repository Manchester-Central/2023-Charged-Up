// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;
import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private Translation2d m_translation;
  private double m_simdistance;
  private SwerveModuleState m_targetState;
  private TalonFX m_angle;
  private TalonFX m_velocity;
  private CANCoder m_2022AbsoluteCanCoder;
  private double m_absoluteAngleOffset2022;

  /** Creates a new SwerveModule. */
  public SwerveModule(Translation2d translation, int canIdAngle, int canIdVelocity, int canAbsoluteID2022, double absoluteAngleOffset2022) {
    m_translation = translation;
    m_simdistance = 0;
    m_targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    m_angle = new TalonFX(canIdAngle);
    m_angle.setInverted(TalonFXInvertType.Clockwise);
    m_absoluteAngleOffset2022 = absoluteAngleOffset2022;
    m_velocity = new TalonFX(canIdVelocity);
    if (Constants.Is2022Robot) {
      m_2022AbsoluteCanCoder = new CANCoder(canAbsoluteID2022);
    }
  }

  public void setTarget(SwerveModuleState state) {
    double convertedAngle = degreesToEncoder(state.angle.getDegrees());
    double convertedVelocity = meterPerSecondToVelocityUnit(state.speedMetersPerSecond);
    m_angle.set(TalonFXControlMode.Position, convertedAngle);
    m_velocity.set(TalonFXControlMode.Velocity, convertedVelocity);
    m_targetState = state;
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

  public void getModuleInfo(String name) {
    SmartDashboard.putNumber("Swerve Module " + name + "/Angle", getModuleState().angle.getDegrees());
    SmartDashboard.putNumber("Swerve Module " + name + "/Speed", getModuleState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module " + name + "/VelocityEncoderPosition", m_velocity.getSelectedSensorPosition());
    SmartDashboard.putNumber("Swerve Module " + name + "/AngleEncoder", m_angle.getSelectedSensorPosition());
    SmartDashboard.putNumber("Swerve Module " + name + "/Position", getPosition().distanceMeters);
    SmartDashboard.putNumber("Swerve Module " + name + "/VelocityEncoderVelocity", m_velocity.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Swerve Module " + name + "/AbsoluteAngle", getAbsoluteAngle());
  }

  public double encoderToDegrees(double counts) {
    counts = counts / 2048;
    counts = counts / SwerveConstants.AngleEncoderRatio;
    return counts * 360;
  }

  public double degreesToEncoder(double angle) {
    angle = angle * 2048;
    angle = angle * SwerveConstants.AngleEncoderRatio;
    return angle / 360;

  }

  public double encoderToDistanceMeters(double counts) {
    counts = counts / 2048;
    counts = counts / SwerveConstants.VelocityEncoderRatio;
    return counts * SwerveConstants.WheelCircumference;

  }

  public double distanceMetersToEncoders(double distance) {
    distance = distance * 2048;
    distance = distance * SwerveConstants.VelocityEncoderRatio;
    return distance / SwerveConstants.WheelCircumference;

  }
  public double velocityUnitToMeterPerSecond(double velocityUnit){
    return encoderToDistanceMeters(velocityUnit) * 10;
  }
  public double meterPerSecondToVelocityUnit(double metersPerSecond){
    return distanceMetersToEncoders(metersPerSecond) / 10;
  }
  public double getAbsoluteAngle(){
    return m_2022AbsoluteCanCoder.getAbsolutePosition() - m_absoluteAngleOffset2022;
  }
}