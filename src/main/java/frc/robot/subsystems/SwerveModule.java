// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private Translation2d m_translation;
  private double m_simdistance;
  private SwerveModuleState m_targetState;
  private TalonFX m_angle;
  private TalonFX m_velocity;

  /** Creates a new SwerveModule. */
  public SwerveModule(Translation2d translation, int canIdAngle, int canIdVelocity) {
    m_translation = translation;
    m_simdistance = 0;
    m_targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    m_angle = new TalonFX(canIdAngle);
    m_velocity = new TalonFX(canIdVelocity);
  }

  public void setTarget(SwerveModuleState state) {
    double convertedAngle = degreesToEncoder(state.angle.getDegrees());
    double convertedVelocity = distanceMetersToEncoders(state.speedMetersPerSecond);
    m_angle.set(TalonFXControlMode.Position, convertedAngle);
    m_velocity.set(TalonFXControlMode.Velocity, convertedVelocity);
    m_targetState = state;
  }

  public SwerveModuleState getModuleState() {
    if (Robot.isSimulation()) {
      return m_targetState;
    }
    double velocity = m_velocity.getSensorCollection().getIntegratedSensorVelocity();
    double angle = m_angle.getSensorCollection().getIntegratedSensorAbsolutePosition();
    velocity = encoderToDistanceMeters(velocity);
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
    double distance = m_velocity.getSensorCollection().getIntegratedSensorPosition();
    double angle = m_angle.getSensorCollection().getIntegratedSensorAbsolutePosition();
    distance = encoderToDistanceMeters(distance);
    angle = encoderToDegrees(angle);
    return new SwerveModulePosition(distance, Rotation2d.fromDegrees(angle));
  }

  public double encoderToDegrees(double angle) {
    return angle * (360.0 / SwerveConstants.AngleEncoderRatio);
  }

  public double degreesToEncoder(double encoderTicks) {
    return encoderTicks * (SwerveConstants.AngleEncoderRatio / 360.0);
  }

  public double encoderToDistanceMeters(double counts) {
    return (counts / SwerveConstants.VelocityEncoderRatio);
  }

  public double distanceMetersToEncoders(double distance) {
    return distance * SwerveConstants.VelocityEncoderRatio;
  }
}
