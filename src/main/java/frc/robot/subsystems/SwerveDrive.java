// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  AHRS m_gyro = new AHRS(SPI.Port.kMXP); 

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule(frontLeftTranslation);
    m_frontRight = new SwerveModule(frontRightTranslation);
    m_backLeft = new SwerveModule(backLeftTranslation);
    m_backRight = new SwerveModule(backRightTranslation);
    m_kinematics = new SwerveDriveKinematics(
      frontLeftTranslation, frontRightTranslation, backLeftTranslation, backRightTranslation);
    m_odometry = new SwerveDriveOdometry(
      m_kinematics, m_gyro.getRotation2d(),
      getModulePositions());
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public void move(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    m_frontLeft.setTarget(states[0]);
    // TODO
  }

  public void moveDriverRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySidewaySpeedMetersPerSecond,
        -xForwardSpeedMetersPerSecond, omegaRadianPerSecond, new Rotation2d()); // TODO replace Rotation2d
    move(speeds);
  }

  public void moveRobotRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = new ChassisSpeeds(xForwardSpeedMetersPerSecond, ySidewaySpeedMetersPerSecond,
        omegaRadianPerSecond);
    move(speeds);
  }

  public Rotation2d getRotation() {
    return m_gyro.getRotation2d();
  }

  public void stop(){
    // TODO add stop code
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
