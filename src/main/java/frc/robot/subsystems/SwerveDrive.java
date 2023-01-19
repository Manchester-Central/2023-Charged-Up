// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private Field2d m_field;
  private Rotation2d m_simrotation = new Rotation2d();

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2,-SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule(frontLeftTranslation);
    m_frontRight = new SwerveModule(frontRightTranslation);
    m_backLeft = new SwerveModule(backLeftTranslation);
    m_backRight = new SwerveModule(backRightTranslation);
    m_kinematics = new SwerveDriveKinematics(
        getModuleTranslations());
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, getRotation(),
        getModulePositions());
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      m_frontLeft.getTranslation(),
      m_frontRight.getTranslation(),
      m_backLeft.getTranslation(),
      m_backRight.getTranslation()
    };
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getModuleState(),
      m_frontRight.getModuleState(),
      m_backLeft.getModuleState(),
      m_backRight.getModuleState()
    };
  }

  public void move(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    m_frontLeft.setTarget(states[0]);
    m_frontRight.setTarget(states[1]);
    m_backLeft.setTarget(states[2]);
    m_backRight.setTarget(states[3]);
  }


  public void moveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds=ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getRotation());
    move(speeds);
  }
  public void moveRobotRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = new ChassisSpeeds(xForwardSpeedMetersPerSecond, ySidewaySpeedMetersPerSecond,
        omegaRadianPerSecond);
    move(speeds);
  }

  public Rotation2d getRotation() {
    if (Robot.isSimulation()) {
      return m_simrotation;
    }

    return m_gyro.getRotation2d();
    
  }

  public void stop() {
    // TODO add stop code
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    if (Robot.isSimulation()) {
      ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
          m_backLeft.getModuleState(), m_backRight.getModuleState());
      double radians = speeds.omegaRadiansPerSecond / Constants.UpdateFrequency_Hz;
      m_simrotation = m_simrotation.plus(Rotation2d.fromRadians(radians));
    }
    Pose2d robotPose = m_odometry.update(getRotation(), getModulePositions());
    m_field.setRobotPose(robotPose);
    updateModuleOnField(m_frontLeft, robotPose, "FL");
    updateModuleOnField(m_frontRight, robotPose, "FR");
    updateModuleOnField(m_backLeft, robotPose, "BL");
    updateModuleOnField(m_backRight, robotPose, "BR");
    SmartDashboard.putNumber("angle", getRotation().getDegrees());

  }

  public void updateModuleOnField(SwerveModule swerveModule, Pose2d robotPose, String name) {
    Transform2d transform = new Transform2d(swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(name).setPose(swerveModulePose);
  }
}