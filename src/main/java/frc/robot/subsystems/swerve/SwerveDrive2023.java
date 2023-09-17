// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;
import com.chaos131.swerve.BaseSwerveDrive;
import com.chaos131.swerve.BaseSwerveModule;
import com.chaos131.swerve.SwerveConfigs;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;

public class SwerveDrive2023 extends BaseSwerveDrive {

  private AHRS m_gyro;
  private Limelight m_limelightLeft;
  private Limelight m_limelightRight;

  private SwerveDrive2023(BaseSwerveModule[] modules, SwerveConfigs config, AHRS gyro, Limelight limelightLeft, Limelight limelightRight) {
    super(modules, config, () -> gyro.getRotation2d());
    m_gyro = gyro;
    m_limelightLeft = limelightLeft;
    m_limelightRight = limelightRight;
  }

  public static SwerveDrive2023 CreateSwerveDrive(Limelight limelightLeft, Limelight limelightRight) {
    Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2,-SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    var frontLeft = new SwerveModule2023(
      "FL",
      frontLeftTranslation,
      SwerveConstants.CanIdFrontLeftAngle,
      SwerveConstants.CanIdFrontLeftVelocity,
      SwerveConstants.AnalogInputFrontLeftAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetFrontLeft,
      Rotation2d.fromDegrees(45)
    );
    var frontRight = new SwerveModule2023(
      "FR",
      frontRightTranslation,
      SwerveConstants.CanIdFrontRightAngle,
      SwerveConstants.CanIdFrontRightVelocity,
      SwerveConstants.AnalogInputFrontRightAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetFrontRight,
      Rotation2d.fromDegrees(315)
    );
    var backLeft = new SwerveModule2023(
      "BL",
      backLeftTranslation,
      SwerveConstants.CanIdBackLeftAngle,
      SwerveConstants.CanIdBackLeftVelocity,
      SwerveConstants.AnalogInputBackLeftAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetBackLeft,
      Rotation2d.fromDegrees(135)
    );
      var backRight = new SwerveModule2023(
      "BR",
      backRightTranslation,
      SwerveConstants.CanIdBackRightAngle,
      SwerveConstants.CanIdBackRightVelocity,
      SwerveConstants.AnalogInputBackRightAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetBackRight,
      Rotation2d.fromDegrees(225)
    );
    BaseSwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    var gyro = new AHRS(SPI.Port.kMXP);

    var swerveConfigs = new SwerveConfigs()
      // Max speeds
      .setMaxRobotSpeed_mps(SwerveConstants.MaxRobotSpeed_mps)
      .setMaxRobotRotation_radps(SwerveConstants.MaxRobotRotation_radps)
      // Translation PID settings
      .setDefaultTranslationPIDValues(new PIDValue(0.6, 0.05, 0.1))
      .setDefaultDriveToTargetTolerance(SwerveConstants.DriveToTargetTolerance)
      // Rotation PID settings
      .setDefaultRotationPIDValues(new PIDValue(0.01, 0.0001, 0.00))
      .setDefaultRotationTolerance(SwerveConstants.AnglePIDTolerance)
      // Module PID settings
      .setDefaultModuleAnglePIDValues(new PIDValue(0.2, 0, 0))
      .setDefaultModuleVelocityPIDFValues(new PIDFValue(0.0375, 0.0, 0.0, 0.054))
      // Debug/simulation settings
      .setDebugMode(true)
      .setUpdateFrequency_hz(Constants.UpdateFrequency_Hz);

    return new SwerveDrive2023(modules, swerveConfigs, gyro, limelightLeft, limelightRight);
  }

  public Pose2d getPoseFromLimelights() {
    Pose2d LLLeftPose = m_limelightLeft.getPose();
    Pose2d LLRightPose = m_limelightRight.getPose();
    if (LLLeftPose != null && LLRightPose != null){
      double leftDistance = Math.abs(m_limelightLeft.getTargetXDistancePixels());
      double rightDistance = Math.abs(m_limelightRight.getTargetXDistancePixels());
      if (leftDistance < rightDistance){
        return LLLeftPose;
      }
      else{
        return LLRightPose;
      }
    }
    else if (LLRightPose != null){
      return LLRightPose;
    }
    else if (LLLeftPose != null){
      return LLLeftPose;
    }
    return null;
  }

  public void updatePoseFromLimelights() {
    var limelightPose = getPoseFromLimelights();
    if(limelightPose != null) {
      var poseWithGyro = new Pose2d(limelightPose.getX(), limelightPose.getY(), getOdometryRotation());
      addVisionMeasurement(poseWithGyro);
    }
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(m_gyro.getRoll());
  }
}
// “I love polyester.” -Kenny