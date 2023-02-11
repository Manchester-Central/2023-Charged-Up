// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.SwerveConstants2022;
import frc.robot.commands.RecalibrateModules;
import frc.robot.logging.LogManager;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private Field2d m_field;
  private Rotation2d m_simrotation = new Rotation2d();

  private PIDController m_XPid;
  private PIDController m_YPid;
  private PIDController m_AnglePid;
  private PIDTuner m_XPidTuner;
  private PIDTuner m_YPidTuner;
  private PIDTuner m_AnglePidTuner;
  private PIDTuner m_moduleVelocityPIDTuner;
  private PIDTuner m_moduleAnglePIDTuner;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    if (Constants.Is2022Robot) {
      Translation2d frontLeftTranslation = new Translation2d(SwerveConstants2022.RobotLength_m / 2, SwerveConstants2022.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants2022.RobotLength_m / 2,-SwerveConstants2022.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants2022.RobotLength_m / 2, SwerveConstants2022.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants2022.RobotLength_m / 2, -SwerveConstants2022.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule2022(
      frontLeftTranslation, 
      SwerveConstants2022.CanIdFrontLeftAngle, 
      SwerveConstants2022.CanIdFrontLeftVelocity,
      SwerveConstants2022.CanIdFrontLeftAbsoluteEncoder,
      SwerveConstants2022.AbsoluteAngleOffsetFrontLeft);
    m_frontRight = new SwerveModule2022(
      frontRightTranslation, 
      SwerveConstants2022.CanIdFrontRightAngle, 
      SwerveConstants2022.CanIdFrontRightVelocity, 
      SwerveConstants2022.CanIdFrontRightAbsoluteEncoder,
      SwerveConstants2022.AbsoluteAngleOffsetFrontRight);
    m_backLeft = new SwerveModule2022(
      backLeftTranslation, 
      SwerveConstants2022.CanIdBackLeftAngle, 
      SwerveConstants2022.CanIdBackLeftVelocity,
      SwerveConstants2022.CanIdBackLeftAbsoluteEncoder,
      SwerveConstants2022.AbsoluteAngleOffsetBackLeft);
    m_backRight = new SwerveModule2022(
      backRightTranslation, 
      SwerveConstants2022.CanIdBackRightAngle, 
      SwerveConstants2022.CanIdBackRightVelocity, 
      SwerveConstants2022.CanIdBackRightAbsoluteEncoder,
      SwerveConstants2022.AbsoluteAngleOffsetBackRight);
    }
    else {
      Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2,-SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule2023(
      frontLeftTranslation, 
      SwerveConstants.CanIdFrontLeftAngle, 
      SwerveConstants.CanIdFrontLeftVelocity, 
      SwerveConstants.AnalogInputFrontLeftAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetFrontLeft
);
    m_frontRight = new SwerveModule2023(
      frontRightTranslation, 
      SwerveConstants.CanIdFrontRightAngle, 
      SwerveConstants.CanIdFrontRightVelocity,
      SwerveConstants.AnalogInputFrontRightAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetFrontRight
      

);
    m_backLeft = new SwerveModule2023(
      backLeftTranslation, 
      SwerveConstants.CanIdBackLeftAngle, 
      SwerveConstants.CanIdBackLeftVelocity,
      SwerveConstants.AnalogInputBackLeftAbsoluteEncoder,
      SwerveConstants.AbsoluteAngleOffsetBackLeft
);
    m_backRight = new SwerveModule2023(
      backRightTranslation, 
      SwerveConstants.CanIdBackRightAngle, 
      SwerveConstants.CanIdBackRightVelocity,
      SwerveConstants.AnalogInputBackRightAbsoluteEncoder, 
      SwerveConstants.AbsoluteAngleOffsetBackRight
);
    }

    
    m_kinematics = new SwerveDriveKinematics(
        getModuleTranslations());
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, getGyroRotation(),
        getModulePositions());
    resetPose(new Pose2d(8, 4, Rotation2d.fromDegrees(0)));
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);
    m_XPid = new PIDController(1, 0, 0);
    m_YPid = new PIDController(1, 0, 0);
    m_AnglePid = new PIDController(1, 0, 0);
    m_AnglePid.enableContinuousInput(-Math.PI, Math.PI);
    m_XPidTuner = new PIDTuner("X PID Tuner", true, m_XPid);
    m_YPidTuner = new PIDTuner("Y PID Tuner", true, m_YPid);
    m_AnglePidTuner = new PIDTuner("Angel PID Tuner", true, m_AnglePid);
    Robot.logManager.addNumber("SwerveDrive/X_m", () -> m_odometry.getPoseMeters().getX());
    Robot.logManager.addNumber("SwerveDrive/Y_m", () -> m_odometry.getPoseMeters().getY());
    Robot.logManager.addNumber("SwerveDrive/Rotation_deg", () -> getOdometryRotation().getDegrees());
    double velocityP = 0.1;
    double velocityI = 0;
    double velocityD = 0;
    // `this::updateVelocityPIDConstants` is basically shorthand for `(PIDUpdate update) -> updateVelocityPIDConstants(update)`
    m_moduleVelocityPIDTuner = new PIDTuner("Swerve/ModuleVelocity", true, velocityP, velocityI, velocityD, this::updateVelocityPIDConstants);
    double angleP = 0.2;
    double angleI = 0;
    double angleD = 0;
    m_moduleAnglePIDTuner = new PIDTuner("Swerve/ModuleAngle", true, angleP, angleI, angleD, this::updateAnglePIDConstants);

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
    chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1) * SwerveConstants.MaxRobotSpeed_mps;
    chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1)* SwerveConstants.MaxRobotSpeed_mps;
    chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1) * SwerveConstants.MaxRobotRotation_radps;
    if (chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
      stop();
      return;
    }
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    ChassisSpeeds currentSpeed = m_kinematics.toChassisSpeeds(getModuleStates()); // TODO TEST THIS
    SwerveDriveKinematics.desaturateWheelSpeeds(states, currentSpeed, SwerveConstants.MaxSwerveModuleSpeed_mps, SwerveConstants.MaxRobotSpeed_mps, SwerveConstants.MaxRobotRotation_radps);
    m_frontLeft.setTarget(states[0]);
    m_frontRight.setTarget(states[1]);
    m_backLeft.setTarget(states[2]);
    m_backRight.setTarget(states[3]);
    
  }


  public void swerveXMode() {
    m_frontLeft.setTarget(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setTarget(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backLeft.setTarget(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
    m_backRight.setTarget(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
  }


  public void moveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation());
    move(speeds);
  }

  public void moveFieldRelativeAngle(double xMetersPerSecond, double yMetersPerSecond, Rotation2d angle, double magnitude){
    double omega = 0;
    if (Math.abs(magnitude) >= 0.2) {
      omega = m_AnglePid.calculate(getOdometryRotation().getRadians(), angle.getRadians());
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omega, getOdometryRotation());
    move(speeds);
  }

  public void moveRobotRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = new ChassisSpeeds(xForwardSpeedMetersPerSecond, ySidewaySpeedMetersPerSecond,
        omegaRadianPerSecond);
    move(speeds);
  }

  public void resetPids() {
    m_XPid.reset();
    m_YPid.reset();
    m_AnglePid.reset();
  }

  public boolean atTarget() {
    return m_XPid.atSetpoint() && m_YPid.atSetpoint() && m_AnglePid.atSetpoint();
  }

  public void setTarget(double x, double y, Rotation2d angle) {
    m_XPid.setSetpoint(x);
    m_YPid.setSetpoint(y);
    m_AnglePid.setSetpoint(angle.getRadians());
  }

  public void moveToTarget() {
    Pose2d pose = m_odometry.getPoseMeters();
    double x = m_XPid.calculate(pose.getX());
    double y = m_YPid.calculate(pose.getY());
    double angle = m_AnglePid.calculate(pose.getRotation().getRadians());
    moveFieldRelative(x, y, angle);
  }

  public Rotation2d getGyroRotation() {
    if (Robot.isSimulation()) {
      return m_simrotation;
    }
    return m_gyro.getRotation2d().times(-1);
  }

  public Rotation2d getOdometryRotation() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void recalibrateModules(){
    m_frontLeft.recalibrate();
    m_frontRight.recalibrate();
    m_backLeft.recalibrate();
    m_backRight.recalibrate();
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
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
    Pose2d robotPose = m_odometry.update(getGyroRotation(), getModulePositions());
    SmartDashboard.putNumber("SwerveDrive/X", robotPose.getX());
    SmartDashboard.putNumber("SwerveDrive/y", robotPose.getY());
    SmartDashboard.putNumber("SwerveDrive/Rotation", robotPose.getRotation().getDegrees());
    m_field.setRobotPose(robotPose);
    updateModuleOnField(m_frontLeft, robotPose, "FL");
    updateModuleOnField(m_frontRight, robotPose, "FR");
    updateModuleOnField(m_backLeft, robotPose, "BL");
    updateModuleOnField(m_backRight, robotPose, "BR");
    SmartDashboard.putNumber("gyro_angle", getGyroRotation().getDegrees());
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
    m_frontLeft.getModuleInfo("FL");
    m_frontRight.getModuleInfo("FR");
    m_backLeft.getModuleInfo("BL");
    m_backRight.getModuleInfo("BR");
  }

  public void resetPose(Pose2d targetPose){
    if (Robot.isSimulation()) {
    m_simrotation = targetPose.getRotation();
    }

    m_odometry.resetPosition(getGyroRotation(), getModulePositions(), targetPose);
  }

  public void updateModuleOnField(SwerveModule swerveModule, Pose2d robotPose, String name) {
    Transform2d transform = new Transform2d(swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(name).setPose(swerveModulePose);
  }

  private void updateVelocityPIDConstants(PIDUpdate update) {
    m_frontLeft.UpdateVelocityPIDConstants(update);
    m_frontRight.UpdateVelocityPIDConstants(update);
    m_backRight.UpdateVelocityPIDConstants(update);
    m_backLeft.UpdateVelocityPIDConstants(update);
  }

  private void updateAnglePIDConstants(PIDUpdate update) {
    m_frontLeft.UpdateAnglePIDConstants(update);
    m_frontRight.UpdateAnglePIDConstants(update);
    m_backRight.UpdateAnglePIDConstants(update);
    m_backLeft.UpdateAnglePIDConstants(update);
  }

}