// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
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
import frc.robot.logging.LogManager;

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

  private PIDController m_XPid;
  private PIDController m_YPid;
  private PIDController m_AnglePid;
  private PIDTuner m_XPidTuner;
  private PIDTuner m_YPidTuner;
  private PIDTuner m_AnglePidTuner;

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
    m_XPid = new PIDController(1, 0, 0);
    m_YPid = new PIDController(1, 0, 0);
    m_AnglePid = new PIDController(1, 0, 0);
    m_XPidTuner = new PIDTuner("X PID Tuner", true, m_XPid);
    m_YPidTuner = new PIDTuner("Y PID Tuner", true, m_YPid);
    m_AnglePidTuner = new PIDTuner("Angel PID Tuner", true, m_AnglePid);
    Robot.logManager.addNumber("SwerveDrive/X_m", () -> m_odometry.getPoseMeters().getX());
    Robot.logManager.addNumber("SwerveDrive/Y_m", () -> m_odometry.getPoseMeters().getY());
    Robot.logManager.addNumber("SwerveDrive/Rotation_deg", () -> m_odometry.getPoseMeters().getRotation().getDegrees());
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

  public void setCoordinates(double x, double y, Rotation2d angle) {
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
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
  }

  public void updateModuleOnField(SwerveModule swerveModule, Pose2d robotPose, String name) {
    Transform2d transform = new Transform2d(swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(name).setPose(swerveModulePose);
  }

  public void resetOdometry(double x, double y, double angle) {
    // m_odometry.resetPosition(getRotation(), getModulePositions(), new Pose2d(x, y, getRotation()));
  }
}