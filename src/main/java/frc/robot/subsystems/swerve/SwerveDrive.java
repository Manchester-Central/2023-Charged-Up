// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RecalibrateModules;
import frc.robot.logging.LogManager;
import frc.robot.subsystems.Limelight;

public class SwerveDrive extends SubsystemBase {

  public static double SpeedModifier = 1.0;

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
  private PIDController m_AngleDegreesPid;
  private PIDTuner m_XPidTuner;
  private PIDTuner m_YPidTuner;
  private PIDTuner m_AnglePidTuner;
  private PIDTuner m_moduleVelocityPIDTuner;
  private PIDTuner m_moduleAnglePIDTuner;
  private double m_driveToTargetTolerance = SwerveConstants.DriveToTargetTolerance;
  
  private Limelight m_limelightLeft;
  private Limelight m_limelightRight;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(Limelight limelightLeft, Limelight limelightRight) {
    m_limelightLeft = limelightLeft;
    m_limelightRight = limelightRight;

    Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2,-SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule2023(
        "FL",
        frontLeftTranslation,
        SwerveConstants.CanIdFrontLeftAngle,
        SwerveConstants.CanIdFrontLeftVelocity,
        SwerveConstants.AnalogInputFrontLeftAbsoluteEncoder,
        SwerveConstants.AbsoluteAngleOffsetFrontLeft);
    m_frontRight = new SwerveModule2023(
        "FR",
        frontRightTranslation,
        SwerveConstants.CanIdFrontRightAngle,
        SwerveConstants.CanIdFrontRightVelocity,
        SwerveConstants.AnalogInputFrontRightAbsoluteEncoder,
        SwerveConstants.AbsoluteAngleOffsetFrontRight
    );
    m_backLeft = new SwerveModule2023(
        "BL",
        backLeftTranslation,
        SwerveConstants.CanIdBackLeftAngle,
        SwerveConstants.CanIdBackLeftVelocity,
        SwerveConstants.AnalogInputBackLeftAbsoluteEncoder,
        SwerveConstants.AbsoluteAngleOffsetBackLeft);
    m_backRight = new SwerveModule2023(
        "BR",
        backRightTranslation,
        SwerveConstants.CanIdBackRightAngle,
        SwerveConstants.CanIdBackRightVelocity,
        SwerveConstants.AnalogInputBackRightAbsoluteEncoder,
        SwerveConstants.AbsoluteAngleOffsetBackRight);
    
    m_kinematics = new SwerveDriveKinematics(
        getModuleTranslations());
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, getGyroRotation(),
        getModulePositions());
    resetPose(new Pose2d(8, 4, Rotation2d.fromDegrees(0)));
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);
    m_XPid = new PIDController(0.6, 0.05, 0.1);
    m_YPid = new PIDController(0.6, 0.05, 0.1);
    m_AngleDegreesPid = new PIDController(0.01, 0.0001, 0.00);
    m_AngleDegreesPid.enableContinuousInput(-180, 180);
    m_XPidTuner = new PIDTuner("SwerveDrive/X_PID_Tuner", DebugConstants.EnableDriveDebug, m_XPid);
    m_YPidTuner = new PIDTuner("SwerveDrive/Y_PID_Tuner", DebugConstants.EnableDriveDebug, m_YPid);
    m_XPid.setTolerance(SwerveConstants.DriveToTargetTolerance);
    m_YPid.setTolerance(SwerveConstants.DriveToTargetTolerance);
    m_AngleDegreesPid.setTolerance(SwerveConstants.AnglePIDTolerance.getDegrees());
    m_AnglePidTuner = new PIDTuner("SwerveDrive/Angle_PID_Tuner", true, m_AngleDegreesPid);
    Robot.logManager.addNumber("SwerveDrive/X_m", DebugConstants.EnableDriveDebug, () -> m_odometry.getPoseMeters().getX());
    Robot.logManager.addNumber("SwerveDrive/Y_m", DebugConstants.EnableDriveDebug, () -> m_odometry.getPoseMeters().getY());
    Robot.logManager.addNumber("SwerveDrive/Rotation_deg", DebugConstants.EnableDriveDebug, () -> getOdometryRotation().getDegrees());
    Robot.logManager.addNumber("SwerveDrive/Gyro_angle_deg", DebugConstants.EnableDriveDebug, () -> getGyroRotation().getDegrees());
    Robot.logManager.addNumber("SwerveDrive/Gyro_pitch_deg", DebugConstants.EnableDriveDebug, () -> getPitch().getDegrees());
    Robot.logManager.addNumber("SwerveDrive/Gyro_roll_deg", DebugConstants.EnableDriveDebug, () -> getRoll().getDegrees());
    double velocityP = 0.1;
    double velocityI = 0;
    double velocityD = 0;
    double velocityF = 0.054;
    // `this::updateVelocityPIDConstants` is basically shorthand for `(PIDUpdate update) -> updateVelocityPIDConstants(update)`
    m_moduleVelocityPIDTuner = new PIDTuner("Swerve/ModuleVelocity_PID_Tuner", DebugConstants.EnableDriveDebug, velocityP, velocityI, velocityD, velocityF, this::updateVelocityPIDConstants);
    double angleP = 0.2;
    double angleI = 0;
    double angleD = 0;
    m_moduleAnglePIDTuner = new PIDTuner("Swerve/ModuleAngle_PID_Tuner", DebugConstants.EnableDriveDebug, angleP, angleI, angleD, this::updateAnglePIDConstants);

  }
    
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
    m_frontLeft.addCoachTabDashboardValues(coachTab);
    m_frontRight.addCoachTabDashboardValues(coachTab);
    m_backLeft.addCoachTabDashboardValues(coachTab);
    m_backRight.addCoachTabDashboardValues(coachTab);
  }

  public void driverModeInit() {
    m_frontLeft.driverModeInit();
    m_frontRight.driverModeInit();
    m_backLeft.driverModeInit();
    m_backRight.driverModeInit();
  }

  public void driveToPositionInit() {
    m_frontLeft.driveToPositionInit();
    m_frontRight.driveToPositionInit();
    m_backLeft.driveToPositionInit();
    m_backRight.driveToPositionInit();
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
    chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1) * SwerveConstants.MaxRobotSpeed_mps * SpeedModifier;
    chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1)* SwerveConstants.MaxRobotSpeed_mps * SpeedModifier;
    chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1) * SwerveConstants.MaxRobotRotation_radps * SpeedModifier;
    if (chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
      stop();
      return;
    }
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MaxRobotSpeed_mps);
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

  public void debug_setSwerveModule(SwerveModuleState swerveModuleState) {
    m_frontLeft.setTarget(swerveModuleState);
    m_frontRight.setTarget(swerveModuleState);
    m_backLeft.setTarget(swerveModuleState);
    m_backRight.setTarget(swerveModuleState);
  }

  public void moveFieldRelativeForPID(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation());
    move(speeds);
  }

  public void moveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds;
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation().minus(new Rotation2d(Math.PI)));
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getOdometryRotation());
    }
    move(speeds);
  }

  public void moveFieldRelativeAngle(double xMetersPerSecond, double yMetersPerSecond, Rotation2d angle, double magnitude){
    double omega = 0;
    ChassisSpeeds speeds;
    if (Math.abs(magnitude) >= 0.2) {
      omega = m_AngleDegreesPid.calculate(getOdometryRotation().getDegrees(), angle.getDegrees()) * magnitude;
    }
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omega, getOdometryRotation().minus(new Rotation2d(Math.PI)));
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omega, getOdometryRotation());
    }
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
    m_AngleDegreesPid.reset();
    setDriveTranslationTolerance(SwerveConstants.DriveToTargetTolerance);
  }

  public boolean atTarget() {
    boolean isXTolerable = Math.abs(m_odometry.getPoseMeters().getX() - m_XPid.getSetpoint()) <= m_driveToTargetTolerance;
    boolean isYTolerable = Math.abs(m_odometry.getPoseMeters().getY() - m_YPid.getSetpoint()) <= m_driveToTargetTolerance;
    return isXTolerable && isYTolerable && m_AngleDegreesPid.atSetpoint();

  }

  public void setTarget(double x, double y, Rotation2d angle) {
    m_XPid.setSetpoint(x);
    m_YPid.setSetpoint(y);
    m_AngleDegreesPid.setSetpoint(angle.getDegrees());
  }

  public void moveToTarget(double maxTranslationSpeedPercent) {
    Pose2d pose = m_odometry.getPoseMeters();
    double x = MathUtil.clamp(m_XPid.calculate(pose.getX()), -maxTranslationSpeedPercent, maxTranslationSpeedPercent);
    double y = MathUtil.clamp(m_YPid.calculate(pose.getY()), -maxTranslationSpeedPercent, maxTranslationSpeedPercent);
    double angle = m_AngleDegreesPid.calculate(pose.getRotation().getDegrees());
    moveFieldRelativeForPID(x, y, angle);
  }

  public Rotation2d getGyroRotation() {
    if (Robot.isSimulation()) {
      return m_simrotation;
    }
    return m_gyro.getRotation2d().times(-1);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    m_field.setRobotPose(robotPose);
    updateModuleOnField(m_frontLeft, robotPose, "FL");
    updateModuleOnField(m_frontRight, robotPose, "FR");
    updateModuleOnField(m_backLeft, robotPose, "BL");
    updateModuleOnField(m_backRight, robotPose, "BR");
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
    m_moduleVelocityPIDTuner.tune();
    m_moduleAnglePIDTuner.tune();
    m_frontLeft.updateDashboard();
    m_frontRight.updateDashboard();
    m_backLeft.updateDashboard();
    m_backRight.updateDashboard();
  }

  public void resetPose(Pose2d targetPose){
    if (Robot.isSimulation()) {
    m_simrotation = targetPose.getRotation();
    }
    m_odometry.resetPosition(getGyroRotation(), getModulePositions(), targetPose);
  }

  public void resetHeading(Rotation2d targetHeading) {
    var currentPose = m_odometry.getPoseMeters();
    var updatedPose = new Pose2d(currentPose.getX(), currentPose.getY(), targetHeading);
    resetPose(updatedPose);
  }

  public void updateModuleOnField(SwerveModule swerveModule, Pose2d robotPose, String name) {
    if (!DebugConstants.EnableDriveDebug) {
      return;
    }
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

  public void setDriveTranslationTolerance(double tolerance) {
    m_driveToTargetTolerance = tolerance;
  }

  public void updatePoseFromLimelights() {
    Pose2d LLLeftPose = m_limelightLeft.getPose();
    Pose2d LLRightPose = m_limelightRight.getPose();
    if (LLLeftPose != null && LLRightPose != null){
      // System.out.println(LLLeftPose.toString() + LLRightPose.toString()); 
      double leftDistance = Math.abs(m_limelightLeft.getTargetXDistancePixels());
      double rightDistance = Math.abs(m_limelightRight.getTargetXDistancePixels());
      Pose2d LLLeftPoseWithGyro = new Pose2d(LLLeftPose.getX(), LLLeftPose.getY(), getOdometryRotation());
      Pose2d LLRightPoseWithGyro = new Pose2d(LLRightPose.getX(), LLRightPose.getY(), getOdometryRotation());
      if (leftDistance < rightDistance){
        resetPose(LLLeftPoseWithGyro);
      }
      else{
        resetPose(LLRightPoseWithGyro);
      }
      
    }
    else if (LLRightPose != null){
      // System.out.println(LLRightPose.toString()); 
      Pose2d LLRightPoseWithGyro = new Pose2d(LLRightPose.getX(), LLRightPose.getY(), getOdometryRotation());
      resetPose(LLRightPoseWithGyro);
    }
    else if (LLLeftPose != null){
      // System.out.println(LLLeftPose.toString()); 
      Pose2d LLLeftPoseWithGyro = new Pose2d(LLLeftPose.getX(), LLLeftPose.getY(), getOdometryRotation());
      resetPose(LLLeftPoseWithGyro);
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