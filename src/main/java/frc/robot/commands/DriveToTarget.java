// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.DrivePose;
import frc.robot.subsystems.swerve.SwerveDrive2023;

public class DriveToTarget extends CommandBase {
  protected SwerveDrive2023 m_swerveDrive;
  private Supplier<Pose2d> m_poseSupplier;
  private double m_translationTolerance;
  private double m_maxPercentSpeed;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(SwerveDrive2023 swerveDrive, Supplier<Pose2d> poseSupplier, double translationTolerance, double maxPercentSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_poseSupplier = poseSupplier;
    m_translationTolerance = translationTolerance;
    m_maxPercentSpeed = maxPercentSpeed;
    addRequirements(m_swerveDrive);
  }

    /** Creates a new DriveToTarget. */
  public DriveToTarget(SwerveDrive2023 swerveDrive, Pose2d pose, double translationTolerance, double maxPercentSpeed) {
    this(swerveDrive, () -> pose, translationTolerance, maxPercentSpeed);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, SwerveDrive2023 swerveDrive) {
    double translationTolerance = AutoUtil.getTranslationTolerance(parsedCommand);
    double maxPercentSpeed = AutoUtil.getMaxPercentSpeed(parsedCommand);
    Pose2d pose = AutoUtil.getDrivePose(parsedCommand);
    if(pose == null) {
      return new InstantCommand();
    }
    return new DriveToTarget(swerveDrive, pose, translationTolerance, maxPercentSpeed);
  }

  public static DriveToTarget toClosestScoreTarget(SwerveDrive2023 swerveDrive) {
    return new DriveToTarget(swerveDrive, () -> {
      var robotPose = swerveDrive.getPose();
      var closestPose = DrivePose.getClosestPose(robotPose);
      if (closestPose.getTranslation().getDistance(robotPose.getTranslation()) > 1) {
        // If not close to a target, don't do anything
        return null;
      }
      return closestPose;
    }, SwerveConstants.DriveToTargetTolerance, SwerveConstants.MaxTranslationPIDSpeedPercent);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.driveToPositionInit();
    m_swerveDrive.resetPids();
    Pose2d pose = m_poseSupplier.get();
    if (pose == null) {
      return;
    }
    m_swerveDrive.setTarget(pose.getX(), pose.getY(), pose.getRotation());
    m_swerveDrive.setDriveTranslationTolerance(m_translationTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_poseSupplier.get();
    if (pose == null) {
      return;
    }
    m_swerveDrive.moveToTarget(m_maxPercentSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerveDrive.atTarget();
  }
}
