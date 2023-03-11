// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.ScorePose;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveToTarget extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private Supplier<Pose2d> m_poseSupplier;
  private double m_translationTolerance;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(SwerveDrive swerveDrive, Supplier<Pose2d> poseSupplier, double translationTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_poseSupplier = poseSupplier;
    m_translationTolerance = translationTolerance;
    addRequirements(m_swerveDrive);
  }

    /** Creates a new DriveToTarget. */
  public DriveToTarget(SwerveDrive swerveDrive, Pose2d pose, double translationTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_poseSupplier = () -> pose;
    m_translationTolerance = translationTolerance;
    addRequirements(m_swerveDrive);
  }

  public static DriveToTarget createAutoCommand(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
    double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
    double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
    double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
    double translationTolerance = AutoUtil.ParseDouble(parsedCommand.getArgument("translationTolerance"), Constants.DriveToTargetTolerance);
    return new DriveToTarget(swerveDrive, new Pose2d(x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees)), translationTolerance);
  }

  public static DriveToTarget createAutoCommandForScorePose(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
    String poseName = parsedCommand.getArgument("pose");
    Pose2d pose = poseName == null ? null : ScorePose.ScorePoses.get(poseName);
    return new DriveToTarget(swerveDrive, pose, Constants.DriveToTargetTolerance);
  }

  public static DriveToTarget toClosestScoreTarget(SwerveDrive swerveDrive) {
    return new DriveToTarget(swerveDrive, () -> {
      var robotPose = swerveDrive.getPose();
      var closestPose = ScorePose.getClosestPose(robotPose);
      if (closestPose.getTranslation().getDistance(robotPose.getTranslation()) > 1) {
        // If not close to a target, don't do anything
        return null;
      }
      return closestPose;
    }, Constants.DriveToTargetTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    m_swerveDrive.moveToTarget();
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
