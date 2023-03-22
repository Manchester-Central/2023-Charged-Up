// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveToTargetWithLimelights extends DriveToTarget {

    /** Creates a new DriveToTarget. */
  public DriveToTargetWithLimelights(SwerveDrive swerveDrive, Pose2d pose, double translationTolerance) {
    super(swerveDrive, pose, translationTolerance);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
    double translationTolerance = AutoUtil.getTranslationTolerance(parsedCommand);
    Pose2d pose = AutoUtil.getDrivePose(parsedCommand);
    if(pose == null) {
      return new InstantCommand();
    }
    return new DriveToTargetWithLimelights(swerveDrive, pose, translationTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.updatePoseFromLimelights();
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.updatePoseFromLimelights();
    super.execute();
  }

}
