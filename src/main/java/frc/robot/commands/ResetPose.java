// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetPose extends CommandBase {
  private SwerveDrive m_SwerveDrive;
  private Pose2d m_pose;
  /** Creates a new ResetPose. */
  public ResetPose(SwerveDrive swerveDrive, Pose2d pose) {
    m_SwerveDrive = swerveDrive;
    addRequirements(m_SwerveDrive);
    m_pose = pose;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public static Command createAutoCommand(ParsedCommand parsedCommand, SwerveDrive swerve){
    Pose2d pose = AutoUtil.getDrivePose(parsedCommand);
    if(pose == null) {
      return new InstantCommand();
    }
    return new ResetPose(swerve, pose);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveDrive.resetPose(m_pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
