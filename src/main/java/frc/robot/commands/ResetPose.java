// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.DrivePose;
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
  
  public static ResetPose createAutoCommand(ParsedCommand parsedCommand, SwerveDrive swerve){
    var poseName = parsedCommand.getArgument("pose");
    if(poseName != null && DrivePose.DrivePoses.containsKey(poseName)) {
      return new ResetPose(swerve, DrivePose.DrivePoses.get(poseName).getCurrentAlliancePose());
    }
    double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
    double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
    double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
    return new ResetPose(swerve, new Pose2d(x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees)));
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
