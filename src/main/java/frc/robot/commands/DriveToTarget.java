// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.SwerveDrive;

public class DriveToTarget extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private double m_x;
  private double m_y;
  private Rotation2d m_angle;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(SwerveDrive swerveDrive, double x, double y, Rotation2d angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_x = x;
    m_y = y;
    m_angle = angle;
    addRequirements(m_swerveDrive);
  }

  public static DriveToTarget createAutoCommand(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
    double x_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
    double y_meters = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
    double angle_degrees = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
    return new DriveToTarget(swerveDrive, x_meters, y_meters, Rotation2d.fromDegrees(angle_degrees));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.resetPids();
    m_swerveDrive.setCoordinates(m_x, m_y, m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.moveToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.resetPids();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerveDrive.atTarget();
  }
}
