// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
