// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ResetHeading extends CommandBase {
  /** Creates a new ResetHeading. */
  private SwerveDrive m_SwerveDrive;
  private Rotation2d m_heading;
  public ResetHeading(SwerveDrive swervedrive, Rotation2d heading) {
    m_SwerveDrive = swervedrive;
    m_heading = heading;
    addRequirements(swervedrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveDrive.resetHeading(m_heading);
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
