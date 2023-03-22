// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DriveDirection;

public class ResetHeading extends CommandBase {

  /** Creates a new ResetHeading. */
  private SwerveDrive m_SwerveDrive;
  private DriveDirection m_direction;
  public ResetHeading(SwerveDrive swervedrive, DriveDirection direction) {
    m_SwerveDrive = swervedrive;
    m_direction = direction;
    addRequirements(swervedrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveDrive.resetHeading(m_direction.getAllianceAngle());
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
