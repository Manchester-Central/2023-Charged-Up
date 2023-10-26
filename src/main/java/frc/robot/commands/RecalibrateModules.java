// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive2023;

public class RecalibrateModules extends CommandBase {
  private SwerveDrive2023 m_SwerveDrive;
  /** Creates a new RecalibrateModules. */
  public RecalibrateModules(SwerveDrive2023 swerveDrive) {
    m_SwerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveDrive.recalibrateModules();
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
