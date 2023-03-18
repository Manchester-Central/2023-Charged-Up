// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriverRelativeDrive extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private Gamepad m_driverController;
  /** Creates a new FieldRelativeDrive. */
  public DriverRelativeDrive(SwerveDrive swerveDrive, Gamepad driverController) {
    m_swerveDrive = swerveDrive;
    m_driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = -m_driverController.getLeftY();
    double yMetersPerSecond = -m_driverController.getLeftX();
    double omegaRadiansPerSecond = -m_driverController.getRightX();
    m_swerveDrive.moveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


