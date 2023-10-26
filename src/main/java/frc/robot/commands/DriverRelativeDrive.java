// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import frc.robot.subsystems.swerve.SwerveDrive2023;

public class DriverRelativeDrive extends BaseJoystickDrive {
  /** Creates a new FieldRelativeDrive. */
  public DriverRelativeDrive(SwerveDrive2023 swerveDrive, Gamepad driverController) {
    super(swerveDrive, driverController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = m_slewedLeftY.get();
    double yMetersPerSecond = -m_slewedLeftX.get();
    double omegaRadiansPerSecond = -m_driverController.getRightX();
    m_swerveDrive.moveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }
}
