// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotRelativeDrive extends BaseJoystickDrive {
  /** Creates a new RobotRelativeDrive. */
  public RobotRelativeDrive(SwerveDrive swerveDrive, Gamepad driverController) {
    super(swerveDrive, driverController);
    Robot.logManager.addNumber("Driver/left_x", () -> driverController.getLeftX());
    Robot.logManager.addNumber("Driver/left_y", () -> driverController.getLeftY());
    Robot.logManager.addNumber("Driver/right_x", () -> driverController.getRightX());
    Robot.logManager.addNumber("Driver/right_y", () -> driverController.getRightY());
    Robot.logManager.addNumber("Driver/slewed_left_x", () -> m_slewedLeftX.get());
    Robot.logManager.addNumber("Driver/slewed_left_y", () -> m_slewedLeftY.get());
    Robot.logManager.addNumber("Driver/slewed_right_x", () -> m_slewedRightX.get());
    Robot.logManager.addNumber("Driver/slewed_right_y", () -> m_slewedRightY.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = -m_slewedLeftY.get();
    double yMetersPerSecond = -m_slewedLeftX.get();
    double omegaRadiansPerSecond = -m_driverController.getRightX();
    m_swerveDrive.moveRobotRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }
}
