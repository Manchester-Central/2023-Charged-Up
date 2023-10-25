// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriverRelativeDrive extends BaseJoystickDrive {
  Joystick testController = new Joystick(2);
  /** Creates a new FieldRelativeDrive. */
  public DriverRelativeDrive(SwerveDrive swerveDrive, Gamepad driverController) {
    super(swerveDrive, driverController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = -testController.getRawAxis(1);
    double yMetersPerSecond = -testController.getRawAxis(0);
    double omegaRadiansPerSecond = 0; //-m_driverController.getRightX();
    m_swerveDrive.moveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }
}
