// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveDrive2023;
import frc.robot.util.DriveDirection;

public class DriverRelativeSetAngleDrive extends DriverRelativeAngleDrive {
  /** Creates a new DriverRelativeSetAngleDrive. */
  double m_magnitude;
  DriveDirection m_direction;

  public DriverRelativeSetAngleDrive(SwerveDrive2023 swervedrive, Gamepad driverController, DriveDirection direction, double magnitude) {
    super(swervedrive, driverController);
    m_magnitude = magnitude;
    m_direction = direction;
  }

  @Override
  protected double getTargetMagnitude() {
    return m_magnitude;
  }

  @Override
  protected Rotation2d getTargetRotation() {
    return m_direction.getAllianceAngle();
  }
}
