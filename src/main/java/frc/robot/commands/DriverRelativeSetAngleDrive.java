// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriverRelativeSetAngleDrive extends DriverRelativeAngleDrive {
  /** Creates a new DriverRelativeSetAngleDrive. */
  double m_magnitude;
  Rotation2d m_angle;

  public DriverRelativeSetAngleDrive(SwerveDrive swervedrive, Gamepad driverController, Rotation2d angle, double magnitude) {
    super(swervedrive, driverController);
    m_magnitude = magnitude;
    m_angle = angle;
  }

  @Override
  protected double getTargetMagnitude() {
    return m_magnitude;
  }

  @Override
  protected Rotation2d getTargetRotation() {
    return m_angle;
  }
}

// A function that sets the angle of the robot in relation to the Driver. By gathering the the angle of the Driver's controller, the robot's heading will be based upon where the controller's facing.

