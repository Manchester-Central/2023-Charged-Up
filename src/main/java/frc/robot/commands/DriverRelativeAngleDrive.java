// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveDrive2023;

public class DriverRelativeAngleDrive extends BaseJoystickDrive {
  /** Creates a new DriverRelativeAngleDrive. */
  public DriverRelativeAngleDrive(SwerveDrive2023 swerveDrive, Gamepad driverController) {
    super(swerveDrive, driverController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_swerveDrive.resetPids();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = -m_slewedLeftY.get();
    double yMetersPerSecond = -m_slewedLeftX.get();
    Rotation2d rightAngle = getTargetRotation();
    double rightMagnitude = getTargetMagnitude();
    m_swerveDrive.moveFieldRelativeAngle(xMetersPerSecond, yMetersPerSecond, rightAngle, rightMagnitude);
  }

  protected Rotation2d getTargetRotation() {
    return Rotation2d.fromRadians(m_driverController.getRightAngle() - Math.PI/2);
  }

  protected double getTargetMagnitude() {
    return m_driverController.getRightMagnitude();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
    m_swerveDrive.resetPids();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
