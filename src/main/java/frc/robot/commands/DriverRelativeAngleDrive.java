// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriverRelativeAngleDrive extends CommandBase {
  protected SwerveDrive m_swerveDrive;
  protected Gamepad m_driverController;
  /** Creates a new DriverRelativeAngleDrive. */
  public DriverRelativeAngleDrive(SwerveDrive swerveDrive, Gamepad driverController) {
    m_swerveDrive = swerveDrive;
    m_driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.resetPids();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMetersPerSecond = m_driverController.getLeftY();
    double yMetersPerSecond = -m_driverController.getLeftX();
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
