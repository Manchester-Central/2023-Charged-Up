// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriverRelativeAngleDrive extends BaseJoystickDrive {
  Joystick testController = new Joystick(2);
  /** Creates a new DriverRelativeAngleDrive. */
  public DriverRelativeAngleDrive(SwerveDrive swerveDrive, Gamepad driverController) {
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
    var maxSpeedScaling = Math.max((-testController.getRawAxis(3) + 1) / 2, 0.2);
    SwerveDrive.TranslationSpeedModifier = maxSpeedScaling;
    SwerveDrive.RotationSpeedModifier = maxSpeedScaling;
    double xMetersPerSecond = -testController.getRawAxis(1);
    double yMetersPerSecond = -testController.getRawAxis(0);
    for(var i = 0; i < 20; i++) {
      SmartDashboard.putBoolean("joy/" + i, testController.getRawButton(0));
    }
    if (testController.getPOV(0) == -1) {
      var omegaRadiansPerSecond = -testController.getRawAxis(2);
      // if(testController.getRawButton(16)) { // Warthog joystick
      //   omegaRadiansPerSecond = -1.0;
      // } else if(testController.getRawButton(18)) {
      //   omegaRadiansPerSecond = 1.0;
      // }
      m_swerveDrive.moveFieldRelative(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
    } else {
      Rotation2d rightAngle = getTargetRotation();
      double rightMagnitude = getTargetMagnitude();
      m_swerveDrive.moveFieldRelativeAngle(xMetersPerSecond, yMetersPerSecond, rightAngle, rightMagnitude);
    }
  }

  protected Rotation2d getTargetRotation() {
    return Rotation2d.fromDegrees(testController.getPOV(0)).times(-1);
  }

  protected double getTargetMagnitude() {
    return 1.0;
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
