// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.pid.PIDTuner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DashboardNumber;
import frc.robot.util.DriveDirection;

public class AutoBalanceDrive extends CommandBase {
  private static PIDController pid = new PIDController(0.1, 0.0, 0.0);
  public static PIDTuner PIDTuner = new PIDTuner("AutoBalance/PID_Tuner", DebugConstants.EnableDriveDebug, pid);
  private static DashboardNumber MaxSpeed = new DashboardNumber("AutoBalance/MaxPercentPower", 0.5, DebugConstants.EnableDriveDebug, (newSpeed) -> {});
  private static DashboardNumber AngleTolerance = new DashboardNumber("AutoBalance/AngleTolerance", 5, DebugConstants.EnableDriveDebug, (newTolerance) -> {});
  SwerveDrive m_swerveDrive;

  public AutoBalanceDrive(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    pid.reset();
    pid.setSetpoint(0);
  }

  @Override
  public void execute() {
    var pitchDegrees = m_swerveDrive.getPitch().getDegrees();
    if(Math.abs(pitchDegrees) < AngleTolerance.get()) {
      m_swerveDrive.swerveXMode();
    } else {
      var speedX = MathUtil.clamp(pid.calculate(m_swerveDrive.getPitch().getDegrees()), -MaxSpeed.get(), MaxSpeed.get());
      m_swerveDrive.moveFieldRelativeAngle(speedX, 0, DriveDirection.Towards.getAllianceAngle(), 1.0);
    }
  }
}
