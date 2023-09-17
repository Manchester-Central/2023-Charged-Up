// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.pid.PIDTuner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DebugConstants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.swerve.SwerveDrive2023;
import frc.robot.util.DashboardNumber;
import frc.robot.util.DriveDirection;

public class DriveUntilTipped extends CommandBase {
  private double m_speed;
  private double m_minAngleDegrees;
  SwerveDrive2023 m_swerveDrive;

  public DriveUntilTipped(SwerveDrive2023 swerveDrive, double speed, double minAngleDegrees) {
    m_swerveDrive = swerveDrive;
    m_speed = speed;
    m_minAngleDegrees = minAngleDegrees;
    addRequirements(swerveDrive);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, SwerveDrive2023 swerveDrive) {
    double minAngleDegrees = AutoUtil.ParseDouble(parsedCommand.getArgument("minAngleDegrees"), 12);
    double speed = AutoUtil.ParseDouble(parsedCommand.getArgument("speed"), 0.5);
    return new DriveUntilTipped(swerveDrive, speed, minAngleDegrees);
  }

  @Override
  public void execute() {
    m_swerveDrive.moveRobotRelative(m_speed, 0, 0.0);
  }

  @Override
  public boolean isFinished() {
     var pitchDegrees = m_swerveDrive.getPitch().getDegrees();
    return Math.abs(pitchDegrees) >= m_minAngleDegrees;
  }
}
