// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoTImerCommand extends CommandBase {
  private long m_startTime;
  /** Creates a new AutoTImerCommand. */
  public AutoTImerCommand() {
    SmartDashboard.putNumber("AutoTime", -1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Robot.getCurrentTimeMs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentTimeMs = Robot.getCurrentTimeMs();
    SmartDashboard.putNumber("AutoTime", currentTimeMs - m_startTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
