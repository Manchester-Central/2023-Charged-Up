// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveShoulder extends CommandBase {
  private Arm m_arm;
  private Rotation2d m_shoulderAngle;

  public MoveShoulder(Arm arm, Rotation2d shoulderAngle) {
    m_arm = arm;
    m_shoulderAngle = shoulderAngle;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setShoulderTargetManual(m_shoulderAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setShoulderTargetManual(m_shoulderAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.reachedTarget();
  }
}
