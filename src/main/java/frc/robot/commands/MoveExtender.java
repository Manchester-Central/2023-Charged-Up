// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class MoveExtender extends CommandBase {
  private Arm m_arm;
  private double m_extenderMeters;
  
  /** Creates a new MoveArm. */
  public MoveExtender(Arm arm, double extenderLengthMeters) {
    m_arm = arm;
    m_extenderMeters = extenderLengthMeters;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setExtenderTargetManual(m_extenderMeters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setExtenderTargetManual(m_extenderMeters);
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
