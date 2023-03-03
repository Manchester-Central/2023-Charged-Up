// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.GripperMutex;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class GripGSD extends CommandBase {
  GripperMode m_mode;
  Arm m_arm;
  /** Creates a new GripGSD. */
  public GripGSD(Arm arm, GripperMutex gm, GripperMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_mode = mode;
    m_arm = arm;
    addRequirements(gm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setGripperMode(m_mode);
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
