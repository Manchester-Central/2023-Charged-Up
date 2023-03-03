// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class UnGrip extends CommandBase {
  Arm m_arm;
  /** Creates a new UnGrip. */
  public UnGrip(Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Arm pArm) {
    return new UnGrip(pArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setGripperMode(GripperMode.unGrip);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setGripperMode(GripperMode.stop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
