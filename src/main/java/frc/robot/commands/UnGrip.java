// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class UnGrip extends CommandBase {
  Gripper m_gripper;
  /** Creates a new UnGrip. */
  public UnGrip(Gripper gripper) {
    m_gripper = gripper;
    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Gripper pGripper) {
    return new UnGrip(pGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.setGripperMode(GripperMode.unGrip);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.setGripperMode(GripperMode.stop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// A command that tells the gripper on the robot's arm to ungrip and cease holding an object. It uses auto commands and command bases to achieve this.
