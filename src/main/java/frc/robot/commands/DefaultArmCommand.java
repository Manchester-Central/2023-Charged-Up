// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class DefaultArmCommand extends CommandBase {
  
  private Arm m_arm;
  private Gamepad m_operator;

  /** Creates a new DefaultArmCommand. */
  public DefaultArmCommand(Arm arm, Gamepad operator) {
    m_arm = arm;
    m_operator = operator;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_arm.setGripperMode(GripperMode.hold);
    // Gripper.customPower = m_operator.getRightY();
    m_arm.stop();
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
