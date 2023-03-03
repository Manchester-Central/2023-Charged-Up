// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class DefaultArmCommand extends CommandBase {
  
  private Arm m_arm;
  private Gamepad m_tester;

  /** Creates a new DefaultArmCommand. */
  public DefaultArmCommand(Arm arm, Gamepad tester) {
    m_arm = arm;
    m_tester = tester;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setGripperMode(GripperMode.hold);
    // Gripper.customPower = m_tester.getRightY();
    m_arm.maintain(); // TODO: default command should stow the arm
    // m_arm.setArmTarget(ArmPose.StowedPose);
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
