// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class TestWrist extends CommandBase {
  
  private Arm m_arm;
  private Gamepad m_controller;

  /** Creates a new DefaultArmCommand. */
  public TestWrist(Arm arm, Gamepad controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(arm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.manualWrist(m_controller.getLeftY() * 0.3);
  }
}
