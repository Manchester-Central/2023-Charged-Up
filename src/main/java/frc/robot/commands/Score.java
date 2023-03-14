// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Gripper;
import frc.robot.subsystems.arm.Gripper.GripperMode;

public class Score extends CommandBase {
  /** Creates a new Score. */
  Arm m_Arm;
  Gripper m_gripper;
  ArmPose m_armPose;
  BooleanSupplier m_releaseIndicatorBooleanSupplier;

  public Score(Arm arm, Gripper gripper, ArmPose armPose, BooleanSupplier releaseIndicatorSupplier) {
    m_Arm = arm;
    m_gripper = gripper;
    m_armPose = armPose;
    m_releaseIndicatorBooleanSupplier = releaseIndicatorSupplier;
    addRequirements(arm, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setArmTarget(m_armPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setArmTarget(m_armPose);
    if (m_releaseIndicatorBooleanSupplier.getAsBoolean()){
      m_gripper.setGripperMode(GripperMode.unGrip);
    }
    else{
      m_gripper.setGripperMode(GripperMode.grip);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.setGripperMode(GripperMode.grip);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
