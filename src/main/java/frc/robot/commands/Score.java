// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoUtil;
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
  double m_releaseTime_MS;

  public Score(Arm arm, Gripper gripper, ArmPose armPose, BooleanSupplier releaseIndicatorSupplier) {
    m_Arm = arm;
    m_gripper = gripper;
    m_armPose = armPose;
    m_releaseIndicatorBooleanSupplier = releaseIndicatorSupplier;
    m_releaseTime_MS = 0;
    addRequirements(arm, gripper);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Arm arm, Gripper gripper) {
    ArmPose armPose = AutoUtil.getArmPose(parsedCommand);
    if (armPose == null) {
      return new InstantCommand();
    }
    return new Score(arm, gripper, armPose, () -> arm.reachedTarget(armPose));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_releaseTime_MS = 0;
    m_Arm.setArmTarget(m_armPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setArmTarget(m_armPose);
    if (m_releaseIndicatorBooleanSupplier.getAsBoolean()){
      m_gripper.setGripperMode(GripperMode.unGrip);
      if (m_releaseTime_MS == 0){
        m_releaseTime_MS = Robot.getCurrentTimeMs();
      }
    }
    else{
      m_gripper.setGripperMode(GripperMode.hold);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.setGripperMode(GripperMode.hold);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_releaseTime_MS != 0) {
      return Robot.getCurrentTimeMs() - m_releaseTime_MS > 750; 
    }
    return false;
  }

}
