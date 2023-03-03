// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class MoveArmTogglePosition extends CommandBase {
  private Arm m_arm;
  private ArmPose m_mainPose;
  private ArmPose m_altPose;
  private BooleanSupplier m_altSupplier;

  public MoveArmTogglePosition(Arm arm, ArmPose mainPose, ArmPose altPose, BooleanSupplier bs) {
    m_arm = arm;
    addRequirements(arm);
    m_mainPose = mainPose;
    m_altPose = altPose;
    m_altSupplier = bs;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmTarget(m_mainPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_altSupplier.getAsBoolean()) {
      m_arm.setArmTarget(m_altPose);
    } else {
      m_arm.setArmTarget(m_mainPose);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
