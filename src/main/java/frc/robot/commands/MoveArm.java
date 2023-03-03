// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class MoveArm extends CommandBase {
  private Arm m_arm;
  private Supplier<ArmPose> m_armPoseSupplier;
  
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, Rotation2d shoulderPosition, double extenderPosition, Rotation2d wristPosition, CoordinateType wristCoordinateType) {
    this(arm, new ArmPose(shoulderPosition, extenderPosition, wristPosition, wristCoordinateType));
  }

  public MoveArm(Arm arm, ArmPose armPose) {
    this(arm, () -> armPose);
  }

  public MoveArm(Arm arm, Supplier<ArmPose> armPoseSupplier) {
    m_arm = arm;
    m_armPoseSupplier = armPoseSupplier;
    addRequirements(m_arm);
  }

  public static Command createAutoCommand(ParsedCommand parsedCommand, Arm arm) {
    String poseName = parsedCommand.getArgument("pose");
    if (poseName == null) {
      return new InstantCommand();
    }
    ArmPose armPose = ArmPose.ArmPoses.get(poseName);
    if (armPose == null) {
      return new InstantCommand();
    }
    return new MoveArm(arm, armPose);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmTarget(m_armPoseSupplier.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmTarget(m_armPoseSupplier.get());
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
