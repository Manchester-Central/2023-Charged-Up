// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class MoveArm extends CommandBase {
  private Arm m_arm;
  private ArmPose m_armPose;
  
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, Rotation2d shoulderPosition, double extenderPosition, Rotation2d wristPosition, CoordinateType wristCoordinateType) {
    this(arm, new ArmPose(shoulderPosition, extenderPosition, wristPosition, wristCoordinateType));
  }

  public MoveArm(Arm arm, Rotation2d shoulderPosition, double extenderPosition, Rotation2d wristPosition) {
    this(arm, new ArmPose(shoulderPosition, extenderPosition, wristPosition, CoordinateType.FieldRelative));
  }

  public MoveArm(Arm arm, ArmPose armPose) {
    m_arm = arm;
    m_armPose = armPose;
    addRequirements(m_arm);
  }

  public static MoveArm createAutoCommand(ParsedCommand parsedCommand, Arm arm) {
    double shoulderAngle = AutoUtil.ParseDouble(parsedCommand.getArgument("shoulderAngle"), 0.0);
    double extenderPosition = AutoUtil.ParseDouble(parsedCommand.getArgument("extenderPosition"), 0.0);
    double wristAngle = AutoUtil.ParseDouble(parsedCommand.getArgument("wristAngle"), 0.0);
    //TODO setup coordinate type for auto
    return new MoveArm(arm, Rotation2d.fromDegrees(shoulderAngle), extenderPosition, Rotation2d.fromDegrees(wristAngle));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmTarget(m_armPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmTarget(m_armPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.reachedTarget();
  }
}
