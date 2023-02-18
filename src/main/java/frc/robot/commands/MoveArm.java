// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class MoveArm extends CommandBase {
  private Arm m_arm;
  private Rotation2d m_shoulderPosition;
  private double m_extenderPosition;
  private Rotation2d m_wristPosition;
  private CoordinateType m_coordinateType;
  
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, Rotation2d shoulderPosition, double extenderPosition, Rotation2d wristPosition, CoordinateType coordinateType) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_shoulderPosition = shoulderPosition;
    m_extenderPosition = extenderPosition;
    m_wristPosition = wristPosition;
    m_coordinateType = coordinateType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmTarget(m_shoulderPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
