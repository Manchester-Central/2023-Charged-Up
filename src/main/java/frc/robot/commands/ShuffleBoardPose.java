// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class ShuffleBoardPose extends CommandBase {
  public Arm m_Arm;
  private String m_shoulderKey;
  private String m_extenderKey;
  private String m_wristKey;
  /** Creates a new ShuffleBoardPose. */
  public ShuffleBoardPose(Arm arm, String name ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulderKey = "PoseTest/" + name + "/Shoulder";
    m_extenderKey = "PoseTest/" + name + "/Extender";
    m_wristKey = "PoseTest/" + name + "/Wrist";
    SmartDashboard.putNumber(m_shoulderKey, -90);
    SmartDashboard.putNumber(m_extenderKey, ExtenderConstants.MinimumPositionMeters);
    SmartDashboard.putNumber(m_wristKey, 180);
    m_Arm = arm;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderAngle = SmartDashboard.getNumber(m_shoulderKey, -90);
    double ExtenderLength = SmartDashboard.getNumber(m_extenderKey, ExtenderConstants.MinimumPositionMeters);
    double WristAngle = SmartDashboard.getNumber(m_wristKey, 180);
    ArmPose m_ArmPose = new ArmPose(shoulderAngle, ExtenderLength, WristAngle, CoordinateType.ArmRelative);
    m_Arm.setArmTarget(m_ArmPose);
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
