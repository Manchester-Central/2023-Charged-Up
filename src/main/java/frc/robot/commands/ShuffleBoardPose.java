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
  /** Creates a new ShuffleBoardPose. */
  public ShuffleBoardPose(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber("PoseTest/Shoulder", -90);
    SmartDashboard.putNumber("PoseTest/Extend", ExtenderConstants.MinimumPositionMeters);
    SmartDashboard.putNumber("PoseTest/Wrist", 180);
    m_Arm = arm;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderAngle = SmartDashboard.getNumber("PoseTest/Shoulder", -90);
    double ExtenderLength = SmartDashboard.getNumber("PoseTest/Extend", ExtenderConstants.MinimumPositionMeters);
    double WristAngle = SmartDashboard.getNumber("PoseTest/Wrist", 180);
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

// A command that records the poses of the arm on ShuffleBoard. This command has buttons for each pose, when clicked, the robot simulator on Chaosboard will have its arm move according to the command pressed.