// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Always owns shoulder and wrist
package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Arm extends SubsystemBase {
  Shoulder m_shoulder;
  Extender m_extender;
  /** Creates a new Arm. */
  public Arm() {
    m_shoulder = new Shoulder();
    m_extender = new Extender();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/ShoulderRotation", m_shoulder.getRotation().getDegrees());
    SmartDashboard.putNumber("Arm/ExtenderPosition", m_extender.getPositionMeters());
    m_shoulder.periodic();
    m_extender.periodic();
    // This method will be called once per scheduler run
  }

  public void setArmTarget(ArmPose armPose) {
    m_shoulder.setTargetAngle(armPose.m_shoulderAngle);
    m_extender.ExtendToTarget(armPose.m_extenderPos);
    // TODO other joints
  }

  public boolean reachedTarget() {
    return false; // TODO
  }

  public void stop() {
    m_shoulder.setTargetAngle(m_shoulder.getRotation());
  }
}
