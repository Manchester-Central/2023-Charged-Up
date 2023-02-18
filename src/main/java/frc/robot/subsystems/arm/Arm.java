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
  /** Creates a new Arm. */
  public Arm() {
    m_shoulder = new Shoulder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/ShoulderRotation", m_shoulder.getRotation().getDegrees());
    m_shoulder.periodic();
    // This method will be called once per scheduler run
  }

  public void setArmTarget(Rotation2d shoulderTargetAngle) {
    m_shoulder.setTargetAngle(shoulderTargetAngle);
    // TODO other joints
  }

  public boolean reachedTarget() {
    return false; // TODO
  }

  public void stop() {
    m_shoulder.setTargetAngle(m_shoulder.getRotation());
  }
}
