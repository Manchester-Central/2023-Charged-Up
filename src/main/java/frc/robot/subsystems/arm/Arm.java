// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Always owns shoulder and wrist
package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class Arm extends SubsystemBase {
  Shoulder m_shoulder;
  Extender m_extender;
  Wrist m_wrist;
  /** Creates a new Arm. */
  public Arm() {
    m_shoulder = new Shoulder();
    m_extender = new Extender();
    m_wrist = new Wrist();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/ShoulderRotation", m_shoulder.getRotation().getDegrees());
    SmartDashboard.putNumber("Arm/ExtenderPosition", m_extender.getPositionMeters());
    SmartDashboard.putNumber("Arm/WristRotation", m_wrist.getRotation().getDegrees());
    m_shoulder.periodic();
    m_extender.periodic();
    m_wrist.periodic();

    double [] ArmState = {m_shoulder.getRotation().getDegrees(), m_extender.getPositionMeters(), m_wrist.getRotation().getDegrees()};
    SmartDashboard.putNumberArray("Arm/State", ArmState);
    // This method will be called once per scheduler run
  }

  public void setArmTarget(ArmPose armPose) {
    m_shoulder.updateSafetyZones(armPose, m_extender.getPositionMeters());
    m_extender.updateSafetyZones(armPose, m_shoulder.getRotation());
    m_shoulder.setTargetAngle(armPose.shoulderAngle);
    m_extender.ExtendToTarget(armPose.extenderPos);
    if (armPose.wristCoordinate == CoordinateType.ArmRelative){
      m_wrist.setTarget(armPose.wristAngle);
    }
    else {
      m_wrist.setTarget(armPose.wristAngle.minus(armPose.shoulderAngle));
    }
  }

  public boolean reachedTarget() {
    return false; // TODO
  }

  public void stop() {
    m_shoulder.setTargetAngle(m_shoulder.getRotation());
    m_extender.ExtendToTarget(m_extender.getPositionMeters());
    m_wrist.setTarget(m_wrist.getRotation());
  }
}

//“Kenny, Is your mom a color sensor?” -Josh 2/13/23
