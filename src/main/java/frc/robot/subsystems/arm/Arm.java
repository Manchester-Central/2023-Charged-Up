// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Always owns shoulder and wrist
package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.subsystems.arm.Gripper.GripperMode;
import frc.robot.subsystems.arm.Wrist.CoordinateType;

public class Arm extends SubsystemBase {
  Shoulder m_shoulder;
  Extender m_extender;
  Wrist m_wrist;
  Gripper m_gripper;
  /** Creates a new Arm. */
  public Arm(Gripper gripper) {
    m_shoulder = new Shoulder();
    m_extender = new Extender();
    m_wrist = new Wrist();
    m_gripper = gripper;
    //Robot.logManager.addBoolean("Arm/AtTarget", DebugConstants.EnableArmDebug, () -> reachedTarget());
  }
  
  public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
    m_shoulder.addCoachTabDashboardValues(coachTab);
    m_extender.addCoachTabDashboardValues(coachTab);
    m_wrist.addCoachTabDashboardValues(coachTab);
  }

  @Override
  public void periodic() {
    m_shoulder.periodic(m_extender.getPositionMeters());
    m_extender.periodic();
    m_wrist.periodic();

    double [] ArmState = {
      m_shoulder.getRotation().getDegrees(), 
      m_extender.getPositionMeters(), 
      m_wrist.getRotation().getDegrees(), 
      m_gripper.getGripperMode().getPower()
    };
    SmartDashboard.putNumberArray("Arm/State", ArmState);
    // This method will be called once per scheduler run
  }


  public void setShoulderTargetManual(Rotation2d angle) {
    double extensionMeters = m_extender.getPositionMeters();
    m_shoulder.updateSafetyZones(new ArmPose(angle, extensionMeters, m_wrist.getRotation(), CoordinateType.ArmRelative), extensionMeters, m_wrist.getRotation());
    m_shoulder.setTargetAngle(angle, extensionMeters);
  }

  public void setExtenderTargetManual(double positionMeters) {
    m_extender.updateSafetyZones(new ArmPose(m_shoulder.getRotation(), positionMeters, m_wrist.getRotation(), CoordinateType.ArmRelative), m_shoulder.getRotation());
    m_extender.ExtendToTarget(positionMeters);
  }

  public void setWristTargetManual(Rotation2d angle) {
    m_wrist.updateSafetyZones(new ArmPose(m_shoulder.getRotation(), m_extender.getPositionMeters(), angle, CoordinateType.ArmRelative), m_shoulder.getRotation());
    m_wrist.setTarget(angle);
  }

  public void setArmTarget(ArmPose armPose) {
    double [] targetState = {
      armPose.shoulderAngle.getDegrees(), 
      armPose.extenderPos, 
      armPose.wristAngle.getDegrees(), 
      0
    };
    if (DebugConstants.EnableArmDebug) {
      SmartDashboard.putNumberArray("Arm/TargetState", targetState);
    }
    double extensionMeters = m_extender.getPositionMeters();
    double normalizedCurrentAngle = Shoulder.normalize(m_shoulder.getRotation());
    double normalizedTargetAngle = Shoulder.normalize(armPose.shoulderAngle);
    double angleDifference = Math.abs(normalizedCurrentAngle - normalizedTargetAngle);
    double wristAngleDegrees = m_wrist.getRotation().getDegrees();

    m_shoulder.updateSafetyZones(armPose, extensionMeters, m_wrist.getRotation());
    m_extender.updateSafetyZones(armPose, m_shoulder.getRotation());
    m_wrist.updateSafetyZones(armPose, m_shoulder.getRotation());

    if(angleDifference < 6) {
      m_shoulder.setTargetAngle(armPose.shoulderAngle, extensionMeters);
      m_extender.ExtendToTarget(armPose.extenderPos);
      if (armPose.wristCoordinate == CoordinateType.ArmRelative){
        m_wrist.setTarget(armPose.wristAngle);
      }
      else {
        //m_wrist.setTarget(armPose.wristAngle.minus(armPose.shoulderAngle));
      }
    }
    else if (extensionMeters >= ExtenderConstants.ExtenderSafeLimit || wristAngleDegrees >= WristConstants.MaximumSafeAngleDegrees || wristAngleDegrees <= WristConstants.MinimumSafeAngleDegrees) {
      m_extender.ExtendToTarget(ExtenderConstants.MinimumPositionMeters + 0.01);
      m_wrist.setTarget(Rotation2d.fromDegrees(180));
    }
    else {
      m_shoulder.setTargetAngle(armPose.shoulderAngle, extensionMeters);
    }
  }

  public void maintain() {
    // m_shoulder.stop();
    m_extender.stop();
    // m_wrist.stop();
  }

  public boolean reachedTarget(ArmPose targetPose) {
    return m_shoulder.atTarget(targetPose.shoulderAngle.getDegrees()) && m_extender.atTarget(targetPose.extenderPos) && m_wrist.atTarget(targetPose.wristAngle.getDegrees());
  }

  public void manualShoulder(double speed) {
    m_shoulder.setManual(speed);
  }

  public void manualExtend(double speed) {
    m_extender.setManual(speed);
  }

  public void manualWrist(double speed) {
    m_wrist.setManual(speed);
  }

  public void stop() {
    m_shoulder.stop();
    m_extender.stop();
    m_wrist.stop();
  }

  public void recalibrateSensors() {
    m_shoulder.recalibrateSensors();
    m_extender.recalibrateSensors();
  }
}

// “Kenny, Is your mom a color sensor?” - Joshua Allard 2/13/23 