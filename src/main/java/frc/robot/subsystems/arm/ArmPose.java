// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmPose {
    public Rotation2d m_shoulderAngle;
    public double m_extenderPos;
    public Rotation2d m_wristAngle;
    public ArmPose(Rotation2d shoulderAngle, double extenderPos, Rotation2d wristAngle){
        m_shoulderAngle = shoulderAngle;
        m_extenderPos = extenderPos;
        m_wristAngle = wristAngle;
    }
}