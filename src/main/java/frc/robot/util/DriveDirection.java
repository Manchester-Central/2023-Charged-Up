// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum DriveDirection {
    Up(0, 180),
    Down(180, 0),
    Left(90, 270),
    Right(270, 90);

    private Rotation2d m_redAngle;
    private Rotation2d m_blueAngle;

    private DriveDirection(double redAngle, double blueAngle) {
        m_redAngle = Rotation2d.fromDegrees(redAngle);
        m_blueAngle = Rotation2d.fromDegrees(blueAngle);
    }

    public Rotation2d getAllianceAngle() {
        return DriverStation.getAlliance() == Alliance.Red ? m_redAngle : m_blueAngle;
    }
}
