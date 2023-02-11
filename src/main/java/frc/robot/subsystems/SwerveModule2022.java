// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants2022;

/** Add your docs here. */
public class SwerveModule2022 extends SwerveModule {
    private CANCoder m_2022AbsoluteCanCoder;
    private double m_absoluteAngleOffset;
        
    public SwerveModule2022(Translation2d translation, int canIdAngle, int canIdVelocity, int canAbsoluteID,
            double absoluteAngleOffset) {
        super(translation, canIdAngle, canIdVelocity);
        m_2022AbsoluteCanCoder = new CANCoder(canAbsoluteID);
        m_absoluteAngleOffset = absoluteAngleOffset;
        recalibrate();
    }

    public double getRawAbsoluteAngle(){
        return m_2022AbsoluteCanCoder.getAbsolutePosition();
    }

    public double getAbsoluteAngle() {
        return Rotation2d.fromDegrees(m_2022AbsoluteCanCoder.getAbsolutePosition() - m_absoluteAngleOffset)
                .getDegrees();
    }

    @Override
    public double getAngleEncoderRatio() {
        return SwerveConstants2022.AngleEncoderRatio;
    }

    @Override
    public double getVelocityEncoderRatio() {
        return SwerveConstants2022.VelocityEncoderRatio;
    }

    @Override
    public double getWheelCircumference() {
        return SwerveConstants2022.WheelCircumference;
    }

}