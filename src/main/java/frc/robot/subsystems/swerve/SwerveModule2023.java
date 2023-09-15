// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.DashboardNumber;

/** Add your docs here. */
public class SwerveModule2023 extends SwerveModule {
    private AnalogEncoder m_absoluteEncoder;
    private double m_absoluteAngleOffset;

    public SwerveModule2023(String moduleName, Translation2d translation, int canIdAngle, int canIdVelocity, int absoluteAnalogPort, double absoluteAngleOffset){
        super(moduleName, translation, canIdAngle, canIdVelocity);

        m_absoluteEncoder = new AnalogEncoder(absoluteAnalogPort);

        new DashboardNumber(getDSKey("absoluteAngleOffset"), absoluteAngleOffset, DebugConstants.EnableDriveDebug,  (newValue) -> {
            m_absoluteAngleOffset = newValue;
            recalibrate();
        });
    }

    public double getRawAbsoluteAngle(){
        return (m_absoluteEncoder.getAbsolutePosition() * 360);
    }

    @Override
    public double getAbsoluteAngle() {
        return Rotation2d.fromDegrees((m_absoluteEncoder.getAbsolutePosition() * 360) - m_absoluteAngleOffset).getDegrees();
    }

    @Override
    public double getAngleEncoderRatio() {
        return SwerveConstants.AngleEncoderRatio;
    }

    @Override
    public double getVelocityEncoderRatio() {
        return SwerveConstants.VelocityEncoderRatio;
    }

    @Override
    public double getWheelCircumference() {
        return SwerveConstants.WheelCircumference;
    }

    public double calculateEncoderError() {
        return Math.abs(getPosition().angle.getDegrees() - getAbsoluteAngle()); // TO-DO: What does this line of code do?
    }

    public boolean areEncodersAligned() {
        if(calculateEncoderError() <= 10) {
            return true;
        } else {
            return false;
        } // TO-DO: Test the number in the if statement.
    }

    // SwerveModuleState getPositon() - returns an object from which a Rotation2D object holding the relative encoder angle can be extracted.
    // getAbsoluteAngle() - returns a Rotation2D object.
}
