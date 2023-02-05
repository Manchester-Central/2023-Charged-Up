// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule2023 extends SwerveModule {
    public SwerveModule2023(Translation2d translation, int canIdAngle, int canIdVelocity){
        super(translation, canIdAngle, canIdVelocity);
    }

    @Override
    public double getAbsoluteAngle() {
        // TODO Auto-generated method stub
        return 0;
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
}
