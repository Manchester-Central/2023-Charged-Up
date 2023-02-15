// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Extender {
    CANSparkMax m_SparkMax;
    public Extender(){
        m_SparkMax = new CANSparkMax(ArmConstants.CanIdExtender, MotorType.kBrushless);
    }

    public void ExtendToTarget(double targetPosition){
        //TODO
    }

    public double getPosition(){
        return 0.0; //TODO not real value
    }
}
