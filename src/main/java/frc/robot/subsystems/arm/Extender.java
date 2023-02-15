// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Extender {
    CANSparkMax m_SparkMax;
    private PIDTuner pidTuner;
    public Extender(){
        m_SparkMax = new CANSparkMax(ArmConstants.CanIdExtender, MotorType.kBrushless);
        pidTuner = new PIDTuner("ExtenderPID", true, 0.09, 0, 0, this::tunePID);
    }

    public void ExtendToTarget(double targetPosition){
        //TODO
    }

    public double getPosition(){
        return 0.0; //TODO not real value
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
    }
}
