// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.lang.annotation.Target;

import frc.robot.Constants.ArmConstants.WristConstants;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Wrist {
    private CANSparkMax m_SparkMax;
    private PIDTuner m_pidTuner;
    private SafetyZoneHelper m_SafetyZoneHelper;
    public Wrist(){
        m_SparkMax = new CANSparkMax(WristConstants.CanIdWrist, MotorType.kBrushless);
        m_pidTuner = new PIDTuner("WristPID", true, 0.09, 0, 0, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(WristConstants.MinimumAngle, WristConstants.MaximumAngle);
    }

    public void setTarget(Rotation2d target) {
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(target.getDegrees());
        m_SparkMax.getPIDController().setReference(targetDegrees, ControlType.kPosition);
    }

    public Rotation2d getLocation(){
        return Rotation2d.fromDegrees(0); //TODO fake value
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
    }
}
