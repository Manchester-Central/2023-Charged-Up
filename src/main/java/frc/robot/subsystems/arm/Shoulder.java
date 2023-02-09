// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Shoulder {
    CANSparkMax m_shoulderL_A;
    CANSparkMax m_shoulderL_B;
    CANSparkMax m_shoulderR_A;
    CANSparkMax m_shoulderR_B;
    SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    PIDTuner m_PidTuner;

    public Shoulder (){
        m_shoulderL_A = new CANSparkMax(ArmConstants.CanIdShoulderL_A, MotorType.kBrushless);
        m_shoulderL_B = new CANSparkMax(ArmConstants.CanIdShoulderL_B, MotorType.kBrushless);
        m_shoulderR_A = new CANSparkMax(ArmConstants.CanIdShoulderR_A, MotorType.kBrushless);
        m_shoulderR_B = new CANSparkMax(ArmConstants.CanIdShoulderR_B, MotorType.kBrushless);
        m_shoulderR_A.setInverted(true);
        m_shoulderR_B.setInverted(true);
        m_AbsoluteEncoder = m_shoulderL_A.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(ArmConstants.ShoulderAngleConversionFactor);
        m_AbsoluteEncoder.setZeroOffset(ArmConstants.ShoulderAngleZeroOffset);
        m_PidTuner = new PIDTuner("ShoulderPID", true, 0.0001, 0, 0, this::tunePID);
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(m_AbsoluteEncoder.getPosition());
    }

    public void setTargetAngle(){

    }

    public void tunePID(PIDUpdate pidUpdate){
        setPID(pidUpdate, m_shoulderL_A.getPIDController());
        setPID(pidUpdate, m_shoulderL_B.getPIDController());
        setPID(pidUpdate, m_shoulderR_A.getPIDController());
        setPID(pidUpdate, m_shoulderR_B.getPIDController());
    }

    public void setPID(PIDUpdate pidUpdate, SparkMaxPIDController controller){
        controller.setP(pidUpdate.P);
        controller.setI(pidUpdate.I);
        controller.setD(pidUpdate.D);
    }
}
