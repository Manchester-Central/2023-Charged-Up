// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.lang.annotation.Target;

import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.WristConstants;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Wrist {
    public enum CoordinateType{
        FieldRelative,
        ArmRelative
    }
    private CANSparkMax m_SparkMax;
    private SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    private PIDTuner m_pidTuner;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simAngle = 0;
    private double m_simTarget;

    public Wrist(){
        m_SparkMax = new CANSparkMax(WristConstants.CanIdWrist, MotorType.kBrushless);
        m_AbsoluteEncoder = m_SparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(WristConstants.AbsoluteAngleConversionFactor);
        m_AbsoluteEncoder.setZeroOffset(WristConstants.AbsoluteAngleZeroOffset);
        m_pidTuner = new PIDTuner("WristPID", true, 0.09, 0, 0, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(WristConstants.MinimumAngle, WristConstants.MaximumAngle);
        initializeSparkMaxEncoder(m_SparkMax, getRotation());
    }

    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
        RelativeEncoder encoder = sparkMax.getEncoder();
        encoder.setPositionConversionFactor(WristConstants.SparkMaxEncoderConversionFactor);
        encoder.setPosition(absoluteAngle.getDegrees());
    }

    public void setTarget(Rotation2d target) {
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(target.getDegrees());
        m_SparkMax.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_simTarget = targetDegrees;
    }

    public Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_simAngle);
        }
        return Rotation2d.fromDegrees(m_AbsoluteEncoder.getPosition());
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
    }

    public void periodic() {
        double increment = 2;
        if (Math.abs(m_simAngle - m_simTarget) <= Math.abs(increment)) {
            m_simAngle = m_simTarget;
        } else if(m_simTarget <= m_simAngle) {
            m_simAngle = m_simAngle - increment;
        } else {
            m_simAngle = m_simAngle + increment; 
        }
    }
}
