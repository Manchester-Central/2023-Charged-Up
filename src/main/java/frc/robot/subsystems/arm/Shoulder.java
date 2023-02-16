// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.logging.LogManager;

/** Add your docs here. */
public class Shoulder {
    CANSparkMax m_shoulderL_A;
    CANSparkMax m_shoulderL_B;
    CANSparkMax m_shoulderR_A;
    CANSparkMax m_shoulderR_B;
    SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    PIDTuner m_PidTuner;
    SafetyZoneHelper m_SafetyZoneHelper;

    public Shoulder () {
        m_shoulderL_A = new CANSparkMax(ShoulderConstants.CanIdShoulderL_A, MotorType.kBrushless);
        m_shoulderL_B = new CANSparkMax(ShoulderConstants.CanIdShoulderL_B, MotorType.kBrushless);
        m_shoulderR_A = new CANSparkMax(ShoulderConstants.CanIdShoulderR_A, MotorType.kBrushless);
        m_shoulderR_B = new CANSparkMax(ShoulderConstants.CanIdShoulderR_B, MotorType.kBrushless);
        m_shoulderR_A.setInverted(true);
        m_shoulderR_B.setInverted(true);
        m_AbsoluteEncoder = m_shoulderL_A.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(ShoulderConstants.AbsoluteAngleConversionFactor);
        m_AbsoluteEncoder.setZeroOffset(ShoulderConstants.AbsoluteAngleZeroOffset);
        m_PidTuner = new PIDTuner("ShoulderPID", true, 0.0001, 0, 0, this::tunePID);
        Robot.logManager.addNumber("Shoulder/Shoulder_rotation", () -> getRotation().getDegrees());
        initializeSparkMaxEncoder(m_shoulderL_A, getRotation());
        initializeSparkMaxEncoder(m_shoulderL_B, getRotation());
        initializeSparkMaxEncoder(m_shoulderR_A, getRotation());
        initializeSparkMaxEncoder(m_shoulderR_B, getRotation());
        m_SafetyZoneHelper = new SafetyZoneHelper(ShoulderConstants.MinimumAngle, ShoulderConstants.MaximumAngle);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_AbsoluteEncoder.getPosition());
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(targetAngle.getDegrees());
        m_shoulderL_A.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderL_B.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderR_A.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_shoulderR_B.getPIDController().setReference(targetDegrees, ControlType.kPosition);
    }

    public void tunePID(PIDUpdate pidUpdate) {
        setPID(pidUpdate, m_shoulderL_A.getPIDController());
        setPID(pidUpdate, m_shoulderL_B.getPIDController());
        setPID(pidUpdate, m_shoulderR_A.getPIDController());
        setPID(pidUpdate, m_shoulderR_B.getPIDController());
    }

    public void setPID(PIDUpdate pidUpdate, SparkMaxPIDController controller) {
        controller.setP(pidUpdate.P);
        controller.setI(pidUpdate.I);
        controller.setD(pidUpdate.D);
    }
    
    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
       RelativeEncoder encoder = sparkMax.getEncoder();
       encoder.setPositionConversionFactor(ShoulderConstants.SparkMaxEncoderConversionFactor);
       encoder.setPosition(absoluteAngle.getDegrees());
    }
    
}
