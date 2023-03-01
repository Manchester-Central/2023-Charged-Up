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
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private double m_targetDegrees = m_simAngle;

    public Wrist(){
        m_SparkMax = new CANSparkMax(WristConstants.CanIdWrist, MotorType.kBrushless);
        m_SparkMax.restoreFactoryDefaults();
        m_SparkMax.setInverted(true);
        m_AbsoluteEncoder = m_SparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(WristConstants.AbsoluteAngleConversionFactor); 
        m_AbsoluteEncoder.setInverted(true);
        m_AbsoluteEncoder.setZeroOffset(WristConstants.AbsoluteAngleZeroOffset);
        m_pidTuner = new PIDTuner("WristPID", true, 0.09, 0, 0, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(WristConstants.MinimumAngle, WristConstants.MaximumAngle);
        initializeSparkMaxEncoder(m_SparkMax, getRotation());
        m_SparkMax.setOpenLoopRampRate(WristConstants.RampUpRate);
        m_SparkMax.setClosedLoopRampRate(WristConstants.RampUpRate);
        m_SparkMax.burnFlash();
        Robot.logManager.addNumber("Wrist/Rotation", () -> getRotation().getDegrees());
        Robot.logManager.addNumber("Wrist/appliedOutput", () -> m_SparkMax.getAppliedOutput());
        Robot.logManager.addNumber("Wrist/targetDegrees", () -> m_targetDegrees);
        SmartDashboard.putNumber("Wrist/maxOutput", WristConstants.MaxPIDOutput);
        SmartDashboard.putNumber("Wrist/rampRate", WristConstants.RampUpRate);
  
    }

    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
        RelativeEncoder encoder = sparkMax.getEncoder();
        encoder.setPositionConversionFactor(WristConstants.SparkMaxEncoderConversionFactor);
        encoder.setPosition(absoluteAngle.getDegrees());
        Robot.logManager.addNumber("Wrist/EncoderRotation", () -> encoder.getPosition());
    }

    public void updateSafetyZones(ArmPose targetArmPose, Rotation2d shoulderAngle) {
        double normalizedCurrentAngle = Shoulder.normalize(shoulderAngle);
        double normalizedTargetAngle = Shoulder.normalize(targetArmPose.shoulderAngle);
        if ((normalizedCurrentAngle < ShoulderConstants.MinDangerAngle && normalizedTargetAngle < ShoulderConstants.MinDangerAngle)
        || (normalizedCurrentAngle > ShoulderConstants.MaxDangerAngle && normalizedTargetAngle > ShoulderConstants.MaxDangerAngle)) {
            m_SafetyZoneHelper.resetToDefault();
        } else if (getRotation().getDegrees() > WristConstants.MaximumSafeAngleDegrees) {
            m_SafetyZoneHelper.excludeUp(WristConstants.MaximumSafeAngleDegrees - 5);
        } else if (getRotation().getDegrees() < WristConstants.MinimumSafeAngleDegrees) {
            m_SafetyZoneHelper.excludeDown(WristConstants.MinimumSafeAngleDegrees + 5);
        }
    }

    public void setTarget(Rotation2d target) {
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(target.getDegrees());
        m_SparkMax.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_targetDegrees = targetDegrees;
    }

    public Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_simAngle);
        }
        SmartDashboard.putNumber("Wrist/rawAngle", m_AbsoluteEncoder.getPosition());
        var rawValue = m_AbsoluteEncoder.getPosition();
        var belowWrapAround = rawValue > 400;
        var shiftedValue = rawValue;
        if(belowWrapAround) {
            shiftedValue = rawValue - 460;
        }
        return Rotation2d.fromDegrees(shiftedValue);
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
        var maxOutput = SmartDashboard.getNumber("Wrist/maxOutput", WristConstants.MaxPIDOutput);
        var rampRate = SmartDashboard.getNumber("Wrist/rampRate", WristConstants.RampUpRate);
        m_SparkMax.getPIDController().setOutputRange(-maxOutput, maxOutput);
        m_SparkMax.setOpenLoopRampRate(rampRate);
        m_SparkMax.setClosedLoopRampRate(rampRate);
    }

    public boolean atTarget(){
        return Math.abs(getRotation().getDegrees() - m_targetDegrees) < WristConstants.ToleranceDegrees;
    }

    public void periodic() {
        double increment = 2;
        if (Math.abs(m_simAngle - m_targetDegrees) <= Math.abs(increment)) {
            m_simAngle = m_targetDegrees;
        } else if(m_targetDegrees <= m_simAngle) {
            m_simAngle = m_simAngle - increment;
        } else {
            m_simAngle = m_simAngle + increment; 
        }
        m_pidTuner.tune();
    }

    public void setManual(double speed) {
        m_SparkMax.set(speed);
    }

    public void stop(){
        //TODO After testing, should remain at current position instead.
        m_SparkMax.stopMotor();
        m_targetDegrees = m_simAngle;
    }
}
