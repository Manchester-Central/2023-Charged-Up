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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.lang.annotation.Target;

import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.util.DashboardNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Wrist {
    public enum CoordinateType{
        FieldRelative,
        ArmRelative
    }
    private CANSparkMax m_sparkMax;
    private SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    private PIDTuner m_pidTuner;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simAngle = 0;
    private double m_targetDegrees = m_simAngle;

    public Wrist(){
        m_sparkMax = new CANSparkMax(WristConstants.CanIdWrist, MotorType.kBrushless);
        m_sparkMax.restoreFactoryDefaults();
        m_sparkMax.setInverted(false);
        m_sparkMax.setIdleMode(IdleMode.kBrake);
        m_AbsoluteEncoder = m_sparkMax.getAbsoluteEncoder(Type.kDutyCycle); 
        new DashboardNumber("Wrist/AbsoluteAngleConversionFactor", WristConstants.AbsoluteAngleConversionFactor, (newValue) -> {
            m_AbsoluteEncoder.setPositionConversionFactor(newValue);
            recalibrateSensors();
            // m_sparkMax.burnFlash();
        });
        m_AbsoluteEncoder.setInverted(false);
        new DashboardNumber("Wrist/AbsoluteAngleZeroOffset", WristConstants.AbsoluteAngleZeroOffset, (newOffset) -> {
            m_AbsoluteEncoder.setZeroOffset(newOffset);
            recalibrateSensors();
            // m_sparkMax.burnFlash();
        });
        m_pidTuner = new PIDTuner("WristPID", true, 0.02, 0, 1.0, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(WristConstants.MinimumAngle, WristConstants.MaximumAngle);
        initializeSparkMaxEncoder(m_sparkMax, getRotation());
        m_AbsoluteEncoder.setInverted(false);
        new DashboardNumber("Wrist/RampUprate", WristConstants.RampUpRate, (newValue) -> {
            m_sparkMax.setOpenLoopRampRate(newValue);
            m_sparkMax.setClosedLoopRampRate(newValue);
        });
        //m_sparkMax.setSmartCurrentLimit(15, 20, 8000);
        m_sparkMax.setSmartCurrentLimit(0, 0, 0);
        m_sparkMax.burnFlash();
        Robot.logManager.addNumber("Wrist/AppliedOutput", () -> m_sparkMax.getAppliedOutput());
    }

    private void initializeSparkMaxEncoder(CANSparkMax sparkMax, Rotation2d absoluteAngle) {
        RelativeEncoder encoder = sparkMax.getEncoder();
        new DashboardNumber("Wrist/EncoderConversionFactor", WristConstants.SparkMaxEncoderConversionFactor, (newConversionFactor) -> {
            encoder.setPositionConversionFactor(newConversionFactor);
            recalibrateSensors();
        });
        Robot.logManager.addNumber("Wrist/EncoderRotation", () -> encoder.getPosition());
    }

    public void updateSafetyZones(ArmPose targetArmPose, Rotation2d shoulderAngle) {
        // double normalizedCurrentAngle = Shoulder.normalize(shoulderAngle);
        // double normalizedTargetAngle = Shoulder.normalize(targetArmPose.shoulderAngle);
        // if ((normalizedCurrentAngle < ShoulderConstants.MinDangerAngle && normalizedTargetAngle < ShoulderConstants.MinDangerAngle)
        // || (normalizedCurrentAngle > ShoulderConstants.MaxDangerAngle && normalizedTargetAngle > ShoulderConstants.MaxDangerAngle)) {
        //     m_SafetyZoneHelper.resetToDefault();
        // } else if (getRotation().getDegrees() > WristConstants.MaximumSafeAngleDegrees) {
        //     m_SafetyZoneHelper.excludeUp(WristConstants.MaximumSafeAngleDegrees - 5);
        // } else if (getRotation().getDegrees() < WristConstants.MinimumSafeAngleDegrees) {
        //     m_SafetyZoneHelper.excludeDown(WristConstants.MinimumSafeAngleDegrees + 5);
        // }
    }

    public void setTarget(Rotation2d target) {
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(target.getDegrees());
        m_sparkMax.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_targetDegrees = targetDegrees;
    }

    public Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_simAngle);
        }
        var rawValue = m_AbsoluteEncoder.getPosition();
        var belowWrapAround = rawValue > 420;
        var shiftedValue = rawValue;
        if(belowWrapAround) {
            shiftedValue = rawValue - 460;
        }
        return Rotation2d.fromDegrees(shiftedValue);
    }

    public Rotation2d getEncoderRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_simAngle);
        }
        return Rotation2d.fromDegrees(m_sparkMax.getEncoder().getPosition());
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_sparkMax.getPIDController().setP(pidUpdate.P);
        m_sparkMax.getPIDController().setI(pidUpdate.I);        
        m_sparkMax.getPIDController().setD(pidUpdate.D);
        m_sparkMax.getPIDController().setFF(pidUpdate.F);
        // m_sparkMax.burnFlash();
    }

    public boolean atTarget(){
        return Math.abs(getEncoderRotation().getDegrees() - m_targetDegrees) < WristConstants.ToleranceDegrees;
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
        m_sparkMax.set(speed);
    }

    public void stop(){
        //TODO After testing, should remain at current position instead.
        m_sparkMax.stopMotor();
        m_targetDegrees = m_simAngle;
    }

    public void recalibrateSensors() {
        m_sparkMax.getEncoder().setPosition(getRotation().getDegrees());
    }
}
