// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.lang.annotation.Target;

import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.util.DashboardNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private int m_stallLimit = 15;
    private int m_freeLimit = 45;
    private int m_limitRPM = 250;
    private double m_SensorToCodeEncoderOffset = WristConstants.SensorToCodeEncoderOffset;

    public Wrist(){
        m_sparkMax = new CANSparkMax(WristConstants.CanIdWrist, MotorType.kBrushless);
        m_sparkMax.restoreFactoryDefaults();
        m_sparkMax.setInverted(false);
        m_sparkMax.setIdleMode(IdleMode.kBrake);
        m_AbsoluteEncoder = m_sparkMax.getAbsoluteEncoder(Type.kDutyCycle); 
        new DashboardNumber("Wrist/AbsoluteAngleConversionFactor", WristConstants.AbsoluteAngleConversionFactor, DebugConstants.EnableArmDebug, (newValue) -> {
            m_AbsoluteEncoder.setPositionConversionFactor(newValue);
            // m_sparkMax.burnFlash();
        });
        m_AbsoluteEncoder.setInverted(false);
        new DashboardNumber("Wrist/AbsoluteAngleZeroOffset", WristConstants.AbsoluteAngleZeroOffset, DebugConstants.EnableArmDebug, (newOffset) -> {
            m_AbsoluteEncoder.setZeroOffset(newOffset);
            // m_sparkMax.burnFlash();
        });
        m_pidTuner = new PIDTuner("Wrist/PID_Tuner", DebugConstants.EnableArmDebug, 0.02, 0, 1.0, this::tunePID);
        m_SafetyZoneHelper = new SafetyZoneHelper(WristConstants.MinimumAngle, WristConstants.MaximumAngle);
        m_AbsoluteEncoder.setInverted(false);
        new DashboardNumber("Wrist/RampUprate", WristConstants.RampUpRate, DebugConstants.EnableArmDebug, (newValue) -> {
            m_sparkMax.setOpenLoopRampRate(newValue);
            m_sparkMax.setClosedLoopRampRate(newValue);
        });
        new DashboardNumber("Wrist/stallLimit", m_stallLimit, DebugConstants.EnableArmDebug, (newValue) -> {
            int stallLimit = (int)((double) newValue);
            updateCurrentLimit(stallLimit, m_freeLimit, m_limitRPM);
        });
        new DashboardNumber("Wrist/freeLimit", m_freeLimit, DebugConstants.EnableArmDebug, (newValue) -> {
            int freeLimit = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, freeLimit, m_limitRPM);
        });
        new DashboardNumber("Wrist/limitRPM", m_limitRPM, DebugConstants.EnableArmDebug, (newValue) -> {
            int limitRPM = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, m_freeLimit, limitRPM);
        });
        new DashboardNumber("Wrist/sensorToCodeEncoderOffset", m_SensorToCodeEncoderOffset, DebugConstants.EnableArmDebug, (newValue) -> {
            UpdateSensorToCodeEncoderOffset(newValue);
        });

        m_sparkMax.getPIDController().setFeedbackDevice(m_AbsoluteEncoder);
        m_sparkMax.burnFlash();
        Robot.logManager.addNumber("Wrist/AppliedOutput", DebugConstants.EnableArmDebug, () -> m_sparkMax.getAppliedOutput());
        Robot.logManager.addNumber("Wrist/MotorTemperature_C", DebugConstants.EnableArmDebug, () -> m_sparkMax.getMotorTemperature());
        Robot.logManager.addNumber("Wrist/OutputCurrent", DebugConstants.EnableArmDebug, () -> m_sparkMax.getOutputCurrent());
        Robot.logManager.addNumber("Wrist/Rotation_deg", DebugConstants.EnableArmDebug, () -> getRotation().getDegrees());

    }
    
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
      coachTab.addNumber("Wrist Temp_C", () -> m_sparkMax.getMotorTemperature());
      coachTab.addNumber("Wrist Current_A", () -> m_sparkMax.getOutputCurrent());
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
        double targetDegrees = m_SafetyZoneHelper.getSafeValue(target.getDegrees()) - m_SensorToCodeEncoderOffset;
        m_sparkMax.getPIDController().setReference(targetDegrees, ControlType.kPosition);
        m_targetDegrees = targetDegrees;
    }

    private void updateCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
        m_stallLimit = stallLimit;
        m_freeLimit = freeLimit;
        m_limitRPM = limitRPM;
        m_sparkMax.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    }
    private void UpdateSensorToCodeEncoderOffset(double newValue){
        m_SensorToCodeEncoderOffset = newValue;
    }

    public Rotation2d getRotation() {
        if(Robot.isSimulation()) {
            return Rotation2d.fromDegrees(m_simAngle);
        }
        var rawValue = m_AbsoluteEncoder.getPosition();
        var shiftedValue = rawValue + m_SensorToCodeEncoderOffset; 
        return Rotation2d.fromDegrees(shiftedValue);
    }


    public void tunePID(PIDUpdate pidUpdate){
        m_sparkMax.getPIDController().setP(pidUpdate.P);
        m_sparkMax.getPIDController().setI(pidUpdate.I);        
        m_sparkMax.getPIDController().setD(pidUpdate.D);
        m_sparkMax.getPIDController().setFF(pidUpdate.F);
        // m_sparkMax.burnFlash();
    }

    public boolean atTarget(double targetDegrees){
        return Math.abs(getRotation().getDegrees() - targetDegrees) < WristConstants.ToleranceDegrees;
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

}
