// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants;

/** Add your docs here. */
public class Extender {
    private CANSparkMax m_SparkMax;
    private PIDTuner m_pidTuner;
    private SparkMaxAnalogSensor m_linearPot;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simPos = ExtenderConstants.MinimumPositionMeters;
    private double m_simTarget = m_simPos;
    public Extender(){
        m_SparkMax = new CANSparkMax(ExtenderConstants.CanIdExtender, MotorType.kBrushless);
        m_pidTuner = new PIDTuner("ExtenderPID", true, 0.09, 0, 0, this::tunePID);
        m_linearPot = m_SparkMax.getAnalog(Mode.kAbsolute);
        m_linearPot.setPositionConversionFactor(ExtenderConstants.LinearPotConversionFactor);
        double absPos = getPositionMeters();
        m_SparkMax.getEncoder().setPositionConversionFactor(ExtenderConstants.SparkMaxEncoderConversionFactor);
        m_SparkMax.getEncoder().setPosition(absPos);
        m_SafetyZoneHelper = new SafetyZoneHelper(ExtenderConstants.MinimumPositionMeters, ExtenderConstants.MaximumPositionMeters);
        m_SparkMax.setOpenLoopRampRate(ExtenderConstants.RampUpRate);
        m_SparkMax.setClosedLoopRampRate(ExtenderConstants.RampUpRate);
    }

    public void updateSafetyZones(ArmPose targetArmPose, Rotation2d shoulderAngle){
        double normalizedCurrentAngle = Shoulder.normalize(shoulderAngle);
        double normalizedTargetAngle = Shoulder.normalize(targetArmPose.shoulderAngle);
        if ((normalizedCurrentAngle < ShoulderConstants.MinDangerAngle && normalizedTargetAngle < ShoulderConstants.MinDangerAngle)
        || (normalizedCurrentAngle > ShoulderConstants.MaxDangerAngle && normalizedTargetAngle > ShoulderConstants.MaxDangerAngle)) {
            m_SafetyZoneHelper.resetToDefault();
        } else{
            m_SafetyZoneHelper.excludeUp(ExtenderConstants.MinimumPositionMeters);
        }
    }

    public void ExtendToTarget(double targetPositionMeters) {

        double safeTargetPosition = m_SafetyZoneHelper.getSafeValue(targetPositionMeters);

        if (Robot.isSimulation()) {
            m_simTarget = safeTargetPosition;
        }
        m_SparkMax.getPIDController().setReference(safeTargetPosition, ControlType.kPosition);
        
    }

    public double getPositionMeters(){
        if (Robot.isSimulation()) {
            return m_simPos;
        }
        return m_linearPot.getPosition() + ExtenderConstants.LinearPotOffsetMeters;
    }

    public void tunePID(PIDUpdate pidUpdate){
        m_SparkMax.getPIDController().setP(pidUpdate.P);
        m_SparkMax.getPIDController().setI(pidUpdate.I);        
        m_SparkMax.getPIDController().setD(pidUpdate.D);
        m_SparkMax.getPIDController().setFF(pidUpdate.F);
    }

    public void periodic() {
        double increment = 0.004;
        if (Math.abs(m_simPos - m_simTarget) <= Math.abs(increment)) {
            m_simPos = m_simTarget;
        } else if(m_simTarget <= m_simPos) {
            m_simPos -= increment;
        } else {
            m_simPos += increment; 
            
        }
    }
    public void stop(){
        //TODO After testing, should remain at current position instead.
        m_SparkMax.stopMotor();
        m_simTarget = m_simPos;
    }
}

//“MMMMMMM, Mcextender.” -John, 2/20/2023