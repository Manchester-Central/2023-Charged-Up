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

import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ExtenderConstants;

/** Add your docs here. */
public class Extender {
    private CANSparkMax m_SparkMax;
    private PIDTuner m_pidTuner;
    private SparkMaxAnalogSensor m_linearPot;
    private SafetyZoneHelper m_SafetyZoneHelper;
    private double m_simPos = ExtenderConstants.MinimumPositionMeters;
    private double m_simTarget;
    public Extender(){
        m_SparkMax = new CANSparkMax(ExtenderConstants.CanIdExtender, MotorType.kBrushless);
        m_pidTuner = new PIDTuner("ExtenderPID", true, 0.09, 0, 0, this::tunePID);
        m_linearPot = m_SparkMax.getAnalog(Mode.kAbsolute);
        m_linearPot.setPositionConversionFactor(ExtenderConstants.LinearPotConversionFactor);
        double absPos = getPositionMeters();
        m_SparkMax.getEncoder().setPositionConversionFactor(ExtenderConstants.SparkMaxEncoderConversionFactor);
        m_SparkMax.getEncoder().setPosition(absPos);
        m_SafetyZoneHelper = new SafetyZoneHelper(ExtenderConstants.MinimumPositionMeters, ExtenderConstants.MaximumPositionMeters);
    }

    public void ExtendToTarget(double targetPositionMeters) {
        if (Robot.isSimulation()) {
            m_simTarget = targetPositionMeters;
        }
        double targetPosition = m_SafetyZoneHelper.getSafeValue(targetPositionMeters);
        m_SparkMax.getPIDController().setReference(targetPosition, ControlType.kPosition);
        
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
}
