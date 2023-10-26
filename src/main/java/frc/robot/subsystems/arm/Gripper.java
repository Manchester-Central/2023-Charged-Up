// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.util.DashboardNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ArmConstants.GripperConstants;

// i got the new forgis on the g
/** Add your docs here. */
public class Gripper extends SubsystemBase {
    public enum GripperMode{
        grip(1.0),
        slowGrip(0.25),
        unGrip(-1.0),
        slowUngrip(-0.25),
        stop(0),
        hold(0.2);

        private double m_power;
        GripperMode(double power){
            new DashboardNumber("Gripper/speed/" + this.name(), power, DebugConstants.EnableGripperDebug, (newPower) -> {
                m_power = newPower;
            });

        }

        public double getPower(){
            return m_power;
        }
    }
    private CANSparkMax m_sparkMax;
    private GripperMode m_gripperMode = GripperMode.stop;
    private int m_stallLimit = 15;
    private int m_freeLimit = 25;
    private int m_limitRPM = 250;
    
    public Gripper() {
        m_sparkMax = new CANSparkMax(GripperConstants.CanIdGripper, MotorType.kBrushless);
        m_sparkMax.setInverted(false);
        m_sparkMax.setOpenLoopRampRate(0.05);
        new DashboardNumber("Gripper/stallLimit", m_stallLimit, DebugConstants.EnableGripperDebug, (newValue) -> {
            int stallLimit = (int)((double) newValue);
            updateCurrentLimit(stallLimit, m_freeLimit, m_limitRPM);
        });
        new DashboardNumber("Gripper/freeLimit", m_freeLimit, DebugConstants.EnableGripperDebug, (newValue) -> {
            int freeLimit = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, freeLimit, m_limitRPM);
        });
        new DashboardNumber("Gripper/limitRPM", m_limitRPM, DebugConstants.EnableGripperDebug, (newValue) -> {
            int limitRPM = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, m_freeLimit, limitRPM);
        });
        m_sparkMax.burnFlash();

        Robot.logManager.addNumber("Gripper/AppliedOutput", DebugConstants.EnableGripperDebug, () -> m_sparkMax.getAppliedOutput());
        Robot.logManager.addNumber("Gripper/OutputCurrent", DebugConstants.EnableGripperDebug, () -> m_sparkMax.getOutputCurrent());
        Robot.logManager.addNumber("Gripper/MotorTemperature_C", DebugConstants.EnableGripperDebug, () -> m_sparkMax.getMotorTemperature());
        Robot.logManager.addBoolean("Gripper/HasPiece", DebugConstants.EnableGripperDebug, () -> hasPiece());
        Robot.logManager.addString("Gripper/Mode", DebugConstants.EnableGripperDebug, () -> getGripperMode().name());
    }
    
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
      coachTab.addBoolean("HasPiece?", () -> hasPiece());
      coachTab.addNumber("Gripper Temp_C", () -> m_sparkMax.getMotorTemperature());
      coachTab.addNumber("Gripper Current_A", () -> m_sparkMax.getOutputCurrent());
    }

    public boolean hasPiece(){
        return (m_sparkMax.getOutputCurrent() > m_stallLimit - 0.1) && (m_gripperMode == GripperMode.grip);
    }

    private void updateCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
        m_stallLimit = stallLimit;
        m_freeLimit = freeLimit;
        m_limitRPM = limitRPM;
        m_sparkMax.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    }

    public void setGripperMode(GripperMode gripperMode) {
        m_gripperMode = gripperMode;

    }
    public GripperMode getGripperMode() {
        return m_gripperMode;
    }

    public void periodic() {
        m_sparkMax.set(m_gripperMode.getPower());
    }
}
