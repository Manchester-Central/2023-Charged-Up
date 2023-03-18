// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.util.DashboardNumber;
// i got the new forgis on the g
/** Add your docs here. */
public class Gripper extends SubsystemBase {
    public enum GripperMode{
        grip(1.0),
        slowGrip(0.25),
        unGrip(-1.0),
        slowUngrip(-0.25),
        stop(0),
        hold(0.03);

        private double m_power;
        GripperMode(double power){
            new DashboardNumber("gripper/speed/" + this.name(), power, (newPower) -> {
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
    private int m_freeLimit = 20;
    private int m_limitRPM = 8000;
    
    public Gripper() {
        m_sparkMax = new CANSparkMax(GripperConstants.CanIdGripper, MotorType.kBrushless);
        m_sparkMax.setInverted(true);
        int m_stallLimit = 15;
        int m_freeLimit = 20;
        int m_limitRPM = 8000;
        new DashboardNumber("gripper/stallLimit", m_stallLimit, (newValue) -> {
            int stallLimit = (int)((double) newValue);
            updateCurrentLimit(stallLimit, m_freeLimit, m_limitRPM);
        });
        new DashboardNumber("gripper/freeLimit", m_freeLimit, (newValue) -> {
            int freeLimit = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, freeLimit, m_limitRPM);
        });
        new DashboardNumber("gripper/limitRPM", m_limitRPM, (newValue) -> {
            int limitRPM = (int)((double) newValue);
            updateCurrentLimit(m_stallLimit, m_freeLimit, limitRPM);
        });
        m_sparkMax.burnFlash();
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
        // m_sparkMax.set(m_gripperMode.getPower());
    }
}
