// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants.GripperConstants;
// i got the new forgis on the g
/** Add your docs here. */
public class Gripper {
    public enum GripperMode{
        grip,
        unGrip,
        stop
    }
    private CANSparkMax m_sparkMax;
    private GripperMode m_gripperMode = GripperMode.stop;
    
    public Gripper() {
        m_sparkMax = new CANSparkMax(GripperConstants.CanIdGripper, MotorType.kBrushless);
    }

    public void setGripperMode(GripperMode gripperMode) {
        m_gripperMode = gripperMode;

    }
    public GripperMode getGripperMode() {
        return m_gripperMode;
    }

    public void periodic() {
        switch (m_gripperMode) {
            case grip:
                grip();
                break;
            case unGrip:
                unGrip();
                break;
            case stop:
                stop();
                break;
        } 
    }

    private void grip() {
        m_sparkMax.getPIDController().setReference(0.5, ControlType.kDutyCycle);
    }

    private void unGrip() {
        m_sparkMax.getPIDController().setReference(-0.5, ControlType.kDutyCycle);
    }

    private void stop() {
        m_sparkMax.getPIDController().setReference(0.0, ControlType.kDutyCycle);
    }
    
    

}
