// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.GripperConstants;
// i got the new forgis on the g
/** Add your docs here. */
public class Gripper extends SubsystemBase {
    public static double customPower = 0;
    public enum GripperMode{
        grip(0.5),
        slowGrip(0.25),
        unGrip(-0.5),
        slowUngrip(-0.25),
        stop(0),
        hold(0.03),
        custom((double)Double.NaN);

        private double m_power;
        GripperMode(double power){
            m_power = power;
        }

        public double getPower(){
            if(Double.isNaN(m_power)) {
                return customPower;
            }
            return m_power;
        }
    }
    private CANSparkMax m_sparkMax;
    private GripperMode m_gripperMode = GripperMode.stop;
    
    public Gripper() {
        m_sparkMax = new CANSparkMax(GripperConstants.CanIdGripper, MotorType.kBrushless);
        m_sparkMax.setInverted(true);
        m_sparkMax.setSecondaryCurrentLimit(20);
        m_sparkMax.burnFlash();
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

// The metaphorical hand for the robot, the UnGrip and Grip commands allow it to grab and ungrab game pieces. The gripper is given set power levels in order to 