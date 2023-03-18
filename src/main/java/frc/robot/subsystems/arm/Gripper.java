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
    
    public Gripper() {
        m_sparkMax = new CANSparkMax(GripperConstants.CanIdGripper, MotorType.kBrushless);
        m_sparkMax.setInverted(true);
        m_sparkMax.setSmartCurrentLimit(15, 20, 8000);
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
