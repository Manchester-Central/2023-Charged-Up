package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.RobotContainer.ArmMode;

public class LEDs extends SubsystemBase {
    private AddressableLED m_lightStrip;
    private AddressableLEDBuffer m_buffer;
    private Supplier<ArmMode> m_armModeSupplier;
    private Supplier<Boolean> m_griperHasPieceSupplier;
    private Object m_mutex = new Object();
    public static boolean flashing = false;
    public static boolean strobeTeamColors = false;

    public LEDs(Supplier<ArmMode> armModeSupplier, Supplier<Boolean> gripperHasPiece) {
        m_armModeSupplier = armModeSupplier;
        m_griperHasPieceSupplier = gripperHasPiece;
        SmartDashboard.putNumber("red", 255);
        SmartDashboard.putNumber("green", 0);
        SmartDashboard.putNumber("blue", 100);
        m_buffer = new AddressableLEDBuffer(LedConstants.NumLeds);
        m_lightStrip = new AddressableLED(PWMConstants.LightStripPWMPort);
        m_lightStrip.setLength(m_buffer.getLength());
        m_lightStrip.start();
    }

    @Override
    public void periodic() {
        /*
         * int red = (int) SmartDashboard.getNumber("red", 255);
         * int green = (int) SmartDashboard.getNumber("green", 0);
         * int blue = (int) SmartDashboard.getNumber("blue", 100);
         * setRGB(red, green, blue);
         */

         if(strobeTeamColors == true) {
            for(int i = 0; i < LedConstants.NumLeds; i++) {
                m_buffer.setLED(i, new Color(255, 30, 0));
                m_lightStrip.setData(m_buffer);
            }
            for(int i = 0; i < LedConstants.NumLeds; i++) {
                m_buffer.setLED(i, new Color(0, 0, 0));
                m_lightStrip.setData(m_buffer);
            }
            return;
         }

        if(flashing == true) {
            if(Robot.getCurrentTimeMs() % 200 < 100) {
                setRGB(0, 0, 0);
                return;
            }
        }
        if (DriverStation.isDSAttached() == false) {
            setRGB(255, 30, 0);
            return;
        }

        // Check if we are in autonomous OR if we are disabled. If so, we will show our
        // current alliance color.
        if (DriverStation.isAutonomous() || DriverStation.isDisabled()) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                setRGB(255, 0, 0);
            } else if (DriverStation.getAlliance() == Alliance.Blue) {
                setRGB(0, 0, 255);
            }
            return;
        }

        if(m_griperHasPieceSupplier.get() == true) {
            setRGB(0, 255, 0);
            return;
        }

        // Check whether we are in cubee mode or cone mode. Set the led color
        // appropriately.
        if (m_armModeSupplier.get() == ArmMode.Cone) {
            setRGB(255, 100, 0);
            return;
        } else if (m_armModeSupplier.get() == ArmMode.Cube) {
            setRGB(115, 0, 75);
            return;
        } else if (m_armModeSupplier.get() == ArmMode.Unset) {
            setRGB(200, 200, 200);
            return;
        }
    }

    public void setRGB(int red, int green, int blue) {
        synchronized (m_mutex) {
            for (int i = 0; i < m_buffer.getLength(); i++) {
                m_buffer.setRGB(i, red, green, blue);
            }
            m_lightStrip.setData(m_buffer);
        }
    }
}
