package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PWMConstants;
import frc.robot.RobotContainer.ArmMode;

public class LEDs extends SubsystemBase {
    private AddressableLED m_lightStrip;
    private AddressableLEDBuffer m_buffer;
    private Supplier<ArmMode> m_armModeSupplier;
    private Supplier<Boolean> m_griperHasPieceSupplier;
    private Object m_mutex = new Object();
    public static boolean flashing = false;
    private int m_rainbowFirstPixelHue = 0;

    public LEDs(Supplier<ArmMode> armModeSupplier, Supplier<Boolean> gripperHasPiece) {
        m_armModeSupplier = armModeSupplier;
        m_griperHasPieceSupplier = gripperHasPiece;
        SmartDashboard.putNumber("red", 255);
        SmartDashboard.putNumber("green", 0);
        SmartDashboard.putNumber("blue", 100);
        m_buffer = new AddressableLEDBuffer(60);
        m_lightStrip = new AddressableLED(PWMConstants.lightStripPWMPort);
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
        if(flashing == true) {
            rainbow();
            return;
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

        // Hijack cube/cone to get CHAOS colors.
        if (m_armModeSupplier.get() == ArmMode.Cone) { // Green
            setRGB(255, 100, 0);
            return;
        } else if (m_armModeSupplier.get() == ArmMode.Cube) { // Orange
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

    // Ripped straight from the WPILib documentation. Smooth.
    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_buffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_buffer.getLength())) % 180;
        // Set the value
        m_buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_lightStrip.setData(m_buffer);
  }
}
