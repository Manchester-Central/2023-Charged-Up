package frc.robot.commands;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArduinoIO;
import java.awt.Color;

public class SignalLEDs extends CommandBase {
    private ArduinoIO m_arduinoIO;
    private Color m_color;
    public SignalLEDs(ArduinoIO arduino, Color color) {
        m_arduinoIO = arduino;
        m_color = color;
        addRequirements(m_arduinoIO);
    }

    @Override
    public void execute() {
        m_arduinoIO.queueLEDS(m_color.getRed(), m_color.getGreen(), m_color.getBlue());
    }
}

// A command that controls the LEDs powered by the arduino. The LEDs will change color based on what game piece is being held.