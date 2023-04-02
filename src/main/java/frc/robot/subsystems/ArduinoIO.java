package frc.robot.subsystems;


import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoIO extends SubsystemBase {
    private SerialPort m_arduinoPort;
    private String m_rgbMessage;
    private Object m_mutex = new Object();

    public ArduinoIO() {
        setRGB(100, 100, 100);
        SmartDashboard.putNumber("led/red", 100);
        SmartDashboard.putNumber("led/green", 100);
        SmartDashboard.putNumber("led/blue", 100);
        try {
            m_arduinoPort = new SerialPort(9600, SerialPort.Port.kUSB2);
        } catch(Exception e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void periodic() {
        var red = (int)SmartDashboard.getNumber("led/red", 100);
        var green = (int)SmartDashboard.getNumber("led/green", 100);
        var blue = (int)SmartDashboard.getNumber("led/blue", 100);
        setRGB(red, green, blue);
        synchronized(m_mutex) {
            try {
                m_arduinoPort.writeString(m_rgbMessage);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setRGB(int red, int green,  int blue) {  
        synchronized(m_mutex) {
            String redString = String.format("%03d", red);
            String greenString = String.format("%03d", green);
            String blueString = String.format("%03d", blue);
            m_rgbMessage = redString + greenString + blueString + ";";
        }
    }

   


}


