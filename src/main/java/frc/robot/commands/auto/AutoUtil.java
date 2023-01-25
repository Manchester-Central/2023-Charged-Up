package frc.robot.commands.auto;

public class AutoUtil {
    public static double ParseDouble(String stringToParse, double defaultValue) {
        if(stringToParse == null) {
            return defaultValue;
        }

        return Double.parseDouble(stringToParse);
    }
    
}
