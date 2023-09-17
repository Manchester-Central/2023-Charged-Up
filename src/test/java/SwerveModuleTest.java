import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.chaos131.swerve.BaseSwerveModule;


public class SwerveModuleTest {
    @Test
    public void testAngle() {
        assertEquals(0, BaseSwerveModule.closestTarget(0, 0));
        assertEquals(1800, BaseSwerveModule.closestTarget(1800, 360));
        assertEquals(180, BaseSwerveModule.closestTarget(90, 180));
        assertEquals(180, BaseSwerveModule.closestTarget(0, 180));
        assertEquals(-180, BaseSwerveModule.closestTarget(0, -180));
        assertEquals(-1800, BaseSwerveModule.closestTarget(-1800, -360)); 
        assertEquals(-3600, BaseSwerveModule.closestTarget(-3600, 360));
        assertEquals(3600, BaseSwerveModule.closestTarget(3600, -720));
        assertEquals(3600.5, BaseSwerveModule.closestTarget(3600.87, 0.5));
        assertEquals(390, BaseSwerveModule.closestTarget(390, 30 ));
        assertEquals(230, BaseSwerveModule.closestTarget(390, 590));
        
    }

    // "Inflation happens man" - JoshuaAllard2022 
  } 
