package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.auto.commands.BaseAutoCommand;

import frc.robot.subsystems.SwerveDrive;

public class StartingPositon extends BaseAutoCommand {
    private SwerveDrive m_swerveDrive;
    private double m_x, m_y, m_angle;

    public StartingPositon(ParsedCommand parsedCommand, SwerveDrive swerve) {
        super(parsedCommand);
        m_swerveDrive = swerve;
        m_x = AutoUtil.ParseDouble(parsedCommand.getArgument("x"), 0.0);
        m_y = AutoUtil.ParseDouble(parsedCommand.getArgument("y"), 0.0);
        m_angle = AutoUtil.ParseDouble(parsedCommand.getArgument("angle"), 0.0);
    }

    @Override
    protected void afterWorking() {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected void beforeWorking() {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected boolean isWorkDone() {
        return true;
    }

    @Override
    protected void work() {
        m_swerveDrive.resetOdometry(m_x, m_y, m_angle);
    }
    
}
