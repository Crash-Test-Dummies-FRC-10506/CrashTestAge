package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;

public class ShooterButVoltageCommand extends Command{

    private final Shooter shooter;
    private final double voltage;

    public ShooterButVoltageCommand(Shooter shooter, double voltage){
        this.shooter = shooter;
        this.voltage = voltage;
    }

    @Override
    public void execute() {
        this.shooter.setShooterVoltage(voltage);
    }
    
    /*@Override
    public boolean isFinished() {
        return true;
    }*/
    @Override
    public void end(boolean isInterupted) {
        this.shooter.setShooterVoltage(0);
    }
}