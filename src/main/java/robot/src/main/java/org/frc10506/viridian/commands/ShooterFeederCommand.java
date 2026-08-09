package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;

public class ShooterFeederCommand extends Command{

    private final Shooter shooter ;
    private final double power;

    public ShooterFeederCommand(Shooter shooter, double power){
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void execute() {
        this.shooter.setFeederSpeed(power);
    }
    
    /*@Override
    public boolean isFinished() {
        return true;
    }*/
    @Override
    public void end(boolean isInterupted) {
        this.shooter.setFeederSpeed(0);
    }
}
