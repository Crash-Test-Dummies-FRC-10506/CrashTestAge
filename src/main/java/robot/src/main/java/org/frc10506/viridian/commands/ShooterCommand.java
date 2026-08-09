package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;

public class ShooterCommand extends Command{

    private final Shooter shooter ;
    private final double power;

    public ShooterCommand(Shooter shooter, double power){
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void initialize() {
        this.shooter.setShooterSpeed(power);
    }

    @Override
    public void end(boolean isInterupted) {
        this.shooter.setShooterSpeed(0);
    }
}
