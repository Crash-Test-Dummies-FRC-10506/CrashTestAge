package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PowerShooterMotionMagic extends Command {

    private final Shooter m_Shooter;
    private final double m_velocity;

    public PowerShooterMotionMagic(Shooter shooter, double velocity) {
        m_Shooter = shooter;
        m_velocity = velocity;
    }

    @Override
    public void execute() {
        m_Shooter.shootMotionMagic(m_velocity);
    }
}