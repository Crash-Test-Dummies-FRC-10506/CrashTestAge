package framework.src.main.java.org.frc10506.framework.scheduler.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TickedSubsystem extends Subsystem {

    default long getPeriod() {
        return -1L;
    }

    @Override
    void periodic();

    @Override
    default void simulationPeriodic() {

    }
}
