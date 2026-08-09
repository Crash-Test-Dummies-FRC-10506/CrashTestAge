package framework.src.main.java.org.frc10506.framework;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public record AutoProfile(String name, Supplier<Command> supplier, int id) {

    public AutoProfile   {
        Objects.requireNonNull(name);
        Objects.requireNonNull(supplier);
    }
}
