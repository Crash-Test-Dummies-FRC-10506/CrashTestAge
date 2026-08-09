package framework.src.main.java.org.frc10506.framework.scheduler.task.lock;

import java.util.Set;

public record TaskLock(int priority, Set<?> keys) {

    public TaskLock(int priority, Set<?> keys) {
        this.priority = priority;
        this.keys = keys;
    }
}
