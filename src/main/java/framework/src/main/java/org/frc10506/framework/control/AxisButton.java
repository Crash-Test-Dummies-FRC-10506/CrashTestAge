package framework.src.main.java.org.frc10506.framework.control;

import framework.src.main.java.org.frc10506.framework.scheduler.task.TaskScheduler;

public record AxisButton(Controller controller, Axis axis) implements Button {

	@Override
	public boolean isActive() {
		return axis.get() != 0;
	}

	@Override
	public TaskScheduler scheduler() {
		return controller.scheduler;
	}
}