package framework.src.main.java.org.frc10506.framework.control;

import framework.src.main.java.org.frc10506.framework.scheduler.task.TaskScheduler;

public class StandardButton implements Button {

	private final Controller controller;
	private final int id;

	public StandardButton(Controller controller, int id) {
		this.controller = controller;
		this.id = id;
	}

	@Override
	public TaskScheduler scheduler() {
		return controller.scheduler;
	}

	@Override
	public boolean isActive() {
		return controller.backingController.getRawButton(id);
	}
}