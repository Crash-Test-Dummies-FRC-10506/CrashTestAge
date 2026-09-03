package framework.src.main.java.org.frc10506.framework;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import framework.src.main.java.org.frc10506.framework.phase.Phase;
import framework.src.main.java.org.frc10506.framework.phase.PhaseController;
import framework.src.main.java.org.frc10506.framework.scheduler.loop.Loop;
import framework.src.main.java.org.frc10506.framework.scheduler.subsystem.SubsystemStore;
import framework.src.main.java.org.frc10506.framework.scheduler.subsystem.TickedSubsystem;
import framework.src.main.java.org.frc10506.framework.scheduler.task.TaskScheduler;
import framework.src.main.java.org.frc10506.framework.util.log.Logger;
import robot.src.main.java.org.frc10506.viridian.util.PhaseTimer;
import framework.src.main.java.org.frc10506.framework.control.Controller;

/**
 * The base robot type that all robots can extend to access the scheduling and
 * phase features. {@link PhaseDrivenRobot}s hold a state machine used for the
 * current phase, a task scheduler, and a subsystem store.
 *
 * <p>
 * Robots are built by scheduling tasks to {@link Phase}s, where there are hooks
 * for each phase that can be used to add commands and buttons.
 * </p>
 *
 * <p>
 * This class is also responsible for handling a robot's subsystems. In order to
 * get a subsystem with a {@link TickedSubsystem#periodic()} handle to work, it
 * must be added through the {@link SubsystemStore}.
 */
public abstract class PhaseDrivenRobot extends TimedRobot {

    private static final Logger LOG = new Logger("Robot");
    private final Controller controller = new Controller(this.scheduler, 0, 0.1);
    private final PhaseTimer phaseTimer = new PhaseTimer();
    private PhaseTimer.Phase lastPhase = null;
    private double[] rumblePattern = null;
    private int rumblePatternIndex = 0;
    private double rumbleStepEndTime = 0;
    private boolean warningRumbleSent = false;

    protected final TaskScheduler scheduler = new TaskScheduler();
    protected final PhaseController phaseController = new PhaseController(scheduler);
    protected final SubsystemStore subsystems = new SubsystemStore(scheduler);

    public PhaseDrivenRobot() {
        super();
    }

    public PhaseDrivenRobot(double period) {
        super(period);
    }

    @Override
    public final void robotPeriodic() {
        if (phaseController.isTransitioning()) {
            LOG.warn("Scheduler tick submitted during transition. Skipped.");
            return;
        }

        // Tick the main loop. This loop just runs on the default robot period.
        scheduler.loopStore.main.tick();

        {
            // Grab the queue of untracked loops.
            var loops = scheduler.loopStore.getUntrackedLoops();
            if (loops.isEmpty()) {
                return; // Optimization: early return
            }
            // This is a fast way to iterate over all untracked loops and to schedule them while popping them.
            // This is optimized to prevent tick overruns as it executes each tick on periodic.
            Loop loop;
            while ((loop = loops.pollFirst()) != null) {
                addPeriodic(loop::tick, loop.getPeriodSeconds());
            }
        }
    }

    // <editor-fold desc="> Phase hooks" defaultstate="collapsed">
    @Override
    public final void robotInit() {
        LOG.info("Robot initialized.");
        // Signal that we're about to transition out of INIT as soon as the scheduler does a sweep
        phaseController.beginTransition();
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    /**
     * The hook for when a robot enters the {@link Phase#DISABLED} phase. It
     * should be used to set up any tasks that should be executed in this phase.
     */
    protected void disabledSequence() {

    }

    /**
     * The hook for when a robot enters the {@link Phase#AUTONOMOUS} phase. It
     * should be used to set up any tasks that should be executed in this phase.
     */
    protected void autonomousSequence() {

    }

    /**
     * The hook for when a robot enters the {@link Phase#TELEOP} phase. It
     * should be used to set up any tasks that should be executed in this phase.
     */
    protected void teleopSequence() {

    }

    /**
     * The hook for when a robot enters the {@link Phase#TEST} phase. It should
     * be used to set up any tasks that should be executed in this phase.
     */
    protected void testSequence() {

    }

    // Initialization methods.
    @Override
    public final void disabledInit() {
        phaseController.transition(Phase.DISABLED);
        disabledSequence();
    }

    @Override
    public final void autonomousInit() {
        phaseController.transition(Phase.AUTONOMOUS);
        autonomousSequence();
    }

    @Override
    public final void teleopInit() {
        phaseController.transition(Phase.TELEOP);
        phaseTimer.markTeleopStart();
        teleopSequence();
    }

    @Override
    public final void testInit() {
        phaseController.transition(Phase.TEST);
        testSequence();
    }

    @Override
    public final void disabledExit() {
        phaseController.beginTransition();
    }

    @Override
    public final void autonomousExit() {
        phaseController.beginTransition();
    }

    @Override
    public final void teleopExit() {
        phaseController.beginTransition();
    }

    @Override
    public final void testExit() {
        phaseController.beginTransition();
    }

    // These methods have a default implementation that prints a warning. This adds overhead to the scheduler
    // that we want to avoid, so we just have blank methods.
    @Override
    public final void simulationPeriodic() {
        this.robotPeriodic();
    }

    @Override
    public final void disabledPeriodic() {

    }

    @Override
    public final void autonomousPeriodic() {

    }

    @Override
    public final void teleopPeriodic() {
        PhaseTimer.Phase currentPhase = phaseTimer.getCurrentPhase();
        double remaining = phaseTimer.getSecondsRemainingInCurrentPhase();

        if (remaining <= 5.0 && remaining > 4.5 && !warningRumbleSent && rumblePattern == null) {
      // Pattern: on 0.15s, off 0.1s, on 0.15s, off 0.1s, on 0.15s
      rumblePattern = new double[]{0.15, 0.1, 0.15, 0.1, 0.15};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = true;
    }

    if (lastPhase != null && currentPhase != lastPhase) {
      rumblePattern = new double[]{0.8};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = false;
    }
    lastPhase = currentPhase;

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (rumblePattern != null) {
      if (rumbleStepEndTime == 0) {
        boolean isOn = (rumblePatternIndex % 2) == 0;
        this.controller.setRumble(isOn);
        if (isOn) System.out.println("Rumble triggered");
        rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
      } else if (now >= rumbleStepEndTime) {
        rumblePatternIndex++;
        if (rumblePatternIndex >= rumblePattern.length) {
          // Pattern done
          this.controller.setRumble(false);
          rumblePattern = null;
        } else {
          boolean isOn = (rumblePatternIndex % 2) == 0;
          if (isOn) System.out.println("Rumble triggered");
          this.controller.setRumble(isOn);
          rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
        }
      }
    }

        SmartDashboard.putString("Current Phase", currentPhase.name());
        //SmartDashboard.putNumber("Phase/ElapsedSec", phaseTimer.getElapsedSec());
        //SmartDashboard.putNumber("Phase/SecsInPhase", phaseTimer.getSecondsIntoCurrentPhase());
        SmartDashboard.putNumber("Seconds Remaining In Phase", phaseTimer.getSecondsRemainingInCurrentPhase());
    }

    @Override
    public final void testPeriodic() {

    }
    // </editor-fold>
}
