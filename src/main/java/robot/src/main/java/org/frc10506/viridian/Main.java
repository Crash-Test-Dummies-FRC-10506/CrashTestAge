//WAIT! this package below should read "robot.src.main.java.org.frc10506.viridian" IF you are on Viridian Framework, if you've forked Viridian into the current year(like Rebuilt2026)
//this should robot.src.main.java.org.frc10506.[namehere] AND your file path should read src\main\java\robot\org\frc10506\[name here]
package robot.src.main.java.org.frc10506.viridian;

import edu.wpi.first.wpilibj.RobotBase;

public interface Main {

    static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }
}
