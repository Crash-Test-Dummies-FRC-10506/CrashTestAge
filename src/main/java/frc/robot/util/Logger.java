package frc.robot.util;

import dev.doglog.DogLog;
import com.revrobotics.spark.SparkMax;

public class Logger extends DogLog {
    public static void log(String key, SparkMax motor) { // idk only things that I can think of
        log(key + "/Voltage", motor.getBusVoltage());
        log(key + "/Temperature", motor.getMotorTemperature());
    }
}
