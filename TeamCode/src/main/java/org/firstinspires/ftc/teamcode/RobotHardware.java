package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    public DistanceSensor sensorRange;
    public Servo grabber;
    public Servo vertical;
    DigitalChannel limit;

    public void init(HardwareMap HW) {

        grabber = HW.get(Servo.class, "grabber");
        vertical = HW.get(Servo.class, "vertical");

        limit = HW.get(DigitalChannel.class, "limit");

        limit.setMode(DigitalChannel.Mode.INPUT);

        sensorRange = HW.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

    }
}
