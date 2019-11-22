package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

    public DistanceSensor sensorRange;
    public Servo grabber;
    public Servo vertical;
    public Servo L;
    public Servo R;
    DigitalChannel limit;
    BNO055IMU imu;
    Orientation angles;

    public void init(HardwareMap HW) {

        grabber =     HW.get(Servo.class, "grabber");
        vertical =    HW.get(Servo.class, "vertical");
        limit =       HW.get(DigitalChannel.class, "limit");
        sensorRange = HW.get(DistanceSensor.class, "sensor_range");
        L = HW.get(Servo.class, "GrabberLeft");
        R = HW.get(Servo.class, "GrabberRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";

        imu = HW.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
         */

        limit.setMode(DigitalChannel.Mode.INPUT);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

    }
}
