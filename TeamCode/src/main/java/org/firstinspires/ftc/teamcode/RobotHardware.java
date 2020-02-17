package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {
    /**
     *  to use RobotHardware use this:
     *
     *      RobotHardware H = new RobotHardware();
     *
     *  then run this in runOpMode()
     *
     *      H.init(hardwareMap)
     *
     *  if you want to read a sensor value or change motor speed use this:
     *
     *      H.[sensor/motor name].[function name]([var 1], [var 2] ...);
     */

    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor upperRange;
    public DistanceSensor lowerRange;
    public DistanceSensor sensorRange;
    public AnalogInput    vertpos;
    public DigitalChannel limit;
    public BNO055IMU      imu;
    public Orientation    angles;

    ////////////////////////////// Motors //////////////////////////////

    public DcMotor        leftfront;
    public DcMotor        rightfront;
    public DcMotor        leftback;
    public DcMotor        rightback;
    public DcMotor        vertical;
    public Servo          grabber;
    public Servo          Vertical;
    public Servo          sensorServo;
    public Servo          centerServo;
    public Servo          L;
    public Servo          R;

    public void init(HardwareMap HM) {

        ////////////////////////////// Hardware Map //////////////////////////////

        leftfront   = HM.get(DcMotor.class, "LF_drive");
        rightfront  = HM.get(DcMotor.class, "RF_drive");
        leftback    = HM.get(DcMotor.class, "LB_drive");
        rightback   = HM.get(DcMotor.class, "RB_drive");

        grabber     = HM.get(Servo.class, "grabber");  // 1 = open, 0 = closed
        sensorServo = HM.get(Servo.class, "sensor");
        centerServo = HM.get(Servo.class, "center");
        vertical    = HM.get(DcMotor.class, "vertical");
        L           = HM.get(Servo.class, "GrabberLeft");
        R           = HM.get(Servo.class, "GrabberRight");

        //vertpos     = HM.get(AnalogInput.class, "vert_pos");
        upperRange  = HM.get(DistanceSensor.class, "upper_range");
        lowerRange  = HM.get(DistanceSensor.class, "lower_range");
        sensorRange = HM.get(DistanceSensor.class, "sensor_range");
        imu         = HM.get(BNO055IMU.class, "imu");

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        Rev2mDistanceSensor SensorTimeOfFlight1 = (Rev2mDistanceSensor) upperRange;
        Rev2mDistanceSensor SensorTimeOfFlight2 = (Rev2mDistanceSensor) lowerRange;
        Rev2mDistanceSensor SensorTimeOfFlight3 = (Rev2mDistanceSensor) sensorRange;

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vertical.setDirection(DcMotor.Direction.REVERSE);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public double getheading() {
        // returns a value between 0 and 360
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 180;

    }

    public void grab(boolean down) {

        if (down) {
            L.setPosition(1);
            R.setPosition(0);
        } else {
            L.setPosition(0);
            R.setPosition(1);
        }

    }

    public void block(boolean closed) {

        if (closed) {
            centerServo.setPosition(0.9);
            grabber.setPosition(0);
        } else {
            centerServo.setPosition(0.4);
            grabber.setPosition(1);
        }

    }

}
