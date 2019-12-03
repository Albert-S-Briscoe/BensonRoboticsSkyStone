package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor FrontRange;
    public DistanceSensor BackRange;
    public AnalogInput    vertpos;
    public DigitalChannel limit;
    public BNO055IMU      imu;
    public Orientation    angles;

    ////////////////////////////// Motors //////////////////////////////

    public DcMotor        leftfront;
    public DcMotor        rightfront;
    public DcMotor        leftback;
    public DcMotor        rightback;
    public Servo          grabber;
    public Servo          vertical;
    public Servo          L;
    public Servo          R;

    public void init(HardwareMap HM) {

        ////////////////////////////// Hardware Map //////////////////////////////

        leftfront   = HM.get(DcMotor.class, "LF_drive");
        rightfront  = HM.get(DcMotor.class, "RF_drive");
        leftback    = HM.get(DcMotor.class, "LB_drive");
        rightback   = HM.get(DcMotor.class, "RB_drive");

        grabber     = HM.get(Servo.class, "grabber");
        vertical    = HM.get(Servo.class, "vertical");
        L           = HM.get(Servo.class, "GrabberLeft");
        R           = HM.get(Servo.class, "GrabberRight");

        limit       = HM.get(DigitalChannel.class, "limit");
        vertpos     = HM.get(AnalogInput.class, "vert_pos");
        FrontRange  = HM.get(DistanceSensor.class, "front_range");
        BackRange   = HM.get(DistanceSensor.class, "back_range");
        imu         = HM.get(BNO055IMU.class, "imu");

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        limit.setMode(DigitalChannel.Mode.INPUT);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)FrontRange;

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double getheading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 180;
    }
}
