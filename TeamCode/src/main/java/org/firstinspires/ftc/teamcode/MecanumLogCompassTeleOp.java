/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRCompass;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TeleOp_Compass", group="Linear Opmode")
public class MecanumLogCompassTeleOp extends LinearOpMode {

    private BNO055IMU           compass = hardwareMap.get(BNO055IMU.class, "compass");
    Orientation             lastAngles = new Orientation();
    double globalAngle;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime         runtime = new ElapsedTime();
        DcMotor           leftfront = hardwareMap.get(DcMotor.class, "LF_drive");
        DcMotor          rightfront = hardwareMap.get(DcMotor.class, "RF_drive");
        DcMotor            leftback = hardwareMap.get(DcMotor.class, "LB_drive");
        DcMotor           rightback = hardwareMap.get(DcMotor.class, "RB_drive");
        DistanceSensor  sensorRange = hardwareMap.get(DistanceSensor .class, "sensor_range");
        DigitalChannel        limit = hardwareMap.get(DigitalChannel.class, "limit");
        //CompassSensor       compass = hardwareMap.get(CompassSensor.class, "compass");
        Servo               grabber = hardwareMap.get(Servo.class, "grabber");
        Servo              vertical = hardwareMap.get(Servo.class, "vertical");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        compass.initialize(parameters);

        while (!isStopRequested() && !compass.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        limit.setMode(DigitalChannel.Mode.INPUT);

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        final double logCurve = 15;
        double y;
        double x;
        double Rotate = 0;
        double Radius;
        double stickTotal;
        double Angle;
        double cosAngle;
        double sinAngle;
        double LF_RB = 0;  //leftfront and rightback motors
        double RF_LB = 0;  //rightfront and leftback motors
        double multiplier;

        double  position = 1;
        boolean button = false;
        double V_pos;

        double agl_frwd = getAngle();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.x && Math.abs(LF_RB) + Math.abs(RF_LB) < .001 && Rotate < .001) {
                agl_frwd = getAngle();
            }

            if (gamepad1.right_bumper | gamepad1.b) {
                if (!button) {
                    if (position == 1) {
                        position = 0;
                    } else {
                        position = 1;
                    }
                    button = true;
                }
            } else if (button) {
                button = false;
            }
            grabber.setPosition(position);

            if (!limit.getState()) {
                V_pos = (gamepad1.left_trigger + 1) / 2;
            } else {
                V_pos = (gamepad1.left_trigger - gamepad1.right_trigger + 1) / 2;
            }
            vertical.setPosition(V_pos);

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            Rotate = gamepad1.right_stick_x;
            Radius = (Math.log10((Math.hypot(x, y) + 1 / logCurve) * logCurve)) / Math.log10(logCurve + 1);
            stickTotal = Radius + Math.abs(Rotate);
            Angle = Math.atan2(y, x) - Math.PI / 4 + Math.toRadians(agl_frwd - getAngle());
            cosAngle = Math.cos(Angle);
            sinAngle = Math.sin(Angle);

            if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
                multiplier = 1 / Math.abs(cosAngle);
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            } else {
                multiplier = 1 / Math.abs(sinAngle);
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            }

            leftfrontPower = LF_RB * Radius + Rotate; //then add the rotate speed
            rightfrontPower = RF_LB * Radius - Rotate;
            leftbackPower = RF_LB * Radius + Rotate;
            rightbackPower = LF_RB * Radius - Rotate;

            if (Math.abs(stickTotal) > 1) {
                leftfrontPower = leftfrontPower / stickTotal;
                rightfrontPower = rightfrontPower / stickTotal;
                leftbackPower = leftbackPower / stickTotal;
                rightbackPower = rightbackPower / stickTotal;
            }

            if (gamepad1.left_bumper | gamepad1.a) {
                leftfront.setPower(leftfrontPower / 2);
                rightfront.setPower(rightfrontPower / 2);//lower servo a little
                leftback.setPower(leftbackPower / 2);
                rightback.setPower(rightbackPower / 2);
            } else {
                leftfront.setPower(leftfrontPower);
                rightfront.setPower(rightfrontPower);//lower servo a little
                leftback.setPower(leftbackPower);
                rightback.setPower(rightbackPower);
            }

            // Show run time, wheel power and sensor values.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("range", "%.01f cm", sensorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("compass", "%.1f", getAngle());
            telemetry.addData("frontMotors", "left (%.2f), right (%.2f)", leftfrontPower, rightfrontPower);
            telemetry.addData("backMotors", "left (%.2f), right (%.2f)", leftbackPower, rightbackPower);
            telemetry.update();
        }
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = compass.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}
