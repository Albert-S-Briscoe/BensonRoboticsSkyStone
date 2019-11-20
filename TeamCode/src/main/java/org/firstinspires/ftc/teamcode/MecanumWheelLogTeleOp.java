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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class MecanumWheelLogTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorRange;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    private DcMotor leftback = null;
    private DcMotor rightback = null;
    DigitalChannel limit;
    Servo   grabber;
    Servo   vertical;
    double  position = 1;
    boolean button = false;
    double V_pos;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sensorRange = hardwareMap.get(DistanceSensor .class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        leftfront  = hardwareMap.get(DcMotor.class, "LF_drive");
        rightfront = hardwareMap.get(DcMotor.class, "RF_drive");
        leftback  = hardwareMap.get(DcMotor.class, "LB_drive");
        rightback = hardwareMap.get(DcMotor.class, "RB_drive");
        limit = hardwareMap.get(DigitalChannel.class, "limit");

        limit.setMode(DigitalChannel.Mode.INPUT);

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grabber = hardwareMap.get(Servo.class, "grabber");
        vertical = hardwareMap.get(Servo.class, "vertical");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double Rotate = gamepad1.right_stick_x;
            double logCurve = 15;
            double Radius = (Math.log10((Math.hypot(x, y) + 1 / logCurve) * logCurve)) / Math.log10(logCurve + 1);
            double stickTotal = Radius + Math.abs(Rotate);
            double Angle = Math.atan2(y, x) - Math.PI / 4;
            double cosAngle = Math.cos(Angle);
            double sinAngle = Math.sin(Angle);
            double LF_RB;  //leftfront and rightback motors
            double RF_LB;  //rightfront and leftback motors
            double multiplier;

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



            // Show run time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("frontMotors", "left (%.2f), right (%.2f)", leftfrontPower, rightfrontPower);
            telemetry.addData("backMotors", "left (%.2f), right (%.2f)", leftbackPower, rightbackPower);
            telemetry.update();
        }
    }
}
