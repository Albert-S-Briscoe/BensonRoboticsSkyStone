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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name="TeleOp Compass Toggle", group="Linear Opmode")
public class MecanumLogCompassToggleTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ////////////////////////////// Init //////////////////////////////

        ElapsedTime         runtime = new ElapsedTime();
        RobotHardware H = new RobotHardware();
        H.init(hardwareMap);

        ////////////////////////////// Init Variables //////////////////////////////

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors

        final double logCurve = 35;

        double y;
        double x;
        double Rotate;
        double Radius;
        double stickTotal;
        double multiplier;
        double Angle;
        double cosAngle;
        double sinAngle;

        boolean slowDown = false;
        boolean speedButton = false;
        boolean useCompass = true;
        boolean compassButton = false;
        boolean vertPosButton = false;
        double  position = 1;
        double V_pos;

        double agl_frwd = 180;
        double heading = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            ////////////////////////////// Set Variables //////////////////////////////

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                if (gamepad1.right_stick_x > 0) {
                    Rotate = -((Math.log10((-Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.85 + 1;
                } else {
                    Rotate = -(-((Math.log10((Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.85 + 1);
                }
            } else {
                Rotate = 0;
            }
            //Radius = (Math.log10((Math.hypot(x, y) + 1 / logCurve) * logCurve)) / Math.log10(logCurve + 1);
            if (Math.hypot(x, y) > 0.05) {
                Radius = -((Math.log10((-Range.clip(Math.hypot(x, y), -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.85 + 1;
            } else {
                Radius = 0;
            }
            stickTotal = Radius + Math.abs(Rotate);
            Angle = Math.atan2(y, x) + Math.toRadians(agl_frwd - heading - 45);
            cosAngle = Math.cos(Angle);
            sinAngle = Math.sin(Angle);

            ////////////////////////////// Mecanum Wheel Stuff //////////////////////////////

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

            if (stickTotal > 1) {
                leftfrontPower = leftfrontPower / stickTotal;
                rightfrontPower = rightfrontPower / stickTotal;
                leftbackPower = leftbackPower / stickTotal;
                rightbackPower = rightbackPower / stickTotal;
            }

            ////////////////////////////// Buttons //////////////////////////////

            if (!H.limit.getState()) {
                V_pos = (gamepad1.left_trigger + 1) / 2;
            } else if (H.vertpos.getVoltage() < .15 ) {
                V_pos = (1 - gamepad1.right_trigger) / 2;
            } else {
                V_pos = (gamepad1.left_trigger - gamepad1.right_trigger + 1) / 2;
            }
            H.Vertical.setPosition(V_pos);

            if (gamepad1.right_bumper | gamepad1.b) {
                if (!vertPosButton) {
                    if (position == 1) {
                        position = 0;
                    } else {
                        position = 1;
                    }
                    vertPosButton = true;
                }
            } else {
                vertPosButton = false;
            }
            H.grabber.setPosition(position);

            if (gamepad1.dpad_up) {
                H.grab(false);
            } else if (gamepad1.dpad_down) {
                H.grab(true);
            }

            if (gamepad1.y) {
                if (!compassButton) {
                    useCompass = !useCompass;
                    compassButton = true;
                }
            } else {
                compassButton = false;
            }

            if (useCompass) {
                heading = H.getheading();
            } else {
                heading = agl_frwd;
            }

            if (gamepad1.x && stickTotal < 0.1) {
                agl_frwd = heading;
            }


            if (gamepad1.left_bumper | gamepad1.left_stick_button) {
                if (!speedButton) {
                    speedButton = true;
                    slowDown = !slowDown;
                }
            } else {
                speedButton = false;
            }
    
            if (slowDown) {
                H.driveMotor[0].setPower(leftfrontPower / 2);
                H.driveMotor[1].setPower(rightfrontPower / 2);
                H.driveMotor[2].setPower(rightbackPower / 2);
                H.driveMotor[3].setPower(leftbackPower / 2);
            } else {
                H.driveMotor[0].setPower(leftfrontPower);
                H.driveMotor[1].setPower(rightfrontPower);
                H.driveMotor[2].setPower(rightbackPower);
                H.driveMotor[3].setPower(leftbackPower);
            }

            ////////////////////////////// Telemetry //////////////////////////////

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("front", "%.01f cm", H.upperRange.getDistance(DistanceUnit.CM));
            telemetry.addData("back", "%.01f cm", H.lowerRange.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", "%.1f", heading);
            telemetry.addData("speed before log", "%.3f", Math.hypot(x, y));
            telemetry.addData("speed", "%.3f", Radius);
            telemetry.addData( "vertpos", "%.2f", H.vertpos.getVoltage());
            telemetry.addData("frontMotors", "left (%.2f), right (%.2f)", leftfrontPower, rightfrontPower);
            telemetry.addData("backMotors", "left (%.2f), right (%.2f)", leftbackPower, rightbackPower);
            telemetry.update();
        }
    }
}
