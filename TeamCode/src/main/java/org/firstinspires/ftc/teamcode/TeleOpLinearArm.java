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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Lite", group="Linear Opmode")
public class TeleOpLinearArm extends LinearOpMode {

    RobotHardware H = new RobotHardware();

    boolean first = true;

    double LF_RB = 0;  //leftfront and rightback motors
    double RF_LB = 0;  //rightfront and leftback motors

    double rotateSpeed;
    double offset;
    double heading;
    double Target;
    double rightRadius;
    byte drect;
    final private double rampDownAngl = 50;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ////////////////////////////// Init //////////////////////////////

        ElapsedTime         runtime = new ElapsedTime();
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
        boolean grabberPosButton = false;
        boolean usePOV = true;
        boolean POVButton = false;
        double  position = 0;

        double agl_frwd = 180;
        double heading = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            ////////////////////////////// Set Variables //////////////////////////////

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;

            if (usePOV) {

                rightRadius = -((Math.log10((-Range.clip(Math.hypot(gamepad1.right_stick_x, (-gamepad1.right_stick_y)), 0, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1;

                if (rightRadius > 0.2) {
                    Target = addDegree(Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)), 90);
                    Rotate = POVRotate();
                } else {
                    Rotate = 0;
                }

            } else {
                if (Math.abs(gamepad1.right_stick_x) > 0.05) {

                    if (gamepad1.right_stick_x > 0) {

                        Rotate = -((Math.log10((-Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1;

                    } else {

                        Rotate = -(-((Math.log10((Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1);

                    }

                } else {

                    Rotate = 0;

                }
            }

            if (Math.hypot(x, y) > 0.05) {

                Radius = -((Math.log10((-Range.clip(Math.hypot(x, y), -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1;

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

            if (Math.abs(stickTotal) > 1) {

                leftfrontPower = leftfrontPower / stickTotal;
                rightfrontPower = rightfrontPower / stickTotal;
                leftbackPower = leftbackPower / stickTotal;
                rightbackPower = rightbackPower / stickTotal;

            }

                if (slowDown) {

                    H.leftfront.setPower(leftfrontPower / 2);
                    H.rightfront.setPower(rightfrontPower / 2);
                    H.leftback.setPower(leftbackPower / 2);
                    H.rightback.setPower(rightbackPower / 2);

                } else {

                    H.leftfront.setPower(leftfrontPower);
                    H.rightfront.setPower(rightfrontPower);
                    H.leftback.setPower(leftbackPower);
                    H.rightback.setPower(rightbackPower);

                }

                if (LF_RB != 0 || RF_LB != 0) {

                    this.LF_RB = 0;
                    this.RF_LB = 0;
                    first = true;

                    H.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }

            ////////////////////////////// Move Arm //////////////////////////////



            ////////////////////////////// Buttons //////////////////////////////

            if (gamepad1.x) {

                if (!POVButton) {

                    usePOV = !usePOV;
                    POVButton = true;

                }

            } else {

                POVButton = false;

            }

            if (gamepad1.right_bumper || gamepad1.a) {

                if (!grabberPosButton) {

                    if (position == 0) {

                        position = 1;

                    } else {

                        position = 0;

                    }

                    grabberPosButton = true;

                }

            } else {

                grabberPosButton = false;

            }

            H.grabber.setPosition(position);

            if (gamepad1.dpad_up) {

                H.grab(false);

            } else if (gamepad1.dpad_down) {

                H.grab(true);

            }

            if (gamepad1.back) {

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

            if (gamepad1.start && stickTotal < 0.1) {

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

        }

    }

    private double POVRotate() {

        heading = H.getheading();
        offset = -FindDegOffset(heading, Target);
        rotateSpeed = Range.clip( Math.abs(offset / rampDownAngl), 0.19, rightRadius);
        drect = (byte)Range.clip(offset * 100, -1, 1);

        if (Math.abs(offset) > 5) {
            return rotateSpeed * drect;
        } else {
            return 0;
        }

    }

    private double FindDegOffset(double DegCurrent, double TargetDeg) {

        /**DegCurrent, the current degree of the robot value between 0 and 360
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -180 and 180
         * output will be negative if the current degree is left of the target, positive if on the right
         *    0
         * 90   -90
         *   180
         */

        double offset = TargetDeg - DegCurrent;
        if (offset > 180) {
            offset -= 360;
        } else if (offset < -180) {
            offset += 360;
        }
        return offset;
    }

    private double addDegree(double DegCurrent, double addDeg) {

        /**adds a number of degrees to the current degree with rapping around from 360 to 0
         * returns a value between 0 and 360
         */

        double output = DegCurrent + addDeg;
        while (output < 0 || output > 360) {
            if (output >= 360) {
                output -= 360;
            } else if (output < 0) {
                output += 360;
            }
        }
        return output;
    }

}
