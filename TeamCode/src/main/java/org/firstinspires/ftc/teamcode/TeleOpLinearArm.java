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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="TeleOp Linear Arm", group="Linear Opmode")
public class TeleOpLinearArm extends LinearOpMode {

    boolean first = true;

    private double LF_RB = 0;  //leftfront and rightback motors
    private double RF_LB = 0;  //rightfront and leftback motors
    
    private boolean[] button = {false, false, false, false, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = grabber, 4 = arm up, 5 = arm down
    private boolean[] toggle = {false, true, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = grabber

    private double agl_frwd;

    RobotHardware H = new RobotHardware();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ////////////////////////////// Init //////////////////////////////

        MecanumWheelDriver drive = new MecanumWheelDriver(H);
        ElapsedTime runtime = new ElapsedTime();
        ExecutorService pool = Executors.newFixedThreadPool(1);
        LinearArmDriver arm = new LinearArmDriver(H);
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
        double Target;
        double rotateRadius;
        int blockpos = 0;
        double upstartTime = 0;
        double downstartTime = 0;

        //boolean  position = false;

        double agl_frwd = 180;
        double heading = 0;

        final byte maxblocks = 7;
        final double repeatDelay = 0.425;
        final double sideOffset = 3.5;
        final double forwardOffset = 0.75;
        final double tolerance = 0.50;
        double block_X = 0;
        double block_Y = 0;
        double angleToBlockPos;
        double distanceToBlockPos;
        boolean atPos = false;


        waitForStart();
        runtime.reset();
        arm.zero();
        pool.execute(arm);

        while (opModeIsActive()) {

            ////////////////////////////// Set Variables //////////////////////////////

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;

            if (toggle[2]) {

                rotateRadius = -((Math.log10((-Range.clip(Math.hypot(gamepad1.right_stick_x, (-gamepad1.right_stick_y)), 0, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1;

                if (rotateRadius > 0.2) {
                    Target = drive.addDegree(Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)), -90);
                    Rotate = drive.POVRotate(Target + agl_frwd, rotateRadius);
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

            if (gamepad1.b) {
                if (!atPos) {
                    
                    if (!first) {
        
                        H.leftfront  .setPower(0);
                        H.rightfront .setPower(0);
                        H.leftback   .setPower(0);
                        H.rightback  .setPower(0);
        
                        H.sensorServo.setPosition(1);
                        sleep(500);
                        
                        first = true;
                        
                    }
    
                    block_X = (H.sensorRange.getDistance(DistanceUnit.INCH) - sideOffset) * 1.19;
                    block_Y = H.lowerRange.getDistance(DistanceUnit.INCH) - forwardOffset;
    
                    if (block_Y < 1.5) {
        
                        angleToBlockPos = Math.atan2(block_X, block_Y);
                        distanceToBlockPos = Math.hypot(block_X, block_Y) + 0.4;
        
                    } else {
        
                        angleToBlockPos = 0;
                        distanceToBlockPos = block_Y;
        
                    }
    
                    if (distanceToBlockPos - 0.4 > tolerance) {
        
                        MoveInches(angleToBlockPos, distanceToBlockPos);
                        MoveLoop(1);
        
                    } else {
        
                        H.sensorServo.setPosition(0.05);
                        arm.moveInches(-0.75);
                        toggle[3] = false; // grabber
                        H.block(false);
                        sleep(450);
                        arm.moveToBlock(blockpos);
                        atPos = true;
        
                    }
    
                }

            } else {

                if (first) {

                    first = false;
                    atPos = false;
                    this.LF_RB = 0;
                    this.RF_LB = 0;

                    H.sensorServo.setPosition(0.05);
                    H.leftfront  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.rightfront .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.leftback   .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.rightback  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }

                if (toggle[0]) {

                    H.leftfront  .setPower(leftfrontPower / 2);
                    H.rightfront .setPower(rightfrontPower / 2);
                    H.leftback   .setPower(leftbackPower / 2);
                    H.rightback  .setPower(rightbackPower / 2);

                } else {

                    H.leftfront  .setPower(leftfrontPower);
                    H.rightfront .setPower(rightfrontPower);
                    H.leftback   .setPower(leftbackPower);
                    H.rightback  .setPower(rightbackPower);

                }

            }

            ////////////////////////////// Move Arm //////////////////////////////

            if (gamepad1.left_trigger - gamepad1.right_trigger > 0.25) {

                if (!button[4] || runtime.seconds() > upstartTime + repeatDelay) {

                    upstartTime = runtime.seconds();
                    blockpos = Range.clip(blockpos - 1, 0, maxblocks);
                    button[4] = true;

                }

            } else {
    
                button[4] = false;

            }

            if (gamepad1.right_trigger - gamepad1.left_trigger > 0.25) {

                if (!button[5] || runtime.seconds() > downstartTime + repeatDelay) {

                    downstartTime = runtime.seconds();
                    blockpos = Range.clip(blockpos + 1, 0, maxblocks);
                    button[5] = true;

                }

            } else {
    
                button[5] = false;

            }

            arm.moveToBlock(blockpos);

            ////////////////////////////// Buttons //////////////////////////////
    
            testButton(gamepad1.left_bumper | gamepad1.left_stick_button, 0); // half speed
            
            testButton(gamepad1.back, 1); // compass
            
            if (toggle[1]) {
        
                heading = H.getheading();
        
            } else {
            
                heading = agl_frwd;
            
            }
            
            testButton(gamepad1.x, 2); // POV
            
            testButton(gamepad1.right_bumper || gamepad1.a, 3); // grabber
            
            H.block(toggle[3]);

            if (gamepad1.dpad_up) {

                H.grab(false);

            } else if (gamepad1.dpad_down) {

                H.grab(true);

            }
            
            if (gamepad1.start && stickTotal < 0.1) {

                agl_frwd = heading;

            }

            telemetry.addData("encoders", H.vertical.getCurrentPosition());
            telemetry.addData("difference", H.vertical.getTargetPosition() - H.vertical.getCurrentPosition());
            telemetry.addData("add", arm.addTarget);
            telemetry.addData("inches", arm.inches);
            telemetry.addData("x", block_X);
            telemetry.addData("y", block_Y);
            telemetry.addData("stop behavior", H.vertical.getZeroPowerBehavior());
            telemetry.update();

        }

        arm.stop();
        pool.shutdownNow();

    }
    
    private void testButton(boolean gamepadIn, int numb) {
    
        if (gamepadIn) {
        
            if (!button[numb]) {
            
                toggle[numb] = !toggle[numb];
                button[numb] = true;
            
            }
        
        } else {
        
            button[numb] = false;
        
        }
        
    }

    private void MoveInches(double Angle_input, double inches) {

        /**Angle_input, the angle relative to the robot that it should move
         * inches, the number of inches to move. Is not accurate when going sideways due to the mecanum wheels
         * speed, the speed at which to run the motors
         * agl_frwd, the direction the robot should face while moving
         * it will be set automatically if = -1
         * forward = 0 degrees, right = 90, left = -90, back = 180
         */
        
        final double COUNTS_PER_REVOLUTION = 288;
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * 3.141592653589793248);
        
        double Angle = Angle_input + Math.PI / 4;
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double multiplier;
        int LF_RBtarget;
        int RF_LBtarget;

        if (LF_RB == 0 && RF_LB == 0) {
            agl_frwd = H.getheading() - 180;
        }

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1 / Math.abs(cosAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        } else {
            multiplier = 1 / Math.abs(sinAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        }

        LF_RBtarget = (int) (LF_RB * inches * COUNTS_PER_INCH);
        RF_LBtarget = (int) (RF_LB * inches * COUNTS_PER_INCH);

        H.leftfront  .setTargetPosition(H.leftfront  .getCurrentPosition() + LF_RBtarget);
        H.rightfront .setTargetPosition(H.rightfront .getCurrentPosition() + RF_LBtarget);
        H.leftback   .setTargetPosition(H.leftback   .getCurrentPosition() + RF_LBtarget);
        H.rightback  .setTargetPosition(H.rightback  .getCurrentPosition() + LF_RBtarget);

        H.leftfront  .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.rightfront .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.leftback   .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.rightback  .setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    private void MoveLoop(double speed) {

        double offset;
        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;
        double multiplier;

        offset = FindDegOffset(H.getheading(), agl_frwd + 180);

        leftfrontPower = LF_RB * speed - offset / 45;
        rightfrontPower = RF_LB * speed + offset / 45;
        leftbackPower = RF_LB * speed - offset / 45;
        rightbackPower = LF_RB * speed + offset / 45;

        if (LF_RB > RF_LB) {
            if (offset > 0) {
                multiplier = 1 / Math.abs(rightbackPower);
            } else {
                multiplier = 1 / Math.abs(leftfrontPower);
            }
        } else {
            if (offset > 0) {
                multiplier = 1 / Math.abs(rightfrontPower);
            } else {
                multiplier = 1 / Math.abs(leftbackPower);
            }
        }

        H.leftfront.setPower(Range.clip(leftfrontPower * multiplier * speed, -1, 1));
        H.rightfront.setPower(Range.clip(rightfrontPower * multiplier * speed, -1, 1));
        H.leftback.setPower(Range.clip(leftbackPower * multiplier * speed, -1, 1));
        H.rightback.setPower(Range.clip(rightbackPower * multiplier * speed, -1, 1));
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

}