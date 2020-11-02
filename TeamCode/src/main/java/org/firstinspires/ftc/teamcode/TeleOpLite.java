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
public class TeleOpLite extends LinearOpMode {

    RobotHardware H = new RobotHardware();

    private int Angle_Degrees;
    private double inches;
    public double speed;
    private double agl_frwd;

    private final double COUNTS_PER_REVOLUTION = 288;
    private final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //final double ROBOT_DIAMETER_INCHES = 23;
    private final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * 3.14159);

    boolean selfcorrect = true;
    boolean first = true;

    int LF_RBtarget;
    int RF_LBtarget;

    double LF_RB = 0;  //leftfront and rightback motors
    double RF_LB = 0;  //rightfront and leftback motors

    int leftfrontStartPos;
    int rightfrontStartPos;
    int leftbackStartPos;
    int rightbackStartPos;

    double rotateSpeed;
    double offset;
    double heading;
    double Target;
    double rotateRadius;
    byte drect;
    final private double rampDownAngl = 50;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ////////////////////////////// Init //////////////////////////////

        ElapsedTime runtime = new ElapsedTime();
        MecanumWheelDriver drive = new MecanumWheelDriver(H);
        H.init(hardwareMap);

        ////////////////////////////// Init Variables //////////////////////////////

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors

        final double logCurve = 35;
        final double zeroVolts = 0.33;
        final double degreesPerVolt = 111;
        final int maxDeg = 120;
        final double armLength = 11.75;
        final double armOffset = 0;
        final double rampDownAngle = 20;

        double y;
        double x;
        double Rotate;
        double Radius;
        double stickTotal;
        double multiplier;
        double Angle;
        double cosAngle;
        double sinAngle;

        double armAngle;
        double grabberToRobot;
        double previousArmPos = Math.cos(Math.toRadians((H.vertpos.getVoltage() - zeroVolts) * degreesPerVolt)) * armLength - armOffset;
        double inchesToCompensate = 0;

        boolean slowDown = false;
        boolean speedButton = false;
        boolean useCompass = true;
        boolean compassButton = false;
        boolean compensatebutton = false;
        boolean useArmCompensate = true;
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

                rotateRadius = -((Math.log10((-Range.clip(Math.hypot(gamepad1.right_stick_x, (-gamepad1.right_stick_y)), 0, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) * 0.825 + 1;

                if (rotateRadius > 0.2) {
                    Target = drive.addDegree(Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)), 90);
                    Rotate = drive.POVRotate(Target, rotateRadius);
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

            armAngle = (H.vertpos.getVoltage() - zeroVolts) * degreesPerVolt;
            grabberToRobot = Math.cos(Math.toRadians(armAngle)) * armLength - armOffset;

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


            ////////////////////////////// Compensate For Arm //////////////////////////////

            if (stickTotal < 0.02 && useArmCompensate) {

                inchesToCompensate += grabberToRobot - previousArmPos;


                if (first) {

                    inchesToCompensate = grabberToRobot - previousArmPos;

                    setMoveInches(180, inchesToCompensate, 0.7, -1);


                } else {

                    changeTargetInches(inchesToCompensate);

                }

                MoveInches();
                previousArmPos = grabberToRobot;

            } else {
    
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

                if (LF_RB != 0 || RF_LB != 0) {

                    this.LF_RB = 0;
                    this.RF_LB = 0;
                    first = true;

                    H.driveMotor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.driveMotor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.driveMotor[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    H.driveMotor[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
            }

            ////////////////////////////// Move Arm //////////////////////////////

                H.Vertical.setPosition((Range.clip(gamepad1.left_trigger, 0, armAngle / rampDownAngle) - Range.clip(gamepad1.right_trigger, 0, (maxDeg - armAngle) / rampDownAngle)) / 2 + 0.5);

            ////////////////////////////// Buttons //////////////////////////////

            if (gamepad1.b) {

                if (!compensatebutton) {

                    useArmCompensate = !useArmCompensate;
                    compensatebutton = true;

                    previousArmPos = grabberToRobot;

                }

            } else {

                compensatebutton = false;

            }

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

        setMoveInches(0, 0, 0, -2);

    }

    private void MoveInches(/*int Angle_Degrees, double inches, double speed, double agl_frwd*/) {

        /**Angle_Degrees, the angle relative to the robot that it should move
         * inches, the number of inches to move. Is not accurate when going sideways due to the mecanum wheels
         * speed, the speed at which to run the motors
         * agl_frwd, the direction the robot should face while moving
         * it will be set automatically if = -1
         * forward = 0 degrees, right = 90, left = -90, back = 180
         *
         * DO NOT set speed to a negative number
         * if you want to move backwards set Angle_degrees to 180
         */

        double Angle = Math.toRadians(Angle_Degrees + 45);// - Math.PI / 4;
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double multiplier;

        if (LF_RB == 0 && RF_LB == 0) {

            selfcorrect = true;

            if (agl_frwd == -1) {
                agl_frwd = H.getheading() - 180;
                selfcorrect = true;
            } else if (agl_frwd == -2) {
                selfcorrect = false;
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

            leftfrontStartPos = H.driveMotor[0].getCurrentPosition();
            rightfrontStartPos = H.driveMotor[1].getCurrentPosition();
            rightbackStartPos = H.driveMotor[2].getCurrentPosition();
            leftbackStartPos = H.driveMotor[3].getCurrentPosition();

            LF_RBtarget = (int) (LF_RB * inches * COUNTS_PER_INCH);
            RF_LBtarget = (int) (RF_LB * inches * COUNTS_PER_INCH);

            H.driveMotor[0].setTargetPosition(leftfrontStartPos + LF_RBtarget);
            H.driveMotor[1].setTargetPosition(rightfrontStartPos + RF_LBtarget);
            H.driveMotor[2].setTargetPosition(rightbackStartPos + LF_RBtarget);
            H.driveMotor[3].setTargetPosition(leftbackStartPos + RF_LBtarget);

            H.driveMotor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            H.driveMotor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            H.driveMotor[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            H.driveMotor[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

            H.driveMotor[0].setPower(LF_RB * speed);
            H.driveMotor[1].setPower(RF_LB * speed);
            H.driveMotor[2].setPower(LF_RB * speed);
            H.driveMotor[3].setPower(RF_LB * speed);

    }

    void setMoveInches(int Angle_Degrees, double inches, double speed, double agl_frwd) {

        this.Angle_Degrees = Angle_Degrees;
        this.inches = inches;
        this.speed = speed;
        this.agl_frwd = agl_frwd;

        first = false;

    }

    private void changeTargetInches(double inches) {

        double Angle = Math.toRadians(Angle_Degrees + 45);// - Math.PI / 4;
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

        LF_RBtarget = (int)(LF_RB * inches * COUNTS_PER_INCH);
        RF_LBtarget = (int)(RF_LB * inches * COUNTS_PER_INCH);

        H.driveMotor[0].  setTargetPosition(leftfrontStartPos + LF_RBtarget);
        H.driveMotor[1]. setTargetPosition(rightfrontStartPos + RF_LBtarget);
        H.driveMotor[2].  setTargetPosition(rightbackStartPos + LF_RBtarget);
        H.driveMotor[3].   setTargetPosition(leftbackStartPos + RF_LBtarget);

    }

}
