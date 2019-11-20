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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**     put the following code inside runOpMode()

            MecanumWheelDriver drive = new MecanumWheelDriver();
            drive.init(hardwareMap);

        if you are going to run using encoders then include this as well

            drive.initEncoder();
 */

public class MecanumWheelDriver {

    final double COUNTS_PER_REVOLUTION = 288;
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double ROBOT_DIAMETER_INCHES = 22.5;
    final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * 3.14159);
    final double COUNTS_PER_DEGREE = ((ROBOT_DIAMETER_INCHES * 3.14159) / 360) * COUNTS_PER_INCH;

    public DcMotor leftfront = null;
    public DcMotor rightfront = null;
    public DcMotor leftback = null;
    public DcMotor rightback = null;
    ElapsedTime runtime = new ElapsedTime();

    public void move(double Angle_Degrees, double Radius, double Rotate) {

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        double rTotal = Radius + Math.abs(Rotate);
        double Angle = Math.toRadians(Angle_Degrees - 45);
        double cosAngle = Math.cos(Angle);
        double sinAngle = -Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;

            if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
                multiplier = 1/Math.abs(cosAngle);
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            } else {
                multiplier = 1/Math.abs(sinAngle);
                LF_RB = multiplier * cosAngle;
                RF_LB = multiplier * sinAngle;
            }

            leftfrontPower    = LF_RB * Radius + Rotate;
            rightfrontPower   = RF_LB * Radius - Rotate;
            leftbackPower     = RF_LB * Radius + Rotate;
            rightbackPower    = LF_RB * Radius - Rotate;

            if (Math.abs(rTotal) > 1) {
                leftfrontPower    = leftfrontPower/rTotal;
                rightfrontPower   = rightfrontPower/rTotal;
                leftbackPower     = leftbackPower/rTotal;
                rightbackPower    = rightbackPower/rTotal;
            }

            leftfront.setPower(leftfrontPower);
            rightfront.setPower(rightfrontPower);
            leftback.setPower(leftbackPower);
            rightback.setPower(rightbackPower);
    }

    public void stop() {
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
    }

    public void moveInches(int Angle_Degrees, int inches, double speed) {

        double Angle = Math.toRadians(Angle_Degrees - 45);// - Math.PI / 4;
        double cosAngle = Math.cos(Angle);
        double sinAngle = -Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1 / Math.abs(cosAngle);//then add the rotate speed
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        } else {
            multiplier = 1 / Math.abs(sinAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        }

        int LF_RBtarget = (int)(LF_RB * inches * COUNTS_PER_INCH);
        int RF_LBtarget = (int)(RF_LB * inches * COUNTS_PER_INCH);

        leftfront.setTargetPosition(leftfront.getCurrentPosition() + LF_RBtarget);
        rightfront.setTargetPosition(rightfront.getCurrentPosition() + RF_LBtarget);
        leftback.setTargetPosition(leftback.getCurrentPosition() + RF_LBtarget);
        rightback.setTargetPosition(rightback.getCurrentPosition() + LF_RBtarget);

        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfront.setPower(LF_RB * speed);
        rightfront.setPower(RF_LB * speed);
        leftback.setPower(RF_LB * speed);
        rightback.setPower(LF_RB * speed);

        while (leftfront.isBusy() && rightfront.isBusy() && leftback.isBusy() && rightback.isBusy()) {
        }

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotate(int Degrees, double speed) {

        int Lefttarget = (int)(Degrees * COUNTS_PER_DEGREE);
        int Righttarget = -((int)(Degrees * COUNTS_PER_DEGREE));

        leftfront.setTargetPosition(leftfront.getCurrentPosition() + Lefttarget);
        rightfront.setTargetPosition(rightfront.getCurrentPosition() + Righttarget);
        leftback.setTargetPosition(leftback.getCurrentPosition() + Lefttarget);
        rightback.setTargetPosition(rightback.getCurrentPosition() + Righttarget);

        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfront.setPower(speed);
        rightfront.setPower(-speed);
        leftback.setPower(speed);
        rightback.setPower(-speed);

        while (leftfront.isBusy() && rightfront.isBusy() && leftback.isBusy() && rightback.isBusy()) {
            //idle();
        }

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(HardwareMap HW) {
        leftfront  = HW.get(DcMotor.class, "LF_drive");
        rightfront = HW.get(DcMotor.class, "RF_drive");
        leftback  = HW.get(DcMotor.class, "LB_drive");
        rightback = HW.get(DcMotor.class, "RB_drive");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void RunWithEncoders(boolean On) {
        if (On) {
            leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
