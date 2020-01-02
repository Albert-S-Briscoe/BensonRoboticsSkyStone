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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**     put the following code inside runOpMode()

            ArmDriver arm = new ArmDriver(hardwareMap);

        if you are going to run anything on another thread then add this
        at the top of runOpMode()

            ExecutorService pool = Executors.newFixedThreadPool(2);

        to call a function use this format

            arm.[function name]([var 1], [var 2], ...);

        info for individual functions are included at the top of the function

 */

public class ArmDriver implements Runnable {

    public boolean moveDone = false;
    private boolean isToDeg;

    private final double zeroVolts = 0.38;
    private final double degreesPerVolt = 111;
    private double maxArmPower = 0.3;
    private double armTargetAngle;
    private double armAngle;
    private double armOffAngle;
    private double inches;

    RobotHardware H = new RobotHardware();

    void init(HardwareMap HM) {

        /**initializes RobotHardware
         * requires hardwaremap as input
         */
        H.init(HM);
    }

    public void run() {
        /**
         * to run a function use either
         *
         *    arm.setMoveToDeg(double, double);
         *
         *    or
         *
         *    arm.setMoveToHeight();
         *
         * then run this to start it
         *    pool.execute(arm);
         *
         */

        if (isToDeg) {

            MoveToDeg();

        } else {

            MoveToHeight();

        }

        moveDone = true;
    }

    void setMoveToDeg(double MoveToAngle, double maxArmPower) {

        this.armTargetAngle = Range.clip(MoveToAngle, 0, 135);
        this.maxArmPower = maxArmPower;

        isToDeg = true;
        moveDone = false;

    }

    private void MoveToDeg() {

         do {

             armAngle = (H.vertpos.getVoltage() - zeroVolts) * degreesPerVolt;
             armOffAngle = Math.abs(armAngle - armTargetAngle);

             if (armAngle > armTargetAngle) {

                 H.vertical.setPower(maxArmPower - Range.clip(1 / armOffAngle, 0, maxArmPower - 0.25 ));

             } else {

                 H.vertical.setPower(-maxArmPower + Range.clip(1 / armOffAngle, 0, maxArmPower - 0.25 ));

             }

        } while (armOffAngle > 2.5);

        H.vertical.setPower(0);

    }

    public void setMoveToHeight(double inches, double maxArmPower) {

        this.maxArmPower = maxArmPower;
        this.inches = Range.clip(inches, 0, 11.74);

        isToDeg = false;
        moveDone = false;
    }

    private void MoveToHeight() {

        final double armLength = 11.75;
        double armPos;

        do {

            armAngle = (H.vertpos.getVoltage() - zeroVolts) * degreesPerVolt;
            armPos = Math.sin(Math.toRadians(armAngle)) * armLength;
            armOffAngle = Math.abs(armAngle - Math.toDegrees(Math.asin(inches / armLength)));

            if (armPos > inches) {

                H.vertical.setPower(maxArmPower - Range.clip(1 / armOffAngle, 0, maxArmPower - 0.25));

            } else {

                H.vertical.setPower(-maxArmPower + Range.clip(1 / armOffAngle, 0, maxArmPower - 0.25));

            }

        } while (Math.abs(inches - armPos) > 0.075);

        H.vertical.setPower(0);

    }

}
