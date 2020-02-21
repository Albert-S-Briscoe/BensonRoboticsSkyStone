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
import com.qualcomm.robotcore.util.Range;

/**     put the following code inside runOpMode()

            LinearArmDriver arm = new LinearArmDriver(H);
            ExecutorService pool = Executors.newFixedThreadPool(1);

        to call a function use this format

            arm.[function name]([var 1], [var 2] ...);

        info for individual functions are included at the top of the function

        In runOpMode() after all the game stuff, add this to shutdown the thread

            arm.stop();
            pool.shutdownNow();

 */

public class LinearArmDriver implements Runnable {


    private final double maxHeight = 29.25;
    private final double power = 1;

    public boolean moveDone = false;
    public boolean stop = false;
    public double inches = 0;
    int addTarget = 0;

    RobotHardware H;

    LinearArmDriver(RobotHardware H) {

        this.H = H;

    }

    public void run() {
        /**
         * run this at start:
         *
         *    pool.execute(arm);
         *
         * then use either
         *
         *    moveToInch(double);
         *
         *    or
         *
         *    moveToBlock(int);
         */

        final double COUNTS_PER_REVOLUTION = 1120; // maybe 2240 or 560
        final double WHEEL_DIAMETER_INCHES = 1.495;
        final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        int target;

        while (!stop) {

            addTarget = 3 * Range.clip((int)(inches * COUNTS_PER_INCH) - H.vertical.getCurrentPosition(), -33, 33);
            
            if ((inches * COUNTS_PER_INCH) + addTarget < 0) {
                
                addTarget *= 3;
                
            }
            
            target = (int)(inches * COUNTS_PER_INCH) + addTarget;
            H.vertical.setTargetPosition(target);
            H.vertical.setPower(power);

            moveDone = !H.vertical.isBusy();

        }
    }

    void moveToInch(double inch) {

        /**
         *  moves the arm to specified number of inches above the ground
         */

        this.inches = Range.clip(inch, 0, maxHeight);
        moveDone = false;
        stop = false;

    }

    void moveInches(double inches) {

        /**
         *  moves the arm a specified number of inches up or down
         */

        this.inches = Range.clip(this.inches + inches, 0, maxHeight);
        moveDone = false;
        stop = false;

    }

    void moveToBlock(int blocknum) {

        /**
         *  moves the arm to a specific block
         *  block 0 = ground
         *  block 1 = foundation
         *  block 2 = foundation + 1 block
         *  block n = foundation + (n - 1) blocks
         */

        inches = Range.clip((blocknum * 4) - 0.75, 0, maxHeight);
        moveDone = false;
        stop = false;

    }

    void zero() {

        /**
         *  sets the current position as zero
         */

        H.vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        H.vertical.setTargetPosition(0);
        H.vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void stop() {

        /**
         *  stops moving the arm
         *  use this to start again:
         *
         *     pool.execute(arm);
         */

        stop = true;
        inches = 0;

    }

    void pause() {

        /**
         *  temporarily stops the arm while keeping the current destination
         */

        H.vertical.setPower(0);

    }

    void play() {

        /**
         *  resumes arm movement after pause(); was used
         */

        H.vertical.setPower(power);

    }

}
