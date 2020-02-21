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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Autonomous Blue", group="Linear Opmode")
public class autonomous extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Skystone";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private ElapsedTime         runtime  = new ElapsedTime();
    private RobotHardware       H        = new RobotHardware();
    private MecanumWheelDriver  drive    = new MecanumWheelDriver(H);
    private LinearArmDriver     arm      = new LinearArmDriver(H);
    private ExecutorService     pool     = Executors.newFixedThreadPool(2);

    private int mode = 1;

    private int objleft;
    private int objright;
    private int objcenter;
    private int offset;
    private int blockPos;

    //final double speed_slow = .35;
    final double sidewaysInches = 1.35;
    final double speed_norm = .6;
    final double speed_fast = 1;
    final double armPower = 0.3;
    final byte field_side = 1;   // -1 = red, 1 = blue
    final int maxOffsetForMiddelBlock = 175;
    final double reasonability = 2;
    
    
    private boolean found = false;
    private double inches_to_move;

    @Override
    public void runOpMode() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        //drive.init(hardwareMap);
        //arm.init(hardwareMap);
        H.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        arm.zero();
        pool.execute(arm);

        H.grab(false);
        arm.moveInches(4);
        H.block(false);
        drive.RunWithEncoders(true);
    
        if (opModeIsActive()) {
            telemetry.addData("Status", "loop started");
            telemetry.update();
            findSkystone();
        }
        if (opModeIsActive()) {
            _1A();
        }
        if (opModeIsActive()) {
            _2A();
        }
        if (opModeIsActive()) {
            placeSkystoneAndGrab();
        }
        if (opModeIsActive()) {
            _3A();
        }
        if (opModeIsActive()) {
            _4A();
        }

        arm.stop();
        drive.stop();
        pool.shutdownNow();
        H.leftfront.setPower(0);
        H.rightfront.setPower(0);
        H.leftback.setPower(0);
        H.rightback.setPower(0);
    }



    private void _1A() {
        
        drive.stop();
        
        drive.setrotate(0, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);

        arm.moveToInch(0);
        
        drive.setMoveInches(0,30, speed_fast, 0);
        pool.execute(drive);
        waitForMoveDone(0);
        
        /*while (!isStopRequested() && runtime.seconds() < 7 && !drive.moveDone && Math.abs(H.upperRange.getDistance(DistanceUnit.INCH) - (drive.H.leftback.getTargetPosition() - drive.H.leftback.getCurrentPosition()) / drive.COUNTS_PER_INCH) < reasonability) {
            
            drive.changeTargetInches(H.upperRange.getDistance(DistanceUnit.INCH) - 1, false);
            
        }

        /*while (!isStopRequested() && H.upperRange.getDistance(DistanceUnit.MM) > 650) {
            drive.move(0, speed_norm, 0);
            if (runtime.seconds() > 4.5) {
                drive.stop();
                while (!isStopRequested()) {
                    idle();
                }
            }
        }*/

    }

    private void _1B() {

    }

    private void _2A() {
        
        H.block(true);
        sleep(500);
        arm.moveToInch(6);
        drive.setMoveInches(180, 5, speed_fast, 0);
        pool.execute(drive);
        waitForMoveDone(0);

        drive.setrotate(90 * field_side, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);

        arm.moveToInch(0);

        drive.setMoveInches(0, 84 + 8 * blockPos, speed_fast, 90 * field_side);
        pool.execute(drive);
        waitForMoveDone(0);

    }

    private void _2B() {

    }

    private void _3A() {
        drive.move(-90 * field_side, speed_fast, 0);
        sleep(500);
        drive.stop();
        drive.setMoveInches(180, 18, speed_fast, 0);
        pool.execute(drive);
        waitForMoveDone(0);
        drive.setRampDown(25, 0.4);
        drive.setrotate(90 * field_side, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);
        drive.setMoveInches(0, 11, speed_fast, 90 * field_side);
        pool.execute(drive);
        drive.setRampDown(0, 0);
        sleep(1000);
        drive.stop = true;
        waitForMoveDone(0);

        /*drive.move(0, speed_fast, 0);
        sleep(500);
        drive.stop();*/

        H.grab(false);



        //drive.MoveInches(0, 12, speed_norm, -2);
    }

    private void _4A() {
        drive.move(180, speed_fast, 0);
        sleep(150);
        drive.stop();
        drive.move(-90, speed_fast, 0);
        sleep(300);
        drive.stop();
        drive.setrotate(90 * field_side, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);
        drive.setMoveInches(180, 40, speed_fast, -1);
        pool.execute(drive);
        sleep(500);
        arm.moveToInch(0);
        waitForMoveDone(2);
    }

    private void _4B() {

    }

    private void _5A() {

    }

    private void _5B() {

    }

    private void _6A() {

    }

    private void _6B() {

    }

    private void _7A() {

    }

    private void _7B() {

    }

    private void findSkystone() {
        while (!isStopRequested() && runtime.seconds() < 6 && !found) {
            if (tfod != null) {
                telemetry.addData("TFmode = ", mode);
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            objleft = (int) recognition.getLeft();
                            objright = (int) recognition.getRight();
                            objcenter = (objleft + objright) / 2;
                            offset = objcenter - 640;
                            if (objleft - objright < 250) {
                                if (Math.abs(offset) < maxOffsetForMiddelBlock) {
                                    blockPos = 0;
                                } else {
                                    blockPos = (int)Math.signum(offset);
                                }
                                found = true;
                            }
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                }
            }
        }
        if (!isStopRequested() && blockPos != 0) {
            drive.setMoveInches(-90 * blockPos, 8.5 * sidewaysInches, speed_fast, 0);
            pool.execute(drive);
            sleep(750);
        }
    }

    /*private void findSkystone() {
        offset = -500 * field_side;
        while (!isStopRequested() && !found && runtime.seconds() < 8) {

            if (tfod != null) {
                telemetry.addData("TFmode = ", mode);
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Skystone") {
                            objright = (int) recognition.getTop();
                            objleft = (int) recognition.getBottom();
                            objcenter = (objleft + objright) / 2;
                            offset = objcenter - 640;
                            break;
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                }
            }
            if (Math.abs(offset) < 25) {
                drive.stop();
                found = true;
            } else {
                drive.move(-90, speed_norm * -Range.clip(offset / 30, -1, 1), 0);

            }
            telemetry.update();
        }
    }*/

    private void pickUpSkystone() {
        H.grabber.setPosition(1);
        /*while (!isStopRequested() && (H.upperRange.getDistance(DistanceUnit.MM) < 420 || H.vertpos.getVoltage() > .4)) {
            if (!isStopRequested() && H.upperRange.getDistance(DistanceUnit.MM) < 420) {
                drive.move(180, speed_norm, 0);
            } else {
                drive.stop();
            }
            if (!isStopRequested() && H.vertpos.getVoltage() > .4) {
                H.Vertical.setPosition(1);
            } else {
                H.Vertical.setPosition(.5);
            }
        }*/
        //drive.setMoveInches(180, 2, speed_norm, 0);
        //pool.execute(drive);
        if (!drive.moveDone) {
            sleep(1250);
        }
        drive.stop = true;
        waitForMoveDone(0);
        //arm.setMoveToDeg(5);
        drive.setrotate(0, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);
        drive.setMoveInches(0, 7.5, speed_norm, 0);
        pool.execute(drive);
        waitForMoveDone(2);
        //arm.setMoveToDeg(2);
        waitForMoveDone(1);
        H.grabber.setPosition(0);
        sleep(750); // wait for servo to move
        //arm.setMoveToDeg(35);
        drive.setMoveInches(0, 6, speed_norm, 0);
        pool.execute(drive);
        waitForMoveDone(2);
    }

    private void placeSkystoneAndGrab() {
    
        arm.moveToInch(3);
        drive.setrotate(0, speed_fast, true);
        pool.execute(drive);
        waitForMoveDone(0);
        inches_to_move = H.lowerRange.getDistance(DistanceUnit.INCH) + .5;
        drive.setMoveInches(0, inches_to_move, speed_fast, 0);
        pool.execute(drive);
        arm.moveToInch(3);
        waitForMoveDone(0);
        H.block(false);
        H.grab(true);
        sleep(500);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.72;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);//, LABEL_SECOND_ELEMENT);
    }

    private void waitForMoveDone(int moveDriver) {

        // 0 = MecanumWheelDriver
        // 1 = ArmDriver
        // 2 = both

        switch (moveDriver) {
            case 0:

                while (!isStopRequested() && !drive.moveDone) {

                    idle();

                }
                break;

            case 1:

                while (!isStopRequested() && !arm.moveDone) {

                    idle();

                }
                break;

            case 2:

                while (!isStopRequested() && (!drive.moveDone || !arm.moveDone)) {

                    idle();

                }
                break;

        }

    }

}