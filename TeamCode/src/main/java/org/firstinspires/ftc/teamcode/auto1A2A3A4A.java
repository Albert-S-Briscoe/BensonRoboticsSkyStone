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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Disabled
@Autonomous(name="blue_1A2A3A4B", group="Linear Opmode")
public class auto1A2A3A4A extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Skystone";
    //private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private ElapsedTime runtime = new ElapsedTime();

    //final float servoGear = 15/125;
    //final long degPerSec = (long)((60/.14) * servoGear);

    int mode = 1;

    int objleft;
    int objright;
    int objcenter;
    double offset;

    final long degPerSec = 30;

    //final double speed_slow = .35;
    final double speed_norm = .35;
    final double speed_fast = 1;
    final int field_side = 1;// -1 = red, 1 = blue

    //int inches_to_move;

    @Override
    public void runOpMode() {

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        MecanumWheelDriver drive = new MecanumWheelDriver();
        drive.init(hardwareMap);
        RobotHardware H = new RobotHardware();
        H.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        H.grab(false);

        /*while(!isStopRequested() && H.limit.getState()) {
            H.vertical.setPosition(0);
        }
        H.vertical.setPosition(1);
        sleep(110000 / degPerSec);//                           <-----------------
        H.vertical.setPosition(.5);*/

        while (!isStopRequested() && mode < 9) {

            switch (mode) {
                case 0: // Error
                    telemetry.addLine("There has been an Error!");
                    telemetry.addLine("Now it's time for you to spend very very large number of hours debugging");
                    telemetry.update();
                    while (opModeIsActive()) {
                        idle();
                    }
                case 1: // move forward to stones (1A)
                    telemetry.addData("mode = ", mode);
                    telemetry.update();
                        while (!isStopRequested() && (H.upperRange.getDistance(DistanceUnit.MM) > 355 || H.vertpos.getVoltage() > .5)) {
                            if (H.upperRange.getDistance(DistanceUnit.MM) > 355) {
                                drive.move(0, speed_norm, 0);
                            } else {
                                drive.stop();
                            }
                            if (H.vertpos.getVoltage() > .3) {
                                H.vertical.setPosition(1);
                            } else {
                                H.vertical.setPosition(.5);
                            }
                        }
                    H.vertical.setPosition(.5);
                    drive.stop();
                    mode = 2;
                    break;
                case 2: // looking for skystone
                    telemetry.addData("mode = ", mode);
                    offset = -500 * field_side;
                    while (!isStopRequested() && mode == 2 && runtime.seconds() < 6) {

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
                                mode = 3;
                        } else {
                            drive.move(90 * field_side, speed_norm * -Range.clip(offset / 30, -1, 1), 0);
                            telemetry.update();
                        }
                    }
                    mode = 3;
                    break;
                case 3: // pickup skystone
                    telemetry.addData("mode = ", mode);
                    telemetry.update();
                    H.grabber.setPosition(1);
                    while (!isStopRequested() && (H.upperRange.getDistance(DistanceUnit.MM) > 185 || H.vertpos.getVoltage() > .3)) {
                        if (!isStopRequested() && H.upperRange.getDistance(DistanceUnit.MM) > 185) {
                            drive.move(0, speed_norm, 0);
                        } else {
                            drive.stop();
                        }
                        if (!isStopRequested() && H.vertpos.getVoltage() > .3) {
                            H.vertical.setPosition(1);
                        } else {
                            H.vertical.setPosition(.5);
                        }
                    }
                    drive.stop();
                    H.vertical.setPosition(.5);
                    //lower servo a little
                    drive.RunWithEncoders(true);
                    drive.setMoveInches(0, 2, speed_norm, 0);
                    while (!isStopRequested() && H.vertpos.getVoltage() > .13) {
                        H.vertical.setPosition(1);
                    }
                    H.vertical.setPosition(.5);
                    H.grabber.setPosition(0);
                    sleep(750); //wait for servo to move
                    while (!isStopRequested() && H.vertpos.getVoltage() < .4) {
                        H.vertical.setPosition(0);
                    }
                    H.vertical.setPosition(.5);
                    /*H.vertical.setPosition(1);
                    sleep(10000 / degPerSec);//                                     <--------------
                    H.vertical.setPosition(.5);*/
                    //drive.stop();
                    //lower the servo all the way


                    /*H.vertical.setPosition(1);
                    sleep(27000 / degPerSec);//                                      <----------
                    H.vertical.setPosition(.5);
                    drive.MoveInches(0, 2, speed_norm, 0);
                    H.vertical.setPosition(1);
                    sleep(10000 / degPerSec);//                                      <----------
                    H.vertical.setPosition(.5);
                    H.grabber.setPosition(0);
                    sleep(500); //wait for servo to move
                    H.vertical.setPosition(0);
                    sleep(41000 / degPerSec);
                    H.vertical.setPosition(.5);*/
                    drive.setMoveInches(180, 1, speed_norm, 0);
                    mode = 4;
                    break;
                case 4: // navigate to foundation (2A)
                    drive.setrotate(90 * field_side, speed_norm, true);
                    /*inches_to_move = 96 - (int)H.lowerRange.getDistance(DistanceUnit.INCH);
                    if (inches_to_move < 0 || inches_to_move > 90) {
                        mode = 0;
                        break;
                    }*/
                    //drive.rotate(180, speed_norm);
                    drive.move(0, speed_fast, 0);
                    sleep(500);
                    while (H.upperRange.getDistance(DistanceUnit.INCH) > 28) {
                        drive.moveWithGyro(0, speed_fast, 90 * field_side);
                    }
                    drive.stop();
                    //drive.MoveInches(0, inches_to_move, speed_fast, 90);
                    mode = 5;
                    break;
                /*case 4: // navigate to foundation (2A)
                    while (H.sensorRange.getDistance(DistanceUnit.MM) > 750) {//  <--------------
                        drive.move(0, speed_slow, 0);
                        targetVisible = false;
                        for (VuforiaTrackable trackable : allTrackables) {
                            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                                telemetry.addData("Visible Target", trackable.getName());
                                targetVisible = true;

                                // getUpdatedRobotLocation() will return null if no new information is available since
                                // the last time that call was made, or if the trackable is not currently visible.
                                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                                if (robotLocationTransform != null) {
                                    lastLocation = robotLocationTransform;
                                }
                                break;
                            }
                        }

                        // Provide feedback as to where the robot is located (if we know).
                        if (targetVisible) {
                            // express position (translation) of robot in inches.
                            VectorF translation = lastLocation.getTranslation();
                            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                            // express the rotation of the robot in degrees.
                            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                                while (Math.abs(rotation.thirdAngle) > 15) {
                                    drive.move(0, 0, Range.clip(rotation.thirdAngle, -speed_slow, speed_slow));
                                }
                        }
                        else {
                            telemetry.addData("Visible Target", "none");
                        }
                        telemetry.update();
                    }
                    drive.stop();
                    mode = 5;*/
                case 5: // place skystone
                    drive.setrotate(0, speed_fast, true);
                    drive.setMoveInches(0, 8, speed_norm, 0);
                    /*H.vertical.setPosition(1);
                    sleep(25000 / degPerSec);
                    H.vertical.setPosition(.5);*/
                    H.grab(true);
                    H.grabber.setPosition(1);
                    sleep(750);
                    //drive.MoveInches(180, 4, speed_slow, -1);
                    /*H.vertical.setPosition(0);
                    sleep(25000  / degPerSec);
                    H.vertical.setPosition(.5);*/
                    mode = 7;
                    break;
                /*case 6: // position and grab foundation
                    drive.rotateToDeg(180, speed_fast);
                    inches_to_move = 1 + (int)H.lowerRange.getDistance(DistanceUnit.INCH);
                    drive.MoveInches(180, inches_to_move, speed_norm, 180);
                    H.grab(true);
                    sleep(500);
                    mode = 7;*/
                case 7: // move foundation (3A)
                    drive.move(90 * field_side, speed_fast, 0);
                    sleep(750);
                    drive.stop();
                    drive.setRampDown(1, 0.15);
                    drive.setrotate(75 * field_side, speed_fast, false);
                    drive.setRampDown(0, 0);
                    drive.move(0, speed_fast, 0);
                    sleep(1300);
                    drive.stop();
                    //drive.MoveInches(0, 12, speed_norm, -2);
                    H.grab(false);
                    mode = 8;
                    break;
                case 8: // park (4B)
                    //drive.MoveInches(-90 * field_side, 7, speed_norm, -2);
                    drive.move(-90 * field_side, speed_norm, 0);
                    sleep(600);
                    drive.setMoveInches(180, 40, speed_norm, 90 * field_side);
                    mode = 9;
                    break;
            }

        }

        H.vertical.setPosition(.5);
        H.leftfront.setPower(0);
        H.rightfront.setPower(0);
        H.leftback.setPower(0);
        H.rightback.setPower(0);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);//, LABEL_SECOND_ELEMENT);
    }
}
