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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="auto center", group="Linear Opmode")
//@Disabled
public class autocenter extends LinearOpMode {


    private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Skystone";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    int mode = 2;

    int objleft;
    int objright;
    int objcenter;
    int offset;
    private int blockPos;

    final long degPerSec = 30;
    final double sidewaysInches = 1.19;
    final int maxOffsetForMiddelBlock = 50;

    private boolean found = false;

    //final double speed_slow = .35;
    final double speed_norm = .65;
    final double speed_fast = 1;
    final int field_side = 1;// -1 = red, 1 = blue

    @Override
    public void runOpMode() {

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
        ExecutorService pool = Executors.newFixedThreadPool(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ////////////////////////////////////////////////////////////////////////////////////////////

        /*offset = -500 * field_side;
        while (opModeIsActive() && mode == 2) {

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
                            telemetry.addData("label" , recognition.getLabel());
                            telemetry.addData("left, top", "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData("right, bottom", "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            break;
                        }
                    }
                }
            }
            if (Math.abs(offset) < 25) {
                drive.stop();
                mode = 3;
            } else {
                drive.move(90 * field_side, speed_norm * -Range.clip(offset / 200, -1, 1), 0);
                telemetry.addData("offset: ", offset);
                telemetry.addData("power: ", speed_norm * -Range.clip(offset / 200, -1, 1));
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }*/

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
                            objright = (int) recognition.getTop();
                            objleft = (int) recognition.getBottom();
                            objcenter = (objleft + objright) / 2;
                            offset = objcenter - 640;
                            if (objright - objleft < 500) {
                                if (Math.abs(offset) < maxOffsetForMiddelBlock) {
                                    blockPos = 0;
                                } else {
                                    blockPos = Range.clip(offset, -1, 1);
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
            drive.setMoveInches(90 * blockPos, 8 * sidewaysInches, speed_norm, 0);
            pool.execute(drive);
        }

        telemetry.addData("block pos", blockPos);
        telemetry.update();
        while (!drive.moveDone && !isStopRequested()) {
            idle();
        }
    }
        ////////////////////////////////////////////////////////////////////////////////////////////

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);//, LABEL_SECOND_ELEMENT);
    }
}
