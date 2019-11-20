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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="1A2A3A4B", group="Linear Opmode")
public class auto1A2A3A4B extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private TFObjectDetector tfod;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private ElapsedTime runtime = new ElapsedTime();

    //final float servoGear = 15/125;//                                                  <<--------------------------
    //final long degPerSec = (long)((60/.14) * servoGear);

    @Override
    public void runOpMode() {

        int mode = 1;

        int objleft;
        int objright;
        int objcenter;
        int offset;

        final long degPerSec = 30;

        final double speed_slow = .2;
        final double speed_norm = .35;
        final double speed_fast = .75;
        final int field_side = 1;// -1 = red, 1 = blue

        int inches_to_move;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

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

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        waitForStart();
        runtime.reset();

        while(H.limit.getState()) {
            H.vertical.setPosition(0);
        }
        H.vertical.setPosition(1);
        sleep(110000 / degPerSec);//                           <-----------------
        H.vertical.setPosition(.5);

        while (opModeIsActive() && mode < 9) {

            switch (mode) {
                case 0: // Error
                    telemetry.addLine("There has been an Error!");
                    telemetry.addLine("Now it's time for you to spend n hours debugging");
                    telemetry.addLine("where n is a very very large number");
                    telemetry.update();
                    while (opModeIsActive()) {
                        idle();
                    }
                case 1: // move forward to stones (1A)
                    telemetry.addData("mode = ", mode);
                    telemetry.update();
                    while (H.sensorRange.getDistance(DistanceUnit.MM) > 355) {
                    drive.move(0, speed_norm, 0);
                    }
                    drive.stop();
                    mode = 2;
                    break;
                case 2: // looking for skystone
                    telemetry.addData("mode = ", mode);
                    targetsSkyStone.activate();
                    offset = 500 * field_side;
                    while (!isStopRequested() && mode == 2 && runtime.seconds() < 10) {

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
                                    }
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                }
                            }
                        }
                        if (Math.abs(offset) < 80) {
                            drive.stop();
                            mode = 3;
                        }
                        drive.move(90, speed_slow * -Range.clip(offset, -1, 1), 0);
                        telemetry.update();
                    }
                    mode = 3;
                    break;
                case 3: // pickup skystone
                    telemetry.addData("mode = ", mode);
                    telemetry.update();
                    H.grabber.setPosition(1);
                    while (H.sensorRange.getDistance(DistanceUnit.MM) > 135) {//   <----------------
                        drive.move(0, speed_norm, 0);
                    }
                    drive.stop();
                    //lower servo a little
                    drive.RunWithEncoders(true);
                    /*H.vertical.setPosition(1);
                    sleep(10000 / degPerSec);//                                     <--------------
                    H.vertical.setPosition(.5);*/
                    drive.moveInches(0, 1, speed_norm);
                    drive.stop();
                    //lower the servo all the way
                    H.vertical.setPosition(1);
                    sleep(35000 / degPerSec);//                                      <----------
                    H.vertical.setPosition(.5);
                    H.grabber.setPosition(0);
                    sleep(500); //wait for servo to move
                    H.vertical.setPosition(0);
                    sleep(75000 / degPerSec);
                    H.vertical.setPosition(.5);
                    drive.moveInches(180, 10, speed_norm);
                    mode = 4;
                    break;
                case 4:
                    drive.rotate(90, speed_fast * field_side);
                    inches_to_move = 84 - ((int) H.sensorRange.getDistance(DistanceUnit.INCH));
                    //drive.rotate(180, speed_norm);
                    drive.moveInches(180, inches_to_move, speed_fast);
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
                    drive.rotate(90, -speed_norm * field_side);
                    H.vertical.setPosition(1);
                    sleep(25000  / degPerSec);//                                            <-----------------
                    H.vertical.setPosition(.5);
                    H.grabber.setPosition(1);
                    drive.moveInches(180, 4, speed_slow);
                    H.vertical.setPosition(0);
                    sleep(25000  / degPerSec);//                                            <-----------------
                    H.vertical.setPosition(.5);
                    mode = 8;
                    break;
                /*case 6: // position and grab foundation
                    drive.move(0, -speed_slow, 0);
                    sleep(500);                                     //-----------
                    drive.move(0, 0, speed_slow);
                    sleep(2000);
                    drive.move(0, -speed_slow, 0);
                    sleep(500);                                     //-----------
                    drive.stop();
                    // grab base                                  <-----------------------------------
                    mode = 7;
                case 7: // move foundation (3A)
                    while (H.sensorRange.getDistance(DistanceUnit.MM) > 355) {//    <------------
                        drive.move(0, speed_norm, 0);
                    }
                    drive.stop();
                    mode = 8;*/
                case 8: // park (4B)
                    drive.rotate(90, -speed_fast * field_side);
                    inches_to_move = 64 - ((int) H.sensorRange.getDistance(DistanceUnit.INCH));
                    drive.moveInches(180, inches_to_move, speed_norm);
                    drive.stop();
                    mode = 9;
                    break;
            }

        }

        targetsSkyStone.deactivate();
        H.vertical.setPosition(.5);
        drive.leftfront.setPower(0);
        drive.rightfront.setPower(0);
        drive.leftback.setPower(0);
        drive.rightback.setPower(0);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
