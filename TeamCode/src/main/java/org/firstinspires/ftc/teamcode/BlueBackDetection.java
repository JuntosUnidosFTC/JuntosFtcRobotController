/* Copyright (c) 2019 FIRST. All rights reserved.
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: Back Blue Diabolo", group = "Concept")
//@Disabled
public class BlueBackDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "Blue_Diabolo_Recognition.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Blue_Diabolo_Recognition.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue_Diabolo",
    };

    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");
        leftDriveBack = hardwareMap.get(DcMotor.class, "left_drive_back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD); //FORWARD
        rightDrive.setDirection(DcMotor.Direction.REVERSE); //REVERSE
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        initTfod();
        int position;

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            position = FindPixel();
            telemetry.addData("Position",position);
            visionPortal.stopStreaming();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "PropDetector"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int FindPixel() {
        // Camera settings
        tfod.setZoom(1.0); // Change this value to 2.0 for farther objects, default 1.0

        // add the pseudo code from the 15th here
        double conf = 0.0d;
        double x = 0.0d;
        double y = 0.0d;
        int position = 2;

        //Recognition myrecognition = null;
        runtime.reset();

        while (conf < 0.75 && opModeIsActive() && (runtime.seconds() <= 3.0)) {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
                conf = recognition.getConfidence();

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                sleep (500);      //originally 20 ms
                //myrecognition = recognition;
            } // end for() loop

            // Push telemetry to the Driver Station.
            telemetry.update();

        } // end while() loop
        if (runtime.seconds() > 3.0) {
            position = 2; // Right spike mark

            // Putting Pixel On Right Spike Mark (Base done)
            TurnRight(0.5,0.2);
            MoveForward(0.3,0.5);
            TurnRight(0.3,0.2);
            MoveForward(0.4,0.5);
            TurnRight(0.3,0.3);
            MoveForward(0.5,0.7);

            //Parking Backstage
            MoveBackward(0.4, 0.5);
            MoveLeft(0.5, 2);
            MoveForward(0.5, 2);
            MoveLeft(0.5, 10);

        }
        else {
            if (x <= 250)
            {
                position = 0; // Left spike mark
                TurnRight(0.4,0.4);
                MoveForward(0.2,1.0);
                TurnLeft(0.4,0.8);
                MoveForward(0.3,0.5);
            }
            else {
                position = 1; // Middle spike mark

                //Place Pixel On Middle Spike Mark (base done)
                TurnRight(0.5,0.2);
                MoveForward(0.4,2.3);

                //Park Backstage
                MoveBackward(0.3, 0.5);
                MoveRight(0.5, 1.3);
                MoveForward(0.3, 1);
                MoveLeft(0.7, 7);


            }
        }
        return(position);
    } // end method telemetryTfod()

    public void MoveForward(double speed, double time_in_seconds) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed);

        sleep(((long) (time_in_seconds * 475)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        sleep(((long)(time_in_seconds * 100)));
    }
    public void MoveBackward(double speed, double time_in_seconds) {
        leftDrive.setPower(speed*-1);
        rightDrive.setPower(speed*-1);
        leftDriveBack.setPower(speed*-1);
        rightDriveBack.setPower(speed*-1);

        sleep(((long) (time_in_seconds * 475)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        sleep(((long)(time_in_seconds * 100)));
    }

    public void MoveLeft(double speed, double time_in_seconds) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed*-1);
        leftDriveBack.setPower(speed*-1);
        rightDriveBack.setPower(speed);

        sleep(((long)(time_in_seconds * 400)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        sleep(((long)(time_in_seconds * 100)));

    }
    public void MoveRight(double speed, double time_in_seconds) {
        leftDrive.setPower(speed*-1);
        rightDrive.setPower(speed);
        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed*-1);

        sleep(((long) (time_in_seconds * 475)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        sleep(((long) (time_in_seconds * 475)));

    }

    public void TurnRight(double speed, double time_in_seconds) {
        leftDrive.setPower(speed*-1);
        rightDrive.setPower(speed);

        sleep(((long) (time_in_seconds * 1000)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        sleep(((long) (time_in_seconds * 1000)));
    }

    public void TurnLeft(double speed, double time_in_seconds) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed*-1);

        sleep(((long) (time_in_seconds * 1000)));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        sleep(((long) (time_in_seconds * 1000)));
    }

}   // end class
