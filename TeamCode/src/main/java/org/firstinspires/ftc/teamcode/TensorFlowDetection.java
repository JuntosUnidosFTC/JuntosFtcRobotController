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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

/*
* THE CAMERA IS CALLED PropDetector
 */
@TeleOp(name = "Concept: TensorFlow Object Detection Easy", group = "Concept")
// @Disabled
public class TensorFlowDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

        /* Declare OpMode members. */
        private DcMotor leftDrive   = null;
        private DcMotor rightDrive  = null;
        private DcMotor rightDriveBack = null;
        private DcMotor leftDriveBack = null;

        private ElapsedTime runtime = new ElapsedTime();

        Teleop practiceRobot = new Teleop(this);

        static final double     FORWARD_SPEED = 0.6;
        static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");
        leftDriveBack = hardwareMap.get(DcMotor.class, "left_drive_back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        Recognition myrecognition = null;

        initTfod();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                FindPixel();

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
        }

    /*    while (opModeIsActive()) {        //normally an if
            sleep (1000);
            telemetry.addData("Ready","Findpixel");
            telemetry.update();
            myrecognition = FindPixel();

        }   */

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "PropDetector"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private Recognition FindPixel() {

        // add the pseudo code from the 15th here
        double conf = 0.0d;
        Recognition myrecognition = null;

        while (conf < 75.0 && opModeIsActive()) {
            
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                conf = recognition.getConfidence() * 100;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.update();
                sleep (500);      //originally 20 ms
                myrecognition = recognition;
            } // end for() loop

            // Push telemetry to the Driver Station.
            telemetry.update();
            
        } // end while() loop
     return(myrecognition);
    }
    // end method telemetryTfod()
    public void MoveForward(double speed, int time_in_seconds) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed);

        sleep((time_in_seconds * 475));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        sleep((time_in_seconds * 100));
    }
    public void MoveLeft(double LFspeed, double RFspeed, double LBspeed, double RBspeed, int time_in_seconds) {
        leftDrive.setPower(LFspeed);
        rightDrive.setPower(RFspeed);
        leftDriveBack.setPower(LBspeed);
        rightDriveBack.setPower(RBspeed);

        sleep((time_in_seconds * 400));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        sleep((time_in_seconds * 100));
    }
    public void MoveRight(double LFspeed, double RFspeed, double LBspeed, double RBspeed, int time_in_seconds) {
        leftDrive.setPower(LFspeed);
        rightDrive.setPower(RFspeed);
        leftDriveBack.setPower(LBspeed);
        rightDriveBack.setPower(RBspeed);

        sleep((time_in_seconds * 70));
        telemetry.addData("Reached and passed time", "yes");
        telemetry.update();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        sleep((time_in_seconds * 100));
    }
}   // end class
