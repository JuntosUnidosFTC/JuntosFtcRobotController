//test

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
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="13727", group="Linear Opmode")
//@Disabled
public class OpMode13727 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Motors F = Front B = Back
    Teleop practiceRobot = new Teleop(this);
    //Encoders13727 practiceRobot = new Encoders13727(this);
    private Servo BoxServo = null;

    private Servo droneServo = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        practiceRobot.init();

        BoxServo = hardwareMap.get(Servo.class, "box_servo");
        droneServo = hardwareMap.get(Servo.class, "drone_servo");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double driveY = -gamepad1.left_stick_y; //Y Direction
            double driveX = gamepad1.left_stick_x; //X Direction
            double turn  =  gamepad1.right_stick_x; //Turn

            //Wheels
            double LFWheel = Range.clip(driveY - driveX - turn, -1.0, 1.0);
            double RFWheel = Range.clip(driveY + driveX + turn, -1.0, 1.0);
            double LBWheel = Range.clip(driveY + driveX - turn, -1.0, 1.0);
            double RBWheel = Range.clip(driveY - driveX + turn, -1.0, 1.0);

            //double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            practiceRobot.Move(LFWheel, RFWheel, LBWheel, RBWheel);

            // This section is what controls the intake, as well as reversing it and stopping it.
            if(gamepad1.right_bumper)
            {
                practiceRobot.Intake(0.85);
            } else if (gamepad1.left_bumper) {
                practiceRobot.Outtake(0.75);
            } else {
                practiceRobot.IntakeStop();
            }

            //This section is what controls the slider movement, as well as reversing it and stopping it.

            //This section is what controls the box servo
            if (gamepad1.b) {
                practiceRobot.OpenBox();
                telemetry.addData("Servo","MiddleBox");
                telemetry.addData("ServoPosition", BoxServo.getPosition());
                telemetry.update();
            } else {
                practiceRobot.CloseBox();
                telemetry.addData("Servo","MiddleBox");
                telemetry.addData("ServoPosition", BoxServo.getPosition());
                telemetry.update();
            }

            if(gamepad1.y)
            {
                practiceRobot.SlideUpStart();
            } else if (gamepad1.x) {
                practiceRobot.SlideDownStart();
            } else {
                practiceRobot.SlideStop();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}