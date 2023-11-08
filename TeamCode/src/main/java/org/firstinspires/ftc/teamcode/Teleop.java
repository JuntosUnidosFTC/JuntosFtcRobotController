package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Teleop {
    private DcMotor leftDriveF = null; //left_drive_front
    private DcMotor leftDriveB = null; //left_drive_back
    private DcMotor rightDriveF = null; //right_drive_front
    private DcMotor rightDriveB = null; //right_drive_back
    private DcMotor intakeMotor = null; //intake_motor

    private Servo boxServo = null; //gate_servo
    private LinearOpMode CurrentOpMode = null;
    public Teleop(LinearOpMode opMode) {
        CurrentOpMode = opMode;
    }
    public void init()
    {
        leftDriveF = CurrentOpMode.hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveB = CurrentOpMode.hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveF = CurrentOpMode.hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveB = CurrentOpMode.hardwareMap.get(DcMotor.class, "right_drive_back");
        intakeMotor = CurrentOpMode.hardwareMap.get(DcMotor.class, "intake_motor");
        boxServo = CurrentOpMode.hardwareMap.get(Servo.class, "box_servo");

        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Move(double LFPower, double RFPower, double LBPower, double RBPower)
    {
        leftDriveF.setPower(LFPower/1.5);
        leftDriveB.setPower(LBPower/1.5);
        rightDriveF.setPower(RFPower/1.5);
        rightDriveB.setPower(RBPower/1.5);
    }

    public void Stop()
    {
        leftDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveF.setPower(0);
        rightDriveB.setPower(0);
    }
    public void Intake(double Power)
    {
        intakeMotor.setPower(Power);
    }
    public void Outtake(double Power)
    {
        intakeMotor.setPower(-Power);
    }
    public void IntakeStop()
    {
        intakeMotor.setPower(0);
    }

    public void RotateServo(double position)
    {
        boxServo.setPosition(position);
    }
}