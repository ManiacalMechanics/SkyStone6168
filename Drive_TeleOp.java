package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOP", group="Main")
public class Drive_TeleOp extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorDriveLeftBack;
    private DcMotor motorDriveLeftFront;
    private DcMotor motorDriveRightBack;
    private DcMotor motorDriveRightFront;
    private DcMotor motorLift;
    private DcMotor Extender;
    public DcMotor plateMover;

    private Servo wideGrabber;
    private Servo capstone;

    public void coustomInit() {



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    @Override
    public void runOpMode()
    {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        coustomInit();



        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        motorDriveLeftBack = hardwareMap.get(DcMotor.class, "Backleft");
        motorDriveLeftFront = hardwareMap.get(DcMotor.class, "Frontleft");
        motorDriveRightBack = hardwareMap.get(DcMotor.class, "Backright");
        motorDriveRightFront = hardwareMap.get(DcMotor.class, "Frontright");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        Extender = hardwareMap.get(DcMotor.class, "Extender");
        plateMover = hardwareMap.get(DcMotor.class, "Plate Mover");


        wideGrabber = hardwareMap.get(Servo.class, "wideGrab");
        capstone = hardwareMap.get(Servo.class, "Capstone");


        wideGrabber.setPosition(1);
        capstone.setPosition(1);

        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        motorDriveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorDriveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorDriveRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorDriveRightFront.setDirection(DcMotor.Direction.REVERSE);

        motorDriveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDriveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plateMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Liftpower;

        waitForStart();

        boolean SlowMode = false;

        while (opModeIsActive())
        {
            //Driving Code


            if(gamepad1.y) {
                SlowMode = true;
                telemetry.addData("drive mode","Slow Mode");
                telemetry.update();
            }

            if(gamepad1.x)
            {
                SlowMode = false;
                telemetry.addData("drive mode","FastBoi");
                telemetry.update();
            }

            if (SlowMode) {
                Slowmode();
            }else {
                FastBoi();
            }






            //Lift Code
            if (gamepad2.left_stick_y > 0.1)
            {
                motorLift.setPower(0.4);
            } else
            {
                motorLift.setPower(gamepad2.left_stick_y);
            }

            //extender code
            Extender.setPower(gamepad2.right_stick_x);


            //wide grabber
            if (gamepad2.dpad_up)
            {
                wideGrabber.setPosition(1);
            } else if (gamepad2.dpad_down)
            {
                wideGrabber.setPosition(0.33);
            }

            //slight release
            if (gamepad2.dpad_left)
            {
                wideGrabber.setPosition(0.6);
            }

            if (gamepad2.a)
            {
                capstone.setPosition(1);
            }
            else if (gamepad2.y)
            {
                capstone.setPosition(0.3);
            }

            //Plate Mover
            if (gamepad2.x)
            {
                plateMover.setPower(0.4);
            } else if (gamepad2.b)
            {
                plateMover.setPower(-0.4);
            } else
            {
                plateMover.setPower(0);
            }
        }

    }
    public void setSplitPower ( double power){
        motorDriveLeftBack.setPower(power);
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(-power);
        motorDriveRightFront.setPower(-power);
    }

    public void strafe ( double power){
        motorDriveLeftFront.setPower(power);
        motorDriveRightBack.setPower(power);
        motorDriveRightFront.setPower(-power);
        motorDriveLeftBack.setPower(-power);
    }

//    public void setPower ( double power){
//        motorDriveLeftBack.setPower(-power);
//        motorDriveLeftFront.setPower(power);
//        motorDriveRightBack.setPower(-power);
//        motorDriveRightFront.setPower(power);
//
//    }

    public void Slowmode () {
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        motorDriveLeftBack.setPower(0.5*(secondaryDiagonalSpeed - rotation));
        motorDriveRightFront.setPower(0.5*(secondaryDiagonalSpeed + rotation));
        motorDriveLeftFront.setPower(0.5*(primaryDiagonalSpeed - rotation));
        motorDriveRightBack.setPower(0.5*(primaryDiagonalSpeed + rotation));
        telemetry.addData("Primary:" , primaryDiagonalSpeed);
        telemetry.addData("Secondary:", secondaryDiagonalSpeed);
        telemetry.addData("LeftRear:", motorDriveLeftBack.getPower());
        telemetry.addData("RightRear:", motorDriveRightBack.getPower());
        telemetry.update();
    }

    public void FastBoi() {
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        motorDriveLeftBack.setPower(secondaryDiagonalSpeed - rotation);
        motorDriveRightFront.setPower(secondaryDiagonalSpeed + rotation);
        motorDriveLeftFront.setPower(primaryDiagonalSpeed - rotation);
        motorDriveRightBack.setPower(primaryDiagonalSpeed + rotation);
        telemetry.addData("Primary:" , primaryDiagonalSpeed);
        telemetry.addData("Secondary:", secondaryDiagonalSpeed);
        telemetry.update();

    }

}