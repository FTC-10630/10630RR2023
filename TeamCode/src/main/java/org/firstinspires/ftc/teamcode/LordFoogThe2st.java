package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


// ALWAYS START TeleOp WITH SPOOLS AT THE LOWEST POSITION

/*
 * Controls (11/04/22):
 *
 * Gamepad 1:
 * Dpad Up - lift up,
 * Dpad Down - lift down
 * Y - grab / ungrab with claw
 * B - flip axel
 * A - flip wrist
 * *removed* X - wrist vertical position (to grab fallen cones)
 *
 * Gamepad 2:
 * left stick - ride + strafe
 * right stick - turn (x axis only)
 * Right Bumper - high speed
 * Left Bumper - low speed
 * Dpad - slow movement in 4 directions
 * *test* Left Trigger - slightly moves axel out
 * *test* Right Trigger - slightly moves axel in
 */

// Remember to comment everything

@TeleOp(name="LordFoogThe2st", group = "TeleOp")
public class LordFoogThe2st extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor spoolRight;
    private DcMotor spoolLeft;
    private Servo axel;
    private Servo wrist;
    private Servo claw;

    // Mutable variables
    private double speed;
    private double[] oldV; // old velocities
    private double liftLevel;

    // States
    private boolean isGrabbing;
    private boolean isAxelFront;
    private boolean isWristFront;
    private boolean ypressed;
    private boolean apressed;
    private boolean bpressed;
    private boolean xpressed;

    // Constants
    final double MAX_STICK_COORD = 1.0; // stick goes from -MAX_STICK_COORD to +MAX_STICK_COORD
    final double HIGH_SPEED = 2.0;
    final double LOW_SPEED = 0.7;
    final double TURN_SPEED = 1.0;
    final double A_LIMIT = 0.05; // acceleration per tick limit
    final double PRECISE_SPEED = 0.2;
    
    final double LIFT_POWER = 1.0; // TODO: try changing it somehow or adding acceleration limit to make lifting smoother
    final double LIFT_SPEED = 1.5;
    final int MAX_LIFT_LEVEL = 2735; // MAX value is ~38.5 in. * ~77 = ~2926 // every 1500 ticks is ~2.5 in ~77 tpi.
    final int MIN_LIFT_LEVEL = 4;
    final int START_LIFT_LEVEL = 0;

    final double FRONT_AXEL_POSITION = 0.83;
    final double BACK_AXEL_POSITION = 0.17;
    final double FRONT_WRIST_POSITION = 0.83;
    final double BACK_WRIST_POSITION = 0.17;
    final double VERTICAL_WRIST_POSITION = 0.5;
    final double GRAB_CLAW_POSITION = 0.55;
    final double UNGRAB_CLAW_POSITION = 0.42;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything
        initialize();

        // wait for the start button
        waitForStart();

        // main loop
        while (opModeIsActive()) {
            // TODO: make it work without this if-else
            if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) { 
                preciseset();
            }
            else {
                driveset();
            }

            liftset();
            telemetry.addData("Lift Level", (int) (liftLevel * 10) / 10.0);

            handset();

            telemetry.update();
        }
    }


    // Driver with gamepad 2 drives the robot
    // Robot can go in all four directions with the left stick and turn with the right stick
    private void driveset() {
        if (gamepad2.right_bumper) {
            speed = HIGH_SPEED;
        }
        if (gamepad2.left_bumper) {
            speed = LOW_SPEED;
        }
        
        double ls_y = -gamepad2.left_stick_y, ls_x = -gamepad2.left_stick_x; // -1 - 1
        double rs_x = -gamepad2.right_stick_x; // -1 - 1

        //                   move  strafe    turn
        double frontLeftV  = ls_x - ls_y + TURN_SPEED*rs_x;
        double frontRightV = ls_x + ls_y + TURN_SPEED*rs_x;
        double backLeftV   = ls_x + ls_y - TURN_SPEED*rs_x;
        double backRightV  = ls_x - ls_y - TURN_SPEED*rs_x;

        // shrinking speed's range from [-(2.0 + TURN_SPEED), +(2.0 + TURN_SPEED)] to [-1.0, +1.0]
        // then multiplying it by "speed" to make higher / lower
        frontLeftV  = frontLeftV  / ((2 + TURN_SPEED) * MAX_STICK_COORD) * speed;
        frontRightV = frontRightV / ((2 + TURN_SPEED) * MAX_STICK_COORD) * speed;
        backLeftV   = backLeftV   / ((2 + TURN_SPEED) * MAX_STICK_COORD) * speed;
        backRightV  = backRightV  / ((2 + TURN_SPEED) * MAX_STICK_COORD) * speed;

        double[] accelerationV = new double[4]; // acceleration of each motor
        accelerationV[0] = frontLeftV - oldV[0];
        accelerationV[1] = frontRightV - oldV[1];
        accelerationV[2] = backLeftV - oldV[2];
        accelerationV[3] = backRightV - oldV[3];

        // acceleration's absolute value can't be bigger than acceleration limit
        // a =             if a is positive   then               this               else                this
        accelerationV[0] = accelerationV[0] > 0 ? Math.min(A_LIMIT, accelerationV[0]) : Math.max(-A_LIMIT, accelerationV[0]);
        accelerationV[1] = accelerationV[1] > 0 ? Math.min(A_LIMIT, accelerationV[1]) : Math.max(-A_LIMIT, accelerationV[1]);
        accelerationV[2] = accelerationV[2] > 0 ? Math.min(A_LIMIT, accelerationV[2]) : Math.max(-A_LIMIT, accelerationV[2]);
        accelerationV[3] = accelerationV[3] > 0 ? Math.min(A_LIMIT, accelerationV[3]) : Math.max(-A_LIMIT, accelerationV[3]);

        // set new speed
        frontLeft.setPower(oldV[0] + accelerationV[0]);
        frontRight.setPower(oldV[1] + accelerationV[1]);
        backLeft.setPower(oldV[2] + accelerationV[2]);
        backRight.setPower(oldV[3] + accelerationV[3]);

        // save new speed
        oldV[0] += accelerationV[0];
        oldV[1] += accelerationV[1];
        oldV[2] += accelerationV[2];
        oldV[3] += accelerationV[3];
    }


    // Move with super low speed to precise the robot's position
    // (use dpad)
    private void preciseset() {
        // set speed
        if (gamepad2.dpad_up) { // move forward
            frontLeft.setPower(-PRECISE_SPEED);
            frontRight.setPower(PRECISE_SPEED);
            backLeft.setPower(PRECISE_SPEED);
            backRight.setPower(-PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_down) { // move backward
            frontLeft.setPower(PRECISE_SPEED);
            frontRight.setPower(-PRECISE_SPEED);
            backLeft.setPower(-PRECISE_SPEED);
            backRight.setPower(PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_left) { // strafe left
        frontLeft.setPower(PRECISE_SPEED);
            frontRight.setPower(PRECISE_SPEED);
            backLeft.setPower(PRECISE_SPEED);
            backRight.setPower(PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_right) { // strafe right
            frontLeft.setPower(-PRECISE_SPEED);
            frontRight.setPower(-PRECISE_SPEED);
            backLeft.setPower(-PRECISE_SPEED);
            backRight.setPower(-PRECISE_SPEED);
        }
    }


    // Code that controlls the spools
    // (use gamepad1 dpad)
    private void liftset() {
        /* 
        // old lift code (moves slower, but less smoothly)
        if (gamepad1.dpad_up && liftLevel < MAX_LIFT_LEVEL) {
            liftLevel += 3.0 * LIFT_SPEED;
        }
        if (gamepad1.dpad_down) {
            liftLevel =  Math.max(liftLevel - 2.0 * LIFT_SPEED, MIN_LIFT_LEVEL);
        }
        */
        
        // current spools position
        double currentAvg = (spoolLeft.getCurrentPosition() + spoolRight.getCurrentPosition()) / 2.0;
        //       if   button pressed  then    move    else   stay
        liftLevel = gamepad1.dpad_up   ? MAX_LIFT_LEVEL : currentAvg;
        liftLevel = gamepad1.dpad_down ? MIN_LIFT_LEVEL : liftLevel;
        
        //telemetry.addLine("level" + liftLevel);

        // set new position for spools
        spoolLeft.setTargetPosition((int) (liftLevel));
        spoolRight.setTargetPosition((int) (liftLevel));

    }


    // Code that controls 3 Servos
    // (use gamepad1 A, B, Y buttons)
    private void handset() {
        if (gamepad1.y && !ypressed) { // grab with claw
            if (isGrabbing) {
                claw.setPosition(UNGRAB_CLAW_POSITION);
            }
            else {
                claw.setPosition(GRAB_CLAW_POSITION);
            }
            isGrabbing = !isGrabbing;
        }
        ypressed = gamepad1.y;

        if (gamepad1.b && !bpressed) { // control axel
            if (isAxelFront) {
                axel.setPosition(BACK_AXEL_POSITION + gamepad1.right_trigger / 20.0 - gamepad1.left_trigger / 20.0);
            }
            else {
                axel.setPosition(FRONT_AXEL_POSITION - gamepad1.right_trigger / 20.0 + gamepad1.left_trigger / 20.0);
            }
            isAxelFront = !isAxelFront;
        }
        bpressed = gamepad1.b;

        if (gamepad1.a && !apressed) { // control wrist
            if (isWristFront) {
                wrist.setPosition(BACK_WRIST_POSITION);
            }
            else {
                wrist.setPosition(FRONT_WRIST_POSITION);
            }
            isWristFront = !isWristFront;
        }
        apressed = gamepad1.a;
    }


    // Initialization code
    // Robot should not yet move on intitalization
    public void initialize() {
        // set initial speed
        speed = HIGH_SPEED;
        oldV = new double[4]; // four zeros
        liftLevel = START_LIFT_LEVEL;

        // set initial hand state
        isGrabbing = true;
        isAxelFront = true;
        isWristFront = false;
        ypressed = false;
        apressed = false;
        bpressed = false;

        // initialize DCMotors
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        spoolRight = hardwareMap.get(DcMotor.class, "spoolr");
        spoolLeft = hardwareMap.get(DcMotor.class, "spooll");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spoolLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolLeft.setTargetPosition(START_LIFT_LEVEL);
        spoolLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolLeft.setPower(LIFT_POWER);
        //spoolRight.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolRight.setTargetPosition(START_LIFT_LEVEL);
        spoolRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spoolRight.setPower(LIFT_POWER);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // initialize Servos
        axel = hardwareMap.get(Servo.class, "axel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(GRAB_CLAW_POSITION);
        axel.setPosition(FRONT_AXEL_POSITION);
        wrist.setPosition(BACK_WRIST_POSITION);
    }
}
