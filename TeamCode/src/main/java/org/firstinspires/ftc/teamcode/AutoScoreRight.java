package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous(name="AutoScoreRight", group="Robot")
public class AutoScoreRight extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor spoolRight;
    private DcMotor spoolLeft;
    private Servo axel;
    private Servo wrist;
    private Servo claw;
    private OpenCvWebcam webcam;
    private ConeAnalysingPipeline pipeline;

    // Mutable variables
    private int webcamResponce; 
    private final String[] Colors = {"Red", "Green", "Blue", "Error"}; // possible colors

    // States
    private boolean isAxelFront;
    private boolean isWristFront;

    // Constants
    private final double ROBOT_LENGTH = 17.0; // inches
    private final double ROBOT_WIDTH = 15.0; // inches
    private final double COUNTS_PER_MOTOR_REV = 537.68984; // goBILDA 5202 Motor, 312 RMP
    private final double DRIVE_GEAR_REDUCTION = 1.0; // no external gearing
    private final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm Mecanum Wheel
    private final double COUNTS_PER_INCH = 1.09 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private final double TURN90_INCHES = Math.PI * (Math.sqrt(ROBOT_LENGTH * ROBOT_LENGTH + ROBOT_WIDTH * ROBOT_WIDTH) / 2.0) / 2.0; // sqrt(514) / 2 - radius of the rotot's circle
    private final double TILE = 23.5; // tile length (inches)
    private final double HIGH_SPEED = 0.7;
    private final double LOW_SPEED = 0.5;

    private final double LIFT_POWER = 1.5;
    private final int MAX_LIFT_LEVEL = 2690; // was 2735
    private final int MIN_LIFT_LEVEL = 30;
    private final int START_LIFT_LEVEL = 0;

    private final double FRONT_AXEL_POSITION = 0.85;
    private final double BACK_AXEL_POSITION = 0.13;
    private final double FRONT_WRIST_POSITION = 0.83; 
    private final double BACK_WRIST_POSITION = 0.17;
    private final double GRAB_CLAW_POSITION = 0.55; 
    private final double UNGRAB_CLAW_POSITION = 0.42; 


    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything
        initialize();

        // works until you hit Play on the Driver Hub
        while (!isStarted()) {
            // Possible responses: 0 - Red, 1 - Green, 2 - Blue, 3 - Error
            webcamResponce = pipeline.getAnalysis();
            
            telemetry.addData("Avg", pipeline.getAvg());
            telemetry.addData("Color", Colors[webcamResponce]);
            telemetry.update();
        }

        telemetry.addData("Result", Colors[webcamResponce]);
        telemetry.update();
 
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Example: moveSomewhere(double speed, double inches);

        /*=-- MOVE CODE SEGMENT START --=*/
        lift(200);
        sleep(600);
        moveForward(LOW_SPEED, TILE * 2 + 1);
        strafeLeft(LOW_SPEED, 12);
        axelFlip();
        lift(MAX_LIFT_LEVEL);
        sleep(1500);
        moveForward(LOW_SPEED, 4.6);
        sleep(100);
        axelFlip();
        sleep(1500);
        ungrab();
        sleep(1000);
        moveBackward(LOW_SPEED, 4.6 + 0.5);

        // RED -- RIGHT ZONE
        if (webcamResponce == 0) {
            strafeRight(LOW_SPEED, TILE + 12 + 1);
        }
        // GREEN -- CENTRE ZONE
        else if (webcamResponce == 1) {
            strafeRight(LOW_SPEED, 12 + 0.5);
        }
        // BLUE -- LEFT ZONE
        else if (webcamResponce == 2) {
            strafeLeft(LOW_SPEED, TILE - 12 - 1);
        }
        // ERROR
        else if (webcamResponce == 3) {
            telemetry.addData("COLOR", "ERROR");
            strafeLeft(LOW_SPEED, 12);
        }
        
        lift(0);
        sleep(1500);
        /*=-- MOVE CODE SEGMENT END --=*/

        telemetry.addData("Auto", "Complete");
        telemetry.update();

        sleep(500); // pause to display final telemetry message.
    }


    // Move all motors forward
    private void moveForward(double speed, double inches) {
        encoderDrive(speed, inches, inches, inches, inches, 4.0);
    }

    // Move all motors backward
    private void moveBackward(double speed, double inches) {
        encoderDrive(speed, -inches, -inches, -inches, -inches, 4.0);
    }

    // Turn left by setting frontleft and backleft backward and frontright and backright to forward
    private void turnLeft(double speed, double inches) {
        encoderDrive(speed, -inches, inches, -inches, inches, 4.0);
    }

    // Turn right by setting frontleft and backleft forward and frontright and backright to backward
    private void turnRight(double speed, double inches) {
        encoderDrive(speed, inches, -inches, inches, -inches, 4.0);
    }

    // Strafe left by setting frontleft and backright to backward and frontright and backleft to forward
    private void strafeLeft(double speed, double inches) {
        encoderDrive(speed, -inches, inches, inches, -inches, 4.0);
    }

    // Strafe left by setting frontleft and backright to backward and frontright and backleft to forward
    private void strafeRight(double speed, double inches) {
        encoderDrive(speed, inches, -inches, -inches, inches, 4.0);
    }


    // Set power to all motors
    private void motorsSetPower(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }


    // Returns true if all motors are busy (rinning), else - false
    private boolean allMotorsAreBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    // Returns true if at least one motor is busy (rinning), else - false
    private boolean oneMotorIsBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeoutSec) {
        // extra check that the robot is still running
        if (!opModeIsActive()) return;

        // set new target positions
        int frontLeftTarget  = frontLeft.getCurrentPosition()  + (int) (frontLeftInches * COUNTS_PER_INCH);
        int frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
        int backLeftTarget   = backLeft.getCurrentPosition()   + (int) (backLeftInches * COUNTS_PER_INCH);
        int backRightTarget  = backRight.getCurrentPosition()  + (int) (backRightInches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        // turn on RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motorsSetPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // allMotorsAreBusy() - when at least one motor hits its target position, it stops (its safer)
        // oneMotorIsBusy() - stops only when all at the target position (try it if something isn't working with &&) 
        while (opModeIsActive() && (runtime.seconds() < timeoutSec) && oneMotorIsBusy()) {
            // display positions
            telemetry.addLine("Target position:");
            telemetry.addLine("Front  L: " + frontLeftTarget + "  R: " + frontRightTarget);
            telemetry.addLine("Back   L: " + backLeftTarget + "  R: " + backRightTarget);
            
            telemetry.addLine("Current position:");
            telemetry.addLine("Front  L: " + frontLeft.getCurrentPosition() + "  R: " + frontRight.getCurrentPosition());
            telemetry.addLine("Back   L: " + backLeft.getCurrentPosition() + "  R: " + backRight.getCurrentPosition());
            
            telemetry.update();
        }

        // stop all motors
        motorsSetPower(0);

        // turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100); // optional pause after each move.
    }


    // Set lift level
    private void lift(double liftLevel) {
        spoolLeft.setTargetPosition((int) (liftLevel));
        spoolRight.setTargetPosition((int) (liftLevel));
    }

    // Grab with claw
    private void grab() {
        claw.setPosition(GRAB_CLAW_POSITION);
    }

    // Ungrab with claw
    private void ungrab() {
        claw.setPosition(UNGRAB_CLAW_POSITION);
    }

    // Switch wrist between front and back positions
    private void wristFlip() {
        if (isWristFront)
            wrist.setPosition(BACK_WRIST_POSITION);
        else
            wrist.setPosition(FRONT_WRIST_POSITION);
        isWristFront = !isWristFront;
    }

    // Switch axel between front and back positions
    private void axelFlip() {
        if (isAxelFront)
            axel.setPosition(BACK_AXEL_POSITION);
        else
            axel.setPosition(FRONT_AXEL_POSITION);
        isAxelFront = !isAxelFront;
    }
    

    // Initialization code
    // Robot should not yet move on intitalization
    public void initialize() {
        // set initial states
        isAxelFront = true;
        isWristFront = false;

        // initialize Wheels
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");

        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("Start positions:");
        telemetry.addLine("Front  L: " + frontLeft.getCurrentPosition() + "  R: " + frontRight.getCurrentPosition());
        telemetry.addLine("Back   L: " + backLeft.getCurrentPosition() + "  R: " + backRight.getCurrentPosition());
        telemetry.update();

        // initialize Spools
        spoolRight = hardwareMap.get(DcMotor.class, "spoolr");
        spoolLeft = hardwareMap.get(DcMotor.class, "spooll");

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

        // initialize Servos
        axel = hardwareMap.get(Servo.class, "axel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(GRAB_CLAW_POSITION);
        axel.setPosition(FRONT_AXEL_POSITION);
        wrist.setPosition(BACK_WRIST_POSITION);

        // initialize Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new ConeAnalysingPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Test", "Webcam Error");
                telemetry.update();
            }
        });
    }


    public static class ConeAnalysingPipeline extends OpenCvPipeline
    {
        /* 
         * Possible results:
         * 0 - Red, 1 - Green, 2 - Blue, 3 - Error
         */

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GRAY = new Scalar(128, 128, 128);

        /*
         * The core values which define the location and size of the analysed region
         */
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(625, 345);
        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 30;
        static final Point COLOR_REGION_TOPLEFT_ANCHOR_POINT = new Point(700, 200);
        static final int COLOR_REGION_WIDTH = 300;
        static final int COLOR_REGION_HEIGHT = 300;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point color_region_pointA = new Point(
                COLOR_REGION_TOPLEFT_ANCHOR_POINT.x,
                COLOR_REGION_TOPLEFT_ANCHOR_POINT.y);
        Point color_region_pointB = new Point(
                COLOR_REGION_TOPLEFT_ANCHOR_POINT.x + COLOR_REGION_WIDTH,
                COLOR_REGION_TOPLEFT_ANCHOR_POINT.y + COLOR_REGION_HEIGHT);
        

        /*
         * Working variables
         */
        Mat region_H;
        Mat HSV = new Mat();
        Mat H = new Mat();
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile int avg = 0;
        // same as above
        private volatile int color = 3;
        /* 
         * Possible results:
         * 0 - Red, 1 - Green, 2 - Blue, 3 - Error
         */


        /*
         * This function takes the RGB frame, converts to HSV,
         * and extracts the H (Hue) channel to the 'H' variable
         */
        void inputToH(Mat input)
        {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, H, 0);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */

            region_H = firstFrame.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to HSV (or HSL (HSV = HSL)) color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, Hue and
             * Saturation are intertwined. In HSV, Hue is separated.
             * HSV is a 3-channel color space, just like RGB. HSV's 3 channels
             * are H, the Hue channel (which essentially just a color), the
             * S channel (Saturation) and the V channel (Value or Light). Because Hue is separated
             * from everything else in HSV, vision code written to look for certain values
             * in the Hue channel will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the S and V channels.
             *
             * After we've converted to H, we extract just the H channel that 
             * contains color that we need.
             *
             * We then take the average pixel value of a region on that H channel.
             *
             * We also draw rectangles on the screen showing what the region are
             * we checking.
             *
             * In order for this whole process to work correctly, the region
             * should be positioned in the center of the color cone and should 
             * contain only a color part.
             */

            /*
             * Compute the average pixel value of the submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 0 here.
             */
            inputToH(input);
            Mat copyMat = input;
            avg = (int) Core.mean(region_H).val[0];
            

            /*
             * Draw a rectangle showing the region on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_pointA, // First point which defines the rectangle
                    region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    5); // Thickness of the rectangle lines


            /*
             * Now check what color is the closest to our avg.
             * So we can find what avg color is.
             */
            // We expected: Red - 0,       Green - 120,     Blue - 240
            // We have:
            // In lab:      Red - 180-200, Green - 100-120, Blue - 70-90
            // At 2nd meet: Red - 150-170, Green - 90-110,  Blue - 70-90
            int closeToRed = Math.abs(160 - avg), closeToGreen = Math.abs(100 - avg), closeToBlue = Math.abs(80 - avg);
            int closest = Math.min(closeToRed, Math.min(closeToGreen, closeToBlue));

            if (closest == closeToRed) {
                color = 0;
                Imgproc.rectangle( // Visualising what color we have 
                    input, // Buffer to draw on
                    color_region_pointA, // First point which defines the rectangle
                    color_region_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Thickness of the rectangle lines
            }
            else if (closest == closeToGreen) {
                color = 1;
                Imgproc.rectangle( // Visualising what color we have
                    input, // Buffer to draw on
                    color_region_pointA, // First point which defines the rectangle
                    color_region_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Thickness of the rectangle lines
            }
            else if (closest == closeToBlue) {
                color = 2;
                Imgproc.rectangle( // Visualising what color we have
                    input, // Buffer to draw on
                    color_region_pointA, // First point which defines the rectangle
                    color_region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Thickness of the rectangle lines
            }
            else {
                color = 3;
                Imgproc.rectangle( // Visualising what color we have
                    input, // Buffer to draw on
                    color_region_pointA, // First point which defines the rectangle
                    color_region_pointB, // Second point which defines the rectangle
                    GRAY, // The color the rectangle is drawn in
                    -1); // Thickness of the rectangle lines
            }


            return input;
        }

        
        public int getAvg() {
            // Returns Hue value of color that camera see
            return avg;
        }


        public int getAnalysis() {
            // Returns 0 (Red) or 1 (Green) or 2 (Blue) or 3 (Error)
            return color;
        }
    }
}
