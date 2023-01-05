package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvViewport;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous(name="Auto3", group="Basic")
public class Auto3 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor spoolRight;
    private DcMotor spoolLeft;
    private Servo axel;
    private Servo wrist;
    private Servo claw;
    private OpenCvWebcam webcam;
    private ConeAnalysingPipeline pipeline;

    // Mutable variables
    private double speed;
    private double liftLevel;
    private int webcamResponce;
    private String[] Colors = {"Red", "Green", "Blue", "Error"}; // possible colors
    
    // States
    private boolean isAxelFront;
    private boolean isWristFront;

    // Constants
    private final double LIFT_POWER = 1.5;
    private final double HIGH_SPEED = 0.9; // CHANGE VALUE
    private final double LOW_SPEED = 0.7; // CHANGE VALUE
    private final int MAX_LIFT_LEVEL = 2735; // CHANGE VALUE MAX value is ~38.5 in. * ~77 = ~2926
    //every 1500 ticks is ~2.5 in ~77 tpi. 
    private final int MIN_LIFT_LEVEL = 30;
    private final int START_LIFT_LEVEL = 0;
    private final double FRONT_AXEL_POSITION = 0.0;
    private final double BACK_AXEL_POSITION = 1.0;
    private final double FRONT_WRIST_POSITION = 0.83; // CHANGE VALUE
    private final double BACK_WRIST_POSITION = 0.17; // CHANGE VALUE
    private final double GRAB_CLAW_POSITION = 0.35; // OLD CLAW VALUE
    private final double UNGRAB_CLAW_POSITION = 0.25; // OLD CLAW VALUEu
    private final double TURN_SPEED = 0.8;
    private final long STRAFE_TIME = 700; // USELESS VARIABLE NOW // times in milliseconds
    private final long MOVE_TIME = 540; // USELESS VARIABLE NOW // times in milliseconds
    private final long TURN90_TIME = 300; // USELESS VARIABLE NOW
    

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize everything
        initialize();

        // works until you hit Play on the Driver Hub
        while (!isStarted()) {
            // Possible responses: 0 - Red, 1 - Green, 2 - Blue, 3 - Error
            webcamResponce = pipeline.getAnalysis();
            
            //telemetry.addData("Color", Colors[pipeline.getAnalysis()]);
            telemetry.addData("Avg", pipeline.getAvg());
            telemetry.addData("Color", Colors[webcamResponce]);
            telemetry.update();
        }

        telemetry.addData("Result", Colors[webcamResponce]);
        telemetry.update();

/*
        grab();
        msleep(350); // change sleep value
        strafeRight(HIGH_SPEED);
        lift(MAX_LIFT_LEVEL);
        msleep(1.5 * TILE_TIME); // change sleep value
        moveForward(HIGH_SPEED);
        msleep(0.94 * TILE_TIME); // change sleep value
        motorStop();
        moveForward(LOW_SPEED);
        msleep(210); // change sleep value
        motorStop();
        msleep(100);
        ungrab(); // score a cone

*/
//PARKING CODE
        moveForward(LOW_SPEED);
        sleep(60);
        motorStop();
        //wristFlip();
        sleep(100);

        // RED -- RIGHT ZONE
        if (webcamResponce == 0){
            strafeRight(HIGH_SPEED);
            msleep(1.0 * STRAFE_TIME);
            motorStop();
            sleep(100);
            moveForward(HIGH_SPEED);
            msleep(1.5 * MOVE_TIME);
            motorStop();
        }
        // GREEN -- CENTRE ZONE
        else if (webcamResponce == 1){
            moveForward(HIGH_SPEED);
            msleep (1.5 * MOVE_TIME);
            motorStop();
        }
        // BLUE -- LEFT ZONE
        else if (webcamResponce == 2){
            strafeLeft(HIGH_SPEED);
            msleep(1.0 * STRAFE_TIME);
            motorStop();
            sleep(100);
            moveForward(HIGH_SPEED);
            msleep(1.5 * MOVE_TIME);
            motorStop();
        }
        else if (webcamResponce == 3){}
        /*
        //msleep(200);
        grab();
        lift(MIN_LIFT_LEVEL);
        turnLeft(speed);
        msleep(300);
        */
        
        sleep(100);
        ungrab();
        sleep(100);
        //moveBackward(LOW_SPEED);
        //msleep(0.5 * MOVE_TIME);
        //motorStop();
        
        /*boolean temp = true;
        while (opModeIsActive()) {
            temp = true;
        }*/
    }

    private DcMotor getMotor(String motorName) {
        return hardwareMap.get(DcMotor.class, motorName);
    }

    // Stop the motors
    private void motorStop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    // Modified sleep()
    private void msleep(double time) {
        sleep((long) time);
    }

    // Move all motors forward
    private void moveForward(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }


    // Move all motors backward
    private void moveBackward(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
    }

    // Turn left by setting frontleft and backleft backward and frontright and backright to forward
    private void turnLeft(double speed) {
        frontLeft.setPower(-speed * TURN_SPEED);
        frontRight.setPower(-speed * TURN_SPEED);
        backLeft.setPower(speed * TURN_SPEED);
        backRight.setPower(speed * TURN_SPEED);
    }

    // Turn right by setting frontleft and backleft forward and frontright and backright to backward
    private void turnRight(double speed) {
        frontLeft.setPower(speed * TURN_SPEED);
        frontRight.setPower(speed * TURN_SPEED);
        backLeft.setPower(-speed * TURN_SPEED);
        backRight.setPower(-speed * TURN_SPEED);
    }

    // Strafe left by setting frontleft and backright to backward and frontright and backleft to forward
    private void strafeLeft(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }

    // Strafe left by setting frontleft and backright to backward and frontright and backleft to forward
    private void strafeRight(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
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

    // Reset all wheel motors
    /*private void reset() {
        // This resets the encoder value to be 0
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Put them back into running mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // reset() + zero power behavior to Brake
    private void hold() {
        // This resets the encoder value to be 0
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Put them back into running mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior to Brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }*/

    // Initialization code
    // Robot should not yet move on intitalization
    public void initialize() {
        // set initial values
        speed = 0.8;
        liftLevel = START_LIFT_LEVEL;

        isAxelFront = true;
        isWristFront = false;
        
        // initialize motors
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        spoolRight = hardwareMap.get(DcMotor.class, "spoolr");
        spoolLeft = hardwareMap.get(DcMotor.class, "spooll");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Initialize Webcam
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
            // We have:     Red - 180-200, Green - 100-120, Blue - 70-90
            int closeToRed = Math.abs(190 - avg), closeToGreen = Math.abs(110 - avg), closeToBlue = Math.abs(80 - avg);
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