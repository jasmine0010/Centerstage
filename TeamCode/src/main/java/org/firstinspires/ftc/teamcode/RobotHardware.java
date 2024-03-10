package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.features2d.ORB;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public static final boolean USE_WEBCAM = true;

    ElapsedTime timeout = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_INCH = 50;
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20231109_095305.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "RED PROP",
            "BLUE PROP"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    SampleMecanumDrive drive = null;

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightSlides = null;
    public DcMotor leftSlides = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo clawJoint = null;
    public Servo drone = null;
    public Servo leftDoorOpener = null;
    public Servo rightDoorOpener = null;
    public DcMotorEx pivot = null;
    public DcMotor arm = null;
    public IMU imu;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController;
    private PIDController pivotPID;
    public static double PIVOT_P = 0.05, PIVOT_I = 0.01, PIVOT_D = 0.005;
    public static double PIVOT_F = 0.1;

    public static int PIVOT_TARGET = 150;

    private final double ticks_in_degree = 288 / 360.0;

    boolean targetFound = false;
    boolean ORBIT_ACTIVATED = false;
    int PIVOT_UP_POS = 330;
    double CLAW_JOINT_UP = 0;
    double CLAW_JOINT_DOWN = 0.28 ;
    double OPENER_ON = 1;
    double OPENER_OFF = 0.5;
    double ARM_POWER = 1;
    double CLAW_OPENED = 0.35;
    double CLAW_CLOSED = 0.55;
    double DRONE_UP = 0.2;
    double DRONE_DOWN = 0.03;

    // Adjust these numbers to suit your robot.

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    public enum PivotMode {
        MANUAL,
        SCORING,
        INTAKE
    }

    public enum DriveMode {
        STANDARD,
        FIELD_CENTRIC,
        DRIVE_TO_DOOR,
        ALIGN_BACKDROP,
        APRILTAGS,
        ORBIT
    }

    public PivotMode pivotMode = PivotMode.MANUAL;
    public DriveMode driveMode = DriveMode.STANDARD;

    // The coordinates we want the bot to automatically go to
    Vector2d targetVector = new Vector2d(0, 0);
    // The heading we want the bot to end on for targetVector
    double targetHeading = Math.toRadians(180);

    // The angle we want to align
    double targetAlignAngle = Math.toRadians(180);

    // Point to orbit
    private final Vector2d blueOrbitPosition = new Vector2d(-62.50, -60);
    private final Vector2d redOrbitPosition = new Vector2d(-62.50, 60);

    public Pose2d myPose = new Pose2d(44, 6, Math.toRadians(180));

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        initCameraSystem();

        drive = new SampleMecanumDrive(myOpMode.hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));

        pivotPID = new PIDController(PIVOT_P, PIVOT_I, PIVOT_D);
        headingController = new PIDFController(SampleMecanumDrive.HEADING_PID_TELE);
        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftSlides = myOpMode.hardwareMap.get(DcMotor.class, "leftSlides");
        rightSlides = myOpMode.hardwareMap.get(DcMotor.class, "rightSlides");
        leftClaw = myOpMode.hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = myOpMode.hardwareMap.get(Servo.class, "rightClaw");
        clawJoint = myOpMode.hardwareMap.get(Servo.class, "clawJoint");
        drone = myOpMode.hardwareMap.get(Servo.class, "drone");
        leftDoorOpener = myOpMode.hardwareMap.get(Servo.class, "leftDoorOpener");
        rightDoorOpener = myOpMode.hardwareMap.get(Servo.class, "rightDoorOpener");
        pivot = myOpMode.hardwareMap.get(DcMotorEx.class, "pivot");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftSlides.setDirection(DcMotor.Direction.FORWARD);
        rightSlides.setDirection(DcMotor.Direction.REVERSE);
        leftDoorOpener.setDirection(Servo.Direction.FORWARD);
        rightDoorOpener.setDirection(Servo.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        drone.setDirection(Servo.Direction.REVERSE);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        myOpMode.telemetry.addData(">", "Initialized");
    }

    public void initCameraSystem() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Build the Vision Portal, using the above settings.
        // visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
    }

    public void driveToAprilTag(int targetTag, double targetDistance) {

        double rangeError = 100;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        while (rangeError > 1 && myOpMode.opModeIsActive()) {

            if (myOpMode.gamepad1.dpad_up) {
                driveMode = DriveMode.STANDARD;
                break;
            }

            targetFound = false;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == targetTag) || (detection.id == targetTag + 3)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                myOpMode.telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                myOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                myOpMode.telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                myOpMode.telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                myOpMode.telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError      = (desiredTag.ftcPose.range - targetDistance);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)*-1;
                turn   = Range.clip(headingError * -TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)*-1;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)*-1;

                myOpMode.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Calculate wheel powers.
                double leftFrontPower    =  drive -strafe -turn;
                double rightFrontPower   =  drive +strafe +turn;
                double leftBackPower     =  drive +strafe -turn;
                double rightBackPower    =  drive -strafe +turn;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                SampleMecanumDrive.leftFront.setPower(leftFrontPower);
                SampleMecanumDrive.rightFront.setPower(rightFrontPower);
                SampleMecanumDrive.leftRear.setPower(leftBackPower);
                SampleMecanumDrive.rightRear.setPower(rightBackPower);
            } else {
                myOpMode.telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                SampleMecanumDrive.leftFront.setPower(0);
                SampleMecanumDrive.rightFront.setPower(0);
                SampleMecanumDrive.leftRear.setPower(0);
                SampleMecanumDrive.rightRear.setPower(0);
                driveMode = DriveMode.STANDARD;
                break;
            }

            if (desiredTag.id == 1) {
            //    Pose2d pose = new Pose2d(-63, -63, Math.toRadians(0));
            //    drive.setPoseEstimate(pose);
            //    PoseStorage.currentPose = drive.getPoseEstimate();
            //    myPose = PoseStorage.currentPose;
            } else if (desiredTag.id == 2) {

            } else if (desiredTag.id == 3) {

            }

            myOpMode.telemetry.update();
        }

        SampleMecanumDrive.leftFront.setPower(0);
        SampleMecanumDrive.rightFront.setPower(0);
        SampleMecanumDrive.leftRear.setPower(0);
        SampleMecanumDrive.rightRear.setPower(0);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

    /* Declare OpMode members. */

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            targetCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(targetCounts);
            leftBackDrive.setTargetPosition(targetCounts);
            rightFrontDrive.setTargetPosition(targetCounts);
            rightBackDrive.setTargetPosition(targetCounts);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (timeout.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", targetCounts);
                myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void autoStrafe (double speed, double distance, double timeoutS){
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        targetCounts = (int)(distance * COUNTS_PER_INCH);
        leftFrontDrive.setTargetPosition(targetCounts);
        leftBackDrive.setTargetPosition(-targetCounts);
        rightFrontDrive.setTargetPosition(-targetCounts);
        rightBackDrive.setTargetPosition(targetCounts);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timeout.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() && (timeout.seconds() < timeoutS) && (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {
            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", targetCounts);
            myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnCCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)+degrees;

        //start the motors
        leftFrontDrive.setPower(-motorPower);
        leftBackDrive.setPower(-motorPower);
        rightFrontDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle > orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
            myOpMode.telemetry.update();
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)-degrees;

        //start the motors
        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(-motorPower);
        rightBackDrive.setPower(-motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle < orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void encoderPivot(double speed, int target, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            pivot.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            pivot.setPower(Math.abs(speed));

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", target);
            myOpMode.telemetry.addData("Pivot at", pivot.getCurrentPosition());
        }
    }

    public void encoderPivotAuto(double speed, int target, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            pivot.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            pivot.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (timeout.seconds() < timeoutS) && (pivot.isBusy())) {

                // claw joint
                if (pivot.getCurrentPosition() > 160) {
                    clawJoint.setPosition(CLAW_JOINT_UP);
                } else {
                    clawJoint.setPosition(CLAW_JOINT_DOWN);
                }

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", target);
                myOpMode.telemetry.addData("Pivot at", pivot.getCurrentPosition());
            }

            pivot.setPower(0);
        }
    }

    public void pivotPID(int target) {
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(target-pivot.getCurrentPosition()) > 10 && myOpMode.opModeIsActive()) {

            // claw joint
            if (pivot.getCurrentPosition() > 160) {
                clawJoint.setPosition(CLAW_JOINT_UP);
            } else {
                clawJoint.setPosition(CLAW_JOINT_DOWN);
            }

            pivotPID.setPID(PIVOT_P, PIVOT_I, PIVOT_D);
            int pivotPos = pivot.getCurrentPosition();
            double pid = pivotPID.calculate(pivotPos, target);
            double ff = Math.cos(Math.toRadians(PIVOT_TARGET / ticks_in_degree)) * PIVOT_F;

            double power = pid + ff;

            pivot.setPower(power);
        }

        pivot.setPower(0);
    }

    public void pivotTelePID(int target) {
        if (myOpMode.opModeIsActive()) {
            pivotPID.setPID(PIVOT_P, PIVOT_I, PIVOT_D);
            int pivotPos = pivot.getCurrentPosition();
            double pid = pivotPID.calculate(pivotPos, target);
            double ff = Math.cos(Math.toRadians(PIVOT_TARGET / ticks_in_degree)) * PIVOT_F;

            double power = pid + ff;

            pivot.setPower(power);
        }
    }

    public void encoderArm(double speed, int target, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            arm.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            arm.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (timeout.seconds() < timeoutS) && (arm.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", target);
                myOpMode.telemetry.addData("Arm at", arm.getCurrentPosition());
            }

            // Stop all motion;
            arm.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderLift(double speed, int target, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftSlides.setTargetPosition(target);
            rightSlides.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            leftSlides.setPower(Math.abs(speed));
            rightSlides.setPower(Math.abs(speed));

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", target);
            myOpMode.telemetry.addData("Slides at", leftSlides.getCurrentPosition());
            myOpMode.telemetry.addData("Slides at", rightSlides.getCurrentPosition());
            myOpMode.telemetry.update();

            // Stop all motion;
            leftSlides.setPower(0);
            rightSlides.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void teleOpRed() {

        // Make sure to call drive.update() on *every* loop
        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
        drive.update();

        // Retrieve your pose
        myPose = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        // telemetry
        myOpMode.telemetry.addData("Drive mode: ", driveMode);
        myOpMode.telemetry.addData("Pivot mode: ", pivotMode);
        myOpMode.telemetry.addData("x", myPose.getX());
        myOpMode.telemetry.addData("y", myPose.getY());
        myOpMode.telemetry.addData("heading", myPose.getHeading());
        myOpMode.telemetry.addData("orbit ", ORBIT_ACTIVATED);

        myOpMode.telemetry.addData("pivotPosition", pivot.getCurrentPosition());
        myOpMode.telemetry.addData("armPosition", arm.getCurrentPosition());
        myOpMode.telemetry.addData("leftSlidesPosition", leftSlides.getCurrentPosition());
        myOpMode.telemetry.addData("rightSlidesPosition", rightSlides.getCurrentPosition());
        myOpMode.telemetry.update();

        if (myOpMode.gamepad1.y) {
            ORBIT_ACTIVATED = !ORBIT_ACTIVATED;
        }

        //Drivetrain TeleOp Code
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double LEFT_FRONT_POWER = axial + lateral + yaw;
        double RIGHT_FRONT_POWER = axial - lateral - yaw;
        double LEFT_BACK_POWER = axial - lateral + yaw;
        double RIGHT_BACK_POWER = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(LEFT_FRONT_POWER), Math.abs(RIGHT_FRONT_POWER));
        max = Math.max(max, Math.abs(LEFT_BACK_POWER));
        max = Math.max(max, Math.abs(RIGHT_BACK_POWER));

        if (max > 1.0) {
            LEFT_FRONT_POWER /= max;
            RIGHT_FRONT_POWER /= max;
            LEFT_BACK_POWER /= max;
            RIGHT_BACK_POWER /= max;
        }

        if (myPose.getX() < -40 && myPose.getY() > 40 && ORBIT_ACTIVATED) {
            driveMode = DriveMode.ORBIT;
        } else if (myOpMode.gamepad1.dpad_up) {
            driveMode = DriveMode.STANDARD;
        } else if (myOpMode.gamepad1.dpad_down) {
            driveMode = DriveMode.FIELD_CENTRIC;
        } else if (myOpMode.gamepad1.dpad_left) {
            driveMode = DriveMode.DRIVE_TO_DOOR;
        } else if (myOpMode.gamepad1.dpad_right) {
            driveMode = DriveMode.ALIGN_BACKDROP;
        } else if (myOpMode.gamepad1.b) {
            driveMode = DriveMode.APRILTAGS;
        }

        if (myOpMode.gamepad1.x) {
            Pose2d pose = new Pose2d(-63, 63, Math.toRadians(0));
            drive.setPoseEstimate(pose);
            PoseStorage.currentPose = drive.getPoseEstimate();
            myPose = PoseStorage.currentPose;
            ORBIT_ACTIVATED = false;
        }

        if (myOpMode.gamepad1.a) {
            SampleMecanumDrive.leftFront.setPower(LEFT_FRONT_POWER / 4);
            SampleMecanumDrive.rightFront.setPower(RIGHT_FRONT_POWER / 4);
            SampleMecanumDrive.leftRear.setPower(LEFT_BACK_POWER / 4);
            SampleMecanumDrive.rightRear.setPower(RIGHT_BACK_POWER / 4);

        } else if (driveMode == DriveMode.ORBIT) {
            if (!ORBIT_ACTIVATED || myPose.getX()>-40 || myPose.getY()<40) {
                driveMode = DriveMode.STANDARD;
            }
            // Create a vector from the gamepad x/y inputs which is the field relative movement
            // Then, rotate that vector by the inverse of that heading for field centric control
            Vector2d fieldFrameInput = new Vector2d(
                    myOpMode.gamepad1.left_stick_x,
                    -myOpMode.gamepad1.left_stick_y
            );
            Vector2d robotFrameInput = fieldFrameInput.rotated(-myPose.getHeading());

            // Difference between the target vector and the bot's position
            Vector2d difference = redOrbitPosition.minus(myPose.vec());
            // Obtain the target angle for feedback and derivative for feedforward
            double theta = difference.angle();

            // Not technically omega because its power. This is the derivative of atan2
            double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

            // Set the target heading for the heading controller to our desired angle
            headingController.setTargetPosition(theta);

            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            double headingInput = (headingController.update(myPose.getHeading())
                    * DriveConstants.kV + thetaFF)
                    * DriveConstants.TRACK_WIDTH;

            // Combine the field centric x/y velocity with our derived angular velocity
            driveDirection = new Pose2d(
                    robotFrameInput,
                    headingInput
            );
            drive.setWeightedDrivePower(driveDirection);

        } else if (driveMode == DriveMode.FIELD_CENTRIC) {
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -myOpMode.gamepad1.left_stick_y,
                    -myOpMode.gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()-Math.toRadians(90));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -myOpMode.gamepad1.right_stick_x
                    )
            );
        } else if (driveMode == DriveMode.APRILTAGS) {
            setManualExposure(6, 250);
            driveToAprilTag(-1, 12);
        } else if (driveMode == DriveMode.STANDARD) {
            SampleMecanumDrive.leftFront.setPower(LEFT_FRONT_POWER*1.1);
            SampleMecanumDrive.rightFront.setPower(RIGHT_FRONT_POWER*1.1);
            SampleMecanumDrive.leftRear.setPower(LEFT_BACK_POWER*1.1);
            SampleMecanumDrive.rightRear.setPower(RIGHT_BACK_POWER*1.1);
        } else if (driveMode == DriveMode.DRIVE_TO_DOOR) {
            Trajectory traj1 = drive.trajectoryBuilder(myPose)
                    .splineTo(targetVector, targetHeading)
                    .build();
            drive.followTrajectory(traj1);
            driveMode = DriveMode.STANDARD;
        } else if (driveMode == DriveMode.ALIGN_BACKDROP) {
            drive.turn(Angle.normDelta(targetAlignAngle - myPose.getHeading()));
            driveMode = DriveMode.STANDARD;
        }


        // PIVOT
        if (Math.abs(myOpMode.gamepad2.left_trigger)>0.1 || Math.abs(myOpMode.gamepad2.right_trigger)>0.1) {
            pivotMode = PivotMode.MANUAL;
        } else if (myOpMode.gamepad2.dpad_up) {
            pivotMode = PivotMode.SCORING;
        } else if (myOpMode.gamepad2.dpad_down) {
            pivotMode = PivotMode.INTAKE;
        }

        if (pivotMode == PivotMode.MANUAL) {
            double PIVOT_POWER_ADD;
            if (pivot.getCurrentPosition()<170) {
                PIVOT_POWER_ADD = 0.2;
            } else {
                PIVOT_POWER_ADD = -0.2;
            }
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivot.setPower(myOpMode.gamepad2.right_trigger*-1+myOpMode.gamepad2.left_trigger+PIVOT_POWER_ADD);
        } else if (pivotMode == PivotMode.SCORING) {
            pivotPID(330);
        } else if (pivotMode == PivotMode.INTAKE) {
            pivotPID(0);
        }

        // linear slides - R lower, L raise
        rightSlides.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);
        leftSlides.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);

        // arm
        if (myOpMode.gamepad1.left_bumper) {
            arm.setPower(ARM_POWER*-1);
        } else if (myOpMode.gamepad1.right_bumper) {
            arm.setPower(ARM_POWER);
        } else {
            arm.setPower(0);
        }

        // drone
        if (myOpMode.gamepad2.b) {
            drone.setPosition(DRONE_UP);
        } else {
            drone.setPosition(DRONE_DOWN);
        }

        // claw
        if (myOpMode.gamepad2.left_bumper) {
            leftClaw.setPosition(CLAW_OPENED);
            rightClaw.setPosition(CLAW_OPENED);
        } else {
            leftClaw.setPosition(CLAW_CLOSED);
            rightClaw.setPosition(CLAW_CLOSED);
        }

        // claw joint
        if (pivot.getCurrentPosition() > 160) {
            clawJoint.setPosition(CLAW_JOINT_UP);
        } else {
            clawJoint.setPosition(CLAW_JOINT_DOWN);
        }

        // door opener
        if (myOpMode.gamepad1.a) {
            leftDoorOpener.setPosition(OPENER_ON);
            rightDoorOpener.setPosition(OPENER_ON);
        } else {
            leftDoorOpener.setPosition(OPENER_OFF);
            rightDoorOpener.setPosition(OPENER_OFF);
        }
    }

    public void teleOpBlue() {

        // Make sure to call drive.update() on *every* loop
        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
        drive.update();

        // Retrieve your pose
        myPose = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        // telemetry
        myOpMode.telemetry.addData("Drive mode: ", driveMode);
        myOpMode.telemetry.addData("Pivot mode: ", pivotMode);
        myOpMode.telemetry.addData("x", myPose.getX());
        myOpMode.telemetry.addData("y", myPose.getY());
        myOpMode.telemetry.addData("heading", myPose.getHeading());
        myOpMode.telemetry.addData("orbit ", ORBIT_ACTIVATED);

        myOpMode.telemetry.addData("pivotPosition", pivot.getCurrentPosition());
        myOpMode.telemetry.addData("armPosition", arm.getCurrentPosition());
        myOpMode.telemetry.addData("leftSlidesPosition", leftSlides.getCurrentPosition());
        myOpMode.telemetry.addData("rightSlidesPosition", rightSlides.getCurrentPosition());
        myOpMode.telemetry.update();

        if (myOpMode.gamepad1.y) {
            ORBIT_ACTIVATED = !ORBIT_ACTIVATED;
        }

        //Drivetrain TeleOp Code
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double LEFT_FRONT_POWER = axial + lateral + yaw;
        double RIGHT_FRONT_POWER = axial - lateral - yaw;
        double LEFT_BACK_POWER = axial - lateral + yaw;
        double RIGHT_BACK_POWER = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(LEFT_FRONT_POWER), Math.abs(RIGHT_FRONT_POWER));
        max = Math.max(max, Math.abs(LEFT_BACK_POWER));
        max = Math.max(max, Math.abs(RIGHT_BACK_POWER));

        if (max > 1.0) {
            LEFT_FRONT_POWER /= max;
            RIGHT_FRONT_POWER /= max;
            LEFT_BACK_POWER /= max;
            RIGHT_BACK_POWER /= max;
        }

        if (myPose.getX() < -40 && myPose.getY() < -40 && ORBIT_ACTIVATED) {
            driveMode = DriveMode.ORBIT;
        } else if (myOpMode.gamepad1.dpad_up) {
            driveMode = DriveMode.STANDARD;
        } else if (myOpMode.gamepad1.dpad_down) {
            driveMode = DriveMode.FIELD_CENTRIC;
        } else if (myOpMode.gamepad1.dpad_left) {
            driveMode = DriveMode.DRIVE_TO_DOOR;
        } else if (myOpMode.gamepad1.dpad_right) {
            driveMode = DriveMode.ALIGN_BACKDROP;
        } else if (myOpMode.gamepad1.b) {
            driveMode = DriveMode.APRILTAGS;
        }

        if (myOpMode.gamepad1.x) {
            Pose2d pose = new Pose2d(-63, -63, Math.toRadians(0));
            drive.setPoseEstimate(pose);
            PoseStorage.currentPose = drive.getPoseEstimate();
            myPose = PoseStorage.currentPose;
            ORBIT_ACTIVATED = false;
        }

        if (myOpMode.gamepad1.a) {
            SampleMecanumDrive.leftFront.setPower(LEFT_FRONT_POWER / 4);
            SampleMecanumDrive.rightFront.setPower(RIGHT_FRONT_POWER / 4);
            SampleMecanumDrive.leftRear.setPower(LEFT_BACK_POWER / 4);
            SampleMecanumDrive.rightRear.setPower(RIGHT_BACK_POWER / 4);

        } else if (driveMode == DriveMode.ORBIT) {
            if (!ORBIT_ACTIVATED || myPose.getX()>-40 || myPose.getY()>-40) {
                driveMode = DriveMode.STANDARD;
            }
            // Create a vector from the gamepad x/y inputs which is the field relative movement
            // Then, rotate that vector by the inverse of that heading for field centric control
            Vector2d fieldFrameInput = new Vector2d(
                    -myOpMode.gamepad1.left_stick_x,
                    myOpMode.gamepad1.left_stick_y
            );
            Vector2d robotFrameInput = fieldFrameInput.rotated(-myPose.getHeading());

            // Difference between the target vector and the bot's position
            Vector2d difference = blueOrbitPosition.minus(myPose.vec());
            // Obtain the target angle for feedback and derivative for feedforward
            double theta = difference.angle();

            // Not technically omega because its power. This is the derivative of atan2
            double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

            // Set the target heading for the heading controller to our desired angle
            headingController.setTargetPosition(theta);

            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            double headingInput = (headingController.update(myPose.getHeading())
                    * DriveConstants.kV + thetaFF)
                    * DriveConstants.TRACK_WIDTH;

            // Combine the field centric x/y velocity with our derived angular velocity
            driveDirection = new Pose2d(
                    robotFrameInput,
                    headingInput
            );
            drive.setWeightedDrivePower(driveDirection);

        } else if (driveMode == DriveMode.FIELD_CENTRIC) {
            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -myOpMode.gamepad1.left_stick_y,
                    -myOpMode.gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()+Math.toRadians(90));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -myOpMode.gamepad1.right_stick_x
                    )
            );
        } else if (driveMode == DriveMode.APRILTAGS) {
            setManualExposure(6, 250);
            driveToAprilTag(-1, 12);
        } else if (driveMode == DriveMode.STANDARD) {
            SampleMecanumDrive.leftFront.setPower(LEFT_FRONT_POWER*1.1);
            SampleMecanumDrive.rightFront.setPower(RIGHT_FRONT_POWER*1.1);
            SampleMecanumDrive.leftRear.setPower(LEFT_BACK_POWER*1.1);
            SampleMecanumDrive.rightRear.setPower(RIGHT_BACK_POWER*1.1);
        } else if (driveMode == DriveMode.DRIVE_TO_DOOR) {
            Trajectory traj1 = drive.trajectoryBuilder(myPose)
                    .splineTo(targetVector, targetHeading)
                    .build();
            drive.followTrajectory(traj1);
            driveMode = DriveMode.STANDARD;
        } else if (driveMode == DriveMode.ALIGN_BACKDROP) {
            drive.turn(Angle.normDelta(targetAlignAngle - myPose.getHeading()));
            driveMode = DriveMode.STANDARD;
        }


        // PIVOT
        if (Math.abs(myOpMode.gamepad2.left_trigger)>0.1 || Math.abs(myOpMode.gamepad2.right_trigger)>0.1) {
            pivotMode = PivotMode.MANUAL;
        } else if (myOpMode.gamepad2.dpad_up) {
            pivotMode = PivotMode.SCORING;
        } else if (myOpMode.gamepad2.dpad_down) {
            pivotMode = PivotMode.INTAKE;
        }

        if (pivotMode == PivotMode.MANUAL) {
            double PIVOT_POWER_ADD;
            if (pivot.getCurrentPosition()<170) {
                PIVOT_POWER_ADD = 0.2;
            } else {
                PIVOT_POWER_ADD = -0.2;
            }
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivot.setPower(myOpMode.gamepad2.right_trigger*-1+myOpMode.gamepad2.left_trigger+PIVOT_POWER_ADD);
        } else if (pivotMode == PivotMode.SCORING) {
            pivotPID(330);
        } else if (pivotMode == PivotMode.INTAKE) {
            pivotPID(0);
        }

        // linear slides - R lower, L raise
        rightSlides.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);
        leftSlides.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);

        // arm
        if (myOpMode.gamepad1.left_bumper) {
            arm.setPower(ARM_POWER*-1);
        } else if (myOpMode.gamepad1.right_bumper) {
            arm.setPower(ARM_POWER);
        } else {
            arm.setPower(0);
        }

        // drone
        if (myOpMode.gamepad2.b) {
            drone.setPosition(DRONE_UP);
        } else {
            drone.setPosition(DRONE_DOWN);
        }

        // claw
        if (myOpMode.gamepad2.left_bumper) {
            leftClaw.setPosition(CLAW_OPENED);
            rightClaw.setPosition(CLAW_OPENED);
        } else {
            leftClaw.setPosition(CLAW_CLOSED);
            rightClaw.setPosition(CLAW_CLOSED);
        }

        // claw joint
        if (pivot.getCurrentPosition() > 160) {
            clawJoint.setPosition(CLAW_JOINT_UP);
        } else {
            clawJoint.setPosition(CLAW_JOINT_DOWN);
        }

        // door opener
        if (myOpMode.gamepad1.a) {
            leftDoorOpener.setPosition(OPENER_ON);
            rightDoorOpener.setPosition(OPENER_ON);
        } else {
            leftDoorOpener.setPosition(OPENER_OFF);
            rightDoorOpener.setPosition(OPENER_OFF);
        }
    }

    public void checkTelemetry(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }
}