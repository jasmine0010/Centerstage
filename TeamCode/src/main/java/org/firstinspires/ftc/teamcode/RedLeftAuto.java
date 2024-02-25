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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RedLeftAuto", group = "Concept")

public class RedLeftAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20231109_095305.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "RED PROP",
            "BLUE PROP"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //Add instance of RobotHardware
    RobotHardware robot;

    //Declare a variable to store the Pixel's position, assign a default value
    String position = "LEFT";

    //Create a runtime object so we can time loops
    ElapsedTime runtime = new ElapsedTime(0);

    @Override
    public void runOpMode() throws InterruptedException {

        initTfod();

        //Initialize robot object with reference to this opMode
        //We can now call all the functions from the RobotHardware Class
        robot = new RobotHardware(this);
        //We need to initialize the robot hardware
        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        robot.clawJoint.setPosition(robot.CLAW_JOINT_DOWN);
        robot.rightClaw.setPosition(robot.CLAW_CLOSED);
        robot.leftClaw.setPosition(robot.CLAW_CLOSED);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // TRAJECTORY

        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // Center
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .back(44)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToSplineHeading(new Pose2d(-52, -12, Math.toRadians(180) + 1e-6))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, -96, 5);
                    robot.clawJoint.setPosition(0.303);
                })
                .forward(
                        10.1,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .lineToConstantHeading(new Vector2d(33, -13))
                .strafeLeft(23)
                .build();

        // Right
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-32, -33, Math.toRadians(0)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToSplineHeading(new Pose2d(-52, -8, Math.toRadians(180) + 1e-6))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, -100, 5);
                    robot.clawJoint.setPosition(0.303);
                })
                .forward(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .lineToConstantHeading(new Vector2d(33, -13))
                .strafeLeft(23)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(12, -61, Math.toRadians(180)))
                .forward(40)
                .lineToConstantHeading(new Vector2d(-52, -38))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, -100, 5);
                    robot.clawJoint.setPosition(0.303);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                })
                .forward(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .build();

        // Left
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-48, -25))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToSplineHeading(new Pose2d(-52, -12, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, -105, 5);
                    robot.clawJoint.setPosition(0.303);
                })
                .forward(
                        10.1,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .lineToConstantHeading(new Vector2d(33, -13))
                .strafeLeft(23)
                .build();

        waitForStart();

        if (opModeIsActive()) {

            //this loop detects for five seconds, storing the outcome
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                detectPosition();
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

            //now you can move your robot based on the value of the 'position' variable
            if (Objects.equals(position, "CENTER")) {
                drive.followTrajectorySequence(trajSeq1);
            } else if (Objects.equals(position, "RIGHT")) {
                drive.followTrajectorySequence(trajSeq2);
            } else {
                drive.followTrajectorySequence(trajSeq2);
            }
        }
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

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
     * This function will detect the pixel location and store it's location in the position variable
     */
    private void detectPosition() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //These thresholds will need to be adjusted based on testing on the field
        int LEFT_THRESHOLD = 100;
        int RIGHT_THRESHOLD = 460;

        //This is pulling from the first object it detects...if it detects more than one...
        //you may need to choose based on size or position
        if (currentRecognitions.size() > 0) {
            Recognition rec = currentRecognitions.get(0);

            telemetry.addData("- Position", rec.getLeft());

            if (rec.getLeft() < LEFT_THRESHOLD) {
                position = "LEFT";
            } else if (rec.getLeft() > RIGHT_THRESHOLD) {
                position = "RIGHT";
            } else {
                position = "CENTER";
            }
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class