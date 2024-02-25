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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Objects;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RedRightAuto", group = "Concept")

public class RedRightAuto extends LinearOpMode {

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

        int PIVOT_STACK_POS = -105;
        double JOINT_STACK_POS = 0.303;

        // TRAJECTORY

        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // CENTER
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(17, -21))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .splineToSplineHeading(new Pose2d(33, -38, Math.toRadians(180)), Math.toRadians(270))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .build();

        // RIGHT
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(27, -30, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(33, -38))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .build();

        // LEFT
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(4, -31, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(50, -30))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, 0, 5);
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .lineToConstantHeading(new Vector2d(50, 37))
                .build();

        // CYCLE
        // Toward Stack
        TrajectorySequence trajSeqToStack = drive.trajectorySequenceBuilder(trajSeq3.end())
                .splineToConstantHeading(new Vector2d(-5, -9), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftDoorOpener.setPosition(robot.OPENER_ON);
                    robot.rightDoorOpener.setPosition(robot.OPENER_ON);
                })
                .forward(
                        7,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        // Right Stack
        TrajectorySequence trajSeqStackRight = drive.trajectorySequenceBuilder(trajSeqToStack.end())
                .splineToConstantHeading(new Vector2d(-52, -12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
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
                .lineToConstantHeading(new Vector2d(50, -40))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, 0, 5);
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .build();

        // Left Stack
        TrajectorySequence trajSeqStackLeft = drive.trajectorySequenceBuilder(trajSeqToStack.end())
                .splineToConstantHeading(new Vector2d(-52, -35), Math.toRadians(180))
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
                .lineToConstantHeading(new Vector2d(44, -38))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, 0, 5);
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .build();

        robot.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData(">", "Initialized");

        waitForStart();

        if (opModeIsActive()) {

            //this loop detects for five seconds, storing the outcome
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                position = robot.detectPosition();
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }

            // Save more CPU resources when camera is no longer needed.
            // VisionPortal.close();

            //now you can move your robot based on the value of the 'position' variable
            if (Objects.equals(position, "CENTER")) {
                drive.followTrajectorySequence(trajSeq1);
            } else if (Objects.equals(position, "RIGHT")) {
                drive.followTrajectorySequence(trajSeq2);
            } else {
                drive.followTrajectorySequence(trajSeq3);
                // robot.driveToAprilTag(4, 12);
                drive.followTrajectorySequence(trajSeqToStack);
                drive.followTrajectorySequence(trajSeqStackLeft);
            }
        }
    }   // end runOpMode()
}   // end class