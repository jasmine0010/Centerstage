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
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

    //Add instance of RobotHardware
    RobotHardware robot;

    //Declare a variable to store the Pixel's position, assign a default value
    String position = null;

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

        // TRAJECTORY

        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // CENTER
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

        // RIGHT
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-32, -33, Math.toRadians(0)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToSplineHeading(new Pose2d(-52, -8, Math.toRadians(180) + 1e-6))
                .build();

        TrajectorySequence  trajSeq2_2 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .forward(
                        10,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .lineToConstantHeading(new Vector2d(33, -13))
                .lineToConstantHeading(new Vector2d(50, -10))
                .build();


        // LEFT
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-48, -25))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToSplineHeading(new Pose2d(-52, -12, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivotAuto(1, -105, 5);
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

        // PARK
        TrajectorySequence parkInner = drive.trajectorySequenceBuilder(trajSeq3.end())
                .forward(4)
                .strafeRight(22)
                .build();

        TrajectorySequence parkOuter = drive.trajectorySequenceBuilder(trajSeq3.end())
                .forward(4)
                .strafeLeft(29)
                .build();

        robot.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData(">", "Initialized");

        waitForStart();

        if (opModeIsActive()) {

            //this loop detects for five seconds, storing the outcome
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                position = robot.detectPosition("LEFT");
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }

            // Save more CPU resources when camera is no longer needed.
            //visionPortal.close();

            //now you can move your robot based on the value of the 'position' variable
            if (Objects.equals(position, "CENTER")) {
                drive.followTrajectorySequence(trajSeq1);
            } else if (Objects.equals(position, "RIGHT")) {
                drive.followTrajectorySequence(trajSeq2);
            } else {
                drive.followTrajectorySequence(trajSeq2);
                robot.encoderPivotAuto(1, 37, 5);
                robot.clawJoint.setPosition(0.303);
                drive.followTrajectorySequence(trajSeq2_2);
                drive.followTrajectorySequence(parkInner);
            }

            // Transfer the current pose to PoseStorage so we can use it in TeleOp
            PoseStorage.currentPose = drive.getPoseEstimate();

        }
    }   // end runOpMode()
}   // end class