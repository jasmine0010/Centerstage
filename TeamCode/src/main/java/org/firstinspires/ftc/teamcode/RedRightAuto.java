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
@Autonomous(name = "RedRightAuto", group = "Concept")

public class RedRightAuto extends LinearOpMode {

    //Add instance of RobotHardware
    RobotHardware robot;

    //Declare a variable to store the Pixel's position, assign a default value
    String position = null;
    int PIVOT_STACK_POS = -105;
    double JOINT_STACK_POS = 0.303;

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

        // TRAJECTORIES

        Pose2d startPose = new Pose2d(14.50, -63.00, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // CENTER
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24.00, -24.00, Math.toRadians(180.00)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(50.15, -42.00))
                //.lineToLinearHeading(new Pose2d(30.00, -36.00, Math.toRadians(180)))
                .build();

        // RIGHT
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30.00, -32.00, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(52.50, -47.50))
                //.lineToLinearHeading(new Pose2d(35.00, -36.00, Math.toRadians(180)))
                .build();

        // LEFT
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(6.70, -33.00, Math.toRadians(180)))
                .addDisplacementMarker(31, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(49.50, -32.00))
                //.lineToLinearHeading(new Pose2d(30.00, -36.00, Math.toRadians(180)))
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

        // CYCLE
        // Left Stack Through Truss
        TrajectorySequence trajSeqToStackLeft = drive.trajectorySequenceBuilder(trajSeq2.end())
                .lineToLinearHeading(new Pose2d(23.00, -66.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-39.50, -66.00))
                .splineToConstantHeading(new Vector2d(-55, -34), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivotAuto(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .forward(
                        8,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .back(5)
                .splineToConstantHeading(new Vector2d(-63, -39), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .back(1e-2) // to adjust spline shape
                .splineToConstantHeading(new Vector2d(-39.50, -62.00), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(23, -62.00))
                .lineToConstantHeading(new Vector2d(49.50, -40.00))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivotAuto(1, robot.PIVOT_UP_POS, 5);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivotAuto(1, 0, 5);
                    robot.clawJoint.setPosition(robot.CLAW_JOINT_DOWN);
                })
                .lineToLinearHeading(new Pose2d(30.00, -36.00, Math.toRadians(180))).waitSeconds(1)
                .build();

        robot.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData(">", "Initialized");

        waitForStart();

        if (opModeIsActive()) {

            //this loop detects for five seconds, storing the outcome
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
                position = robot.detectPosition("CENTER");
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
                robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderPivotAuto(1, robot.PIVOT_UP_POS, 5);
                robot.rightClaw.setPosition(robot.CLAW_OPENED);
                robot.encoderPivotAuto(1, 0, 5);
                robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderArm(1, 25, 5);
                drive.followTrajectorySequence(parkInner);
            } else if (Objects.equals(position, "RIGHT")) {
                drive.followTrajectorySequence(trajSeq2);
                robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderPivotAuto(1, robot.PIVOT_UP_POS, 5);
                robot.rightClaw.setPosition(robot.CLAW_OPENED);
                robot.encoderPivotAuto(1, 0, 5);
                robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderArm(1, 25, 5);
                drive.followTrajectorySequence(parkOuter);
            } else {
                drive.followTrajectorySequence(trajSeq3);
                robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderPivotAuto(1, robot.PIVOT_UP_POS, 5);
                robot.rightClaw.setPosition(robot.CLAW_OPENED);
                robot.encoderPivotAuto(1, 0, 5);
                robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                robot.encoderArm(1, 25, 5);
                drive.followTrajectorySequence(parkInner);
            }
        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

    }   // end runOpMode()
}   // end class