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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    String position = "LEFT";
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

        // Score + Set Position
        Trajectory setArm = drive.trajectoryBuilder(null)
                .addDisplacementMarker(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .build();

        // CENTER
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24.00, -21.00, Math.toRadians(180.00)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })

                //.addDisplacementMarker(() -> drive.followTrajectoryAsync(setArm))

                .lineToConstantHeading(new Vector2d(50.00, -37.50))
                //.UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                //    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                //    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                //    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                //    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                //    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                //    robot.clawJoint.setPosition(JOINT_STACK_POS);
                //})
                .lineToConstantHeading(new Vector2d(37.00, -36.00))
                .build();

        // RIGHT
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(29.50, -32.00, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(50.00, -45.00))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .lineToConstantHeading(new Vector2d(37.00, -36.00))
                .build();

        // LEFT
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(6.50, -33.00, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(50.00, -32.00))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .lineToConstantHeading(new Vector2d(37.00, -36.00))
                .build();

        // CYCLE
        // Right Stack Through Door
        TrajectorySequence trajSeqToStackRight = drive.trajectorySequenceBuilder(trajSeq1.end())
                .turn(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-2.50, -11.00, Math.toRadians(180)), Math.toRadians(180))
                // Through Door
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftDoorOpener.setPosition(robot.OPENER_ON);
                    robot.rightDoorOpener.setPosition(robot.OPENER_ON);
                })
                .forward(
                        7,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(-59, -9), Math.toRadians(180))
                // Left Claw
                .forward(
                        4,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                // Right Claw
                .back(4)
                .splineToConstantHeading(
                        new Vector2d(-63, -14),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                // Back to Backdrop
                .lineToConstantHeading(new Vector2d(33, -9))
                .lineToConstantHeading(new Vector2d(50, -40))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .lineToConstantHeading(new Vector2d(37.00, -36.00))
                .build();

        // Left Stack Through Truss
        TrajectorySequence trajSeqToStackLeft = drive.trajectorySequenceBuilder(trajSeq1.end())
                .lineToLinearHeading(new Pose2d(23.00, -59.00, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(-39.50, -59.00))
                .splineToConstantHeading(new Vector2d(-59, -34), Math.toRadians(180))
                .forward(
                        4,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_CLOSED);
                })
                .back(4)
                .splineToConstantHeading(new Vector2d(-63, -39), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.rightClaw.setPosition(robot.CLAW_CLOSED);
                })
                .back(1e-2)
                .splineToConstantHeading(new Vector2d(-39.50, -59.00), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(23, -59.00))
                .lineToConstantHeading(new Vector2d(50.00, -40.00))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.encoderPivot(1, robot.PIVOT_UP_POS, 5);
                    robot.rightClaw.setPosition(robot.CLAW_OPENED);
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                    robot.encoderPivot(1, PIVOT_STACK_POS, 5);
                    robot.clawJoint.setPosition(JOINT_STACK_POS);
                })
                .lineToConstantHeading(new Vector2d(37.00, -36.00))
                .build();

        // Park Outer
        TrajectorySequence trajSeqParkOuter = drive.trajectorySequenceBuilder(trajSeq1.end())
                .splineToConstantHeading(new Vector2d(48.00, -60), Math.toRadians(180))
                .build();

        // Park Inner
        TrajectorySequence trajSeqParkInner = drive.trajectorySequenceBuilder(trajSeq1.end())
                .splineToConstantHeading(new Vector2d(46.00, -10), Math.toRadians(180))
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

                while (opModeIsActive() && (30-runtime.seconds()) > 5) {
                    //drive.followTrajectorySequence(trajSeqToStackRight);
                    drive.followTrajectorySequence(trajSeqToStackLeft);
                    PIVOT_STACK_POS-=3;
                    JOINT_STACK_POS-=0.2;
                }

                drive.followTrajectorySequence(trajSeqParkOuter);
                //drive.followTrajectorySequence(trajSeqParkInner);

            } else if (Objects.equals(position, "RIGHT")) {

                drive.followTrajectorySequence(trajSeq2);

                while (opModeIsActive() && (30-runtime.seconds()) > 5) {
                    //drive.followTrajectorySequence(trajSeqToStackRight);
                    drive.followTrajectorySequence(trajSeqToStackLeft);
                    PIVOT_STACK_POS-=3;
                    JOINT_STACK_POS-=0.2;
                }

                drive.followTrajectorySequence(trajSeqParkOuter);
                //drive.followTrajectorySequence(trajSeqParkInner);

            } else {

                drive.followTrajectorySequence(trajSeq3);

                while (opModeIsActive() && (30-runtime.seconds()) > 5) {
                    drive.followTrajectorySequence(trajSeqToStackRight);
                    //drive.followTrajectorySequence(trajSeqToStackLeft);
                    PIVOT_STACK_POS-=3;
                    JOINT_STACK_POS-=0.2;
                }

                drive.followTrajectorySequence(trajSeqParkOuter);
                //drive.followTrajectorySequence(trajSeqParkInner);

            }
        }

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

    }   // end runOpMode()
}   // end class