/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Sensor: HuskyLens", group = "Sensor")

public class SensorHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    //Add instance of RobotHardware
    RobotHardware robot;

    @Override
    public void runOpMode()
    {
        //Create a runtime object so we can time loops
        ElapsedTime runtime = new ElapsedTime(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot = new RobotHardware(this);
        //We need to initialize the robot hardware
        robot.init();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        robot.clawJoint.setPosition(robot.CLAW_JOINT_DOWN);
        robot.rightClaw.setPosition(robot.CLAW_CLOSED);
        robot.leftClaw.setPosition(robot.CLAW_CLOSED);

        // TRAJECTORIES

        Pose2d startPose = new Pose2d(14.50, -63.00, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // CENTER
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24.00, -25.00, Math.toRadians(180.00)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    robot.leftClaw.setPosition(robot.CLAW_OPENED);
                })
                .lineToConstantHeading(new Vector2d(48.00, -41.00))
                .build();

        // PARK
        TrajectorySequence parkOuter = drive.trajectorySequenceBuilder(trajSeq1.end())
                .forward(4)
                .strafeLeft(30)
                .back(15)
                .build();

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        if(opModeIsActive()) {

            //move
            //drive.followTrajectorySequence(trajSeq1);
            robot.leftClaw.setPosition(robot.CLAW_CLOSED);
            robot.pivotPID(200);

            //this loop detects for 5 seconds, storing the outcome
            runtime.reset();

            while(opModeIsActive() && runtime.seconds() < 5) {
                if (!rateLimit.hasExpired()) {
                    continue;
                }
                rateLimit.reset();

                /*
                 * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
                 * Block represents the outline of a recognized object along with its ID number.
                 * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
                 * referenced in the header comment above for more information on IDs and how to
                 * assign them to objects.
                 *
                 * Returns an empty array if no objects are seen.
                 */
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Block count", blocks.length);

                for (HuskyLens.Block block : blocks) {
                    telemetry.addData("Block", block.toString());
                }

                if (blocks.length != 0 && blocks[0].x > 80) {
                    SampleMecanumDrive.leftFront.setPower(-0.3);
                    SampleMecanumDrive.rightFront.setPower(0.3);
                    SampleMecanumDrive.leftRear.setPower(0.3);
                    SampleMecanumDrive.rightRear.setPower(-0.3);
                } else {
                    SampleMecanumDrive.leftFront.setPower(0);
                    SampleMecanumDrive.rightFront.setPower(0);
                    SampleMecanumDrive.leftRear.setPower(0);
                    SampleMecanumDrive.rightRear.setPower(0);
                }

                telemetry.update();
            }

            //robot.pivotPID(330);
            //robot.encoderDrive(0.5, -4, 5);
            //robot.rightClaw.setPosition(robot.CLAW_OPENED);
            //sleep(500);
            //robot.pivotPID(0);
            //robot.rightClaw.setPosition(robot.CLAW_CLOSED);
            //drive.followTrajectorySequence(parkOuter);

        }
    }
}