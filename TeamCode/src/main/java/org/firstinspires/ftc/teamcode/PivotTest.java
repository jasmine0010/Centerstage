package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PivotTest extends OpMode {

    private PIDController pivotPID;
    public static double PIVOT_P = 0.05, PIVOT_I = 0.01, PIVOT_D = 0.005;
    public static double PIVOT_F = 0.1;

    public static int PIVOT_TARGET = 150;

    private final double ticks_in_degree = 288 / 360.0;
    public DcMotorEx pivot;

    public void init() {
        pivotPID = new PIDController(PIVOT_P, PIVOT_I, PIVOT_D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
    }

    public void loop() {
        pivotPID.setPID(PIVOT_P, PIVOT_I, PIVOT_D);
        int pivotPos = pivot.getCurrentPosition();
        double pid = pivotPID.calculate(pivotPos, PIVOT_TARGET);
        double ff = Math.cos(Math.toRadians(PIVOT_TARGET / ticks_in_degree)) * PIVOT_F;

        double power = pid + ff;

        pivot.setPower(power);

        telemetry.addData("pivotPos ", pivotPos);
        telemetry.addData("pivotTarget ", PIVOT_TARGET);
        telemetry.update();
    }
}
