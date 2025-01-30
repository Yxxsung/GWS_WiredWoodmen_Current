package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class PIDF_ARM1 extends OpMode {
    private PIDController controller;
    public static double p=0,i=0,d=0;
    public static double f=0;
    public static int target=0;
    private final double ticks_in_degree=537.6/360;
    private DcMotorEx liftMotor;

    @Override
    public void init() {
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor=hardwareMap.get(DcMotorEx.class,"liftMotor");


    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int liftPos=liftMotor.getCurrentPosition();
        double pid= controller.calculate(liftPos,target);
        double ff=Math.cos(Math.toRadians(target/ticks_in_degree))*f;
        double power=pid+ff;
        liftMotor.setPower(power);
        telemetry.addData("position",liftPos);
        telemetry.addData("target",target);
        telemetry.update();

    }
}