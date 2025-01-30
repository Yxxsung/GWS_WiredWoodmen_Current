package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoTesting extends OpMode {
    ServoImplEx claw, clawRot;

    @Override
    public void init() {
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        clawRot = hardwareMap.get(ServoImplEx.class, "clawPivot");
    }

    @Override
    public void loop() {
        if (gamepad1.a) claw.setPosition(0);
        if (gamepad1.b) claw.setPosition(0.25);
        if (gamepad1.left_bumper) claw.getController().pwmDisable();
        if (gamepad1.left_trigger > 0.75) claw.getController().pwmEnable();

        if (gamepad1.dpad_down) clawRot.setPosition(0.5);
        if (gamepad1.dpad_right) clawRot.setPosition(1);
        if (gamepad1.dpad_left) clawRot.setPosition(0);
        if (gamepad1.right_bumper) clawRot.getController().pwmDisable();
        if (gamepad1.right_trigger > 0.75) clawRot.getController().pwmEnable();
        telemetry.addData("claw cont", claw.getController());
        telemetry.addData("claw rot cont", clawRot.getController());
        telemetry.addData("Claw pwm", claw.isPwmEnabled());
        telemetry.addData("Claw Rot pwm", clawRot.isPwmEnabled());
        telemetry.update();
    }
}
