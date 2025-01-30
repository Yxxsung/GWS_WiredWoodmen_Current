package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="WW_AutoBR2024", group="Autonomous")
public class WW_AutoBR2024 extends AutoLinearAbstract2023 {
    DcMotorEx armMotor,
            liftMotor;

    CRServoImplEx flapper;

    ElapsedTime waitTimer;

    public void wait(double waitTime) {
        waitTimer = new ElapsedTime();
        // waitTimer.reset();
        while (waitTimer.seconds() < waitTime) {
        }
        waitTimer.reset();
    }

    public void grabber() {
        flapper.setPower(0.7);
        wait(0.5);
    }

    public void eject() {
        flapper.setPower(-0.7);
        wait(1.0);
    }

    public void stop1() {
        flapper.setPower(0);
        wait(1.0);
    }

    public void upperBasket() {
        armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_HIGH);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.7);
        wait(1.5);
    }

    public void lowerBasket() {
        armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_LOW);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.7);
        wait(1.7);
    }

    public void slowerBasket() {
        liftMotor.setTargetPosition(1400);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
        wait(0.7);
    }

    public void groundArm() {
        armMotor.setTargetPosition((int) ARM_CLEAR_BARRIER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.7);
        wait(0.7);
    }

    public void supperBasket() {
        liftMotor.setTargetPosition(2000);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
        wait(0.7);
    }

    // Declare OpMode members specific to this Autonomous Opmode variant.
    public void sstop() {
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(-0.7);
        wait(0.7);
    }

    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        RunAutoInput = true;
        super.runOpMode();
        //int ARM_UP = 180;

        //  pixelArm.setTargetPosition(10);
        // objectLocation = scanLocation();
        //Now to take a nice cozy sleep zzzzz
        // sleep(600);
        // telemetry.addData("objectLocation",objectLocation);
        // telemetry.update();


        // turnToHeading(0);


        driveTrain.goStraightToTarget(24.5, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Going to place pixel on spike");
            if (Kill(28)) {
                break;
            }
        }
        turnToHeading(-90);
        driveTrain.goStraightToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Going to place pixel on spike");
            if (Kill(28)) {
                break;
            }
        }
        // waitTimer.wait(10);
        // wait(200);
        upperBasket();
        supperBasket();
        eject();
        stop1();




        telemetry.addLine("Autonomous Done");
        telemetry.update();
    }
}



}
