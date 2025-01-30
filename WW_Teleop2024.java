package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="WW_Teleop2024", group="WiredWoodmen")  //declares the name of the class and the
// group it is in.



public class WW_Teleop2024 extends OpMode {




    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotorEx

            liftMotor = null,
            armMotor=null;
    ServoImplEx intake1;
    CRServoImplEx flapper;

    // pixelArm = null;
    // double hangerpos = 0.0;
    final double ARM_TICKS_PER_DEGREE =
            28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;

    // we want ticks per degree, not per rotation
    /* These constants hold the position that the arm is commanded to run to.
   These are relative to where the arm was located when you start the OpMode. So make sure the
   arm is reset to collapsed inside the robot before you start the program.

   In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
   This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
   set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
   160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
   If you'd like it to move further, increase that number. If you'd like it to not move
   as far from the starting position, decrease it. */
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    // final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_TICKS_PER_MM= 9.27;
    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 356 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
   /*final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;*/
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    /*final double FLAPPER_IN   = -1.0;
    final double FLAPPER_OUT  = 1.0;*/


    double armLiftComp = 0;
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    double liftpos=0;
    double liftpower=0.7;
    double liftposincrement=1.1;
    double minliftposition=2;
    double maxliftposition=500;

    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armMotor=hardwareMap.get(DcMotorEx.class,"armMotor");


//Direction?


        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // pixelArm.setDirection(DcMotor.Direction.REVERSE);


        liftMotor.setPower(0);
        armMotor.setPower(0);


        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        flapper = hardwareMap.get(CRServoImplEx.class, "flapper");
        intake1  = hardwareMap.get(ServoImplEx.class, "intake1");

        /* Make sure that the intake is off, and the wrist is folded in. */
        flapper.setPower(0.0);
        // intake1.setPosition(INTAKE_OFF);

        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetrylift();

        telemetry.update();
        runTime.reset();

    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0.6);
        telemetrylift();

    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {



     /*   if (gamepad1.right_trigger > 0.5 && hangerpos < 2000)
            hangerpos += 50;
        if (gamepad1.left_trigger > 0.5 && hangerpos > 40)
            hangerpos -= 40;*/
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        // ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
         /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
    /*    armMotor.setTargetPosition(speedVariable1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

         /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/
        if (gamepad2.right_bumper){
            liftPosition += 2800 * cycletime;
        }
        else if (gamepad2.left_bumper){
            liftPosition -= 2800 * cycletime;
        }
        /* Program for slides to raise up and down */
     /*  if (gamepad2.right_bumper  && liftpos>minliftposition) {
            liftpos -= liftposincrement;
        }
        if(gamepad2.left_bumper && liftpos<200){
            liftpos+=liftposincrement;
        }*/
        /*here we check to see if the lift is trying to go higher than the maximum extension.
         *if it is, we set the variable to the max.
         */
        if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET){
            liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < 0){
            liftPosition = 0;
        }

        liftMotor.setTargetPosition((int) (liftPosition));
        // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) liftMotor).setVelocity(2100);

        //  liftMotor.setPower(0.7);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;


        //==========================================================//
        //                        Telemetry                           //
        //==========================================================//

        telemetrylift();
        try {
            runTime.wait(10000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetryarm();
        try {
            runTime.wait(10000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


    }

    public void telemetrylift(){
        telemetry.addData("lift variable", liftPosition);
        telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
        telemetry.addData("Lift motor velocity",liftMotor.getVelocity());
        telemetry.addData("lift current position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(liftMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Lift motor velocity",armMotor.getVelocity());
        telemetry.addData("Arm Motor current",armMotor.getCurrent(CurrentUnit.AMPS));
    }




    //Code will run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Sets all motors to zero power except Arms to keep pos

        liftMotor.setPower(0);
    }
    // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());

}
