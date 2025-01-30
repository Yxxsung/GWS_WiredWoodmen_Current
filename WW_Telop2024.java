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


@TeleOp(name="WW_Telop2024", group="WiredWoodmen")  //declares the name of the class and the
// group it is in.



public class WW_Telop2024 extends OpMode {
     //declares all double variables and their values
        double speedVariable = .7;
         int speedVariable1=0;
    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null,
            liftMotor = null,
            armMotor=null,
            hangmotor=null;
   // ServoImplEx intake1;
    CRServoImplEx flapper;

    // pixelArm = null;
   // double hangerpos = 0.0;

    final double ARM_TICKS_PER_DEGREE =76.036;
           // 28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
    // ARM_TICKS_PER_DEGREE is 19.794
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW= 23.5 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH=21.5 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    double hangerPos;


   double liftPosition = 0;
   /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
   /*final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;*/
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    /*final double FLAPPER_IN   = -1.0;
    final double FLAPPER_OUT  = 1.0;*/


    double armLiftComp = 0;

    double liftpos=0;
    double liftpower=0.7;
    double liftposincrement=23;
    double minliftposition=2;
    double maxliftposition=500;

    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armMotor=hardwareMap.get(DcMotorEx.class,"armMotor");
        flapper = hardwareMap.get(CRServoImplEx.class, "flapper");
        hangmotor=hardwareMap.get(DcMotorEx.class,"hangMotor");
        //Direction?
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // pixelArm.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        liftMotor.setPower(0);
        armMotor.setPower(0);
        hangmotor.setPower(0);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
       // ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        //armMotor.setTargetPosition(0);
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0);


        liftMotor.setTargetPosition(0);
      //  liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0);
        hangmotor.setTargetPosition(0);
        hangmotor.setPower(0);
        telemetryarm();

        /* Define and initialize servos.*/

      //  intake1  = hardwareMap.get(ServoImplEx.class, "intake1");

        /* Make sure that the intake is off, and the wrist is folded in. */
        flapper.setPower(0.0);
       // intake1.setPosition(INTAKE_OFF);

        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetryarm();
        telemetrylift();
        telemetry.update();
        runTime.reset();

    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetryarm();
        telemetrylift();
        telemetryhanger();
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        armMotor.setTargetPosition((int)ARM_CLEAR_BARRIER);
        ((DcMotorEx) armMotor).setVelocity(300);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //  liftMotor.setTargetPosition(0);
        liftMotor.setPower(0.0);
      //  liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetrylift();

    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //==========================================================
        //                        GamePad One
        //==========================================================
        // Retrieve the IMU from the hardware map
        // armRotator manual lifting mostly for testing or as a last resort
        if(gamepad1.right_trigger>.1){
            armMotor.setPower(.75);
            //armMotor.setPower(.75);
        }

        else if(gamepad1.left_trigger>.1){
            armMotor.setPower(-.75);

        }

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }




        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        rearLeft.setPower(Range.clip((-BLspeed * speedVariable), -1, 1));
        rearRight.setPower(Range.clip((BRspeed * speedVariable), -1, 1));
        frontLeft.setPower(Range.clip((FLspeed * speedVariable), -1, 1));
        frontRight.setPower(Range.clip((-FRspeed * speedVariable), -1, 1));


        //DriveTrain Speed Controls
        if (gamepad1.dpad_left) speedVariable -= 0.1;
        if (gamepad1.dpad_right) speedVariable += 0.1;


        speedVariable = Range.clip(speedVariable, 0, 1);
     /*   if (gamepad1.right_trigger > 0.5 && hangerpos < 2000)
            hangerpos += 50;
        if (gamepad1.left_trigger > 0.5 && hangerpos > 40)
            hangerpos -= 40;*/

        telemetry.addLine("Robot Ready.");
        telemetry.update();


        /* Program for slides to raise up and down */
        /* When you press right bumper and left bumper for gamepad2 it will move slides*/
       if (gamepad2.right_bumper  && liftPosition>0) {
            liftPosition -= liftposincrement;
        }
        if(gamepad2.left_bumper && liftPosition<2300){
            liftPosition+=liftposincrement;
        }
        /*here we check to see if the lift is trying to go higher than the maximum extension.
         *if it is, we set the variable to the max.
         */
       if (liftPosition > 2300){
            liftPosition= 2300;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < 0){
            liftPosition = 0;
        }

        liftMotor.setTargetPosition((int) (liftPosition));
        ((DcMotorEx) liftMotor).setVelocity(400);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Code to move arm to certain position
       if (gamepad2.a) {
            /* This is the intaking/collecting arm position */
           armPosition = ARM_COLLECT;

            //liftPosition = LIFT_COLLAPSED;

        } else if (gamepad2.b) {
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
           armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad2.x) {
            /* This is the correct height to score the sample in the HIGH BASKET */
           armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;

       } else if (gamepad2.y){
           armPosition=ARM_SCORE_SAMPLE_IN_HIGH;
       }


            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

      if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
            armLiftComp = (0.25568 * liftPosition);
        } else {
            armLiftComp = 0;
        }

           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
    while (runTime.seconds()<20) {
        armMotor.setTargetPosition((int) (armPosition + armLiftComp + FUDGE_FACTOR));

        ((DcMotorEx) armMotor).setVelocity(700);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
        if (gamepad2.left_trigger>0.1){
            flapper.setPower(-0.7);
            telemetry.addData("Intake Status", "Grabbing");
        }else if (gamepad2.right_trigger>0.1){
            flapper.setPower(0.7);
            telemetry.addData("Intake Status", "Ejecting");
        }else{
            flapper.setPower(0);
            telemetry.addData("Intake Status", "Stopped");
        }

         /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at
            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.
             */
         // Hanger
        //Hanger
        if(gamepad1.right_trigger > .5 && hangerPos<5000) hangerPos += 80;
        if(gamepad1.left_trigger > .5&& hangerPos>10) hangerPos -= 40;
        hangmotor.setTargetPosition((int)hangerPos);


        //==========================================================//
        //                        Telemetry                           //
        //==========================================================//
      // telemetrymotorprint();
        telemetry.addData("lift variable", liftpos);
        telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
        telemetry.addData("Lift motor velocity",liftMotor.getVelocity());
        telemetry.addData("lift current position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(liftMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Lift motor velocity",armMotor.getVelocity());
        telemetry.addData("Arm Motor current",armMotor.getCurrent(CurrentUnit.AMPS));

        telemetry.update();


    }
    public void telemetrymotorprint(){
        telemetry.clear();
        telemetry.addData("Drive Train Speed: " , speedVariable);
        telemetry.addData("BRMotor2", "Position : %2d, Power : %.2f", rearRight.getCurrentPosition(), rearRight.getPower());
        telemetry.addData("FRMotor2", "Position : %2d, Power : %.2f", frontRight.getCurrentPosition(), frontRight.getPower());

        telemetry.addData("FLMotor2", "Position : %2d, Power : %.2f", frontLeft.getCurrentPosition(), frontLeft.getPower());
        telemetry.addData("BLMotor2", "Position : %2d, Power : %.2f", rearLeft.getCurrentPosition(), rearLeft.getPower());

        telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y);
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y);

        // this will send a telemetry message to signify robot waiting
        telemetry.addLine("I 'm Ready");
        telemetry.update();

    }
    public void telemetrylift(){
        telemetry.addData("lift variable", liftpos);
        telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
        telemetry.addData("Lift motor velocity",liftMotor.getVelocity());
        telemetry.addData("lift current position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(liftMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("Arm Position",armPosition);
        telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Arm motor velocity",armMotor.getVelocity());
        telemetry.addData("Arm Motor current",armMotor.getCurrent(CurrentUnit.AMPS));
    }
    public void telemetryhanger(){
        telemetry.addData("hanger Position",hangerPos);
        telemetry.addData("Arm Target Position: ",hangmotor.getTargetPosition());
        telemetry.addData("arm Encoder:",hangmotor.getCurrentPosition());
        telemetry.addData("Arm motor velocity",hangmotor.getVelocity());
        telemetry.addData("Arm Motor current",hangmotor.getCurrent(CurrentUnit.AMPS));
    }

      //Code will run ONCE after the driver hits STOP
        @Override
        public void stop() {
            // Sets all motors to zero power except Arms to keep pos
            frontLeft.setPower(0);
            rearLeft.setPower(0);
            frontRight.setPower(0);
            rearRight.setPower(0);
            armMotor.setTargetPosition((int)ARM_CLEAR_BARRIER);
            armMotor.setVelocity(200);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0);
            flapper.setPower(0);
            hangmotor.setPower(0);
        }
        // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());

    }
