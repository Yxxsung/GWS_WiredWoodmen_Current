
 /* Copyright (c) 2017 FIRST. All rights reserved.
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

 

 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.DigitalChannel;

 import android.text.method.Touch;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.dfrobot.HuskyLens;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.robotcore.internal.system.Deadline;

 import java.util.concurrent.TimeUnit;

 public abstract class AutoLinearAbstract2023 extends LinearOpMode {



     /* =======================================================
      * CLASS MEMBERS (i.e., Class Status)
      * Common autonomous opmode members
      * ======================================================= */

     /* -------------------------------------------------------
      * Public (Shared) Class Members
      * Automated objects, timers, variables, constants
      * ------------------------------------------------------- */

     // OBJECTS
     MecanumDrive1
             driveTrain;



     //declares timers that we use
     ElapsedTime
             generalTimer = new ElapsedTime(), // General/multipurpose timer
             autoTimer = new ElapsedTime();
     // Autonomous timer
     final double ARM_TICKS_PER_DEGREE =
             28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
     // ARM_TICKS_PER_DEGREE is 19.794
      double ARM_COLLAPSED_INTO_ROBOT = 0;
      double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
      double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
      double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
      double ARM_SCORE_SAMPLE_IN_LOW= 50.933 * ARM_TICKS_PER_DEGREE;
      double ARM_SCORE_SAMPLE_IN_HIGH=101.867 * ARM_TICKS_PER_DEGREE;
      double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
      double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
     IMU imu;

     //True-false variables
     boolean
             safeStop,
             RunAutoInput;
     // any whole number variables
     final int READ_PERIOD = 1;


     // any decimal values
     double
             robotHeading  = 0,
             headingOffset = 0,
             headingError  = 0,
             turnTarget = 0,
             targetHeading = 0,
             hangerPos = 0;


     // constants: declared here and can never be changed further in the program
     // however, changing one of these numbers will change it for all(speeds)
     final double
             MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = .25,
             DRIVE_TRAIN_STRAIGHT_SPEED = 0.9,
             DRIVE_TRAIN_DEFAULT_SPEED = 0.7,
             HEADING_THRESHOLD       = 0.25,
             P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable

     final boolean
             FORWARD = false,
             REVERSE = true;



     Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
     /* =======================================================
      * CLASS METHODS (i.e., Class Behavior)
      * ======================================================= */

     /* -------------------------------------------------------
      * Method: runOpMode (Overridden Linear OpMode)
      * Purpose: Establish and initialize automated objects, and signal initialization completion
      * ------------------------------------------------------- */
     @Override
     public void runOpMode() {

         safeStop = false; //Used for stopping robot
         /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
         driveTrain = new MecanumDrive1(hardwareMap, "frontLeft",REVERSE, "frontRight", FORWARD, "rearLeft", REVERSE, "rearRight", FORWARD, 538, 1, 2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */




         //imu
         imu = hardwareMap.get(IMU.class, "imu");
         double xRotation = 90;  // enter the desired X rotation angle here.
         double yRotation = -90;  // enter the desired Y rotation angle here.
         double zRotation = 0;  // enter the desired Z rotation angle here.
         Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
         RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
         imu.initialize(new IMU.Parameters(orientationOnRobot));

         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
         orientation.getRoll(AngleUnit.DEGREES);


         rateLimit.expire();


         // Notify drive station that robot objects are being initialized
         telemetry.addLine("Wait - Initializing Robot Objects");
         telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
         driveTrain.resetEncoders();


         /* Lock drive train at current position */
         driveTrain.front.motorLeft.goToAbsoluteDistance(driveTrain.front.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
         driveTrain.front.motorRight.goToAbsoluteDistance(driveTrain.front.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
         driveTrain.rear.motorLeft.goToAbsoluteDistance(driveTrain.rear.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
         driveTrain.rear.motorRight.goToAbsoluteDistance(driveTrain.rear.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);





         imu.resetYaw();







         // WAIT FOR THE GAME TO START (driver presses PLAY)
         waitForStart();

         autoTimer.reset();  // Reset/restart the autotimer

         // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

     }








     /* -------------------------------------------------------
      * Method: driveTrainTelemetry
      * Purpose: Report the position and speed of the drive train wheels
      * ------------------------------------------------------- */

     void DriveTrainTelemetry() {

         telemetry.addLine("Top Motor");
         telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorLeft.getPosition());
         telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorLeft.targetPosition);
         telemetry.addData("  Position in Counts", driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
         telemetry.addData("  Target in Counts", driveTrain.front.motorLeft.targetCount);
         telemetry.addData("  Is Busy", driveTrain.front.motorLeft.targetMotor.isBusy());
         telemetry.addData("  Speed", driveTrain.front.leftSpeed);


         telemetry.addLine("Bottom Motor");
         telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorLeft.getPosition());
         telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorLeft.targetPosition);
         telemetry.addData("  Position in Counts", driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
         telemetry.addData("  Target in Counts", driveTrain.rear.motorLeft.targetCount);
         telemetry.addData("  Is Busy", driveTrain.rear.motorLeft.targetMotor.isBusy());
         telemetry.addData("  Speed", driveTrain.rear.leftSpeed);

         telemetry.addLine("Right  Motor");
         telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorRight.getPosition());
         telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorRight.targetPosition);
         telemetry.addData("  Position in Counts", driveTrain.front.motorRight.targetMotor.getCurrentPosition());
         telemetry.addData("  Target in Counts", driveTrain.front.motorRight.targetCount);
         telemetry.addData("  Is Busy", driveTrain.front.motorRight.targetMotor.isBusy());
         telemetry.addData("  Speed", driveTrain.front.rightSpeed);

         telemetry.addLine("Left Motor");
         telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorRight.getPosition());
         telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorRight.targetPosition);
         telemetry.addData("  Position in Counts", driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
         telemetry.addData("  Target in Counts", driveTrain.rear.motorRight.targetCount);
         telemetry.addData("  Is Busy", driveTrain.rear.motorRight.targetMotor.isBusy());
         telemetry.addData("  Speed", driveTrain.rear.rightSpeed);
     }

     void motorTelemetryDegrees (DeviceTargetMotor motor) {
         telemetry.addLine();
         telemetry.addLine(motor.name);
         telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
         telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
     }

     void motorTelemetry (DeviceTargetMotor motor) {
         telemetry.addLine();
         telemetry.addLine(motor.name);
         telemetry.addData(" Position in EU", "%.2f EU ", motor.getPosition());
         telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
     }


     boolean Kill ( double autoTime) {
         boolean eStop;

         if (!opModeIsActive() || autoTimer.seconds() >= autoTime || safeStop) {
             driveTrain.stop();
             //Stops movement of shooter

             eStop = true;
         }
         else
             eStop = false;
         return eStop;
     }

     public void turnToHeading(double heading) {
         int count = 0;
         // Run getSteeringCorrection() once to pre-calculate the current error
         getSteeringCorrection(heading);

         // keep looping while we are still active, and not on heading.
         while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

             // Determine required steering to keep on heading
             turnTarget = getSteeringCorrection(heading);

             count += 1;
             // Pivot in place by applying the turning correction
             driveTrain.turnCwToTarget (turnTarget, .9);
             while (!driveTrain.isMoveDone(0.25)){
                 telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                 telemetry.addData("count",count);
                 telemetry.update();
                 if(autoTimer.seconds() > 28)
                     break;
             }
             if(autoTimer.seconds() > 28)
                 break;
         }

         // Stop all motion;

     }

     public double getSteeringCorrection(double desiredHeading) {
         targetHeading = desiredHeading;  // Save for telemetry

         // Get the robot heading by applying an offset to the IMU heading
         robotHeading = getRawHeading() - headingOffset;

         // Determine the heading current error
         headingError = targetHeading - robotHeading;

         // Normalize the error to be within +/- 180 degrees
         while (headingError > 180)  headingError -= 360;
         while (headingError <= -180) headingError += 360;

         // Determine the required steering correction
         return(headingError);
     }

     public double getRawHeading() {
         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
         return -1*orientation.getYaw(AngleUnit.DEGREES);
     }

     public void resetHeading() {
         // Save a new heading offset equal to the current raw heading.
         headingOffset = getRawHeading();
         robotHeading = 0;
     }


 }