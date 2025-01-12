/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//    private DcMotor towerMotor = null;
//    private DcMotor flipperMotor = null;
//    private DcMotor hookMotor = null;
//    private Servo claw0 = null;
//    private Servo claw1 = null;


//    @Override
    private double speedControl(double inputSpeed)
    {
        double temp = Math.abs(inputSpeed);
        double outputSpeed = (((temp*temp) - 0.15*temp)) / 1.25;

        return inputSpeed < 0 ? outputSpeed * -1 : outputSpeed;
    }
    public void runOpMode() {

        RobotHardware robot = new RobotHardware();

        robot.init(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
//        towerMotor = hardwareMap.get(DcMotor.class, "tower_motor");
//        flipperMotor = hardwareMap.get(DcMotor.class, "flipper_motor");
//        hookMotor = hardwareMap.get(DcMotor.class, "hook_motor");

//        claw0 = hardwareMap.get(Servo.class, "flipper_left");
//        claw1 = hardwareMap.get(Servo.class,"flipper_right");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double towerDownPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  speedControl(-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            double lateral =  speedControl(gamepad1.left_stick_x);
            double yaw     =  speedControl(gamepad1.right_stick_x);
            //double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            //double lateral =  gamepad1.left_stick_x;
            //double yaw     =  gamepad1.right_stick_x;
            boolean tower_up = gamepad2.dpad_up;
            boolean tower_down = gamepad2.dpad_down;
            boolean pixel_in = gamepad2.a;
            boolean pixel_out = gamepad2.y;
            boolean flipper_up = gamepad2.left_bumper;
            boolean flipper_down = gamepad2.right_bumper;
            boolean hook_up = gamepad1.x;
            boolean hook_release1 = gamepad1.dpad_left;
            boolean hook_release2 = gamepad2.dpad_left;
            boolean hook_servo = gamepad1.b;
            boolean airplane = gamepad1.a;
            boolean resetEncoderLeft = gamepad2.left_stick_button;
            boolean resetEncoderRight = gamepad2.right_stick_button;
            boolean flipper_something = gamepad2.b;
            boolean tower_encoder_value = gamepad2.x;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //airplane Launcher
            if(airplane) {
                robot.airplaneServo.setPosition(0);
            } else {
                robot.airplaneServo.setPosition(.5);

                if (hook_servo) {
                    robot.hookServo.setPosition(0);
                } else {
                    robot.hookServo.setPosition(.5);
                }

                //pixel operation
                if (pixel_in) {
                    robot.flipperServo.setPosition(1);
//                robot.claw1.setPosition(0);
                    sleep(10);
                } else if (pixel_out) {
                    robot.flipperServo.setPosition(0);
//                robot.claw1.setPosition(1);
                    sleep(10);
                } else {
                    robot.flipperServo.setPosition(0.5);
//                robot.claw1.setPosition(0.5);
                }


                //hooking winch
                //TODO: add limit switch at bottom of tower
                if (hook_up) {
                    robot.hookMotor.setPower(-1);
                } else if (hook_release1 && hook_release2){
                    robot.hookMotor.setPower(1);
                }   else robot.hookMotor.setPower(0);


                //if limit switch is active then reset encoder
                

                if(!robot.touchSensor.isPressed() && gamepad2.dpad_right)
                {
                    towerDownPosition = robot.towerMotor.getCurrentPosition();
                    telemetry.addData("tower position:", "%7d", robot.towerMotor.getCurrentPosition());
                    telemetry.update();

                }else{}
                //TODO determine what the number of encoder counts is to lift tower from bottom to top
                //if (tower_up && (robot.towerMotor.getCurrentPosition() - towerDownPosition < 10000)
                if (tower_up && robot.towerMotor.getCurrentPosition() < 8500) {
                    robot.towerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.towerMotor.setPower(1);
//                    telemetry.addData("tower position:", "%7d", robot.towerMotor.getCurrentPosition());
//                    telemetry.update();

                }else if(tower_down && !robot.touchSensor.isPressed()){
                    robot.towerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.towerMotor.setPower(-.75);
                }else {
                    if(!robot.towerMotor.isBusy()){
                        robot.towerMotor.setPower(0);}
                    else{}
                }

                if(robot.touchSensor.isPressed()) {
                    robot.towerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }else {}

                if(tower_encoder_value){
//                    robot.towerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("tower position:", "%7d", robot.towerMotor.getCurrentPosition());
                    telemetry.update();
                }



//                if(resetEncoderLeft && resetEncoderRight){
//                    robot.flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    telemetry.addData("current:", "%7d", robot.flipperMotor.getCurrentPosition());
//                    telemetry.update();
//                }else {}
                if (flipper_up) {
                    robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.flipperMotor.setPower(.5);
                } else if (flipper_down) {
                    robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.flipperMotor.setPower(-0.25);
//                } else robot.flipperMotor.setPower(0);
//                }else if (flipper_something) {
//                    int newTargetPosition = (robot.flipperMotor.getCurrentPosition());
//                    robot.flipperMotor.setTargetPosition(60);
//                    robot.flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.flipperMotor.setPower(1);
//                while (opModeIsActive() && robot.flipperMotor.isBusy()) {
//                }
//                    telemetry.addData("current:", "%7d", robot.flipperMotor.getCurrentPosition());
//                    telemetry.update();
            } else robot.flipperMotor.setPower(0);
                //            } else if (flipper_travel) {
//                int newTargetPosition = (robot.flipperMotor.getCurrentPosition());
//                robot.flipperMotor.setTargetPosition(5500);
//                robot.flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.flipperMotor.setPower(1);
////                while (opModeIsActive() && robot.flipperMotor.isBusy()) {
////                }
//                telemetry.addData("current:", "%7d", robot.flipperMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            else if (flipper_home) {
//                int newTargetPosition = 0;
//                robot.flipperMotor.setTargetPosition(newTargetPosition);
//                robot.flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.flipperMotor.setPower(1);
////                while (opModeIsActive() && robot.flipperMotor.isBusy()) {
////                }
//                telemetry.addData("current:", "%7d", robot.flipperMotor.getCurrentPosition());
//                telemetry.update();





                // Send calculated power to wheels
                robot.leftFrontDrive.setPower(leftFrontPower);
                robot.rightFrontDrive.setPower(rightFrontPower);
                robot.leftBackDrive.setPower(leftBackPower);
                robot.rightBackDrive.setPower(rightBackPower);

                // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
            }
        }
    }
}
