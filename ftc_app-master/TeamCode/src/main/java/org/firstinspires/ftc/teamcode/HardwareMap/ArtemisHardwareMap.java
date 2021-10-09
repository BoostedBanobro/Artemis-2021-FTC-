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

/**
 * This is the HardwareMap package
 * **/
package org.firstinspires.ftc.teamcode.HardwareMap;

/**
 *Imports physical hardware to manipulate
 * **/
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This program is not an OpMode. Instead it initialize all the hardware including motors and servos and
 * provides various methods on how to get it to move and turn(A helper class)
 * **/
public class ArtemisHardwareMap {

    /**
     * These motors are the 4 mecanum wheels of our robot
     * **/
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    /**
     * DO NOT REMOVE. This hardware map will use the parent hardware map which contains all the names of the parts
     * in which we will use in this class to map and set methods for
     * **/
    HardwareMap hwMap;

    /**
     * This method initializes all the motors and servos using the parent hardware map
     * **/
    public void init(HardwareMap ahwMap) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/
        topLeftDriveMotor = hwMap.get(DcMotor.class, "Top-Left-Motor");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "Bottom-Left-Motor");
        topRightDriveMotor = hwMap.get(DcMotor.class, "Top-Right-Motor");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "Bottom-Right-Motor");

        /**
         * Allow the 4 wheel motors to be run without encoders since we are  doing a time based automonous
         */
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         * Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         */
        topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLefttDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         */
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * The 4 mecanum wheel motors are set to 0 power to keep it from moving when the user presses the INIT button
         */
        topLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);
    }
    /**
     * This method will take in 3 inputs : The Left Stick X/Y and Right Stick X
     * - Left Stick Y will make the robot move forward and backwards ( Positive value will = forwwawrd and Negative value will = backwawrds)
     * - Left Stick X will allow the robot to strafe Left and Right ( Positive value makes topLeft and bottomRight motors run wwhich amkes the robot go right and the negative makes bottomLeft and topRight motors move which makes it go left)
     * - Right STick X allows the robot to turn left or right( Positive value will make the left motors turn more which will make the robot turn right and the negative values will make the right motors turn more which will make it turn left)
    **/
    public void moveRobot(double leftStickY, double leftSTickX, double rightStickX) {
        /**
         * These are the calculations for the Wheel powers using the gamepads 1's inputs leftSTickY, leftStickX, and rightStickX)
         */
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double topRightPower = leftStckY - leftStickX - rightStickX;
        double bottomLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        /**
         * Setting the wheel's power
         */
        topLeftDriveMotor.setPower(topLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
        bottomRightDriveMotor.setPower(bottomRightPower);
    }
