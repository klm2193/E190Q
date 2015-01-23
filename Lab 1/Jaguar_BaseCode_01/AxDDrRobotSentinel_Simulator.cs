using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class AxDDrRobotSentinel_Simulator //: AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel 
    {
        //public Navigation navigation;
        private int encoderPulseL, encoderPulseR;
        private int encoderSpeedL, encoderSpeedR;
        private int encoderDirL, encoderDirR;
        private short actuatorL, actuatorR;

        public AxDDrRobotSentinel_Simulator()
        {
            //navigation = nav;
            Reset();
        }

        public void Reset()
        {
            encoderPulseL = 0;
            encoderPulseR = 0;
            encoderSpeedL = 0;
            encoderSpeedR = 0;
            actuatorL = 0;
            actuatorR = 0;
        }

        public void UpdateSensors(int deltaT)
        {
            encoderDirL = Math.Sign(actuatorL);
            encoderDirR = Math.Sign(actuatorR);
            encoderSpeedL = Math.Abs(actuatorL);
            encoderSpeedR = Math.Abs(actuatorR);
            encoderPulseL = limitEncoder(encoderPulseL + encoderSpeedL * encoderDirL * deltaT/1000);
            encoderPulseR = limitEncoder(encoderPulseR + encoderSpeedR * encoderDirR * deltaT/1000);
        }

        private int limitEncoder(int value)
        {
            if (value > 32767)
                value -= 32767;
            else if (value < 0)
                value += 32767;
            return value;
        }

        public int GetEncoderPulse4()
        {
            return encoderPulseL;
        }

        public int GetEncoderSpeed4()
        {
            return encoderSpeedL;
        }

        public int GetEncoderDir4()
        {
            return encoderDirL;
        }


        public int GetEncoderPulse5()
        {
            return encoderPulseR;
        }

        public int GetEncoderSpeed5()
        {
            return encoderSpeedR;
        }

        public int GetEncoderDir5()
        {
            return encoderDirR;
        }

        public void DcMotorVelocityNonTimeCtrAll(short channel1, short channel2, short channel3, short channel4, short channel5, short channel6)
        {
            actuatorL = channel4;
            actuatorR = channel5;
        }

        public void DcMotorPwmNonTimeCtr(short channelNum, short forwardPWM)
        {
            if (channelNum == 4)
                actuatorL = forwardPWM;
            else if (channelNum == 5)
                actuatorR = forwardPWM;
        }
    }
}
