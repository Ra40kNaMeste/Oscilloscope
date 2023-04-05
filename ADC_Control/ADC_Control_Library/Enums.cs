using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ADC_Control_Library
{
    public enum ADCProperty
    {
        ChopMode,
        Delay,
        PolarMeasure,
        ID,
        Series,
        VoltageReferenceToAINOUT,
        VoltageReferenceToINTREV,
        CheckSum,
        StatusBit,
        Timeout,
        SBMAG,
        SBPOL,
        ADC,
        Filter,
        FilterFrenches,
        GAIN,
        InsideAmpfiler,
        IDACMUX1,
        IDACMUX2,
        IDACMAG1,
        IDACMAG2,
        VoltageReferenceNByChannel,
        VoltageReferencePByChannel,
        DACPOutput,
        DACPValue,
        DACNOutput,
        DACNValue,
        GPIOSelect,
        GRIOMode,
        GpioData
    }

    public enum ADCPropertyMode
    {
        ReadOnly,
        WriteOnly,
        ReadAndWrite
    }

    public enum ChopMode
    {
        Off = 0,
        OnChop = 1,
        OnIDAC = 2,
        OnChopAndIDac = 3
    }
    public enum PolarMeasure
    {
        Straight = 0,
        Reverse = 1
    }
    public enum OnAndOffValue
    {
        Off = 0,
        On = 1
    }
    public enum ADCLastBit
    {
        None = 0,
        CheckSum = 1,
        CRC = 2
    }
    public enum SBMAG
    {
        Off = 0,
        A05mc = 1,
        A2mc = 2,
        A10mc = 3,
        A200mc = 4,
        R10M = 5
    }
    public enum PullResistor
    {
        AINPUP = 0,
        AINNUP = 1
    }
    public enum ADC
    {
        ADC1 = 0,
        ADC2 = 1
    }
    public enum Filter
    {
        sinc1 = 0,
        sinc2 = 1,
        sinc3 = 2,
        sinc4 = 3,
        FIRMode = 4
    }
    public enum InsideAmpfiler
    {
        On = 0,
        Off = 1
    }
    public enum InputPorts
    {
        AIN0 = 0,
        AIN1 = 1,
        AIN2 = 2,
        AIN3 = 3,
        AIN4 = 4,
        AIN5 = 5,
        AIN6 = 6,
        AIN7 = 7,
        AIN8 = 8,
        AIN9 = 9,
        AINCOM = 10
    }
    public enum VoltageRefencesByPort
    {
        GenerateInside = 0,
        AIN1 = 1,
        AIN3 = 2,
        AIN4 = 3,
        VAVSS = 4
    }
    public enum DACPOutput
    {
        None = 0,
        AIN6 = 1
    }
    public enum DACNOutput
    {
        None = 0,
        AIN7 = 1
    }
    public enum GPIO
    {
        AIN3 = 0,
        AIN4 = 1,
        AIN5 = 2,
        AIN6 = 3,
        AIN7 = 4,
        AIN8 = 5,
        AIN9 = 6,
        AINOUT = 7
    }

}
