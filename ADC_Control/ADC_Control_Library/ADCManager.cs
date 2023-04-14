using ADC_Control_Library.Properties;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace ADC_Control_Library
{

    enum Commands
    {
        TestMirror = 0,
        Reset = 1,
        ConvertToTime = 2,
        Convert = 3,
        CalibrationInside = 4,
        CalibrationOutside = 5,
        CalibrationScale = 6,
        SelectInput = 7,
        ReadAllPropertiesADC = 8,
        ReadPropertyAdc = 9,
        SetProperty = 10,
        CancelConvert = 11,
        RewindMonochrome = 12,
        StartOnlyMonochrome = 13
    }
    public sealed class ADCManager : INotifyPropertyChanged, IDisposable
    {
        public ADCManager()
        {
            Port = new();
            Port.ReadTimeout = 500;
            Port.WriteTimeout = 500;
            TokenSource = new();
            ADCProperties = new();
            SetProperties();
            UpdatePorts();
        }

        public ADCManager(ILogService logger) : this()
        {
            LogService = logger;
        }

        #region ADCProperties
        public List<IPropertiesTableElementable> ADCProperties { get; init; }


        private void SetProperties()
        {
            ADCProperties.Clear();
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.ChopMode, typeof(ChopMode)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.Delay, new()
            {
                {0, 0 },
                {0.0087, 1 },
                {0.017, 2},
                {0.035, 3 },
                {0.069, 4 },
                {0.139, 5 },
                {0.278, 6 },
                {0.555, 7 },
                {1.1, 8 },
                {2.2, 9 },
                {4.4, 10 },
                {8.8, 11 }
            }));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.PolarMeasure, typeof(PolarMeasure)));
            ADCProperties.Add(new PropertiesTableByteElement(ADCProperty.ID, ADCPropertyMode.ReadOnly));
            ADCProperties.Add(new PropertiesTableByteElement(ADCProperty.Series, ADCPropertyMode.ReadOnly));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.VoltageReferenceToAINOUT, typeof(OnAndOffValue)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.VoltageReferenceToINTREV, typeof(OnAndOffValue)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.CheckSum, typeof(ADCLastBit)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.StatusBit, typeof(OnAndOffValue)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.Timeout, typeof(OnAndOffValue)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.SBMAG, typeof(SBMAG)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.SBPOL, typeof(PullResistor)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.ADC, typeof(ADC)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.Filter, typeof(Filter)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.FilterFrenches, new()
            {
                {2.5, 0 },
                {5, 1 },
                {10, 2 },
                {16.6, 3 },
                {20, 4 },
                {50, 5 },
                {60, 6 },
                {100, 7 },
                {400, 8 },
                {1200, 9 },
                {2400, 10 },
                {4800, 11 },
                {7200, 12},
                {14400, 13 },
                {19200, 14 },
                {38400, 15 }
            }));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.GAIN, new()
            {
                {1, 0 },
                {2, 1 },
                {4, 2 },
                {8, 3 },
                {16, 4 },
                {32, 5 },
            }));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.InsideAmpfiler, typeof(InsideAmpfiler)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.IDACMUX1, typeof(InputPorts)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.IDACMAG1, new()
            {
                {0, 0 },
                {50, 1 },
                {100, 2 },
                {250, 3 },
                {500, 4 },
                {750, 5 },
                {1000, 6 },
                {1500, 7 },
                {2000, 8 },
                {2500, 9 },
                {3000, 10 }
            }));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.IDACMUX2, typeof(InputPorts)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.IDACMAG2, new()
            {
                {0, 0 },
                {50, 1 },
                {100, 2 },
                {250, 3 },
                {500, 4 },
                {750, 5 },
                {1000, 6 },
                {1500, 7 },
                {2000, 8 },
                {2500, 9 },
                {3000, 10 }
            }));

            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.VoltageReferencePByChannel, typeof(VoltageRefencesByPort)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.VoltageReferenceNByChannel, typeof(VoltageRefencesByPort)));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.DACPOutput, typeof(DACPOutput)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.DACPValue, new()
            {
                {4.5, 9 },
                {3.5, 8 },
                {3, 7 },
                {2.75, 6 },
                {2.625, 5 },
                {2.5625, 4 },
                {2.53125, 3 },
                {2.515625, 2 },
                {2.5078125, 1 },
                {2.5, 0 },
                {2.4921875, 17 },
                {2.484375, 18 },
                {2.46875, 19 },
                {2.4375, 20 },
                {2.375, 21 },
                {2.25, 22 },
                {2, 23 },
                {1.5, 24 },
                {0.5, 25 }
            }));
            ADCProperties.Add(new PropertiesTableElementByEnum(ADCProperty.DACNOutput, typeof(DACNOutput)));
            ADCProperties.Add(new PropertiesTableDoubleValueElement(ADCProperty.DACNValue, new()
            {
                {4.5, 9 },
                {3.5, 8 },
                {3, 7 },
                {2.75, 6 },
                {2.625, 5 },
                {2.5625, 4 },
                {2.53125, 3 },
                {2.515625, 2 },
                {2.5078125, 1 },
                {2.5, 0 },
                {2.4921875, 17 },
                {2.484375, 18 },
                {2.46875, 19 },
                {2.4375, 20 },
                {2.375, 21 },
                {2.25, 22 },
                {2, 23 },
                {1.5, 24 },
                {0.5, 25 }
            }));

        }
        #endregion //ADCProperties

        #region SettingUARTPort
        public SerialPort Port { get; init; }

        private List<string>? ports;
        public List<string>? Ports
        {
            get => ports;
            private set
            {
                ports = value;
                OnPropertyChanged();
            }
        }
        #endregion //SettingUARTPort

        #region Commands
        public void UpdatePorts()
        {
            Ports = SerialPort.GetPortNames().ToList();
        }
        public bool TestMirror(ushort value, CancellationToken token)
        {
            try
            {
                if (!Port.IsOpen)
                {
                    LogService?.Write(Resources.LogErrorOpenPort, LogLevels.Error);
                    throw new PortClosedException();
                }
                LogService?.Write(string.Format(Resources.LogTestMirrorSend, value), LogLevels.Info);

                ADCDataReader = new Int32DataFromPortReader();
                var task = ADCDataReader.Read(Port, token);
                task.Start();
                SendMessage(Commands.TestMirror, value);
                task.Wait();
                int res = (int)task.Result >> 16;
                LogService?.Write(string.Format(Resources.LogTestMirrorReceived, res), LogLevels.Info);
                return res == value;
            }
            catch (TimeoutException)
            {
                return false;
            }
            catch (Exception)
            {

                throw;
            }

        }

        public void Reset()
        {
            LogService?.Write(Resources.LogResetStart, LogLevels.Info);
            SendMessage(Commands.Reset, 0);
            LogService?.Write(Resources.LogResetFinish, LogLevels.Info);
        }
        public async Task<List<Point>> ConvertToTimeAsync(short time, CancellationToken token)
        {
            Task<List<Point>> task = new(() => ConvertToTime(time, token));
            await task;
            return task.Result;
        }
        public List<Point> ConvertToTime(short time, CancellationToken token)
        {
            LogService?.Write(Resources.LogConvertStart, LogLevels.Info);
            if (!Port.IsOpen)
            {
                LogService?.Write(Resources.LogErrorOpenPort, LogLevels.Error);
                throw new PortClosedException();
            }
            using CancellationTokenSource source = new();
            if (!(ADCDataReader is GraphdataFromPortReader))
                ADCDataReader = new GraphdataFromPortReader();
            SendMessage(Commands.ConvertToTime, (uint)time);
            Task<List<Point>> task = ConvertGraphAsync(source.Token);
            task.Start();
            for (int i = 0; i < time; i++)
            {
                Thread.Sleep(1);
                if (token.IsCancellationRequested)
                    break;
            }
            source.Cancel();
            task.Wait();
            LogService?.Write(Resources.LogConvertFinish, LogLevels.Info);
            return task.Result;
        }

        public void UpdateFromADCProperties()
        {
            LogService?.Write(Resources.LogUpdateFromADCPropertiesStart, LogLevels.Info);
            SendMessage(Commands.ReadAllPropertiesADC, 0);
            var ADCprops = ReadPropertiesFromADC();
            LogService?.Write(Resources.LogComparerADCPropertiesStart, LogLevels.Debug);
            foreach (var item in ADCprops)
            {
                var property = ADCProperties.Where(i => i.Property == item.Item1).First();
                property.SelectValue = property.ConvertCodeToValue((byte)item.Item2);
            }
            LogService?.Write(Resources.LogComparerADCPropertiesFinish, LogLevels.Debug);
            LogService?.Write(Resources.LogUpdateFromADCPropertiesFinish, LogLevels.Info);
        }
        public void WriteToADCProperties()
        {
            LogService?.Write(Resources.LogWritePropertiesStart, LogLevels.Info);
            if (!Port.IsOpen)
            {
                LogService?.Write(Resources.LogErrorOpenPort, LogLevels.Error);
                throw new PortClosedException();
            }
            var ADCProps = ReadPropertiesFromADC();

            var propsForWrite = ADCProperties.Where(i => i.SelectValue != null &&
                ADCProps.Where(j => j.Item1 == i.Property).First().Item2 != i.ConvertValueToCode(i.SelectValue));
            LogService?.Write(string.Format(Resources.LogWriteCountChangedPorperties, propsForWrite.Count()), LogLevels.Debug);
            foreach (var item in propsForWrite)
            {
                LogService?.Write(string.Format(Resources.LogWritePropertyStart, item.Property.ToString()), LogLevels.Trace);
                SendMessage(Commands.SetProperty, item.ConvertValueToCode(item.SelectValue!));
                LogService?.Write(string.Format(Resources.LogWritePorpertyFinish, item.Property.ToString()), LogLevels.Trace);
            }
            LogService?.Write(Resources.LogWriteProperiesFinish, LogLevels.Info);
        }
        public int Convert(CancellationToken token)
        {
            LogService?.Write(Resources.LogConvertStart, LogLevels.Debug);
            if (!(ADCDataReader is Int32DataFromPortReader))
                ADCDataReader = new Int32DataFromPortReader();
            var task = ADCDataReader.Read(Port, token);
            task.Start();
            SendMessage(Commands.Convert, 0);
            task.Wait();

            int res = (int)task.Result;
            LogService?.Write(Resources.LogConvertFinish, LogLevels.Info);
            return res;
        }
        public void CalibrationInside()
        {
            LogService?.Write(Resources.LogCalibrationInsideStart, LogLevels.Info);
            SendMessage(Commands.CalibrationInside, 0);
            LogService?.Write(Resources.LogCalibrationInsideFinish, LogLevels.Info);
        }
        public void CalibrationOutside()
        {
            LogService?.Write(Resources.LogCalibrationOutsideStart, LogLevels.Info);
            SendMessage(Commands.CalibrationOutside, 0);
            LogService?.Write(Resources.LogCalibrationOutsideFinish, LogLevels.Info);
        }
        public void CalibrationScale()
        {
            LogService?.Write(Resources.LogCalibrationScaleStart, LogLevels.Info);
            SendMessage(Commands.CalibrationScale, 0);
            LogService?.Write(Resources.LogCalibrationScaleFinish, LogLevels.Info);
        }
        public void StartMonochrome(ushort time, CancellationToken token)
        {
            if (!Port.IsOpen)
            {
                LogService?.Write(Resources.LogErrorOpenPort, LogLevels.Error);
                throw new PortClosedException();
            }
            if (!(ADCDataReader is GraphdataFromPortReader))
                ADCDataReader = new GraphdataFromPortReader();
            SendMessage(Commands.StartOnlyMonochrome, time); ;
            //for (int i = 0; i < time; i++)
            //{
            //    Thread.Sleep(1);
            //    if (token.IsCancellationRequested)
            //        break;
            //}
            LogService?.Write(Resources.LogStartMonochrome, LogLevels.Info);
        }
        public void RewindMonochrome(ushort time)
        {
            SendMessage(Commands.RewindMonochrome, time);
        }

        #endregion //Commands
        private void SendMessage(Commands command, uint dates)
        {
            if (!Port.IsOpen)
            {
                LogService?.Write(Resources.LogErrorOpenPort, LogLevels.Error);
                throw new PortClosedException();
            }
            uint com = (uint)command;
            dates <<= 16;
            dates |= com;
            LogService?.Write(string.Format(Resources.LogSendMessageStart, dates.ToString()), LogLevels.Trace);
            if (!Port.IsOpen)
            {
                LogService?.Write(Resources.LogSendMessageNotOpenPort, LogLevels.Error);
                throw new PortClosedException();
            }

            byte[] buffer = BitConverter.GetBytes(dates);
            Port.Write(buffer, 0, 4);
            LogService?.Write(string.Format(Resources.LogSendMessageSuccessfully, dates.ToString()), LogLevels.Trace);
        }

        private async Task<List<Point>> ConvertGraphAsync(CancellationToken token)
        {
            if (!(ADCDataReader is GraphdataFromPortReader))
                ADCDataReader = new GraphdataFromPortReader();
            List<Point> res = new();
            var task = ADCDataReader.Read(Port, token);
            await task;
            return (List<Point>)task.Result;
        }
        private IEnumerable<(ADCProperty, int)> ReadPropertiesFromADC()
        {
            var props = Enum.GetValues(typeof(ADCProperty));
            List<(ADCProperty, int)> ADCprops = new();
            LogService?.Write(Resources.LogReadStartPropertiesFromADC, LogLevels.Debug);
            foreach (ADCProperty prop in props)
            {
                ADCprops.Add((prop, Port.ReadByte()));
            }
            LogService?.Write(Resources.LogReadFinishPropertiesFromADC, LogLevels.Debug);
            return ADCprops;
        }

        private CancellationTokenSource TokenSource { get; init; }

        private ILogService? LogService { get; init; }

        public void Dispose()
        {
            TokenSource.Cancel();
            TokenSource.Dispose();
        }

        private IDataFromPortReaderable? ADCDataReader { get; set; }

        private void OnPropertyChanged([CallerMemberName] string name = "") => PropertyChanged?.Invoke(this, new(name));
        public event PropertyChangedEventHandler? PropertyChanged;
    }


    public enum LogLevels
    {
        Trace, Debug, Info, Warning, Error
    }
    public interface ILogService
    {
        public void Write(string message, LogLevels level);
    }
    public class PortClosedException : Exception
    {
    }
    interface IDataFromPortReaderable
    {
        public Task<object> Read(SerialPort port, CancellationToken token);
    }
    class Int32DataFromPortReader : IDataFromPortReaderable
    {
        public Task<object> Read(SerialPort port, CancellationToken token)
        {
            return new Task<object>(() =>
            {
                byte[] bytes = new byte[4];
                byte[] val = new byte[4];
                int size = 0;
                while (!token.IsCancellationRequested)
                {
                    try
                    {
                        int bufSize = port.Read(bytes, 0, 4);
                        for (int i = size; i < size+bufSize; i++)
                        {
                            val[i] = bytes[i - size];
                        }
                        size+= bufSize;
                        if (size == 4)
                            break;
                    }
                    catch (TimeoutException) { }
                }

                return (int)BitConverter.ToInt32(val, 0);
            });

        }
    }

    class GraphdataFromPortReader : IDataFromPortReaderable
    {
        public Task<object> Read(SerialPort port, CancellationToken token)
        {
            return new Task<object>(() =>
            {
                List<Point> res = new();
                while (token.IsCancellationRequested)
                {
                    res.Add(ReadCeil(port, token));
                }
                return res;
            });

        }

        Int32DataFromPortReader intReader = new();
        private Point ReadCeil(SerialPort port, CancellationToken token)
        {
            var xTask = intReader.Read(port, token);
            xTask.Start();
            xTask.Wait();
            var yTask = intReader.Read(port, token);
            yTask.Start();
            yTask.Wait();
            return new((int)xTask.Result, (int)yTask.Result);
        }
    }
}
