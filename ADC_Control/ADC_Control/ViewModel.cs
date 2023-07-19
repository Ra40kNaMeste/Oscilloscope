using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Input;
using ADC_Control.Properties;
using ADC_Control_Library;
using InteractiveDataDisplay.WPF;

namespace ADC_Control
{
    [AttributeUsage(AttributeTargets.Property)]
    class CommandCanExecudeUpdateByPropertyChangedAttribute : Attribute
    {

    }
    sealed public class ViewModel : INotifyPropertyChanged, IDisposable
    {

        //[DllImport("User32.dll")]
        //private static extern bool SetCursorPos(int X, int Y);

        //[DllImport("user32.dll", CharSet = CharSet.Auto, CallingConvention = CallingConvention.StdCall)]
        //public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint cButtons, uint dwExtraInfo);
        ////Mouse actions
        //private const int MOUSEEVENTF_LEFTDOWN = 0x02;
        //private const int MOUSEEVENTF_LEFTUP = 0x04;
        //private const int MOUSEEVENTF_RIGHTDOWN = 0x08;
        //private const int MOUSEEVENTF_RIGHTUP = 0x10;


        #region Constructor
        public ViewModel()
        {
            //инициализация компонентов
            Graphs = new();
            ADC = new(new ADCLogger());
            SettingPortVM = new();
            tokenSource = new();
            Logger = NLog.LogManager.GetCurrentClassLogger();
            UpdatedCommands = new();
            FillUpdatedCommands();

            //"Привязка" к изменению свойств
            SettingPortVM.PropertyChanged += (sender, e) => UpdatePortCommand();
            SettingPortVM.PropertyChanged += ClosePortIfSelectPortChanged;
            ADC.PropertyChanged += (sender, e) => UpdatePortCommand();
            ADC.PropertyChanged += CallPropertyChangedByGraphs;
            PropertyChanged += (sender, e) => UpdatePortCommand();
            IsRunningOperation = false;
        }

        private void FillUpdatedCommands()
        {
            var t = GetType();
            var props = t.GetProperties();
            foreach (var prop in props)
            {
                if (prop.GetCustomAttribute<CommandCanExecudeUpdateByPropertyChangedAttribute>() != null &&
                    prop.GetValue(this) is UniversalCommand command)
                    UpdatedCommands.Add(command);
            }
        }

        #endregion //Constructor

        #region VisualProperties

        private bool isRunningOperation;
        /// <summary>
        /// Выполняется ли опреция АЦП
        /// </summary>
        public bool IsRunningOperation
        {
            get => isRunningOperation;
            set
            {
                isRunningOperation = value;
                OnPropertyChanged();
            }
        }

        private PointGraph? selectGraph;
        /// <summary>
        /// Выбранный график
        /// </summary>
        public PointGraph? SelectGraph
        {
            get => selectGraph;
            set
            {
                selectGraph = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Коллекция графикоа
        /// </summary>
        public List<PointGraph> Graphs { get; init; }

        private string? selectPort;
        /// <summary>
        /// выбранный порт для связи с АЦП
        /// </summary>
        public string? SelectPort
        {
            get => selectPort;
            set
            {
                selectPort = value;
                OnPropertyChanged();
            }
        }

        private bool? isMirrorTest;
        /// <summary>
        /// Прошёл ли порт проверку на связь с микроконтроллером
        /// </summary>
        public bool? IsMirrorTest
        {
            get => isMirrorTest;
            private set
            {
                isMirrorTest = value;
                OnPropertyChanged();
            }
        }


        private StringBuilder console = new();
        /// <summary>
        /// Консоль для вывода
        /// </summary>
        public StringBuilder Console
        {
            get => console;
            private set
            {
                console = value;
                OnPropertyChanged();
            }
        }

        private DateTime? timeConvertation;
        /// <summary>
        /// Время для непрерывной конвертации
        /// </summary>
        public DateTime? TimeConvertation
        {
            get => timeConvertation;
            set
            {
                timeConvertation = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Свойства АЦП
        /// </summary>
        public List<IPropertiesTableElementable> ADCProperties
        {
            get => ADC.ADCProperties;
        }

        /// <summary>
        /// Доступные порты
        /// </summary>
        public List<string>? Ports { get => ADC.Ports; }

        /// <summary>
        /// Настройки порта для связи с АЦП
        /// </summary>
        public SettingPortViewModel SettingPortVM { get; init; }

        #endregion //VisualProperties

        #region Command

        #region Head

        private UniversalCommand? updatePortsCommand;
        /// <summary>
        /// Обновить доступные порты для подключения
        /// </summary>
        public UniversalCommand UpdatePortsCommand => updatePortsCommand ??= new(UpdatePorts);

        private UniversalCommand? openPortCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Открыть порт для связи
        /// </summary>
        public UniversalCommand OpenPortCommand => openPortCommand ??= new(OpenPort, CanOpenPort);

        private UniversalCommand? closePortCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Закрыть порт
        /// </summary>
        public UniversalCommand ClosePortCommand => closePortCommand ??= new(ClosePort, CanClosePort);

        private UniversalCommand? updatePortParametersCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Открыть другой порт
        /// </summary>
        public UniversalCommand UpdatePortParametersCommand => updatePortParametersCommand ??= new(UpdatePortParameters, CanUpdatePortParameters);

        private UniversalCommand? updateADCPropertiesCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Считать свойства с АЦП модуля
        /// </summary>
        public UniversalCommand UpdateADCPropertiesCommand => updateADCPropertiesCommand ??= new(UpdateADCPropertiesAsync, CanInvokeADCOperation);

        private UniversalCommand? writeADCPropertiesCommand;
        /// <summary>
        /// Записать свойства на АЦП модуль
        /// </summary>
        public UniversalCommand WriteADCPropertiesCommand => writeADCPropertiesCommand ??= new(WriteADCPropertiesAsync, CanInvokeADCOperation);


        private UniversalCommand? testMirrorCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Проверить связь с микроконтроллером
        /// </summary>
        public UniversalCommand TestMirrorCommand => testMirrorCommand ??= new(TestMirrorAsync,
            (obj) => SelectPort != null && ADC.Port.IsOpen);

        private UniversalCommand? cancelADCOpertaionCommand;

        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Отменить выполняемую операцию
        /// </summary>
        public UniversalCommand CancelADCOpertaionCommand => cancelADCOpertaionCommand ??= new(
            (obj) =>
            {
                tokenSource.Cancel();
                IsRunningOperation = false;
            },
            (obj) => IsRunningOperation);

        private UniversalCommand? calibrationADCInsideCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Откалибровать АЦП на 0. Выполнять перед измерениями
        /// </summary>
        public UniversalCommand CalibrationADCInsideCommand => calibrationADCInsideCommand ??= new((obj) => ADC.CalibrationInside(), CanInvokeADCOperation);

        private UniversalCommand? calibrationADCOutsideCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Откалибровать АЦП с учётом проводов. Выполнять при КЗ
        /// </summary>
        public UniversalCommand CalibrationADCOutsideCommand => calibrationADCOutsideCommand ??= new((obj) => ADC.CalibrationOutside(), CanInvokeADCOperation);

        private UniversalCommand? calibrationADCScaleCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Откалибровать масштаб АЦП
        /// </summary>
        public UniversalCommand CalibrationADCScaleCommand => calibrationADCScaleCommand ??= new((obj) => ADC.CalibrationScale(), CanInvokeADCOperation);

        private UniversalCommand? convertADCCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Конвертирует 1 значение сигнала
        /// </summary>
        public UniversalCommand ConvertADCCommand => convertADCCommand ??= new(ConvertValueADCAsync, CanInvokeADCOperation);

        private UniversalCommand? convertADCToTimeCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        /// <summary>
        /// Конвертирует по времени в график
        /// </summary>
        public UniversalCommand ConvertADCToTimeCommand => convertADCToTimeCommand ??= new(ConvertADCToTimeAsync, CanInvokeADCOperationWithInsideTimer);

        private UniversalCommand? runMonochromeCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        public UniversalCommand RunMonochromeCommand => runMonochromeCommand ??= new(RunMonochromeAsync, CanInvokeADCOperation);

        private UniversalCommand? rewindMonochromeCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        public UniversalCommand RewindMonochromeCommand => rewindMonochromeCommand ??= new(RewindMonochromeAsync, CanInvokeADCOperation);


        private UniversalCommand? runByADCTimerMonochromeCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        public UniversalCommand RunByADCTimerMonochromeCommand => runByADCTimerMonochromeCommand ??= new(RunByADCTimerMonochromeAsync, CanInvokeADCOperationWithInsideTimer);

        private UniversalCommand? rewindByADCTimerMonochromeCommand;
        [CommandCanExecudeUpdateByPropertyChanged]
        public UniversalCommand RewindByADCTimerMonochromeCommand => rewindByADCTimerMonochromeCommand ??= new(RewindByADCTimerMonochromeAsync, CanInvokeADCOperationWithInsideTimer);

        #endregion //Head

        #region Body
        private void UpdatePorts(object? parameter)
        {
            ADC.UpdatePorts();
        }
        private bool CanOpenPort(object? parameter) => SelectPort != null && !ADC.Port.IsOpen && SettingPortVM.StopBits != StopBits.None;
        private async void OpenPort(object? parameter)
        {
            try
            {
                if (SettingPortVM.SavePortSettingsCommand.CanExecute(null))
                    SettingPortVM.SavePortSettingsCommand.Execute(null);

                Logger.Info(Resources.LogClosePortStart);
                ADC.Port.PortName = selectPort;
                SettingPortVM.SetPropertiesPort(ADC.Port);
                SettingPortVM.ResetChangedProperties();
                ADC.Port.Open();
                UpdatePortCommand();
                IsMirrorTest = await ADC.RunTestMirrorAsync(GetToken());
                if (IsMirrorTest == true)
                    UpdateADCPropertiesAsync(null);
                Logger.Info(string.Format(Resources.LogClosePortFinish, SelectPort));
                IsRunningOperation = false;
            }
            catch (Exception)
            {
                Logger.Error(string.Format(Resources.LogOpenPortError, SelectPort));
            }
        }


        private async void TestMirrorAsync(object? parameter)
        {
            await ADC.RunTestMirrorAsync(GetToken());
            //Сброс токенов
            if (tokenSource.IsCancellationRequested)
                tokenSource.TryReset();
        }

        private bool CanClosePort(object? parameter) => SelectPort != null && ADC.Port.IsOpen;
        private void ClosePort(object? parameter)
        {
            Logger.Info(string.Format(Resources.LogClosePortStart, ADC.Port.PortName));
            ADC.Port.Close();
            Logger.Info(Resources.LogClosePortStart);
            UpdatePortCommand();
            IsMirrorTest = null;
        }


        private bool CanUpdatePortParameters(object? parameter) => SelectPort != null && ADC.Port.IsOpen && SettingPortVM.IsChangedProperties;
        private void UpdatePortParameters(object? parameter)
        {
            ClosePort(parameter);
            OpenPort(parameter);
        }

        private bool CanInvokeADCOperation(object? parameter)
        {
            return ADC.Port.IsOpen && IsMirrorTest == true && !IsRunningOperation;
        }
        private bool CanInvokeADCOperationWithInsideTimer(object? parameter)
        {
            return CanInvokeADCOperation(parameter) && ADC.TimeFromADC != null && TimeConvertation != null &&
                new TimeOnly(TimeConvertation.Value.Ticks) == ADC.TimeFromADC;
        }
        private async void ConvertValueADCAsync(object? parameter)
        {
            try
            {
                Logger.Info(Resources.LogStartConvertValue);
                IsRunningOperation = true;
                var res = await ADC.ConvertAsync(GetToken());
                MessageBox.Show(res.ToString());
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();
                Logger.Info(Resources.LogFinishCoonvertValue);
            }
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        private async void ConvertADCToTimeAsync(object? parameter)
        {
            try
            {
                Logger.Info(Resources.LogStartConvertGraph);
                IsRunningOperation = true;
                var result = await ADC.ConvertToTimeAsync();
                Logger.Info(Resources.LogEndConvertGraph);
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();

            }
            catch (Exception)
            {

                Logger.Info(Resources.LogErrorConvertGraph);
            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        private async void RunMonochromeAsync(object? parameter)
        {
            try
            {
                Logger.Info(Resources.LogRunMonochromeStart);
                if (TimeConvertation == null)
                {
                    return;
                }
                //var elements = AutomationElement.RootElement.FindAll(TreeScope.Children, System.Windows.Automation.Condition.TrueCondition);
                //AutomationElement? element = null;
                //foreach (AutomationElement item in elements)
                //{
                //    if (item.Current.Name.Contains("SignalExpress"))
                //    {
                //        element = item;
                //        break;
                //    }

                //}
                //if (element == null) { throw new Exception(); }
                //var commandBar = element.FindFirst(TreeScope.Children, new PropertyCondition(AutomationElement.NameProperty, "DockTop"));
                //element.SetFocus();
                //Point pnt = commandBar.Current.BoundingRectangle.Location;
                //uint x = (uint)pnt.X + 140;
                //uint y = (uint)pnt.Y + 30;
                //SetCursorPos((int)x, (int)y);
                //mouse_event(MOUSEEVENTF_LEFTDOWN, x, y, 0, 0);
                //mouse_event(MOUSEEVENTF_LEFTUP, x, y, 0, 0);
                //x -= 20;
                //y += 80;
                //SetCursorPos((int)x, (int)y);
                //mouse_event(MOUSEEVENTF_LEFTDOWN, x, y, 0, 0);
                //mouse_event(MOUSEEVENTF_LEFTUP, x, y, 0, 0);

                IsRunningOperation = true;
                await ADC.StartByTimeMonochromeAsync(new TimeOnly(TimeConvertation.Value.Ticks), GetToken());
                Logger.Info(Resources.LogRunMonochromeFinish);
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();

            }
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        private async void RewindMonochromeAsync(object? parameter)
        {

            try
            {
                Logger.Info(Resources.LogRewindMonochromeStart);
                IsRunningOperation = true;
                if (TimeConvertation != null)
                    await ADC.RewindByTimeMonochromeAsync(new TimeOnly(TimeConvertation.Value.Ticks), GetToken());
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();

                Logger.Info(Resources.LogRewindMonochromeFinish);
            }
            catch (Exception)
            {
            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        private async void RunByADCTimerMonochromeAsync(object? parameter)
        {
            try
            {
                IsRunningOperation = true;
                Logger.Info(Resources.LogStartRunByADCTimerMonochrome);
                await ADC.StartByADCTimerMonochromeAsync();
                Logger.Info(Resources.LogFinishRunByADCTimerMonochrome);
            } 
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }
        private async void RewindByADCTimerMonochromeAsync(object? parameter)
        {
            try
            {
                IsRunningOperation = true;
                Logger.Info(Resources.LogStartRewindByADCTimerMonochrome);
                await ADC.RewindByADCTimerMonochromeAsunc();
                Logger.Info(Resources.LogFinishRewindByADCTimerMonochrome);
            }
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        public async void UpdateADCPropertiesAsync(object? parameter)
        {
            try
            {
                Logger.Info(Resources.LogReadADCPropertiesStart);
                IsRunningOperation = true;
                await ADC.UpdateFromADCPropertiesAsync();
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();

                Logger.Info(Resources.LogReadADCPropertiesFinish);
                OnADCPropertiesChanged();
            }
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        public async void WriteADCPropertiesAsync(object? parameter)
        {
            try
            {
                Logger.Info(Resources.LogWriteADCPropertiesStart);
                IsRunningOperation = true;
                await ADC.WriteToADCPropertiesAsync();
                //Сброс токенов
                if (tokenSource.IsCancellationRequested)
                    tokenSource.TryReset();

                Logger.Info(Resources.LogWriteADCPropertiesFinish);
            }
            catch (Exception)
            {

            }
            finally
            {
                IsRunningOperation = false;
            }
        }

        #endregion //Body

        #endregion //Command

        #region PrivateProperties


        private CancellationTokenSource tokenSource;
        private ADCManager ADC { get; init; }

        private NLog.Logger Logger { get; init; }
        #endregion //PrivateProperties

        #region PrivateMethods

        private CancellationToken GetToken()
        {
            IsRunningOperation = true;
            return tokenSource.Token;
        }

        private void OnADCPropertiesChanged() => OnPropertyChanged("ADCProperties");

        private void ClosePortIfSelectPortChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "SelectPort")
                ClosePort(null);
        }

        private void CallPropertyChangedByGraphs(object? sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case "Ports":
                    OnPropertyChanged("Ports");
                    break;
                default:
                    break;
            }
        }

        private List<UniversalCommand> UpdatedCommands { get; init; }
        private void UpdatePortCommand()
        {
            foreach (var command in UpdatedCommands)
            {
                command.OnCanExecuteChanged();
            }
        }

        #endregion //PrivateMethods
        private void OnPropertyChanged([CallerMemberName] string name = "") => PropertyChanged?.Invoke(this, new(name));

        public void Dispose()
        {
            tokenSource.Dispose();
        }

        public event PropertyChangedEventHandler? PropertyChanged;
    }
}
