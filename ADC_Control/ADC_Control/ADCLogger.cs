using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ADC_Control_Library;
using NLog;
using NLog.Config;

namespace ADC_Control
{
    public class ADCLogger : ILogService
    {
        private static readonly Logger logger = LogManager.GetCurrentClassLogger();
        public void Write(string message, LogLevels level)
        {
            switch (level)
            {
                case LogLevels.Debug:
                    logger.Debug(message);
                    break;
                case LogLevels.Info:
                    logger.Info(message);
                    break;
                case LogLevels.Trace:
                    logger.Trace(message);
                    break;
                case LogLevels.Warning:
                    logger.Warn(message);
                    break;
                case LogLevels.Error:
                    logger.Error(message);
                    break;
            }
        }
    }
}
