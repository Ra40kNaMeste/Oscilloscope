using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace ADC_Control
{
    public class UniversalCommand : ICommand
    {
        public UniversalCommand(Action<object?> action, Func<object?, bool>? condition = null) 
        {
            Action = action;
            Condition = condition;
        }

        private Action<object?> Action { get; init; }

        private Func<object?, bool>? Condition { get; init; }

        public event EventHandler? CanExecuteChanged;

        public bool CanExecute(object? parameter)
        {
            return Condition == null || Condition.Invoke(parameter);
        }

        public void Execute(object? parameter)
        {
            Action.Invoke(parameter);
        }

        public void OnCanExecuteChanged() => CanExecuteChanged?.Invoke(this, new());
    }
}
