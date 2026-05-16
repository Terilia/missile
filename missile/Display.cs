using Sandbox.ModAPI.Ingame;
using System;
using System.Text;
using VRage.Game.GUI.TextPanel;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public void DisplayOnLCD(IMyTextPanel lcd, double distanceToTarget, double startingDistance, double currentSpeed, int _ticks, Vector3D currentVelocity)
        {
            return;
            // Calculate time to impact and progress
            double timeToTargetSec = currentSpeed > 0 ? distanceToTarget / currentSpeed : double.MaxValue;
            double progress = Math.Max(0, Math.Min((startingDistance - distanceToTarget) / startingDistance, 1.0));

            // Setup LCD for text display (camera compatible)
            lcd.ContentType = ContentType.TEXT_AND_IMAGE;
            lcd.Font = "Monospace";
            lcd.FontSize = 0.8f;
            lcd.Alignment = TextAlignment.LEFT;
            lcd.TextPadding = 2f;

            StringBuilder display = new StringBuilder();

            // Header
            display.AppendLine("=====================================");
            display.AppendLine("   NYINAH CORP MISSILE CAM");
            display.AppendLine("=====================================\n");

            // Critical impact warning (last 2.5 seconds)
            if (timeToTargetSec <= 2.5 && timeToTargetSec > 0.1)
            {
                bool flash = (_ticks / 10) % 2 == 0;
                if (flash)
                {
                    display.AppendLine("!!! IMPACT IMMINENT !!!");
                    display.AppendLine($"    {timeToTargetSec:F1} SECONDS");
                    display.AppendLine("!!! IMPACT IMMINENT !!!");
                }
                else
                {
                    display.AppendLine("");
                    display.AppendLine($">>> {timeToTargetSec:F1} SECONDS <<<");
                    display.AppendLine("");
                }
            }
            else
            {
                // Normal telemetry display
                display.AppendLine($"Speed:     {currentSpeed:F0} m/s");
                display.AppendLine($"Distance:  {distanceToTarget:F0} m");

                if (timeToTargetSec < 999)
                    display.AppendLine($"ETA:       {timeToTargetSec:F1} sec");
                else
                    display.AppendLine("ETA:       ---");

                // Target info
                if (!detectedEntity.IsEmpty())
                    display.AppendLine($"Target:    {detectedEntity.Name}");
                else
                    display.AppendLine("Target:    Seeking...");

                display.AppendLine("");

                // Progress bar (25 characters wide)
                int barLength = 25;
                int filled = (int)(barLength * progress);
                string progressBar = "[" + new string('=', filled) + ">" + new string('-', barLength - filled - 1) + "]";
                display.AppendLine($"Progress:  {progressBar}");
                display.AppendLine($"           {(progress * 100):F0}%");

                display.AppendLine("");

                // Status
                if (distanceToTarget < 100)
                    display.AppendLine("Status:    TERMINAL PHASE");
                else if (distanceToTarget < 500)
                    display.AppendLine("Status:    FINAL APPROACH");
                else
                    display.AppendLine("Status:    TRACKING");
            }

            display.AppendLine("\n=====================================");
            display.AppendLine("IGC:");
            if (_igcLog.Count == 0)
            {
                display.AppendLine("  (no traffic)");
            }
            else
            {
                foreach (var entry in _igcLog)
                {
                    display.AppendLine("  " + entry);
                }
            }
            display.AppendLine("=====================================");

            lcd.WriteText(display.ToString());
        }
    }
}
