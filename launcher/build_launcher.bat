@echo off
rem Builds "FC Tuner.exe" in the fc_test folder with the C# compiler that ships with Windows.
cd /d "%~dp0"
"%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe" /nologo /target:winexe /optimize+ ^
  /win32icon:app.ico /r:System.Windows.Forms.dll /r:System.Drawing.dll ^
  /out:"..\FC Tuner.exe" Launcher.cs
