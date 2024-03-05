SET PATH=C:\Keil_v5\ARM\ARMCLANG\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\MATLAB\R2022b\bin;C:\Users\28166\AppData\Local\Programs\Python\Python38\Scripts\;C:\Users\28166\AppData\Local\Programs\Python\Python38\;C:\Users\28166\AppData\Local\Microsoft\WindowsApps;C:\Users\28166\AppData\Local\Programs\Microsoft VS Code\bin;C:\Users\28166\AppData\Local\GitHubDesktop\bin
SET CPU_TYPE=STM32H723VGTx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32H723VG
SET CPU_CLOCK=0x007A1200
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\startup_stm32h723xx._ac"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\main.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\gpio.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\freertos.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\adc.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\dma.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\fdcan.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\octospi.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @"stm32h723vg\spi.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via "STM32H723VG\STM32H723VG.lnp"
