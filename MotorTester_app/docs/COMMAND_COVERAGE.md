# MKS CAN Command Coverage Matrix

Источник команд: `mks_protocol_description.md`.

| Код | Название | Метод в `MksCanMotor` |
|---|---|---|
| 30h | Read Encoder (Carry) | `readEncoderCarry` |
| 31h | Read Encoder (Addition) | `readEncoderAddition` |
| 32h | Read Motor Speed | `readMotorSpeed` |
| 34h | Read I/O Status | `readIoStatus` |
| 39h | Read Shaft Angle Error | `readShaftAngleError` |
| 3Ah | Read EN Pin Status | `readEnPinStatus` |
| 3Dh | Release Stall Protection | `releaseStallProtection` |
| 3Eh | Read Protection State | `readProtectionState` |
| 80h | Calibrate Encoder | `calibrateEncoder` |
| 82h | Set Work Mode | `setWorkMode` |
| 83h | Set Working Current | `setWorkingCurrent` |
| 84h | Set Subdivision | `setSubdivision` |
| 85h | Set EN Pin Active Level | `setEnPinActiveLevel` |
| 89h | Set Subdivision Interpolation | `setSubdivisionInterpolation` |
| 8Bh | Set CAN ID | `setCanId` |
| 8Ch | Set Slave Respond Active | `setSlaveRespondActive` |
| 8Dh | Set Group ID | `setGroupId` |
| 9Bh | Set Holding Current | `setHoldingCurrent` |
| 9Dh | Set EN Trigger & Error Protection | `setEnTriggerErrorProtection` |
| 9Eh | Set Limit Port Remap | `setLimitPortRemap` |
| 3Fh | Restore Default Parameters | `restoreDefaultParameters` |
| 41h | Restart Motor | `restartMotor` |
| 90h | Set Home Parameters | `setHomeParameters` |
| 91h | Go Home | `goHome` |
| 92h | Set Current Axis to Zero | `setCurrentAxisToZero` |
| F1h | Query Motor Status | `queryMotorStatus` |
| F3h | Enable Motor | `enableMotor` |
| F6h | Run Speed Mode | `runSpeedMode` |
| F7h | Emergency Stop | `emergencyStop` |
| FDh | Run Position (Relative Pulses) | `runPositionRelativePulses` |
| FEh | Run Position (Absolute Pulses) | `runPositionAbsolutePulses` |
| F4h | Run Position (Relative Axis) | `runPositionRelativeAxis` |
| F5h | Run Position (Absolute Axis) | `runPositionAbsoluteAxis` |

Дополнительно:
- чтение параметров через префикс-команду `0x00` реализовано методом `readSystemParameter` (через `MksProtocol::readParameter`).
