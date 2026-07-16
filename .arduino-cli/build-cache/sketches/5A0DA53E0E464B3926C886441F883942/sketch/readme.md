#line 1 "/tmp/vicpinky_refresh_inspect/arduino_board2_tb6600/readme.md"
| 기능 | Arduino 핀 |
|---|---:|
| MCP2515 CS | `D10` |
| MCP2515 INT | `D2` |
| MCP2515 SCK | `D13` |
| MCP2515 MISO | `D12` |
| MCP2515 MOSI | `D11` |
| TB6600 STEP/PUL | `D6` |
| TB6600 DIR | `D7` |
| TB6600 ENA+ | `D8` |
| TB6600 ENA- | `GND` |
| Limit switch | `D3` |

STEP pulse width is `5 us`. The default motion and homing limit is `1250 step/s`, matching about `800 us` between STEP pulses.
