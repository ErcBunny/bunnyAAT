# bunnyAAT v0.0.2b

* Warning: a senior high school level project.

## PREFACE

* I know it is strange that this repo starts with v0.0.2b. Well, the earlier versions can be found at tieba. And if you read Chinese you'll know more details about this project, which is not necessary at all since the best part (so far) is posted here at github.
  
  
## HARDWARE SETUP

* In "hardware" folder you can find autocad blueprints (.dwg) and schematic diagrames.
  
* Read the schematic diagrams and you will know what boards and sensors to buy.
  
* "3mm.dwg" illustrates additional parts needed to complete the structure of the antenna tracker and also the payload adapter for blacksheep  quadcopter airframe, which is shown in "blacksheep.dwg".
  
* "adapter.dwg" is only useful if you want to use the 23dbi MFD antenna. Otherwise you can simply glue your antenna to the moving part.
  
* "base.dwg" shows you what bunnyAAT looks like. Most of its parts are bought online. One possible version of the base can be found here: https://item.taobao.com/item.htm?spm=a1z09.2.0.0.28812e8dhvkkIb&id=37631042726&_u=a2ndlmbi166b.
  
* Also, it is recommended to cover the baro_sensor with a sponge and a reflective surface at the top. By doing this, we can increase sensing accuracy (but only a bit).
  
  
## ABOUT SOFTWARE

* There are two apps for android. One of the two is magvar.apk. This app connects automatically to https://www.ngdc.noaa.gov/geomag-web/#declination. By enabling the positioning service the app will return the declination value of your location. The other one is terminal.apk. It is a bluetooth terminal with macro buttons which displays aircraft status and voltage of batteries.
  
* "mwc.ino" should be uploaded into the mwc flight controller board (make sure that your flight controller uses BMP180 and MPU6050, otherwise you need to modify the code). "servo.ino" should be uploaded into another pro mini board. In order to upload sketches you may need the FTDI USB to TTL tool and corresponding driver.
  
* "uno.ino" is for the uno board installed on your aircraft.
  
  
## CONFIGURATION

* Change gps refresh rate to 5hz or higher.
  
* Edit macro buttons in the terminal app. '0' must be set as "continue" and '_local declination value_' is recommended to be set as "default". 
  
* Turn your tracker into debug mode by uncommenting the the first line of "mwc.ino" and put your aircraft at the same height as the antenna. Follow the instructions. When you reach the stage of calibrating the mag_sensor, the tracker should spin clockwise. If not, swap the values of LEFT and RIGHT in line 2 and 4. And if the pan servo doesn't stop, modify the STOP value too. The debug mode returns two statements "#define offAlt _value_" and "#define K _value_". Modify the values in line 5 and 6 then comment line 1 and upload. Now you are good to go.
  
* When the the aircraft and AAT are exposed to direct sunlight, the baro_sensors are very likely to be affected. That is, the value returned in debug mode is not accurate anymore. You need to redo debugging or adjust the offAlt value after estimating the error.
  
* The last thing. You may notice that the V_air and V_gnd is really off the track. Therefore, in "mwc.ino" and "uno.ino" you can find "#define offV", which enables you to calibrate the value by hand. For example, if V_gnd is 3.7 displayed in terminal, while its real value is 4.0, then I'll enter "#define offV 0.3 (4.0 - 3.7)".
  
  
4) CREDITS

* Libraries are very helpful. I saved a lot of time by using them. I'd like to thank contributors who uploaded their libraries.
  
* I don't quite understand how a kalman filter works (just for now), so the code for this part is written by Similar_Fair. Click the link below for more information: https://blog.csdn.net/sunhaobo1996/article/details/53861752.
  
  
