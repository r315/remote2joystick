# remote2joystick
Basic convertor of PPM from from RC receiver USB joystick device.

<pre>
Arquitecture

          !
          |
+---------------------+
|                     |
|     6 channel       |
|  35MHz Transmitter  |
|                     |
+---------------------+




          !
          |
+------------------+                +-----------------+              +--------+
|                  |      PPM       |                 |      USB     |        |
|  35MHz Receiver  |----------------| remote2joystick |--------------|   PC   |
|                  |                |                 |              |        |
+------------------+                +-----------------+              +--------+

</pre>
