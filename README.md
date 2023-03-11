# 1 Specification
 - Main CPU = ESP32
 - Use Arduino-IDE
 - Use Battery as the power source.
 - Cloud server is "Ambient" which is on of the famous IoT Cloud service in Japan
 - Send temperture, Humidity, buttons status, and Vattery voltage in every 5 min.
 - After sending data, the CPU is into power saving mode.
 - Button1: Once I feel headake, I press it.
 - Button2: Once I get medichine for headake, I press it.

 Yes, it is the system to get log of my headack, and status of getting medichine with weather status. 
 <br><br>

# 2 Hardware wiring

 <br><br><br><br>
 From now on, by Japanese...

# 3. 目的

私は頭痛持ちです。<br>
大気圧の上昇下落によって引き起こされているのでは？と思ったり、湿度に弱いのかな？と思ったり。<br>
最近頭痛薬を飲む事が増えたので、どういう時に飲んでいるのか、気候条件と共にログをとりたいと思い、このガジェットを作成しました。<br>
<br><br>

# 4. 仕様概要
頭痛発生で、ボタン１を押す。
頭痛薬飲んだら、ボタン２を押す。
<br>
基本的にメインCPUはスリープ状態にし、５分に一度起き上がりその時の温湿度気圧を取得し、各ボタン(普通は押されていない)の状態とともにサーバーに送信。
<br><br>
ボタンが押されたら、スリープ解除。その時の温湿度気圧を取得し、各ボタンの状態とともにサーバーに送信。<br><br>
どうせなら、電池で動くもの。<br>
せっかくだし、電池電圧も同じタイミングでサーバに送信する。

<br><br>
