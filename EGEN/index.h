const char MAIN_page[] PROGMEM = R"=====(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>Remote Control Interface</title>
    <style>
      body {
      margin: 0px;
      padding: 0px;
      background-color: #333333;
      }
      #wrapper {
      margin: 0px;
      padding: 0px;
      width: 100%;
      }
      #content {
      margin: 0px auto;
      padding: 5px;
      width: 100%;
      text-align: center;
      }
      button {
      background-color: #22B6FF;
      border: none;
      color: #DDD;
      margin: 20px;
      text-align: center;
      border-radius: 15px;
      box-shadow: -10px 20px #222;
      cursor: pointer;
      }
      #LED {
      font-size: 120px;
      }
      #FRONT, #BACK, #RIGHT, #LEFT, #STOP {
      font-size: 200px;
      height: 300px;
      width: 300px;
      }
      #IMU {
      float: right;
      font-size: 120px;
      }
      button:hover {
      background-color: #0FF3FF;
      color: #FFF;
      }
      button:active {
      background-color: #0FF3FF;
      color: #FFF;
      box-shadow: -5px 15px #555;
      transform: translateY(4px);
      }
      .clear {
      clear:both;
      }
    </style>

    <script>
      var websock;
      function start() {
        websock = new WebSocket('ws://' + window.location.hostname + ':81/');
        websock.onopen = function(evt) { console.log('websock open'); };
        websock.onclose = function(evt) { console.log('websock close'); };
        websock.onerror = function(evt) { console.log(evt); };
        websock.onmessage = function(evt) {
        console.log(evt);
        };
      }
      function ledclick() {
         websock.send("LED");
      }
      function front() {
        websock.send("FRONT");
      }
      function back() {
        websock.send("BACK");
      }
      function right() {
        websock.send("RIGHT");
      }
      function left() {
        websock.send("LEFT");
      }
      function stop() {
        websock.send("STOP");
      }
      function slow() {
        websock.send("SLOW");
      }
      function imu() {
        websock.send("IMU");
      }
    </script>
  </head>

  <body onload="javascript:start();">
    <div id="wrapper">
    <div id="heading">
    <button id="LED" onclick="ledclick()">LED</button>
    <button id="IMU" onclick="imu()">IMU</button>
    </div>
    <div class="clear"></div>
    <div id="content">
      <div id="buttons">
        <button id="FRONT" onclick="front()" onmousedown="forwardVar = setInterval(front, 300)" onmouseup="clearInterval(forwardVar)">&#x21D1;</button><br>
        <button id="LEFT" onclick="left()">&#x21D0;</button>
        <button id="STOP" onclick="stop()">&#x25FC;</button>
        <button id="RIGHT" onclick="right()">&#x21D2;</button><br>
        <button id="BACK" onclick="back()" onmousedown="backwardVar = setInterval(back, 300)" onmouseup="clearInterval(backwardVar)">&#x21D3;</button><br><br>
      </div>
    </div>
    </div>
    

  </body>
</html>
)=====";
