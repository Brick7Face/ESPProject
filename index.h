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
      #heading {
      text-align: center;
      }
      #buttons {
      width: 35%;
      float: left;
      }
      #data {
      width: 64.5%;
      }
      #content {
      margin: 0px auto;
      padding: 5px;
      width: 100%;
      text-align: center;
      }
      h1 {
      text-align: center;
      font-size: 200px;
      }
      button {
      background-color: #22B6FF;
      border: none;
      color: #DDD;
      padding: 10px 20px;
      margin: 10px;
      font-size: 120px;
      text-align: center;
      border-radius: 15px;
      box-shadow: -10px 20px #222;
      cursor: pointer;
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
        var e = document.getElementById();
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
    </script>
  </head>

  <body onload="javascript:start();">
    <div id="wrapper">
    <div id="heading">
    <button id="LED" onclick="ledclick()">LED</button>
    </div>
    <div id="content">
      <div id="buttons">
        <button id="FRONT" onclick="front()" onmousedown="forwardVar = setInterval(front, 300)" onmouseup="clearInterval(forwardVar)">F</button><br>
        <button id="LEFT" onclick="left()">L</button>
        <button id="SLOW" onclick="slow()">SL</button>
        <button id="RIGHT" onclick="right()">R</button><br>
        <button id="BACK" onclick="back()" onmousedown="backwardVar = setInterval(back, 300)" onmouseup="clearInterval(backwardVar)">B</button><br><br>
        <button id="STOP" onclick="stop()">STOP</button>
      </div>
      <div id="data">
        
      </div>
    </div>
    </div>
    

  </body>
</html>
)=====";
