
#ifndef FORMULARIOS2_H
#define FORMULARIOS2_H

static const char PROGMEM HTML2[] = R"(
<!DOCTYPE html>
<HTML>
<HEAD>
<meta http-equiv="content-type" content="text/html; charset=utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:," /> <!-- https://www.w3schools.com/tags/att_link_rel.asp -->
<title>MULTIROTORFLEX2 (mrf2)</title>

<style type="text/css">
body {
  margin: 0;
  padding: 0;
  font: 75% arial, sans-serif;
}
div#header h1, h2, h3 {
  margin: 0;
  padding: 0;
  padding-left:0px;
  padding-top:0px;
}
.inputs {
  position: relative;
  top: 0;
  margin-left:1%;
  margin-right:1%;
  /*justify-content: space-between;*/
  font-size: 1.0em;
  color: #FFF;
  border-radius: 10px;
  height: 99%;
  width: 98%;
  vertical-align: -50%; 
  cursor: pointer;
}
.container {
  display: grid;
  height: 100vh;
  grid-template-columns: 1fr 3fr;
  grid-template-rows: 50px 50px 2fr 1fr;
}
.header {
  grid-column : 1 / 3;
  grid-row: 1 / 2;
  background-color:#96b8f8;
  text-align: center;
}
.modos {
  grid-column : 1 / 3;
  grid-row: 2 / 3;
  background: #EAEAEA;
  border: 1px solid black;
}
.throttle {
  grid-column : 1 / 2;
  grid-row: 3 / 4;
  border: 1px solid gray;
}
.joystick {
  grid-column : 2 / 3;
  grid-row: 3 / 4;
  border: 1px solid gray;
}
.google {
  grid-column : 1 / 3;
  grid-row: 3 / 4;
  border: 1px solid gray;
}
.errores {
  grid-column : 1 / 3;
  grid-row: 4 / 5;
  border: 1px solid gray;
  overflow-x: scroll;
  overflow-y: scroll;
}
</style>

<script async defer src="https://maps.googleapis.com/maps/api/js?v=3.exp&libraries=drawing"></script>

<script>
"use strict"; // global scope

const ARMED = 1090;
const UMBRAL = 10;
const INCL = 1580;
const ESC_LIMITE = 1900;
const SERVO_LIMITE = 250; //1msec = 0,4grados = 4,888pwm(5pwm) => *** 250pwm = 20,46grados (aprox) ***

var HOVER = 1450;

var conexion1, conexion2, conexion3;
var botonARRANQUE, estadoARRANQUE = 0;
var pixelsJoystickX, pixelsJoystickY, PROAJoystick, POPAJoystick, autoJoystick, pixelsThrottle, PWMThrottle, autoThrottle;
var panelErrores;
var tiempo; //calculo del tiempo de conexion de los websockets
var CONECTADO = false; //control booleano de la primera conexion
var zero_zero_error = false; //¿como arregar el 1500,1500?
var latitud=37.401214, longitud=-5.980698; // por defecto micasa
var infoWindow = null;
var mapita, centro;    
var wifi_grafico;

function conecta1 () {
  conexion1 = null;
  conexion1 = new WebSocket('ws://'+location.hostname+':81/'); //MANDO y CONTROL
  conexion1.onopen = function () {
  console.log ("WebSocket conectado al puerto 81 (MANDO y CONTROL): ", (new Date()).getTime() - tiempo);
  insertarError("<span style='color:green;'><strong>WebSocket conectado al puerto 81 (MANDO y CONTROL)</strong></span>");
  if (zero_zero_error) {
    console.log ("readyState de la reconexion:"+conexion1.readyState);
    conexion1.send('JS:0,0');
    zero_zero_error = false;
  }
}
conexion1.onerror = function (error) {
  console.log("Error en el WebSocket del puerto 81 (MANDO y CONTROL) ", error);
  conexion1.close();
}
conexion1.onmessage = function (event) {
  let datos = JSON.parse(event.data); //contrario => JSON.stringify(objetoJSON)
  //console.log("onmessage<= ", datos)
  rxDatosJson(datos);
}
conexion1.onclose = function (error) {
  console.log("WebSocket del puerto 81 (MANDO y CONTROL) cerrado ", error);
  console.log('Reconectaremos en 1 segundo.', error.reason);
  insertarError("<span style='color:red;'><strong>Reconexion WS1 en 1 segundos</strong></span>");
  setTimeout(function() {
    conecta1();
    //if (conexion1.readyState == 1) conexion1.send = "STICK:1500,1500"; //0-CONNECTING 1-OPEN 2-CLOSING 3-CLOSED
  }, 1000);
  }
}

function conecta2() {
  conexion2 = null;
  conexion2 = new WebSocket('ws://'+location.hostname+':82/'); //SENSORES
  conexion2.onopen = function () { 
  console.log ("WebSocket conectado al puerto 82 (SENSORES): ", (new Date()).getTime() - tiempo); 
  insertarError("<span style='color:green;'><strong>WebSocket conectado al puerto 82 (SENSORES)</strong></span>");
}
conexion2.onerror = function (error) {
  console.log("Error en el WebSocket del puerto 82 (SENSORES) ", error); 
  conexion2.close();
}
conexion2.onmessage = function (event) {
  let datos = JSON.parse(event.data); //contrario => JSON.stringify(objetoJSON)
  //console.log("onmessage<= ", datos)
  rxDatosJson(datos);
}
conexion2.onclose = function (error) {
  console.log("WebSocket del puerto 82 (SENSORES) cerrado ", error);
  console.log('Reconectaremos en 1 segundo.', error.reason);
  insertarError("<span style='color:red;'><strong>Reconexion WS2 en 1 segundos</strong></span>");
  setTimeout(function() {
    conecta2();
  }, 1000);
  }
}

function mapas() {
  var mapOptions = {
    zoom: 18,
    center: new google.maps.LatLng(latitud, longitud),
    mapTypeControl: true,
    /*
    mapTypeControlOptions: { mapTypeIds: google.maps.MapTypeId.HYBRID // [ google.maps.MapTypeId.ROADMAP, google.maps.MapTypeId.HYBRID, MapTypeId.SATELLITE, MapTypeId.TERRAIN ] }
    */
    panControl: true,
    zoomControl: true,
    scaleControl: true,
    streetViewControl: true
  };
  mapita = new google.maps.Map(document.getElementById("google"), mapOptions);
  centro = mapita.getCenter();
  mapita.setCenter(centro);
  let contenido = '<div>'+'<b>GPS (micasa)</b></br>lat=' + centro.lat().toFixed(7) + '</br>lng=' + centro.lng().toFixed(7) +'</div>';
  infoWindow = new google.maps.InfoWindow({
    map: mapita,
    position: centro,
    content: contenido,
    maxWidth: 200
  });
}

window.onload = function() {
  const joystick = new JOYSTICK (document.getElementById('joystick-base') );
  const throttle = new THROTTLE (document.getElementById('throttle-base') );

  botonARRANQUE=document.getElementById("boton_de_arranque");

  panelErrores=document.getElementById("errores");

  //pixelsJoystickX=document.getElementById("pixelsJoystickX");
  //pixelsJoystickY=document.getElementById("pixelsJoystickY");
  PROAJoystick=document.getElementById("PROAJoystick");
  POPAJoystick=document.getElementById("POPAJoystick");
  autoJoystick=document.getElementById("autoJoystick");
  //pixelsThrottle=document.getElementById("pixelsThrottle");
  PWMThrottle=document.getElementById("PWMThrottle");
  autoThrottle=document.getElementById("autoThrottle");

  wifi_grafico = document.getElementById("wifi-grafico");

  tiempo = (new Date()).getTime(); //calculo del tiempo de conexion de los websockets
  conecta1();
  conecta2();

  google=document.getElementById("google");
  mapas();
};

function insertarError (error) {
  const t = new Date();
  let tt = new Date().toLocaleTimeString('en-GB') + `.${t.getMilliseconds()}`;
  panelErrores.innerHTML = " &nbsp; " + tt + " &emsp; " + error + '</BR>' + panelErrores.innerHTML;
}

function beep() {
  let sound = new Audio('data:audio/wav;base64,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qJrx4KOQjahZxWbcZgztj2c49nKmkId44S71j0c8eV9yDK6uPRzx5X18eDvjvQ6yKo9ZSS6l//8elePK/Lf//IInrOF/FvDoADYAGBMGb7FtErm5MXMlmPAJQVgWta7Zx2go+8xJ0UiCb8LHHdftWyLJE0QIAIsI+UbXu67dZMjmgDGCGl1H+vpF4NSDckSIkk7Vd+sxEhBQMRU8j/12UIRhzSaUdQ+rQU5kGeFxm+hb1oh6pWWmv3uvmReDl0UnvtapVaIzo1jZbf/pD6ElLqSX+rUmOQNpJFa/r+sa4e/pBlAABoAAAAA3CUgShLdGIxsY7AUABPRrgCABdDuQ5GC7DqPQCgbbJUAoRSUj+NIEig0YfyWUho1VBBBA//uQZB4ABZx5zfMakeAAAAmwAAAAF5F3P0w9GtAAACfAAAAAwLhMDmAYWMgVEG1U0FIGCBgXBXAtfMH10000EEEEEECUBYln03TTTdNBDZopopYvrTTdNa325mImNg3TTPV9q3pmY0xoO6bv3r00y+IDGid/9aaaZTGMuj9mpu9Mpio1dXrr5HERTZSmqU36A3CumzN/9Robv/Xx4v9ijkSRSNLQhAWumap82WRSBUqXStV/YcS+XVLnSS+WLDroqArFkMEsAS+eWmrUzrO0oEmE40RlMZ5+ODIkAyKAGUwZ3mVKmcamcJnMW26MRPgUw6j+LkhyHGVGYjSUUKNpuJUQoOIAyDvEyG8S5yfK6dhZc0Tx1KI/gviKL6qvvFs1+bWtaz58uUNnryq6kt5RzOCkPWlVqVX2a/EEBUdU1KrXLf40GoiiFXK///qpoiDXrOgqDR38JB0bw7SoL+ZB9o1RCkQjQ2CBYZKd/+VJxZRRZlqSkKiws0WFxUyCwsKiMy7hUVFhIaCrNQsKkTIsLivwKKigsj8XYlwt/WKi2N4d//uQRCSAAjURNIHpMZBGYiaQPSYyAAABLAAAAAAAACWAAAAApUF/Mg+0aohSIRobBAsMlO//Kk4soosy1JSFRYWaLC4qZBYWFRGZdwqKiwkNBVmoWFSJkWFxX4FFRQWR+LsS4W/rFRb/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////VEFHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAU291bmRib3kuZGUAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAMjAwNGh0dHA6Ly93d3cuc291bmRib3kuZGUAAAAAAAAAACU=');
  //sound.muted = true;
  sound.play();
  //sound.muted = false;
}

function rxDatosJson(datos) {
  if (datos.BMP180) { return; }
  if (datos.MPU6050) { return; }
  if (datos.ADJUST) { return; }
  if (datos.GPSNEO) { 
    //desarrollar
    return; 
  }
  if (datos.ERROR) {
    insertarError(datos.ERROR);
    return;        
  }
  if (datos.PONG) {
    beep();
    return;
  }
  if (datos.vorwaerts) {
    botonARRANQUE.disabled = false;
    CONECTADO = true;
    return;
  }
  if(datos.WIFI) { 
    if (datos.Wifi > -50) {
      wifi_grafico.className="waveStrength-4";
      return;
    }
    if (datos.WIFI > -70) {
      wifi_grafico.className="waveStrength-3";
      return;
    }
    if (datos.WIFI > -90) {
      wifi_grafico.className="waveStrength-2";
      return;
    }
    wifi_grafico.className="waveStrength-1";
    return; 
  }
  if(datos.LIPO) { return; }
}

function funcionARRANQUE() {
  ++estadoARRANQUE;
  if (estadoARRANQUE>2) estadoARRANQUE = 1;
  if (estadoARRANQUE == 0) {
    document.getElementById("throttle").hidden = true;
    document.getElementById("joystick").hidden = true;
    document.getElementById("google").hidden = true;
    document.getElementById("arranque").style.stroke = "gray";
  }
  if (estadoARRANQUE == 1) {
    document.getElementById("throttle").hidden = false;
    document.getElementById("joystick").hidden = false;
    document.getElementById("google").hidden = true;
    document.getElementById("arranque").style.stroke = "#29B6F6"; //cielo
  }
  if (estadoARRANQUE == 2) {
    document.getElementById("throttle").hidden = true;
    document.getElementById("joystick").hidden = true;
    document.getElementById("google").hidden = false;
    document.getElementById("arranque").style.stroke = "#8B4513"; //tierra
  }
}
</script>
</HEAD>

<BODY>
<div class="container">
  <!-- ********** header ************************* -->
  <div class="header" id="header">
    <meter style="position:relative; float:right; margin-left:0px; margin-right:20px; top:10px; height:30px; width:45px; transform: rotate(-90deg);" value="50" max="100" min="0" high="100" low="33"></meter>
    <!-- cambiando la clase (waveStrength-4,3,2,1) para cambiar la intensidad de la senal -->
    <div id="wifi-grafico" class="waveStrength-1" style="float:right">
      <div class="wv4 wave"><div class="wv3 wave"><div class="wv2 wave"><div class="wv1 wave"></div></div></div></div>
    </div>
        <style>
        .wave {
            display: inline-block;
            border: 8px solid transparent;
            border-top-color: green; /*currentColor;*/
            border-radius: 50%;
            border-style: solid;
            margin: 4px;
        }         
        .waveStrength-3 .wv4.wave, .waveStrength-2 .wv4.wave, .waveStrength-2 .wv3.wave, .waveStrength-1 .wv4.wave, .waveStrength-1 .wv3.wave, .waveStrength-1 .wv2.wave {
            border-top-color:gray;
        }
        </style>
  <h2 style="padding-top:15px;padding-left:5px;float:left;">MULTIROTORFLEX2 (mrf2)</h2>
  </div>

  <!-- ********** modos ************************* -->
  <div class="modos">
    <button class="inputs" id="boton_de_arranque" onclick="funcionARRANQUE();" disabled>
      <svg viewBox="0 0 26 27" height="27" width="28" stroke="gray" id="arranque">
        <circle cx="13" cy="15" r="10" stroke-width="4.0" fill="none"/>
        <line x1="13" y1="0" x2="13" y2="14" stroke-width="4.0" fill="none"/>
      </svg>
    </button> 
    <!-- de "gray" a tierra (#8B4513) y al cielo (#29B6F6) -->
  </div>

  <!-- ********** errores ************************* -->
  <div class="errores">
    <span id="errores"></span>
  </div>

  <!-- ********** JOYSTICK ************************* -->
  <div class="joystick" id="joystick" hidden>
    <style type="text/css">
      #joystick-wrapper {
        width: 100%;
        height: 100%;
      }
      #joystick-base {
        position:relative;
        top: calc(50% - 150px - 35px); /* 35px son un simple ajuste */
        left: calc(50% - 150px);        
        height: 300px;
        width: 300px;
        border-radius: 100%;
        border: 2px solid black;
        cursor: pointer;
        touch-action: none;
      }
      .joystick-stick {
        position: relative;
        top: calc(50% - 25px);
        left: calc(50% - 25px);
        height: 50px;
        width: 50px;
        border-radius: 100%;
        border: 2px solid black;
        background: red;
        will-change: transform;
        touch-action: none;
        opacity:0.9;
      }
      
      .joystick-vertical {
        position:relative;
        top: calc(0% - 55px);
        left: calc(50%);       
        height: 300px;
        width: 1px;
        border-radius: 0%;
        border: 1px solid red;
        background: red;
        opacity:0.7;
      }

      .joystick-horizontal {
        position:relative;
        top: calc(0% - 205px);
        left: calc(0%);       
        height: 1px;
        width: 300px;
        border-radius: 0%;
        border: 1px solid black;
        background: black;
        opacity:0.5;
      };
    </style>

    <script>
      function JOYSTICK( parent ) {
        this.dragStart = null;
        this.currentPos = { x: 0, y: 0 };
        this.PWM =  { x: 0, y: 0 };
        this.maxDiff = 150; /* width and height of "joystick-base" = 300 / 2  */
        this.mensaje = "";
        this.stick = document.createElement('div');
        this.stick.classList.add('joystick-stick');
        parent.appendChild( this.stick );
        
        this.vertical = document.createElement('div');
        this.vertical.classList.add('joystick-vertical');
        parent.appendChild( this.vertical );

        this.horizontal = document.createElement('div');
        this.horizontal.classList.add('joystick-horizontal');
        parent.appendChild( this.horizontal );

        this.stick.addEventListener( 'mousedown', this.handleMouseDown.bind( this ) ); 
        document.addEventListener('mousemove', this.handleMouseMove.bind( this ) );
        document.addEventListener('mouseup', this.handleMouseUp.bind( this ) );
        this.stick.addEventListener('touchstart', this.handleMouseDown.bind( this) );
        document.addEventListener('touchmove', this.handleMouseMove.bind( this ) );
        document.addEventListener('touchend', this.handleMouseUp.bind( this ) );
      };
      JOYSTICK.prototype.sensibilidad = function () {
        const desviacion = 10; //en pixeles
        if (Math.abs(this.PWM.x) <= desviacion ) this.PWM.x = 0;
        if (Math.abs(this.PWM.y) <= desviacion ) this.PWM.y = 0;
        if (this.PWM.x == 0 && this.PWM.y == 0)
           return false;
        else
           return true;
      };
      JOYSTICK.prototype.handleMouseDown = function( event ) {
        this.stick.style.transition = '0s';
        if (event.changedTouches) {
        this.dragStart = {
          x: event.changedTouches[0].clientX,
          y: event.changedTouches[0].clientY,
          };
          return;
        }
        this.dragStart = {
          x: event.clientX,
          y: event.clientY,
        };
      };
      JOYSTICK.prototype.handleMouseMove = function( event ) {
        if ( this.dragStart === null ) return;
        event.preventDefault();
        if (event.changedTouches) {
          event.clientX = event.changedTouches[0].clientX;
          event.clientY = event.changedTouches[0].clientY;
        }
        const xDiff = event.clientX - this.dragStart.x;
        const yDiff = event.clientY - this.dragStart.y;
        const angle = Math.atan2( yDiff, xDiff );
        const distance = Math.min( this.maxDiff, Math.hypot( xDiff, yDiff ) );
        //const distanceOld = Math.sqrt( Math.pow( xDiff, 2 ) + Math.pow( yDiff, 2 ) ); // get the distance between the cursor and the center
        const xNew = distance * Math.cos( angle );
        const yNew = distance * Math.sin( angle );
        this.stick.style.transform = `translate3d(${xNew}px, ${yNew}px, 0px)`;
        this.currentPos = { x: xNew, y: yNew };
        this.PWM.x = (this.currentPos.x * SERVO_LIMITE / this.maxDiff).toFixed(0);
        this.PWM.y = (this.currentPos.y * SERVO_LIMITE / this.maxDiff).toFixed(0);
        if (this.sensibilidad()) {
          this.mensaje = "JS:" + this.PWM.x + "," + this.PWM.y;
          const t = Date.now(); //sleep 20 msec.
          let tt = null;
          do {
              tt = Date.now();
          } while (tt - t < 20);
          conexion1.send(this.mensaje);
          //console.log (this.mensaje+" "+(new Date()).getMilliseconds());
          //pixelsJoystickX.value = this.PWM.x;
          //pixelsJoystickY.value = this.PWM.y;
        }
      };
      JOYSTICK.prototype.handleMouseUp = function(event) {
        if ( this.dragStart === null ) return;
        this.stick.style.transition = '0.1s';
        this.stick.style.transform = `translate3d(0px, 0px, 0px)`;
        this.dragStart = null;
        this.currentPos = { x: 0, y: 0 };
        this.mensaje = "JS:" + 0 + "," + 0;        
        zero_zero_error=false;
        if (conexion1.readyState === conexion1.OPEN) {
          conexion1.send(this.mensaje);
          const t = (new Date()).getMilliseconds();
          console.log (this.mensaje+" "+t);
        }
        else zero_zero_error = true;
        //pixelsJoystickX.value = 0;
        //pixelsJoystickY.value = 0;
      };
    </script>
  
    <div id="joystick-wrapper">
      <!--
      <SMALL> 
      JOYSTICK (PWM) 
      X:<input type="text" id="pixelsJoystickX" style="width:35px;" value="0">
      Y:<input type="text" id="pixelsJoystickY" style="width:35px;" value="0">
      </SMALL>
      -->
      <div id="joystick-base"></div>
    </div>
  </div>

  <!-- ********** THROTTLE ************************* -->
  <div class="throttle" id="throttle" hidden>
    <style type="text/css">
      #throttle-wrapper {
        width: 100%;
        height: 100%;
      }
      #throttle-base {
        position:relative;
        top: calc(50% - 150px - 35px); /* 35px son un simple ajuste */
        left: calc(50% - 15px);       
        height: 300px;
        width: 30px;
        border-radius: 1%;
        border: 2px solid black;
        cursor: pointer;
        touch-action: none;
      }
      .throttle-stick {
        position: relative;
        top: calc(100% - 25px);
        left: calc(50% - 25px);
        height: 50px;
        width: 50px;
        border-radius: 100%;
        border: 2px solid black;
        background: red;
        will-change: transform;
        touch-action: none;
        opacity:0.9;
      }
      .throttle-armed {
        position:relative;
        top: calc(100% - 60px - 25px); /* ARMED*height/(ESC_LIMITE-1000) = 85*300/1000 = 25  - ajuste=60px */
        left: calc(50% - 40px);       
        height: 5px;
        width: 80px;
        border-radius: 0%;
        border: 1px solid #8B4513;
        background: #8B4513;
      }
      .throttle-hover {
        position:relative;
        top: calc(100% - 60px - 150px); /* HOVER*height/(ESC_LIMITE-1000) = 500*300/1000 = 150 - ajuste=60px */
        left: calc(50% - 40px);       
        height: 5px;
        width: 80px;
        border-radius: 0%;
        border: 1px solid #29B6F6;
        background: #29B6F6;
      };
    </style>
  
    <script>
      function THROTTLE ( parent ) {
        this.dragStart = null;
        this.dragEnd = null;
        this.firstTouch = true;
        this.currentPosY = 0;
        this.PWM = 0;
        this.mensaje = "";
        this.maxDiffY = 300; /* height of "throttle-base" = 300  */
        this.stick = document.createElement('div');
        this.stick.classList.add('throttle-stick');
        parent.appendChild( this.stick );
        
        this.armed = document.createElement('div');
        this.armed.classList.add('throttle-armed');
        parent.appendChild( this.armed );
        this.hover = document.createElement('div');
        this.hover.classList.add('throttle-hover');
        parent.appendChild( this.hover );
        
        this.stick.addEventListener( 'mousedown', this.handleMouseDown.bind( this ) ); 
        document.addEventListener('mousemove', this.handleMouseMove.bind( this ) );
        document.addEventListener('mouseup', this.handleMouseUp.bind( this ) );
        this.stick.addEventListener('touchstart', this.handleMouseDown.bind( this) );
        document.addEventListener('touchmove', this.handleMouseMove.bind( this ) );
        document.addEventListener('touchend', this.handleMouseUp.bind( this ) );
      };
      THROTTLE.prototype.handleMouseDown = function( event ) {
        this.stick.style.transition = '0s';
        if (event.changedTouches) {
          this.dragStart = {
          y: event.changedTouches[0].clientY,
          };
          if (this.firstTouch == true) this.dragEnd = this.dragStart;
          this.firstTouch = false;
          return;
        }
        this.dragStart = {
          y: event.clientY,
        };
        if (this.firstTouch == true) this.dragEnd = this.dragStart;
        this.firstTouch = false;
      };
      THROTTLE.prototype.handleMouseMove = function( event ) {
        if ( this.dragStart === null ) return;
        event.preventDefault();
        if (event.changedTouches) {
          event.clientY = event.changedTouches[0].clientY;
        }
        let yDiff = this.dragEnd.y - event.clientY;
        if (yDiff < 0) yDiff = 0;
        if (yDiff > this.maxDiffY) yDiff = this.maxDiffY;
        this.currentPosY = -1*yDiff;
        this.stick.style.transform = `translate3d(0px, ${this.currentPosY}px, 0px)`;
        //this.PWM = (1000 - this.currentPosY * (ESC_LIMITE-1000) / this.maxDiffY).toFixed(0);
        this.PWM = (1000 - this.currentPosY * (2000-1000) / this.maxDiffY).toFixed(0);
        //pixelsThrottle.value = this.PWM;
      };
      THROTTLE.prototype.handleMouseUp = function(event) {
        if ( this.dragStart === null ) return;
        this.mensaje = "THROTTLE:" + this.PWM;
        //conexion1.send(this.mensaje);
        //console.log(this.mensaje);
        this.dragStart = null;
      };
    </script>
  
    <div id="throttle-wrapper">
      <!-- <SMALL> THROTTLE (PWM) <input type="text" id="pixelsThrottle" style="width:35px;" value="0"></SMALL> -->
      <div id="throttle-base"></div>
    </div>
  </div>

    <!-- ********** GOOGLE ************************* -->
    <div id="google" class="google" hidden></div>
</div>
</BODY>
</HTML>
)";

#endif
