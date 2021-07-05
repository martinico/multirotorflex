
#ifndef FORMULARIOS_H
#define FORMULARIOS_H

static const char PROGMEM HTML[] = R"(
<!DOCTYPE html>

<HTML>

<HEAD>
<meta http-equiv="content-type" content="text/html; charset=utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:," />
<title>Test de actitud del MULTIROTORFLEX2 (mrf2)</title>

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
margin: 1px;
justify-content: space-between;
font-size: 1.0em;
color: #FFF;
border-radius: 5px;
height: 44px;
width: 155px;
}
.botons {
position: relative;
top: 5px;
border-radius: 15px;
height: 35px;
width: 100px;
}
.container {
display: grid;
height: 100vh;
grid-template-columns: 1fr 2fr 135px 500px;
grid-template-rows: 50px 50px 335px 135px 1fr 1fr;
}
.header {
grid-column : 1 / 5;
grid-row: 1 / 2;
background-color:#96b8f8;
text-align: center;
}
.modos {
grid-column : 1 / 5;
grid-row: 2 / 3;
background: #EAEAEA;
border: 1px solid black;
}
.HA1 {
grid-column : 4 / 5;
grid-row: 3 / 4;
}
.HA2 {
grid-column : 4 / 5;
grid-row: 4 / 5;
}
.HA3 {
grid-column : 3 / 4;
grid-row: 3 / 5;
}
.GPS {
grid-column : 3 / 5;
grid-row: 5 / 7;
background: #29B6F6;
border: 1px solid white;
overflow-y: scroll;
color: yellow;
}
.GPS SMALL {COLOR:#FFF;}
.errores {
grid-column : 1 / 3;
grid-row: 5 / 7;
border: 1px solid gray;
overflow-x: scroll;
overflow-y: scroll;
}
.ajustes {
grid-column: 1/3;
grid-row: 3/5;
border: 1px solid gray;
overflow-y: scroll;
}
.throttle {
grid-column : 1 / 2;
grid-row: 3 / 5;
border: 1px solid gray;
}
.joystick {
grid-column : 2 / 3;
grid-row: 3 / 5;
border: 1px solid gray;
}
.disable-elements {
pointer-events: none;
opacity: 0.4;
}
</style>

<script>
"use strict"; // global scope

const ARMED = 1090;
const UMBRAL = 10;
const INCL = 1580;
const ESC_LIMITE = 1900;
const SERVO_LIMITE = 250; //1msec = 0,4grados = 4,888pwm(5pwm) => *** 250pwm = 20,46grados (aprox) ***

var HOVER = 1450;
var HOLD = 0;
var throttle;
var QNH = 0.0;

var SVGattitude, TAGposition, TAGdial;
var SVGturning, TAGturning;
var SVGaltitude, TAGaltitude, TAGlevel;
var conexion1, conexion2, conexion3;
var botonARRANQUE, estadoARRANQUE = 0;
var pixelsJoystickX, pixelsJoystickY, degsec, PROAJoystick, POPAJoystick, autoJoystick, pixelsThrottle, PWMThrottle, autoThrottle, tmpThrottle;
var panelErrores;
var gps1, gps2, gps3, gps4, gps5, gps6, gps7, gps8, gps9, gps10;
var wifi_grafico, wifi_numerico;
var roll=0, pitch=0, yaw=0, altitud=0.0;
var tiempo; //calculo del tiempo de conexion de los websockets
var CONECTADO = false; //control booleano de la primera conexion
var zero_zero_error = false; //¿como arregar el 1500,1500?
var wifi_grafico, wifi_numerico;

function conecta1 () {
conexion1 = null;
conexion1 = new WebSocket('ws://'+location.hostname+':81/'); //MANDO y CONTROL
//conexion1 = new WebSocket('ws://192.168.1.39:81');
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
//conexion2 = new WebSocket('ws://192.168.1.39:82');
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

window.onload = function() {
const joystick = new JOYSTICK (document.getElementById('joystick-base') );
throttle = new THROTTLE (document.getElementById('throttle-base') ); //OJO

//SVGattitude=document.getElementById("attitude").getSVGDocument();
SVGattitude=document.getElementById("attitude");
TAGposition=SVGattitude.getElementById("attitude-position");
TAGdial=SVGattitude.getElementById("attitude-dial");
//SVGturning=document.getElementById("turning").getSVGDocument();
SVGturning=document.getElementById("turning");
TAGturning=SVGturning.getElementById("turning-position");
//SVGaltitude=document.getElementById("altitude").getSVGDocument();
SVGaltitude=document.getElementById("altitude");
TAGaltitude=SVGaltitude.getElementById("altitude-position");
TAGlevel=SVGaltitude.getElementById("altitude-level");
actualizarHA1 (); //roll, pitch & yaw
actualizarHA2 (); //altitud y nivel

botonARRANQUE=document.getElementById("boton_de_arranque");

panelErrores=document.getElementById("errores");

pixelsJoystickX=document.getElementById("pixelsJoystickX");
pixelsJoystickY=document.getElementById("pixelsJoystickY");
degsec=document.getElementById("degsec");
PROAJoystick=document.getElementById("PROAJoystick");
POPAJoystick=document.getElementById("POPAJoystick");
autoJoystick=document.getElementById("autoJoystick");
pixelsThrottle=document.getElementById("pixelsThrottle");
PWMThrottle=document.getElementById("PWMThrottle");
autoThrottle=document.getElementById("autoThrottle");

gps1=document.getElementById("gps1");
gps2=document.getElementById("gps2");
gps3=document.getElementById("gps3");
gps4=document.getElementById("gps4");
gps5=document.getElementById("gps5");
gps6=document.getElementById("gps6");
gps7=document.getElementById("gps7");
gps8=document.getElementById("gps8");
gps9=document.getElementById("gps9");
gps10=document.getElementById("gps10");

wifi_grafico = document.getElementById("wifi-grafico");
wifi_numerico = document.getElementById("wifi-numerico");

document.addEventListener('keydown', function(event) {
 const key = event.key; // or const {key} = event; in ES6+
 console.log ("keycode", event.keyCode);
 if (key === "Escape") {
   conexion1.send('CUTOFF:true');
 }
 if (key === "F1") {
   HOLD=0; marcadores();
 }
});

tiempo = (new Date()).getTime(); //calculo del tiempo de conexion de los websockets
conecta1();
conecta2();
};

const SerialTest = async function () {
if ("serial" in navigator) { console.log('The "serial" API is supported'); }
}

function actualizarHA1 () { //HA = Horizonte Artificial
let desp;
if (roll > 180) {
  roll=roll-360;
} else if (roll < -180) {
  roll=roll+360;
}
if (pitch > 180) {
  pitch=pitch-360;
} else if (pitch < -180) {
  pitch=pitch+360;
}
if (yaw > 180) {
  yaw=yaw-360;
} else if (yaw < -180) {
  yaw=yaw+360;
}
desp = Math.round(50+(12.5/10)*pitch);
TAGposition.setAttribute("transform","translate(50,"+ String(desp) + ") rotate("+ roll +") ");
TAGdial.setAttribute("transform","translate(50,"+ String(desp) + ") rotate("+ roll +") ");
desp = Math.round (50+(10/10)*yaw);
TAGturning.setAttribute("transform","translate("+ String(desp) + ",2) ");
}

function actualizarHA2 () { //HA = Horizonte Artificial
let desp, alt, nivel;
const SEVILLA = 11.0;
alt = altitud-QNH;
if ((altitud-QNH) > 10.9) {
  alt = 10.9;
} else if ((altitud-QNH) < -1.0) {
  alt = -1.0;
}
if (alt<1.0) {
  desp = Math.round (125-70*parseFloat(alt));
} else {
  desp = Math.round (125-18*parseFloat(1.0*(alt.toFixed(0))-1) - 70); //solo enteros
}
TAGaltitude.setAttribute("transform","translate(0,"+ String(desp)+") ");
nivel = altitud-QNH;
nivel = nivel * 100.0;
nivel = Math.round(nivel) / 100;
TAGlevel.innerHTML="NIVEL:"+nivel;
}

function devicesReset () { 
conexion1.send('RESET:true');
QNH = altitud; 
}

function actualizarGPS(gps) {
gps1.innerHTML = gps.gps1;
gps2.innerHTML = gps.gps2;
gps3.innerHTML = gps.gps3;
gps4.innerHTML = gps.gps4;
gps5.innerHTML = gps.gps5;
gps6.innerHTML = gps.gps6;
gps7.innerHTML = gps.gps7;
gps8.innerHTML = gps.gps8;
gps9.innerHTML = gps.gps9;
gps10.innerHTML = gps.gps10;
}

function insertarError (error) {
const t = new Date();
let tt = new Date().toLocaleTimeString('en-GB') + `.${t.getMilliseconds()}`;
panelErrores.innerHTML = " &nbsp; " + tt + " &emsp; " + error + '</BR>' + panelErrores.innerHTML;
}

function beep() { }

function rxDatosJson(datos) {
if (datos.MS5611) {
 altitud = 1.0 * datos.MS5611;
 actualizarHA2();
 return;
}

if (datos.MPU6050) {
 roll = 1.0 * datos.MPU6050.roll;
 pitch = 1.0 * datos.MPU6050.pitch;
 yaw = 1.0 * datos.MPU6050.yaw;
 actualizarHA1();
 return;
}

if (datos.GPSNEO) {
 actualizarGPS(datos.GPSNEO);
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
 wifi_numerico.innerHTML = datos.WIFI+"dBm";
 if (datos.WIFIi > -50) {
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
 
if(datos.LIPO) {
 //console.log(datos.LIPO);
}
 
if(datos.ADJUST) {
 document.getElementById("gravedadPITCH").value = datos.ADJUST.pitch;
 document.getElementById("gravedadROLL").value = datos.ADJUST.roll;
}
}

function funcionARRANQUE() {
++estadoARRANQUE;
document.getElementById("ocultados").hidden = false;
if (estadoARRANQUE>2) estadoARRANQUE = 1;
if (estadoARRANQUE == 0) {
document.getElementById("ajustes").hidden = true;
document.getElementById("throttle").hidden = true;
document.getElementById("joystick").hidden = true;
document.getElementById("arranque").style.stroke = "gray";
}
if (estadoARRANQUE == 1) {
document.getElementById("ajustes").hidden = false;
document.getElementById("throttle").hidden = true;
document.getElementById("joystick").hidden = true;
document.getElementById("arranque").style.stroke = "#8B4513"; //tierra
document.getElementById("modos").innerHTML = 'AJUSTES en tierra';
}
if (estadoARRANQUE == 2) {
document.getElementById("ajustes").hidden = true;
document.getElementById("throttle").hidden = false;
document.getElementById("joystick").hidden = false;
document.getElementById("arranque").style.stroke = "#29B6F6"; //cielo
document.getElementById("modos").innerHTML = 'MANDOS de vuelo';
}
}

function propulsion(tipo) {
let pwm, inclinacion;
let mensaje = "propulsion:";

if (tipo == "set") {
console.log("set");
return;
}
if(tipo == "PROA") {
pwm=document.getElementById("pwmPROA").value;
mensaje = mensaje + pwm + "," + "0" + "," + "0" + "," + "0" + "," +"0";
conexion1.send(mensaje);
return
}
if(tipo == "POPA") {
pwm=document.getElementById("pwmPOPA").value;
mensaje = mensaje + "0" + "," + pwm + "," + "0" + "," + "0" + "," +"0";
conexion1.send(mensaje);
return
}
if(tipo == "ESTRIBOR") {
pwm=document.getElementById("pwmESTRIBOR").value;
mensaje = mensaje + "0" + "," + "0" + "," + pwm + "," + "0" + "," +"0";
conexion1.send(mensaje);
return
}
if(tipo == "BABOR") {
pwm=document.getElementById("pwmBABOR").value;
mensaje = mensaje + "0" + "," + "0" + "," + "0" + "," + pwm + "," +"0";
conexion1.send(mensaje);
return
}
if(tipo == "SERVO") {
inclinacion=1*document.getElementById("inclinacionSERVO").value; //(INCL)
mensaje = mensaje + "0" + "," + "0" + "," + "0" + "," + "0" + "," + inclinacion;
conexion1.send(mensaje);
return
}
if(tipo == "JOIN") {
pwm=document.getElementById("pwmPROA").value;
mensaje = mensaje + pwm;
pwm=document.getElementById("pwmPOPA").value;
mensaje = mensaje + "," + pwm;
pwm=document.getElementById("pwmESTRIBOR").value;
mensaje = mensaje + "," + pwm;
pwm=document.getElementById("pwmBABOR").value;
mensaje = mensaje + "," + pwm;
inclinacion=1*document.getElementById("inclinacionSERVO").value; //(INCL)
mensaje = mensaje + "," + inclinacion;
conexion1.send(mensaje);
return
}
}

function pwmAgrados() {
//1msec = 0,4grados = 4,888pwm(5pwm)
let pwm, grados;
pwm=1*document.getElementById("inclinacionSERVO").value-1500;
grados = (pwm * 0.4) / 4.888;
document.getElementById("gradosSERVO").value = grados;
}

function trazas () {
let mensaje="trazas:";
if (document.getElementById("Pitch").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("Roll").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("Yaw").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("Altitudes").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";

if (document.getElementById("PROA").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("POPA").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("BABOR").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("ESTRIBOR").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("SERVO").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";

if (document.getElementById("SensoresPre").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("SensoresPos").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";

if (document.getElementById("Tiempos250").checked) mensaje=mensaje+"1,"; else mensaje=mensaje+"0,";
if (document.getElementById("Tiempos100").checked) mensaje=mensaje+"1"; else mensaje=mensaje+"0";
conexion1.send(mensaje);
}

function parametros () {
let mensaje="parametros:";

mensaje += document.getElementById("SPpitch").value + ",";
mensaje += document.getElementById("KPpitch").value + ",";
mensaje += document.getElementById("KIpitch").value + ",";
mensaje += document.getElementById("KDpitch").value + ",";
mensaje += document.getElementById("aggKPpitch").value + ",";
mensaje += document.getElementById("aggPitchAngle").value + ",";

mensaje += document.getElementById("SProll").value + ",";
mensaje += document.getElementById("KProll").value + ",";
mensaje += document.getElementById("KIroll").value + ",";
mensaje += document.getElementById("KDroll").value + ",";
mensaje += document.getElementById("aggKProll").value + ",";
mensaje += document.getElementById("aggRollAngle").value + ",";

mensaje += document.getElementById("KPyaw").value + ",";
mensaje += document.getElementById("KIyaw").value + ",";
mensaje += document.getElementById("KDyaw").value + ",";
mensaje += document.getElementById("DRCG").value + ",";

mensaje += document.getElementById("KPASCaltitude").value + ",";
mensaje += document.getElementById("KPDESaltitude").value + ",";
mensaje += document.getElementById("KIaltitude").value + ",";
mensaje += document.getElementById("KDaltitude").value + ",";
mensaje += document.getElementById("PWMlimit").value;

conexion1.send(mensaje);
}

function gravedad () {
let mensaje="GRAVITY:";
mensaje += document.getElementById("gravedadPITCH").value +",";
mensaje += document.getElementById("gravedadROLL").value;
conexion1.send(mensaje);
}

function drift () {
let mensaje="drift:";
mensaje += document.getElementById("SFFpr").value+",";
mensaje += document.getElementById("SFFy").value+",";
mensaje += document.getElementById("gananciaP").value+",";
mensaje += document.getElementById("gananciaR").value+",";
mensaje += document.getElementById("gananciaY").value+",";
mensaje += document.getElementById("sensibilidad").value+",";
mensaje += document.getElementById("FINE").value+",";
mensaje += document.getElementById("PEAKS").value+",";
mensaje += document.getElementById("1G").value+",";
mensaje += document.getElementById("9G").value+",";
mensaje += document.getElementById("ATT").value+",";
mensaje += document.getElementById("AMP").value+",";
mensaje += document.getElementById("parachute").value+",";
mensaje += document.getElementById("newFLL").value;
conexion1.send(mensaje);
}

function hover () {
let mensaje="hover:";
mensaje += document.getElementById("newHOVER").value;
conexion1.send(mensaje);
HOVER = 1*document.getElementById("newHOVER").value; //variable global
}

function swing (elemento) {
 if (elemento.id == "HOME") {
  if (elemento.name == "NO") {
   elemento.name = "SI";
   elemento.style.opacity=1.0;
  } else {
   elemento.name = "NO";
   elemento.style.opacity=0.2;
  }
 }
 
 if (elemento.id == "GYRO") {
  if (elemento.name == "NO") {
   elemento.name = "SI";
   elemento.style.opacity=1.0;
   document.getElementById("degsec_titulo").hidden = false;
   degsec.hidden = false;
   //console.log('swingYAW:0->1');
   conexion1.send('swingYAW:1');
  } else {
   elemento.name = "NO";
   elemento.style.opacity=0.2;
   document.getElementById("degsec_titulo").hidden = true;
   degsec.hidden = true;
   //console.log('swingYAW:1->0');
   conexion1.send('swingYAW:0');
  }
  return;
 }

 if (elemento.id == "HOVER") {
  throttle.setPos(HOVER);
  conexion1.send('swingHOVER:1');
  //conexion1.send('swingHOVER:0');
  return;
 }

if (elemento.id == "AIR") {
++HOLD;
let kk = document.getElementById("newFLL").value;
marcadores(kk);
return;
}
if (elemento.id == "LAND") {
--HOLD;
let kk = document.getElementById("parachute").value;
marcadores(kk);
return;
}

}

function marcadores (kk) {
document.getElementById("zAIR").innerHTML = '0';
document.getElementById("zLAND").innerHTML = '0';
if (HOLD > 0) document.getElementById("zAIR").innerHTML = HOLD;
if (HOLD < 0) document.getElementById("zLAND").innerHTML = HOLD;
//conexion1.send('swingHOLD:' + HOLD); //logica de niveles
let valor = document.getElementById('pixelsThrottle').value;
document.getElementById('pixelsThrottle').value = 1*valor + 1*kk;
throttle.setPos(document.getElementById('pixelsThrottle').value);
conexion1.send('THROTTLE:'+document.getElementById('pixelsThrottle').value);
return;
}
</script>

</script>
</HEAD>

<BODY>
<div class="container">

 <!-- ***** header ***** -->
 <div class="header" id="header">
   <span style="cursor:pointer;float:right;text-align:center;font-size:35px;" onclick='openGoogle();'><h2>&equiv;&nbsp;</h2></span> 
   <label style="float:right;"> &emsp; </label>

   <meter style="position:relative; float:right; margin-left:0px; margin-right:20px; top:10px; height:30px; width:45px; transform: rotate(-90deg);" value="50" max="100" min="0" high="100" low="33"></meter>
   <label style="position:relative; float:right; margin-left:0px; margin-right:0px; top:5px;">xx,xV</label>

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
   <label id="wifi-numerico" style="position:relative; float:right; margin-left:0px; margin-right:0px; top:5px;">-xxdBm</label>

   <label id="" style="cursor:pointer; position:relative; float:right; margin-left:0px; margin-right:15px; top:0px;" onclick="SerialTest();">
    <svg viewBox="0 0 40 50" width="40" height="50" stroke="gray" stroke-width="3.0" fill="gray" id="USB">
     <circle cx="20" cy="42" r="4" />
     <line x1="20" y1="42" x2="20" y2="5" />
     <polygon points="20,5  24,10  16,10"></polygon>
     <line x1="20" y1="35" x2="5" y2="25" /> <line x1="5" y1="25" x2="5" y2="20"  /> <circle cx="5" cy="22" r="2" />
     <line x1="20" y1="30" x2="35" y2="25"/> <line x1="35" y1="25" x2="35" y2="15"/> <rect x="33" y="15" width="4" height="3"/>
    </svg>
   </label>

   <h1 style="padding-top:10px;">MULTIROTORFLEX2 (mrf2)</h1>
 </div>

 <!-- ***** modos ***** -->
 <div class="modos">
   <button class="inputs" style="vertical-align:-50%; cursor: pointer;" id="boton_de_arranque" onclick="funcionARRANQUE();" disabled> <!-- OJO:quitar el "disabled" para pruebas en un server local -->
   <svg viewBox="0 0 26 27" height="27" width="28" stroke="gray" id="arranque">
   <circle cx="13" cy="15" r="10" stroke-width="4.0" fill="none"/>
   <line x1="13" y1="0" x2="13" y2="14" stroke-width="4.0" fill="none"/>
   </svg>
   <span style="color:black;" id="modos"></span>
   </button>

   <!-- de "gray" a tierra (#8B4513) y al cielo (#29B6F6) -->
   <division7 id="ocultados" hidden>
   <button class="botons" style="width:190px;margin-left:50px;cursor: pointer;" onclick="conexion1.send('CUTOFF:true');"><h2> CUTOFF <small>(1000pwm)</small></h2></button>
   <button class="botons" style="width:190px;margin-left:5px;cursor: pointer;" onclick="conexion1.send('ARMED:true');"><h2> ARMED <small>(1090pwm)</small></h2></button>

   <button class="inputs" name="NO" id="HOME" style="width:75px;vertical-align:-50%;margin-left:50px;cursor:pointer;opacity:0.2;" onclick="swing(this);">
   <svg viewBox="0 0 50 30" height="28" width="50" stroke="#8B4513" fill="#8B4513" stroke-width="1.0">
   <polygon points="25,2  50,15  40,15  40,30  30,30  30,20  20,20  20,30  10,30  10,15  0,15"></polygon>
   <line x1="0" y1="29" x2="50" y2="29"/>
   </svg>
   </button>
   <button class="inputs" name="NO" id="GYRO" style="width:75px;vertical-align:-50%;margin-left:5px;cursor:pointer;opacity:0.2;" onclick="swing(this);">
   <svg viewBox="0 0 50 30" height="30" width="60" stroke="#29B6F6" fill="#29B6F6" stroke-width="0.5">
   <text text-anchor="middle" font-size="28" x="25" y="25">&#128260;</text>
   </svg>
   </button>

   <button class="inputs" id="HOVER" style="width:75px;vertical-align:-50%;margin-left:50px;cursor:pointer;" onclick="swing(this);" HIDDEN>
   <svg viewBox="0 0 50 30" height="30" width="60" stroke="#29B6F6" fill="#29B6F6" stroke-width="1.5">
   <text text-anchor="middle" font-size="16" x="24" y="24">HOVER</text>
   </svg>
   </button>

   <button class="inputs" id="AIR" style="width:75px;vertical-align:-50%;margin-left:50px;cursor:pointer;" onclick="swing(this);" HIDDEN>
   <svg viewBox="0 0 50 30" height="28" width="50" stroke="#29B6F6" fill="#29B6F6" stroke-width="1.0">
   <g transform='translate(0,15) rotate(-30.0)' >
   <polygon points="5,15 45,15 35,10 15,10 10,5 5,5 "></polygon>
   </g>
   <line x1="0" y1="29" x2="50" y2="29"/>
   <text id="zAIR" x="1" y="9">+0</text>
   </svg>
   </button>
   <button class="inputs" id="HOLD" style="width:75px;vertical-align:-50%;margin-left:5px;opacity:0.2;" onclick="" HIDDEN>
   <svg viewBox="0 0 50 30" height="30" width="60" stroke="gray" fill="gray" stroke-width="1.5">
   <text text-anchor="middle" font-size="16" x="24" y="24">HOLD</text>
   </svg>
   </button>
   <button class="inputs" id="LAND" style="width:75px;vertical-align:-50%;margin-left:5px;cursor:pointer;" onclick="swing(this);" HIDDEN>
   <svg viewBox="0 0 50 30" height="28" width="50" stroke="#8B4513" fill="#8B4513" stroke-width="1.0">
   <g transform='translate(5,-5) rotate(20.0)' >
   <polygon points="5,15 45,15 35,10 15,10 10,5 5,5 "></polygon>
   </g>
   <line x1="0" y1="29" x2="50" y2="29"/>
   <text id="zLAND" x="32" y="9">-0</text>
   </svg>
   </button>
   </division7>
 </div>

<!-- ***** HA1 ***** -->
<div class = "HA1">
<!-- <object id="attitude" data="attitude.svg" type="image/svg+xml">Error importing a SVG file</object> -->
<svg viewBox="0 0 100 100" height="335px" width="500px" id="attitude">
<g font-size="5" text-anchor="middle" font-family="sans-serif" transform="translate(50,62.5) rotate(-0.0) " fill="#fff" stroke="#fff" stroke-width="1" id="attitude-position" >
<rect height="600" width="600" y="-300" x="-300" fill="#29B6F6"></rect> <!-- cielo -->
<rect height="600" width="600" y="0" x="-300" fill="#8B4513"></rect>    <!-- tierra -->
<rect height="75" width="600" y="-300" x="-300" fill="#8B4513"></rect>  <!-- tierra arriba -->
<rect height="75" width="600" y="225" x="-300" fill="#29B6F6"></rect>   <!-- cielo abajo -->
<g stroke-width="0.1">
  <g font-size="3">
    <text y="-223.5" x="0">180</text>
    <text y="-211" x="0">170</text>
    <text y="-198.5" x="0">160</text>
    <text y="-186" x="0">150</text>
    <text y="-173.5" x="0">140</text>
    <text y="-161" x="0">130</text>
    <text y="-148.5" x="0">120</text>
    <text y="-136" x="0">110</text>
    <text y="-123.5" x="0">100</text>
    <text y="126.5" x="0">100</text>
    <text y="139" x="0">110</text>
    <text y="151.5" x="0">120</text>
    <text y="164" x="0">130</text>
    <text y="176.5" x="0">140</text>
    <text y="189" x="0">150</text>
    <text y="201.5" x="0">160</text>
    <text y="214" x="0">170</text>
    <text y="226.5" x="0">180</text>
  </g>
  <text y="-110.5" x="0">90</text>
  <text y="-98" x="0">80</text>
  <text y="-85.5" x="0">70</text>
  <text y="-73" x="0">60</text>
  <text y="-60.5" x="0">50</text>
  <text y="-48" x="0">40</text>
  <text y="-35.5" x="0">30</text>
  <text y="-23" x="0">20</text>
  <text y="-10.5" x="0">10</text>
  <text y="14.5" x="0">10</text>
  <text y="27" x="0">20</text>
  <text y="39.5" x="0">30</text>
  <text y="52" x="0">40</text>
  <text y="64.5" x="0">50</text>
  <text y="77" x="0">60</text>
  <text y="89.5" x="0">70</text>
  <text y="102" x="0">80</text>
  <text y="114.5" x="0">90</text>
</g>
<line y2="-225" x2="-4" y1="-225" x1="-6"></line>
<line y2="-225" x2="6" y1="-225" x1="4"></line>
<line y2="-212.5" x2="-4" y1="-212.5" x1="-6"></line>
<line y2="-212.5" x2="6" y1="-212.5" x1="4"></line>
<line y2="-200" x2="-4" y1="-200" x1="-6"></line>
<line y2="-200" x2="6" y1="-200" x1="4"></line>
<line y2="-187.5" x2="-4" y1="-187.5" x1="-6"></line>
<line y2="-187.5" x2="6" y1="-187.5" x1="4"></line>
<line y2="-175" x2="-4" y1="-175" x1="-6"></line>
<line y2="-175" x2="6" y1="-175" x1="4"></line>
<line y2="-162.5" x2="-4" y1="-162.5" x1="-6"></line>
<line y2="-162.5" x2="6" y1="-162.5" x1="4"></line>
<line y2="-150" x2="-4" y1="-150" x1="-6"></line>
<line y2="-150" x2="6" y1="-150" x1="4"></line>
<line y2="-137.5" x2="-4" y1="-137.5" x1="-6"></line>
<line y2="-137.5" x2="6" y1="-137.5" x1="4"></line>
<line y2="-125" x2="-4" y1="-125" x1="-6"></line>
<line y2="-125" x2="6" y1="-125" x1="4"></line>
<line y2="-118.75" x2="5" y1="-118.75" x1="-5"></line>
<line y2="-112.5" x2="-4" y1="-112.5" x1="-10"></line>
<line y2="-112.5" x2="10" y1="-112.5" x1="4"></line>
<line y2="-106.25" x2="5" y1="-106.25" x1="-5"></line>
<line y2="-100" x2="-4" y1="-100" x1="-10"></line>
<line y2="-100" x2="10" y1="-100" x1="4"></line>
<line y2="-93.75" x2="5" y1="-93.75" x1="-5"></line>
<line y2="-87.5" x2="-4" y1="-87.5" x1="-10"></line>
<line y2="-87.5" x2="10" y1="-87.5" x1="4"></line>
<line y2="-81.25" x2="5" y1="-81.25" x1="-5"></line>
<line y2="-68.75" x2="5" y1="-68.75" x1="-5"></line>
<line y2="-75" x2="-4" y1="-75" x1="-10"></line>
<line y2="-75" x2="10" y1="-75" x1="4"></line>
<line y2="-56.25" x2="5" y1="-56.25" x1="-5"></line>
<line y2="-62.5" x2="-4" y1="-62.5" x1="-10"></line>
<line y2="-62.5" x2="10" y1="-62.5" x1="4"></line>
<line y2="-43.75" x2="5" y1="-43.75" x1="-5"></line>
<line y2="-50" x2="-4" y1="-50" x1="-10"></line>
<line y2="-50" x2="10" y1="-50" x1="4"></line>
<line y2="-31.25" x2="5" y1="-31.25" x1="-5"></line>
<line y2="-37.5" x2="-4" y1="-37.5" x1="-10"></line>
<line y2="-37.5" x2="10" y1="-37.5" x1="4"></line>
<line y2="-18.75" x2="5" y1="-18.75" x1="-5"></line>
<line y2="-25" x2="-4" y1="-25" x1="-10"></line>
<line y2="-25" x2="10" y1="-25" x1="4"></line>
<line y2="-6.25" x2="5" y1="-6.25" x1="-5"></line>
<line y2="-12.5" x2="-4" y1="-12.5" x1="-10"></line>
<line y2="6.25" x2="5" y1="6.25" x1="-5"></line>
<line y2="-12.5" x2="10" y1="-12.5" x1="4"></line>
<line y2="18.75" x2="5" y1="18.75" x1="-5"></line>
<line y2="12.5" x2="-4" y1="12.5" x1="-10"></line>
<line y2="12.5" x2="10" y1="12.5" x1="4"></line>
<line y2="31.25" x2="5" y1="31.25" x1="-5"></line>
<line y2="25" x2="-4" y1="25" x1="-10"></line>
<line y2="25" x2="10" y1="25" x1="4"></line>
<line y2="43.75" x2="5" y1="43.75" x1="-5"></line>
<line y2="37.5" x2="-4" y1="37.5" x1="-10"></line>
<line y2="37.5" x2="10" y1="37.5" x1="4"></line>
<line y2="56.25" x2="5" y1="56.25" x1="-5"></line>
<line y2="50" x2="-4" y1="50" x1="-10"></line>
<line y2="50" x2="10" y1="50" x1="4"></line>
<line y2="68.75" x2="5" y1="68.75" x1="-5"></line>
<line y2="62.5" x2="-4" y1="62.5" x1="-10"></line>
<line y2="62.5" x2="10" y1="62.5" x1="4"></line>
<line y2="81.25" x2="5" y1="81.25" x1="-5"></line>
<line y2="75" x2="-4" y1="75" x1="-10"></line>
<line y2="75" x2="10" y1="75" x1="4"></line>
<line y2="93.75" x2="5" y1="93.75" x1="-5"></line>
<line y2="87.5" x2="-4" y1="87.5" x1="-10"></line>
<line y2="87.5" x2="10" y1="87.5" x1="4"></line>
<line y2="106.25" x2="5" y1="106.25" x1="-5"></line>
<line y2="100" x2="-4" y1="100" x1="-10"></line>
<line y2="100" x2="10" y1="100" x1="4"></line>
<line y2="118.75" x2="5" y1="118.75" x1="-5"></line>
<line y2="112.5" x2="-4" y1="112.5" x1="-10"></line>
<line y2="112.5" x2="10" y1="112.5" x1="4"></line>
<line y2="125" x2="-4" y1="125" x1="-6"></line>
<line y2="125" x2="6" y1="125" x1="4"></line>
<line y2="137.5" x2="-4" y1="137.5" x1="-6"></line>
<line y2="137.5" x2="6" y1="137.5" x1="4"></line>
<line y2="150" x2="-4" y1="150" x1="-6"></line>
<line y2="150" x2="6" y1="150" x1="4"></line>
<line y2="162.5" x2="-4" y1="162.5" x1="-6"></line>
<line y2="162.5" x2="6" y1="162.5" x1="4"></line>
<line y2="175" x2="-4" y1="175" x1="-6"></line>
<line y2="175" x2="6" y1="175" x1="4"></line>
<line y2="187.5" x2="-4" y1="187.5" x1="-6"></line>
<line y2="187.5" x2="6" y1="187.5" x1="4"></line>
<line y2="200" x2="-4" y1="200" x1="-6"></line>
<line y2="200" x2="6" y1="200" x1="4"></line>
<line y2="212.5" x2="-4" y1="212.5" x1="-6"></line>
<line y2="212.5" x2="6" y1="212.5" x1="4"></line>
<line y2="225" x2="-4" y1="225" x1="-6"></line>
<line y2="225" x2="6" y1="225" x1="4"></line>
</g>
<g text-anchor="middle" font-size="8" font-family="sans-serif" stroke-width="2" stroke="#ff0" fill="#fff" >
<line y2="50" x2="43" y1="50" x1="30"></line>
<line y2="53" x2="42" y1="50" x1="42"></line>
<line y2="50" x2="51" y1="50" x1="49"></line>
<line y2="50" x2="58" y1="53" x1="58"></line>
<line y2="50" x2="70" y1="50" x1="57"></line>
</g> 
<g transform = "translate(50,62.5)rotate(-0.0) " stroke="#fff" stroke-width="1" id="attitude-dial" >
<g stroke-width="2">
<line y2="-42.44" x2="24.5" y1="-34.64" x1="20"></line>
<line y2="-24.5" x2="42.44" y1="-20" x1="34.64"></line>
<line y2="0" x2="49" y1="0" x1="40"></line>
<line y2="-42.44" x2="-24.5" y1="-34.64" x1="-20"></line>
<line y2="-24.5" x2="-42.44" y1="-20" x1="-34.64"></line>
<line y2="0" x2="-49" y1="0" x1="-40"></line>
</g>
<line y2="-44.32" x2="7.81" y1="-39.39" x1="6.95"></line>
<line y2="-42.29" x2="15.39" y1="-37.59" x1="13.68"></line>
<line y2="-31.82" x2="31.82" y1="-28.28" x1="28.28"></line>
<line y2="-44.32" x2="-7.81" y1="-39.39" x1="-6.95"></line>
<line y2="-42.29" x2="-15.39" y1="-37.59" x1="-13.68"></line>
<line y2="-31.82" x2="-31.82" y1="-28.28" x1="-28.28"></line>
<line y2="-45" x2="0" y1="-40" x1="0"></line>
<path d = "M-3 -45 L3 -45 L 0 -40 z" fill="#fff"></path>
</g>
<path d = "M47 17 L53 17 L50 12 z" fill="#ff0" stroke="#ff0" stroke-width="1"></path>
</svg>
</div>

<!-- ***** HA2 ***** -->
<div class="HA2">
<!-- <object id="turning" data="turning.svg" type="image/svg+xml">Error importing a SVG file</object> -->
<svg viewBox="0 0 100 100" height="135px" width="500px" id="turning">
<g font-size="14" text-anchor="middle" font-family="sans-serif" fill="#fff" stroke="#fff" stroke-width= "1.0" >
<rect height="100" width="500" y="0" x="-200" fill="#29B6F6"></rect> <!-- cielo -->
<!--<rect height="100" width="500" y="0" x="-200" fill="#8B4513"></rect> --> <!-- tierra -->
<line stroke-width="3" y2="1" x2="-200" y1="1" x1="300"></line>
<line stroke-width="2" y2="1" x2="50" y1="20" x1="50"></line>
<text stroke-width="0.1" y="35" x="50"> 0 </text>
<text stroke-width="0.1" y="35" x="140"> 90 </text>
<text stroke-width="0.1" y="35" x="-40"> -90 </text>
<line stroke-width="2" y2="1" x2="140" y1="18" x1="140"></line>
<line stroke-width="2" y2="1" x2="-40" y1="20" x1="-40"></line>
<g font-size="7" stroke-width="0.1">
 <text y="18" x="60"> 10 </text>
 <text y="18" x="70"> 20 </text>
 <text y="18" x="80"> 30 </text>
 <text y="18" x="90"> 40 </text>
 <text y="18" x="100"> 50 </text>
 <text y="18" x="110"> 60 </text>
 <text y="18" x="120"> 70 </text>
 <text y="18" x="130"> 80 </text>
 <text y="18" x="40"> -10 </text>
 <text y="18" x="30"> -20 </text>
 <text y="18" x="20"> -30 </text>
 <text y="18" x="10"> -40 </text>
 <text y="18" x="0"> -50 </text>
 <text y="18" x="-10"> -60 </text>
 <text y="18" x="-20"> -70 </text>
 <text y="18" x="-30"> -80 </text>
</g>
<line y2= "1" x2="60" y1="12" x1="60"></line>
<line y2="1" x2="70" y1="12" x1="70"></line>
<line y2= "1" x2="80" y1="12" x1="80"></line>
<line y2= "1" x2="90" y1="12" x1="90"></line>
<line y2="1" x2="100" y1="12" x1="100"></line>
<line y2="1" x2="110" y1="12" x1="110"></line>
<line y2="1" x2="120" y1="12" x1="120"></line>
<line y2="1" x2="130" y1="12" x1="130"></line>
<line y2="1" x2="150" y1="15" x1="150"></line>
<line y2="1" x2="160" y1="15" x1="160"></line>
<line y2="1" x2="170" y1="15" x1="170"></line>
<line y2="1" x2="180" y1="15" x1="180"></line>
<line y2="1" x2="190" y1="15" x1="190"></line>
<line y2="1" x2="200" y1="15" x1="200"></line>
<line y2="1" x2="210" y1="15" x1="210"></line>
<line y2="1" x2="220" y1="15" x1="220"></line>
<line y2="1" x2="230" y1="15" x1="230"></line> <!-- limite180 -->
<line y2="1" x2="40" y1="12" x1= "40"></line>
<line y2="1" x2="30" y1="12" x1="30"></line>
<line y2="1" x2="20" y1="12" x1="20"></line>
<line y2="1" x2="10" y1="12" x1="10"></line>
<line y2="1" x2="0" y1="12" x1="0"></line>
<line y2="1" x2="-10" y1="12" x1="-10"></line>
<line y2="1" x2="-20" y1="12" x1="-20"></line>
<line y2="1" x2="-30" y1="12" x1="-30"></line>
<line y2="1" x2="-50" y1="15" x1="-50"></line>
<line y2="1" x2="-60" y1="15" x1="-60"></line>
<line y2="1" x2="-70" y1="15" x1="-70"></line>
<line y2="1" x2="-80" y1="15" x1="-80"></line>
<line y2="1" x2="-90" y1="15" x1="-90"></line>
<line y2="1" x2="-100" y1="15" x1="-100"></line>
<line y2="1" x2="-110" y1="15" x1="-110"></line>
<line y2="1" x2="-120" y1="15" x1="-120"></line>
<line y2="1" x2="-130" y1="15" x1="-130"></line> <!-- limite180 -->
</g>
<g transform="translate(50,2) " stroke="yellow" stroke-width="1" id="turning-position" >
<path d = "M0 35 L-10 50 L10 50 z" fill="yellow"></path>
<path d = "M0 0 L0 100 z" fill="yellow"></path>
</g>
</svg>
</div>

<!-- ***** HA3 ***** -->
<div class="HA3">
<!-- <object id="altitude" data="altitude.svg" type="image/svg+xml">Error importing a SVG file</object> -->
<svg viewBox="0 0 100 100" height="470px" width="135" id="altitude">
<g font-size="14" text-anchor="middle" font-family="sans-serif" fill="#fff" stroke="#fff" stroke-width="1.0" >
<rect height="470" width="135" y="-200" x="0" fill="#29B6F6"></rect> <!-- cielo -->
<line stroke-width="2" x1="100" y1="-200" x2="100" y2="470"></line> <!-- SEPARADOR -->
<rect height="320" width="4" y="-125" x="81" fill="#FFF"></rect> <!-- barra vertical -->
<g stroke-width="0.1">
 <text y="-102" x="33"> 10m </text>
 <text y="-84" x="33" > 9m </text>
 <text y="-66" x="33"> 8m </text>
 <text y="-48" x="33"> 7m </text>
 <text y="-30" x="33"> 6m </text>
 <text y="-12" x="33"> 5m </text>
 <text y="6" x="33"> 4m </text>
 <text y="24" x="33"> 3m </text>
 <text y="42" x="33"> 2m </text>
 <text y="60" x="33"> 1m </text>
 <text y="130" x="33"> GND </text>
 <text y="200" x="33"> -1m </text>    
</g>
<g stroke-width="1">
 <line y2="-124" x2="80" y1="-124" x1="50"></line>
 <line y2="-107" x2="80" y1="-107" x1="50"></line>
 <line y2="-89" x2="80" y1="-89" x1="50"></line>
 <line y2="-71" x2="80" y1="-71" x1="50"></line>
 <line y2="-53" x2="80" y1="-53" x1="50"></line>
 <line y2="-35" x2="80" y1="-35" x1="50"></line>
 <line y2="-17" x2="80" y1="-17" x1="50"></line>
 <line y2="1" x2="80" y1="1" x1="50"></line>
 <line y2="19" x2="80" y1="19" x1="50"></line>
 <line y2="37" x2="80" y1="37" x1="50"></line>
 <line y2="55" x2="80" y1="55" x1="50"></line>
 <line y2="62" x2="80" y1="62" x1="70"></line>
 <line y2="69" x2="80" y1="69" x1="70"></line>
 <line y2="76" x2="80" y1="76" x1="70"></line>
 <line y2="83" x2="80" y1="83" x1="70"></line>
 <line y2="90" x2="80" y1="90" x1="60"></line>
 <line y2="97" x2="80" y1="97" x1="70"></line>
 <line y2="104" x2="80" y1="104" x1="70"></line>
 <line y2="111" x2="80" y1="111" x1="70"></line>
 <line y2="118" x2="80" y1="118" x1="70"></line>
 <line y2="125" x2="80" y1="125" x1="50"></line>
 <line y2="132" x2="80" y1="132" x1="70"></line>
 <line y2="139" x2="80" y1="139" x1="70"></line>
 <line y2="146" x2="80" y1="146" x1="70"></line>
 <line y2="153" x2="80" y1="153" x1="70"></line>
 <line y2="160" x2="80" y1="160" x1="60"></line>
 <line y2="167" x2="80" y1="167" x1="70"></line>
 <line y2="174" x2="80" y1="174" x1="70"></line>
 <line y2="181" x2="80" y1="181" x1="70"></line>
 <line y2="188" x2="80" y1="188" x1="70"></line>
 <line y2="195" x2="80" y1="195" x1="50"></line>    
</g>
<text stroke-width="0.5" y="218" x="50" id="altitude-level"> NIVEL = XX,X </text>
</g>
<g transform="translate(0,110) " stroke="yellow" stroke-width="1" id="altitude-position" >
<path d = "M0 0 L100 0 z" fill="yellow"></path>
<path d = "M85 0 L100 -10 L100 10" fill="yellow"></path>
</g>
</svg>
</div>

<!-- ***** valores ***** -->
<div class="GPS">
<h2>
<span><small> Fecha (ddmmaa) : &emsp;</small></span>                    <span id="gps1"></span></br>
<span><small> Hora (hhmmsscc) : &ensp;</small></span>                   <span id="gps2"></span></br>
<span><small> Latitud (gr,m+s) : &emsp;&nbsp;</small></span>            <span id="gps3"></span></br>
<span><small> Longitud (gr,m+s) : &ensp;</small></span>                 <span id="gps4"></span></br>
<span><small>&emsp; HDOP x 100 : &emsp;&emsp;</small></span>            <span id="gps5"></span></br>
<span><small> Altitud (metros) : &emsp;&ensp;</small></span>            <span id="gps6"></span></br>
<span><small>&emsp; VDOP x 100 : &emsp;&emsp;&nbsp;</small></span>      <span id="gps7"></span></br>
<span><small> Velocidad (kms/h) : &ensp;</small></span>                 <span id="gps8"></span></br>
<span><small> Direccion (grados) : &nbsp;</small></span>                <span id="gps9"></span> ( declinacion = 1,36W (ENAIRE)) </br>
<span><small> Satelites : &emsp;&emsp;&emsp;&emsp;&emsp;</small></span> <span id="gps10"></span></br>
</h2>
</div>

<!-- ***** errores ***** -->
<div class="errores">
<span id="errores"></span>
</div>

<!-- ***** ajustes ***** -->
<div class="ajustes" id="ajustes" hidden>
</BR>
<fieldset>
<legend><h2>Tests de los ESC (SimonK) y del SERVO (RDS3115)</h2></legend>
<input type="button" value="Test Propeller PROA" onclick="conexion1.send('testPROA:false');">
&emsp;
<input type="button" value="Test Propeller POPA" onclick="conexion1.send('testPOPA:false');">
&emsp;
<input type="button" value="Test Propeller ESTRIBOR" onclick="conexion1.send('testESTRIBOR:false');">
&emsp;
<input type="button" value="Test Propeller BABOR" onclick="conexion1.send('testBABOR:false');">
</BR></BR>
<input type="button" value="Test SERVOMOTOR" onclick="conexion1.send('testSERVO:false');">
</BR></BR></BR>
<input type="button" value="¡ SYNCRO Propellers  & Servos !" onclick="conexion1.send('SYNCmotor:false');">
</fieldset></BR>

<fieldset>
<legend><h2>PROPULSION</h2></legend>
</br>
<fieldset>
  <legend><strong>ESC (SimonK)</strong></legend>
  Valores iniciales de PWM coincidentes con la I<sub>0</sub> de arranque.
  <TABLE>
  <TR>
  <TD>PWMs PROA</TD>
  <TD><input id="pwmPROA" type="number" min="1000" max="2000" value="1077" step="1" size="4"></TD>
  <TD><input type="button" value="SET PROA" onclick="propulsion('PROA');"></TD>
  </TR>
  <TR>
  <TD>PWMs POPA</TD>
  <TD><input id="pwmPOPA" type="number" min="1000" max="2000" value="1083" step="1" size="4"></TD>
  <TD><input type="button" value="SET POPA" onclick="propulsion('POPA');"></TD>
  </TR>
  <TR>
  <TD>PWMs ESTRIBOR</TD>
  <TD><input id="pwmESTRIBOR" type="number" min="1000" max="2000" value="1088" step="1" size="4"></TD>
  <TD><input type="button" value="SET ESTRIBOR" onclick="propulsion('ESTRIBOR');"></TD>
  </TR>
  <TR>
  <TD>PWMs BABOR</TD>
  <TD><input id="pwmBABOR" type="number" min="1000" max="2000" value="1082" step="1" size="4"></TD>
  <TD><input type="button" value="SET BABOR" onclick="propulsion('BABOR');"></TD>
  </TR>
  </TABLE>
</fieldset>
</br>
<fieldset>
  <legend><strong>SERVO (RDS3115)</strong></legend>
  Inclinacion
  <input id="inclinacionSERVO" type="number" min="1250" max="1750" value="1500" step="1" size="4" onchange="pwmAgrados();">
  en PWMs y equivalentes a
  <input type="text" id="gradosSERVO" value="0.00" disabled>
  grados
  <strong>(+ estribor/proa y - babor/popa)</strong></br></br>
  <input type="button" value="SET SERVO" onclick="propulsion('SERVO');">
</fieldset>
</BR></BR>
<input type="button" value="SET-JOIN ESC+SERVO" onclick="propulsion('JOIN');">
</fieldset></BR>

<fieldset>
<legend><h2>TRAZAS PLOTTER/CONSOLE</h2></legend>
<input id="Pitch" type="checkbox">Pitch (angulos y aceleraciones)</br>
<input id="Roll" type="checkbox">Roll ()</br>
<input id="Yaw" type="checkbox">Yaw ()</br>
<input id="Altitudes" type="checkbox">Altitud 'Kalman' (GND y REF)</br></br>
<input id="PROA" type="checkbox">PROA (PWM)</br>
<input id="POPA" type="checkbox">POPA ()</br>
<input id="BABOR" type="checkbox">BABOR ()</br>
<input id="ESTRIBOR" type="checkbox">ESTRIBOR ()</br>
<input id="SERVO" type="checkbox">SERVO (angulos)</br></br>
<input id="Tiempos250" type="checkbox">Tiempos de proceso del bucle 250HZ</br>
<input id="Tiempos100" type="checkbox">Tiempos de proceso del bucle 100HZ</br></br>
<input id="SensoresPre" type="checkbox" checked>Fusion de sensores PRE</br>
<input id="SensoresPos" type="checkbox" checked>Fusion de sensores POS</br></br>
<input type="button" value="SET TRAZAS" onclick="trazas();">
</fieldset></BR>

<fieldset>
<legend><h2>PID</h2></legend>
</br><fieldset>
  <legend><strong>PITCH</strong></legend>
  Setpoint (PWM) <input id="SPpitch" type="number" min="-250" max="250" value="0" step="1" size="4"> 
  K<sub>p</sub> <input id="KPpitch" type="number" min="0.00" max="10.00" value="1.8" step="0.01" size="5">
  K<sub>i</sub> <input id="KIpitch" type="number" min="0.00" max="10.00" value="6.5" step="0.01" size="5">
  K<sub>d</sub> <input id="KDpitch" type="number" min="0.00" max="10.00" value="0.12" step="0.01" size="5">
  </br></br>
  Aggressive K<sub>p</sub>:<input id="aggKPpitch" type="number" min="0.00" max="10.00" value="1.0" step="0.01" size="5">
  from <small>250PWM/22,22º</small> to <input id="aggPitchAngle" type="number" min="0" max="30" value="45" step="1" size="2"> Pitch error-angle.
</fieldset>
</br><fieldset>
  <legend><strong>ROLL</strong></legend>
  Setpoint (PWM) <input id="SProll" type="number" min="-250" max="250" value="0" step="1" size="4">
  K<sub>p </sub><input id="KProll" type="number" min="0.00" max="10.00" value="1.5" step="0.01" size="5">
  K<sub>i </sub><input id="KIroll" type="number" min="0.00" max="10.00" value="4.5" step="0.01" size="5">
  K<sub>d </sub><input id="KDroll" type="number" min="0.00" max="10.00" value="0.16" step="0.01" size="5">
  </br></br>
  Aggressive K<sub>p</sub>:<input id="aggKProll" type="number" min="0.00" max="10.00" value="1.00" step="0.01" size="5">
  from <small>250PWM/22,22º</small> to <input id="aggRollAngle" type="number" min="0" max="30" value="45" step="1" size="2"> Roll error-angle.
</fieldset>
</br><fieldset>
  <legend><strong>YAW</strong></legend>
  K<sub>p </sub><input id="KPyaw" type="number" min="0.00" max="100.00" value="1.1" step="0.01" size="6">
  K<sub>i </sub><input id="KIyaw" type="number" min="0.00" max="100.00" value="0.05" step="0.01" size="6">
  K<sub>d </sub><input id="KDyaw" type="number" min="0.00" max="100.00" value="0.15" step="0.01" size="6">
  </br></br>
  DRCG (Damped Rate of Change Gain) <input id="DRCG" type="number" min="1" max="10" value="5" step="1" size="2">
</fieldset>
</br><fieldset>
  <legend><strong>ALTITUDE</strong></legend>
  KP<sub>asc</sub> <input id="KPASCaltitude" type="number" min="0.00" max="10.00" value="15.00" step="0.01" size="5">
  KP<sub>des</sub> <input id="KPDESaltitude" type="number" min="0.00" max="10.00" value="5.00" step="0.01" size="5">
  KI <input id="KIaltitude" type="number" min="0.00" max="10.00" value="1.00" step="0.01" size="5">
  KD <input id="KDaltitude" type="number" min="0.00" max="10.00" value="0.00" step="0.01" size="5">
  </br></br>
  ASC/DES Speed Limit (PWM/cycle) <input id="PWMlimit" type="number" min="1" max="100" value="25" step="1" size="3">
</fieldset>
</BR></BR>
<input type="button" value="SET PID" onclick="parametros();">
</fieldset></BR>

<fieldset>
<legend><h2>CONFIGURACION</h2></legend>
<input type="radio" id="conf1" name="conf" checked><label for="conf1"> QuadPlus (&#10010;) </label>
<input type="radio" id="conf2" name="conf"><label for="conf2"> QuadX (&#10006;) </label>
&emsp; <input type="button" value="SET" onclick="newCONF();" DISABLED></BR></BR>
HOVER with GROUND-EFFECT 
<input id="newHOVER" type="number" min="1400" max="1600" value="1445" step="1"> 
PWMs 
<input type="button" value="SET" onclick="hover();"></BR>
</fieldset></BR>

<fieldset>
<legend><h2>Ajustes de los MPU6050 + HMC5883L + MS5611 | BMP180</h2></legend>
<button class="botons" style="width:450px;" onclick="devicesReset();">RESET the Gyroscope, Accelerometer, Barometer & Compass devices</button></br></br>
<fieldset>
 <TABLE>
 <TR>
 <TD>SENSOR FUSION PITCH & ROLL</TD>
 <TD><input id="SFFpr" type="number" min="0.9" max="1.0" value="0.995" step="0.0001" ></TD>
 </TR>
 <TR>
 <TD>SENSOR FUSION YAW</TD>
 <TD><input id="SFFy" type="number" min="0.9" max="1.0" value="0.9996" step="0.0001" ></TD>
 </TR>
 <TR>
 <TD>PITCH GAIN (acc)</TD>
 <TD><input id="gananciaP" type="number" min="1.0" max="20.0" value="10.0" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>ROLL GAIN (acc)</TD>
 <TD><input id="gananciaR" type="number" min="1.0" max="20.0" value="10.0" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>YAW GAIN (acc)</TD>
 <TD><input id="gananciaY" type="number" min="1.0" max="20.0" value="10.0" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>SENSIBILITY (gyro)</TD>
 <TD><input id="sensibilidad" type="number" min="1.0" max="5.0" value="3.0" step="0.1" ></TD>
 </TR>
 </TABLE>
 </br>
 <TABLE>
 <TR>
 <TD>ALTITUDE FINE FILTER</TD>
 <TD><input id="FINE" type="number" min="0.9" max="1.0" value="0.9985" step="0.00001" ></TD>
 </TR>
 <TR>
 <TD>MAXIMUN ALLOWED PEAKS (pascal)</TD>
 <TD><input id="PEAKS" type="number" min="1" max="100" value="4" step="1" ></TD>
 </TR>
 <TR>
 <TD>GRAVITY (1G = 9,81 m/s) DRIFT</TD>
 <TD><input id="1G" type="number" min="0.1" max="10.0" value="4.0" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>MAXIMUN DRIFT</TD>
 <TD><input id="9G" type="number" min="1.0" max="100.0" value="10.045" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>ATTENUATION</TD>
 <TD><input id="ATT" type="number" min="1.0" max="10.0" value="6.0" step="0.1" ></TD>
 </TR>
 <TD>AMPLIFICATION</TD>
 <TD><input id="AMP" type="number" min="1.0" max="10.0" value="9.0" step="0.1" ></TD>
 </TR>
 </TABLE>
 </br>
 <TABLE>
 <TR>
 <TD>FLIGHT LEVEL LOGIC (0.1 meter = 1.116 pascal)</TD>
 <TD><input id="newFLL" type="number" min="0.0" max="1.0" value="0.5" step="0.1" ></TD>
 </TR>
 <TR>
 <TD>PARACHUTE (T/P correction per hour)</TD>
 <TD><input id="parachute" type="number" min="-10.0" max="10.0" value="-0.5" step="0.1" ></TD>
 <TR>
 </TABLE>
 </br>
 <input type="button" value="SET FILTERS" onclick="drift();">
</fieldset>
</br>
<fieldset>
  <input type="button" value="Adjust the Gyro/Acc Process =>" onclick="conexion1.send('ADJUST:true');">&emsp;
  ANGLE PITCH (º) <input style="width:65px;" id="gravedadPITCH" type="number" min="-5.00" max="+5.00" value="-0.44" step="0.01" size="5">&emsp;
  ANGLE ROLL (º) <input style="width:65px;" id="gravedadROLL" type="number" min="-5.00" max="+5.00" value="7.15" step="0.01" size="5">&emsp;
  <input type="button" value=" SET ANGLES " onclick="gravedad();">
</fieldset>
</br>
<fieldset>
  DECLINACION MAGNETICA : 
  <input id="" style="width:50px;" type="number" min="0.00" max="5.00" value="1.36" step="0.01" size="4"> (º) 
  <label for="WE"> </label>
  <select name="WE" id="">
  <option value="W">W</option>
  <option value="E">E</option>
  </select>
  <input type="button" value=" SET " onclick="" disabled>
</fieldset>
</fieldset></BR>

</div>

<!-- ***** JOYSTICK ***** -->
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
  border: 1px solid black;
  background: black;
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
  const xNew = distance * Math.cos( angle );
  const yNew = distance * Math.sin( angle );
  this.stick.style.transform = `translate3d(${xNew}px, ${yNew}px, 0px)`;
  this.currentPos = { x: xNew, y: yNew };
  this.PWM.x = (this.currentPos.x * SERVO_LIMITE / this.maxDiff).toFixed(0);
  this.PWM.y = (this.currentPos.y * SERVO_LIMITE / this.maxDiff).toFixed(0);
  if (this.sensibilidad()) {
    this.mensaje = "JS:" + this.PWM.x + "," + this.PWM.y;
    //setTimeout(() => { }, 15);
    const t = Date.now();
      let tt = null;
      do {
        tt = Date.now();
      } while (tt - t < 15);
    conexion1.send(this.mensaje);
    //console.log (this.mensaje+" "+(new Date()).getMilliseconds());
    pixelsJoystickX.value = this.PWM.x;
    pixelsJoystickY.value = this.PWM.y;
    degsec.innerHTML = this.PWM.x;
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
  } else zero_zero_error = true;
  pixelsJoystickX.value = 0;
  pixelsJoystickY.value = 0;
  degsec.innerHTML = 0;
};
</script>

   <div id="joystick-wrapper">
     <SMALL>
     JOYSTICK (PWM)
     X:<input type="text" id="pixelsJoystickX" style="width:35px;" value="0">
     Y:<input type="text" id="pixelsJoystickY" style="width:35px;" value="0">
     </SMALL>
     <span id="degsec_titulo" hidden>&emsp; GYRO (deg/sec) : </span>
     <span id="degsec" hidden>0</span>
     <div id="joystick-base"></div>
   </div>
 </div>

<!-- ***** THROTTLE ***** -->
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
  top: calc(100% - 60px - 25px); /* ARMED*height/(ESC_LIMITE-1000) = 85*300/1000 = 25  - ajuste=60x */
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

  this.setPos= function (arg) {
    this.currentPosY = -1 * (arg - 1000) * this.maxDiffY / 1000;
    this.stick.style.transform = `translate3d(0px, ${this.currentPosY}px, 0px)`;
    this.PWM = arg;
    pixelsThrottle.value = this.PWM;
    //¿sending data to ESP32?
  }
};
THROTTLE.prototype.handleMouseDown = function( event ) {
  this.stick.style.transition = '0s';
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
  this.PWM = (1000 - this.currentPosY * (2000-1000) / this.maxDiffY).toFixed(0);
  pixelsThrottle.value = this.PWM;
};
THROTTLE.prototype.handleMouseUp = function(event) {
  if ( this.dragStart === null ) return;
  this.mensaje = "THROTTLE:" + this.PWM;
  if (this.PWM > ARMED) {
   document.getElementById("HOVER").hidden = false;
   document.getElementById("LAND").hidden = false;
   document.getElementById("HOLD").hidden = false;
   document.getElementById("AIR").hidden = false;
  } else {
   document.getElementById("HOVER").hidden = true;
   document.getElementById("LAND").hidden = true;
   document.getElementById("HOLD").hidden = true;
   document.getElementById("AIR").hidden = true;
  }
  conexion1.send(this.mensaje);
  //console.log(this.mensaje);
  this.dragStart = null;
};
</script>

    <div id="throttle-wrapper">
      THROTTLE (PWM) 
      <style>
      .quantity {
      width: 70px;
      height: 26px;
      padding: 0px;
      font-size: 15pt;
      border: solid 0.5px #000;
      }
      </style>
      <input type="number" id="pixelsThrottle" class="quantity" min="1000" max="2000" value="1000" step="1" size="4"> 
      <input type="button" value="SET" onclick="throttle.setPos(document.getElementById('pixelsThrottle').value); conexion1.send('THROTTLE:'+document.getElementById('pixelsThrottle').value);";>
      <div id="throttle-base"></div>
    </div>
  </div>
  
</div>

<script>
function openGoogle() {
document.getElementById("mySidenav").style.width = "635px"; //500px+135px
}
function closeGoogle() {
document.getElementById("mySidenav").style.width = "0";
}
</script>

<style>
.sidenav {
 height: 100%;
 width: 0%;
 position: fixed;
 z-index: 1;
 overflow-x: hidden;
 transition: 0.5s;
 padding-top: 50px;
 top: 0;
 right: 0;
 background:#EAEAEA;
 border: 1px solid #EAEAEA;
}
.sidenav a {
 text-decoration: none;
 color: black;
}
.sidenav a:hover, a:focus {
 color: red;
}
.sidenav .closebtn {
 position: absolute;
 top: 0px;
 right: 10px;
 font-size: 65px;
}
</style>

<div id = 'mySidenav' class = 'sidenav'>
 <a href = 'javascript:void(0)' class = 'closebtn' onclick = 'closeGoogle()'>&times;</a>
</div>

</BODY>

</HTML>
)";

#endif
