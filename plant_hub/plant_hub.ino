/*
 * Plant Tracker — hub / base station
 *
 * Runs on a second ESP32 (no sensors needed). Receives ESP-NOW packets
 * from the sensor node, keeps 48 h of history in RAM, and serves a small
 * dashboard over WiFi.
 *
 * Two ways to reach the dashboard:
 *   1. Standalone (default): join the WiFi network AP_SSID / AP_PASS,
 *      then open http://192.168.4.1
 *   2. Home WiFi: fill in HOME_WIFI_SSID / HOME_WIFI_PASS below and open
 *      http://planthub.local (or the IP printed on Serial) from any
 *      device on your network. The standalone AP stays up as a fallback.
 *
 * Pairing: this hub forces its station MAC to HUB_MAC, the address the
 * sensor node transmits to — no MAC copying between sketches required.
 * The sensor node discovers the WiFi channel by itself, so it also keeps
 * working if your router changes channels.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#if !defined(ESP_ARDUINO_VERSION_VAL)
#error "This sketch requires arduino-esp32 core 2.x or newer"
#endif
#define CORE_AT_LEAST_3 (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))

// ---------------- configuration ----------------
#define HOME_WIFI_SSID ""          // optional: your home WiFi (leave "" for standalone)
#define HOME_WIFI_PASS ""
#define AP_SSID        "PlantHub"  // the hub's own network (always on)
#define AP_PASS        "plantpots" // min 8 chars — change me
#define MDNS_NAME      "planthub"  // http://planthub.local on home WiFi

// WiFi regulatory domain. "01" (world-safe) only allows channels 1-11 on
// arduino-esp32 core 3.x. If your router uses channel 12/13 (common outside
// North America), set your ISO country code here, e.g. "GB", "DE", "AU" —
// in BOTH sketches, or the sensor cannot follow the hub onto that channel.
#define WIFI_COUNTRY   "01"

// Fixed, locally administered MAC. Must match HUB_MAC in the sensor sketch.
static const uint8_t HUB_MAC[6] = {0x02, 0x50, 0x4C, 0x41, 0x4E, 0x54};

// 288 samples x 10 min = 48 h of history held in RAM.
const size_t HISTORY_LEN = 288;
const int8_t RSSI_NONE = 127;

// ---------------- shared packet ----------------
// !! Keep this struct byte-identical with the copy in plant_tracker.ino.ino
#define PKT_MAGIC   0x504C4E54UL  // "PLNT"
#define PKT_VERSION 1

#define FLAG_DHT_OK   0x01
#define FLAG_BMP_OK   0x02
#define FLAG_LIGHT_OK 0x04

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t bootCount;
  float    temperatureC;  // NAN when invalid
  float    humidityPct;   // NAN when invalid
  float    pressureHpa;   // NAN when invalid
  float    lux;           // NAN when invalid
  uint16_t soilRaw;
  uint8_t  soilPct;
  uint8_t  flags;
} PlantPacket;

// ---------------- state ----------------
struct Sample {
  uint32_t at;  // millis() when received
  PlantPacket p;
  int8_t rssi;
};

Sample history[HISTORY_LEN];
size_t histCount = 0;
size_t histHead = 0;  // next write index
uint32_t packetsReceived = 0;

portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool packetPending = false;
PlantPacket pendingPkt;
int8_t pendingRssi = RSSI_NONE;

WebServer server(80);

// ---------------- ESP-NOW receive ----------------
#if CORE_AT_LEAST_3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int8_t rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : RSSI_NONE;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int8_t rssi = RSSI_NONE;
#endif
  if (len != (int)sizeof(PlantPacket)) return;
  PlantPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != PKT_MAGIC || p.version != PKT_VERSION) return;

  // Runs in the WiFi task — hand off to loop() and get out fast.
  portENTER_CRITICAL(&pendingMux);
  pendingPkt = p;
  pendingRssi = rssi;
  packetPending = true;
  portEXIT_CRITICAL(&pendingMux);
}

void drainPending() {
  if (!packetPending) return;
  portENTER_CRITICAL(&pendingMux);
  PlantPacket p = pendingPkt;
  int8_t rssi = pendingRssi;
  packetPending = false;
  portEXIT_CRITICAL(&pendingMux);

  history[histHead].at = millis();
  history[histHead].p = p;
  history[histHead].rssi = rssi;
  histHead = (histHead + 1) % HISTORY_LEN;
  if (histCount < HISTORY_LEN) histCount++;
  packetsReceived++;

  Serial.printf("Packet #%lu boot=%u T=%.1f H=%.1f P=%.1f L=%.1f Soil=%u%% rssi=%d\n",
                (unsigned long)packetsReceived, p.bootCount, p.temperatureC, p.humidityPct,
                p.pressureHpa, p.lux, p.soilPct, rssi);
}

// ---------------- JSON ----------------
void addNum(String &s, const char *key, float v, uint8_t dp) {
  s += '"';
  s += key;
  s += "\":";
  if (isnan(v)) s += "null";
  else s += String(v, (unsigned int)dp);
}

void appendMetrics(String &s, const Sample &smp) {
  addNum(s, "temp", smp.p.temperatureC, 1);
  s += ',';
  addNum(s, "hum", smp.p.humidityPct, 1);
  s += ',';
  addNum(s, "pres", smp.p.pressureHpa, 1);
  s += ',';
  addNum(s, "lux", smp.p.lux, 1);
  s += ",\"soil\":" + String(smp.p.soilPct);
}

void handleLatest() {
  String s;
  s.reserve(400);
  s += '{';
  if (histCount == 0) {
    s += "\"hasData\":false";
  } else {
    const Sample &smp = history[(histHead + HISTORY_LEN - 1) % HISTORY_LEN];
    s += "\"hasData\":true,";
    s += "\"ageS\":" + String((uint32_t)((millis() - smp.at) / 1000)) + ',';
    s += "\"boot\":" + String(smp.p.bootCount) + ',';
    appendMetrics(s, smp);
    s += ",\"soilRaw\":" + String(smp.p.soilRaw);
    s += ",\"rssi\":";
    s += (smp.rssi == RSSI_NONE) ? String("null") : String((int)smp.rssi);
  }
  s += ",\"packets\":" + String(packetsReceived);
  s += ",\"ch\":" + String(WiFi.channel());
  s += ",\"mac\":\"" + WiFi.macAddress() + '"';
  s += ",\"apSsid\":\"" AP_SSID "\"";
  s += ",\"apIp\":\"" + WiFi.softAPIP().toString() + '"';
  s += ",\"staIp\":\"";
  if (WiFi.status() == WL_CONNECTED) s += WiFi.localIP().toString();
  s += "\"}";
  server.send(200, "application/json", s);
}

void handleHistory() {
  uint32_t nowMs = millis();
  String s;
  s.reserve(histCount * 96 + 32);
  s += "{\"samples\":[";
  for (size_t i = 0; i < histCount; i++) {
    const Sample &smp = history[(histHead + HISTORY_LEN - histCount + i) % HISTORY_LEN];
    if (i) s += ',';
    s += "{\"ageS\":" + String((uint32_t)((nowMs - smp.at) / 1000)) + ',';
    appendMetrics(s, smp);
    s += '}';
  }
  s += "]}";
  server.send(200, "application/json", s);
}

// ---------------- dashboard page ----------------
static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plant Tracker</title>
<style>
*{box-sizing:border-box;margin:0}
:root{
  color-scheme:light dark;
  --page:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#898781;
  --grid:#e1e0d9;--baseline:#c3c2b7;--ring:rgba(11,11,11,.10);
  --good:#0ca30c;--crit:#d03b3b;
  --c1:#2a78d6;--c2:#eb6834;--c3:#1baf7a;--c4:#eda100;--c5:#e87ba4;
}
@media (prefers-color-scheme:dark){:root{
  --page:#0d0d0d;--surface:#1a1a19;--ink:#ffffff;--ink2:#c3c2b7;--muted:#898781;
  --grid:#2c2c2a;--baseline:#383835;--ring:rgba(255,255,255,.10);
  --c1:#3987e5;--c2:#d95926;--c3:#199e70;--c4:#c98500;--c5:#d55181;
}}
body{background:var(--page);color:var(--ink);padding:16px;max-width:1100px;margin:0 auto;
  font:15px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif}
header{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
h1{font-size:20px;font-weight:600}
.badge{display:inline-flex;align-items:center;gap:6px;font-size:13px;font-weight:600;
  padding:2px 10px;border-radius:999px;border:1px solid var(--ring);color:var(--ink2)}
.badge .dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}
.badge.live .dot{background:var(--good)}
.badge.bad{color:var(--crit)}
.badge.bad .dot{background:var(--crit)}
.muted{color:var(--muted);font-size:13px}
.filters{display:flex;gap:6px;margin:14px 0}
.filters button{font:600 13px system-ui,-apple-system,"Segoe UI",sans-serif;padding:4px 12px;
  border-radius:999px;border:1px solid var(--ring);background:var(--surface);color:var(--ink2);cursor:pointer}
.filters button.active{background:var(--ink);color:var(--page);border-color:var(--ink)}
#grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px}
.card{background:var(--surface);border:1px solid var(--ring);border-radius:12px;padding:14px 14px 10px}
.card .lbl{font-size:13px;color:var(--ink2)}
.card .val{font-size:30px;font-weight:600;line-height:1.15;margin:2px 0 8px}
.card .unit{font-size:15px;font-weight:400;color:var(--ink2);margin-left:2px}
.card canvas{width:100%;height:88px;display:block;touch-action:none}
.card canvas:focus-visible{outline:2px solid var(--ink2);outline-offset:2px}
#tip{position:fixed;pointer-events:none;background:var(--surface);border:1px solid var(--ring);
  border-radius:8px;padding:6px 10px;font-size:13px;box-shadow:0 2px 8px rgba(0,0,0,.15);z-index:10}
#tip .key{display:inline-block;width:12px;height:2px;border-radius:1px;vertical-align:middle;margin-right:6px}
#tip .tv{font-weight:600;color:var(--ink)}
#tip .tt{color:var(--ink2);font-size:12px;margin-left:4px}
details{margin-top:18px}
summary{cursor:pointer;font-weight:600;font-size:14px;color:var(--ink2)}
.scroll{overflow-x:auto}
table{border-collapse:collapse;font-size:13px;margin-top:8px;font-variant-numeric:tabular-nums;min-width:480px}
th,td{text-align:right;padding:4px 10px;border-bottom:1px solid var(--grid)}
th{color:var(--muted);font-weight:500}
td{color:var(--ink2)}
th:first-child,td:first-child{text-align:left}
footer{margin-top:18px;font-size:12px;color:var(--muted)}
</style></head>
<body>
<header>
  <h1>&#127793; Plant Tracker</h1>
  <span id="badge" class="badge"><span class="dot"></span><span id="badgeTxt">Connecting&hellip;</span></span>
  <span id="ago" class="muted"></span>
</header>
<div class="filters" role="group" aria-label="Time range">
  <button data-h="6">6h</button>
  <button data-h="24" class="active">24h</button>
  <button data-h="48">48h</button>
</div>
<main id="grid"></main>
<details>
  <summary>Data table</summary>
  <div class="scroll"><table id="table"></table></div>
</details>
<footer id="hub"></footer>
<div id="tip" hidden><span class="key"></span><span class="tv"></span><span class="tt"></span></div>
<script>
'use strict';
var METRICS=[
  {k:'temp',label:'Temperature',unit:'°C',dp:1,cvar:'--c1'},
  {k:'hum',label:'Humidity',unit:'%',dp:1,cvar:'--c2'},
  {k:'pres',label:'Pressure',unit:'hPa',dp:1,cvar:'--c3'},
  {k:'lux',label:'Light',unit:'lx',dp:0,cvar:'--c4'},
  {k:'soil',label:'Soil moisture',unit:'%',dp:0,cvar:'--c5'}
];
var state={hist:[],latest:null,lastT:null,rangeH:24,cards:[],linkOk:null};
var grid=document.getElementById('grid');
var tip=document.getElementById('tip');
var tipKey=tip.querySelector('.key'),tipVal=tip.querySelector('.tv'),tipTime=tip.querySelector('.tt');

function cssVar(name){return getComputedStyle(document.documentElement).getPropertyValue(name).trim();}
function fmt(v,dp){
  if(v==null||isNaN(v))return '—';
  return Number(v).toLocaleString(undefined,{minimumFractionDigits:dp,maximumFractionDigits:dp});
}
function fmtTime(t){
  var d=new Date(t),o={hour:'2-digit',minute:'2-digit'};
  if(Date.now()-t>72e6)return d.toLocaleDateString(undefined,{weekday:'short'})+' '+d.toLocaleTimeString([],o);
  return d.toLocaleTimeString([],o);
}
function fmtAgo(ms){
  var s=Math.max(0,Math.floor(ms/1000));
  if(s<60)return s+' s';
  if(s<3600)return Math.floor(s/60)+' m '+(s%60)+' s';
  return Math.floor(s/3600)+' h '+Math.floor((s%3600)/60)+' m';
}

METRICS.forEach(function(m){
  var card=document.createElement('div');card.className='card';
  var lbl=document.createElement('div');lbl.className='lbl';lbl.textContent=m.label;
  var val=document.createElement('div');val.className='val';
  var cv=document.createElement('canvas');
  cv.tabIndex=0;
  cv.setAttribute('role','img');
  cv.setAttribute('aria-label',m.label+' history');
  card.appendChild(lbl);card.appendChild(val);card.appendChild(cv);
  grid.appendChild(card);
  var c={m:m,valEl:val,cv:cv,pts:[],hover:null,scale:null};
  state.cards.push(c);

  cv.addEventListener('pointermove',function(e){
    if(!c.scale)return;
    var r=cv.getBoundingClientRect();
    setHover(c,nearestIdx(c,e.clientX-r.left));
    if(c.hover!=null)showTip(c,c.hover,e.clientX,e.clientY);
  });
  cv.addEventListener('pointerleave',function(){setHover(c,null);hideTip();});
  cv.addEventListener('keydown',function(e){
    if(!c.scale)return;
    var i=c.hover==null?c.pts.length-1:c.hover;
    if(e.key==='ArrowLeft')i=Math.max(0,i-1);
    else if(e.key==='ArrowRight')i=Math.min(c.pts.length-1,i+1);
    else if(e.key==='Home')i=0;
    else if(e.key==='End')i=c.pts.length-1;
    else return;
    e.preventDefault();
    setHover(c,i);
    var r=cv.getBoundingClientRect();
    showTip(c,i,r.left+xPos(c,c.pts[i].t),r.top+yPos(c,c.pts[i].v));
  });
  cv.addEventListener('blur',function(){setHover(c,null);hideTip();});
});

function setHover(c,i){c.hover=i;draw(c);}
function nearestIdx(c,px){
  var best=null,bd=1e9;
  for(var i=0;i<c.pts.length;i++){
    var d=Math.abs(xPos(c,c.pts[i].t)-px);
    if(d<bd){bd=d;best=i;}
  }
  return best;
}
function xPos(c,t){
  var s=c.scale;
  return s.padL+(t-s.t0)/(s.t1-s.t0||1)*(s.W-s.padL-s.padR);
}
function yPos(c,v){
  var s=c.scale;
  return s.padT+(s.vmax-v)/(s.vmax-s.vmin||1)*(s.H-s.padT-s.padB);
}

function draw(c){
  var cv=c.cv,ctx=cv.getContext('2d');
  var dpr=window.devicePixelRatio||1;
  var W=cv.clientWidth,H=cv.clientHeight;
  if(!W||!H)return;
  if(cv.width!==Math.round(W*dpr)||cv.height!==Math.round(H*dpr)){
    cv.width=Math.round(W*dpr);cv.height=Math.round(H*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,W,H);
  var col=cssVar(c.m.cvar),gridCol=cssVar('--grid'),baseCol=cssVar('--baseline'),
      mutedCol=cssVar('--muted'),surf=cssVar('--surface');
  var pts=c.pts;
  if(pts.length<2){
    c.scale=null;
    ctx.fillStyle=mutedCol;ctx.font='12px system-ui,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(pts.length?'one sample so far':'waiting for data',W/2,H/2);
    return;
  }
  var vmin=Infinity,vmax=-Infinity,i;
  for(i=0;i<pts.length;i++){if(pts[i].v<vmin)vmin=pts[i].v;if(pts[i].v>vmax)vmax=pts[i].v;}
  if(vmax===vmin){vmax+=1;vmin-=1;}
  else{var pad=(vmax-vmin)*0.08;vmax+=pad;vmin-=pad;}
  c.scale={t0:pts[0].t,t1:pts[pts.length-1].t,vmin:vmin,vmax:vmax,W:W,H:H,padL:36,padR:12,padT:6,padB:6};

  // min/max gridlines + labels
  ctx.font='10px system-ui,sans-serif';
  ctx.textAlign='right';ctx.textBaseline='middle';
  [vmin,vmax].forEach(function(gv,gi){
    var gy=yPos(c,gi?vmax:vmin);
    ctx.strokeStyle=gridCol;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(c.scale.padL,gy);ctx.lineTo(W-c.scale.padR,gy);ctx.stroke();
    ctx.fillStyle=mutedCol;
    var raw=gi?vmax:vmin;
    ctx.fillText(fmt(Math.abs(raw)>=1000?Math.round(raw):raw,Math.abs(raw)>=100?0:c.m.dp),c.scale.padL-6,gy);
  });

  // area wash
  ctx.beginPath();
  ctx.moveTo(xPos(c,pts[0].t),yPos(c,pts[0].v));
  for(i=1;i<pts.length;i++)ctx.lineTo(xPos(c,pts[i].t),yPos(c,pts[i].v));
  ctx.lineTo(xPos(c,pts[pts.length-1].t),H-c.scale.padB);
  ctx.lineTo(xPos(c,pts[0].t),H-c.scale.padB);
  ctx.closePath();
  ctx.globalAlpha=0.1;ctx.fillStyle=col;ctx.fill();ctx.globalAlpha=1;

  // line
  ctx.beginPath();
  ctx.moveTo(xPos(c,pts[0].t),yPos(c,pts[0].v));
  for(i=1;i<pts.length;i++)ctx.lineTo(xPos(c,pts[i].t),yPos(c,pts[i].v));
  ctx.strokeStyle=col;ctx.lineWidth=2;ctx.lineJoin='round';ctx.lineCap='round';ctx.stroke();

  // crosshair
  if(c.hover!=null&&pts[c.hover]){
    var hx=xPos(c,pts[c.hover].t);
    ctx.strokeStyle=baseCol;ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(hx,c.scale.padT);ctx.lineTo(hx,H-c.scale.padB);ctx.stroke();
  }

  // dot with surface ring (hover point, else latest point)
  var di=c.hover!=null?c.hover:pts.length-1;
  var dx=xPos(c,pts[di].t),dy=yPos(c,pts[di].v);
  ctx.beginPath();ctx.arc(dx,dy,6.5,0,7);ctx.fillStyle=surf;ctx.fill();
  ctx.beginPath();ctx.arc(dx,dy,4.5,0,7);ctx.fillStyle=col;ctx.fill();
}

function showTip(c,i,ax,ay){
  var p=c.pts[i];
  tipKey.style.background=cssVar(c.m.cvar);
  tipVal.textContent=fmt(p.v,c.m.dp)+' '+c.m.unit;
  tipTime.textContent=fmtTime(p.t);
  tip.hidden=false;
  var tw=tip.offsetWidth,th=tip.offsetHeight;
  var x=ax+14;if(x+tw>window.innerWidth-8)x=ax-tw-14;
  var y=ay-th-12;if(y<8)y=ay+18;
  tip.style.left=x+'px';tip.style.top=y+'px';
}
function hideTip(){tip.hidden=true;}

function applyData(){
  var cut=Date.now()-state.rangeH*3600e3;
  state.cards.forEach(function(c){
    c.pts=[];
    state.hist.forEach(function(s){
      if(s.t>=cut&&s[c.m.k]!=null)c.pts.push({t:s.t,v:s[c.m.k]});
    });
    c.hover=null;
    draw(c);
    var v=state.latest?state.latest[c.m.k]:null;
    c.valEl.textContent=fmt(v,c.m.dp);
    var u=document.createElement('span');u.className='unit';u.textContent=c.m.unit;
    c.valEl.appendChild(u);
  });
  renderTable(cut);
}

function renderTable(cut){
  var tbl=document.getElementById('table');
  while(tbl.firstChild)tbl.removeChild(tbl.firstChild);
  var tr=document.createElement('tr');
  ['Time','Temp (°C)','Hum (%)','Pres (hPa)','Light (lx)','Soil (%)'].forEach(function(h){
    var th=document.createElement('th');th.textContent=h;tr.appendChild(th);
  });
  tbl.appendChild(tr);
  var rows=state.hist.filter(function(s){return s.t>=cut;}).slice(-96).reverse();
  rows.forEach(function(s){
    var r=document.createElement('tr');
    var cells=[fmtTime(s.t),fmt(s.temp,1),fmt(s.hum,1),fmt(s.pres,1),fmt(s.lux,0),fmt(s.soil,0)];
    cells.forEach(function(v){var td=document.createElement('td');td.textContent=v;r.appendChild(td);});
    tbl.appendChild(r);
  });
}

function updateStatus(){
  var badge=document.getElementById('badge'),txt=document.getElementById('badgeTxt'),
      ago=document.getElementById('ago');
  badge.className='badge';
  if(state.linkOk===null){
    txt.textContent='Connecting…';ago.textContent='';return;
  }
  if(state.linkOk===false){
    badge.className='badge bad';txt.textContent='⚠ No connection to hub';
    ago.textContent='';return;
  }
  if(state.lastT==null){
    txt.textContent='Waiting for first packet…';ago.textContent='';return;
  }
  var age=Date.now()-state.lastT;
  if(age>25*60e3){
    badge.className='badge bad';txt.textContent='⚠ Sensor offline';
  }else{
    badge.className='badge live';txt.textContent='Live';
  }
  ago.textContent='updated '+fmtAgo(age)+' ago';
}

function updateFooter(){
  var j=state.hub;if(!j)return;
  var parts=['Hub '+j.mac,'channel '+j.ch,'AP "'+j.apSsid+'" '+j.apIp];
  if(j.staIp)parts.push('LAN '+j.staIp);
  parts.push(j.packets+' packets received');
  if(state.latest&&state.latest.rssi!=null)parts.push('signal '+state.latest.rssi+' dBm');
  if(state.latest&&state.latest.soilRaw!=null)parts.push('soil raw '+state.latest.soilRaw);
  document.getElementById('hub').textContent=parts.join(' · ');
}

var lastPackets=-1;
function fetchLatest(){
  fetch('/api/latest').then(function(r){return r.json();}).then(function(j){
    state.linkOk=true;
    state.hub=j;
    if(j.hasData){
      state.latest=j;
      state.lastT=Date.now()-j.ageS*1000;
    }
    updateStatus();updateFooter();
    if(j.packets!==lastPackets){fetchHistory(j.packets);}
  }).catch(function(){state.linkOk=false;updateStatus();});
}
function fetchHistory(p){
  fetch('/api/history').then(function(r){return r.json();}).then(function(j){
    lastPackets=p;  // commit only on success so the next poll retries a failure
    var now=Date.now();
    state.hist=j.samples.map(function(s){
      return {t:now-s.ageS*1000,temp:s.temp,hum:s.hum,pres:s.pres,lux:s.lux,soil:s.soil};
    });
    applyData();
  }).catch(function(){});
}

document.querySelectorAll('.filters button').forEach(function(b){
  b.addEventListener('click',function(){
    document.querySelectorAll('.filters button').forEach(function(x){x.classList.remove('active');});
    b.classList.add('active');
    state.rangeH=Number(b.dataset.h);
    applyData();
  });
});

window.addEventListener('resize',function(){state.cards.forEach(draw);});
setInterval(fetchLatest,30000);
setInterval(updateStatus,1000);
fetchLatest();
var mq=window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)');
function onTheme(){state.cards.forEach(draw);updateStatus();}
if(mq){
  if(mq.addEventListener)mq.addEventListener('change',onTheme);
  else if(mq.addListener)mq.addListener(onTheme);
}
</script>
</body></html>
)HTML";

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nPlant Tracker hub starting");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);

  esp_err_t merr = esp_wifi_set_mac(WIFI_IF_STA, HUB_MAC);
  if (merr != ESP_OK) {
    Serial.printf("WARNING: could not set fixed MAC (err %d).\n", (int)merr);
    Serial.printf("Paste this MAC into HUB_MAC in the sensor sketch instead: %s\n",
                  WiFi.macAddress().c_str());
  }
  WiFi.setSleep(false);  // never doze — we must not miss ESP-NOW frames
  if (strcmp(WIFI_COUNTRY, "01") != 0) esp_wifi_set_country_code(WIFI_COUNTRY, false);

  bool staConnected = false;
  if (strlen(HOME_WIFI_SSID) > 0) {
    Serial.printf("Connecting to %s", HOME_WIFI_SSID);
    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    staConnected = WiFi.status() == WL_CONNECTED;
    if (staConnected) {
      Serial.printf("Home WiFi OK: http://%s/\n", WiFi.localIP().toString().c_str());
    } else {
      // Stop retrying: reconnect attempts hop channels and break ESP-NOW.
      WiFi.setAutoReconnect(false);
      WiFi.disconnect();
      Serial.println("Home WiFi failed — running standalone");
    }
  }

  // Always-on fallback AP. With home WiFi connected it shares that channel;
  // standalone it sits on channel 1. The sensor scans, so either works.
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP \"%s\" up: http://%s/ (channel %d)\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), WiFi.channel());
  Serial.printf("Hub MAC for ESP-NOW: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: esp_now_init failed — restarting in 5 s");
    delay(5000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local/\n", MDNS_NAME);
  }

  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/latest", handleLatest);
  server.on("/api/history", handleHistory);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("Web server running");
}

void loop() {
  server.handleClient();
  drainPending();
  delay(2);
}
