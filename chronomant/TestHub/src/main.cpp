/*
 * QUEST HUB — на основе боевого кода с 192.168.8.177 (Silent Hill),
 * + RSSI, + вкладка "Chronomant". Со временем должен заменить собой
 * боевой хаб целиком, поэтому изменения — только аддитивные поверх
 * оригинальной структуры (tabs/power device/NVS), не переписывание.
 *
 * ESP32 (любой с WiFi — на плате прямых указаний на C3/S3 в коде нет).
 * На стенде — DHCP/weasgley, на бою — статик 192.168.8.177 (см. USE_STATIC_IP).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PicoMQTT.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// ====== Сеть ======
// 0 = стенд (DHCP, weasgley), 1 = боевая сеть (статик 192.168.8.177).
#define USE_STATIC_IP 0

#if USE_STATIC_IP
const char* WIFI_SSID     = "LMT-5DA0";
const char* WIFI_PASSWORD = "IH7R9QCREC";
IPAddress LOCAL_IP(192, 168, 8, 177);
IPAddress GATEWAY(192, 168, 8, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS(8, 8, 8, 8);
#else
const char* WIFI_SSID     = "weasgley";
const char* WIFI_PASSWORD = "weasgley123";
#endif

// ====== MQTT broker ======
PicoMQTT::Server mqtt;

// ====== HTTP server ======
WebServer http(80);

// ====== NVS ======
Preferences prefs;

// ====== Цикл хрономага ======
// Общий таймер для всех синхронизированных устройств (маски, весы+лампочки).
// Раз в CYCLE_MS хаб публикует quest/chronomage/cycle/start — устройства,
// подписанные на этот топик, сбрасываются/генерируют новую последовательность
// СИНХРОННО, вместо того чтобы каждое считало свои 3 минуты независимо.
// Содержимое цикла (порядок масок, число кристаллов) пока генерируется
// на местах — хаб только даёт общий "удар по столу". Реальная комната
// будет 100 сек, сейчас тестовый темп — 3 мин.
#define CYCLE_MS (3UL * 60UL * 1000UL)
#define CYCLE_TICK_MS 5000UL

unsigned long cycleStartMs    = 0;
unsigned long cycleNumber     = 0;
unsigned long lastCycleTickMs = 0;

// ====== Constants ======
static const uint8_t MAX_DEVICES  = 16;
static const uint8_t MAX_BUTTONS  = 8;
static const uint8_t MAX_TABS     = 8;
static const unsigned long ONLINE_TIMEOUT_MS = 10000;

// ====== Structs ======
struct Button {
  char label[24];
  char topic[64];
  char payload[32];
};

struct DeviceState {
  bool     used        = false;
  char     type[16];
  char     name[16];
  char     key[32];
  bool     hasState    = false;
  bool     on          = false;
  char     info[64];
  int      rssi        = 0;
  int      rssiMin     = 0;
  int      rssiMax     = -127;
  unsigned long lastSeenMs = 0;
  uint8_t  buttonCount = 0;
  Button   buttons[MAX_BUTTONS];
  int8_t   tabIndex    = -1;  // -1 = Ungrouped
};

struct Tab {
  bool used            = false;
  char label[32];
  char powerKey[32];
};

DeviceState devices[MAX_DEVICES];
Tab         tabs[MAX_TABS];

// ====== NVS ======
void saveSettings() {
  prefs.begin("qhub", false);
  for (uint8_t t = 0; t < MAX_TABS; t++) {
    char kU[5], kL[5], kP[5];
    snprintf(kU, sizeof(kU), "t%du", t);
    snprintf(kL, sizeof(kL), "t%dl", t);
    snprintf(kP, sizeof(kP), "t%dp", t);
    prefs.putBool(kU, tabs[t].used);
    prefs.putString(kL, tabs[t].label);
    prefs.putString(kP, tabs[t].powerKey);
  }
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    if (!devices[i].used) continue;
    char nvsKey[16];
    strncpy(nvsKey, devices[i].key, sizeof(nvsKey) - 1);
    nvsKey[sizeof(nvsKey) - 1] = '\0';
    for (char* p = nvsKey; *p; p++) if (*p == '/') *p = '-';
    prefs.putInt(nvsKey, devices[i].tabIndex);
    Serial.print(F("[NVS] Save ")); Serial.print(devices[i].key);
    Serial.print(F(" -> ")); Serial.print(nvsKey);
    Serial.print(F(" tab=")); Serial.println(devices[i].tabIndex);
  }
  prefs.end();
  Serial.println(F("[NVS] Saved"));
}

void loadSettings() {
  prefs.begin("qhub", true);
  bool anyTab = false;
  for (uint8_t t = 0; t < MAX_TABS; t++) {
    char kU[5], kL[5], kP[5];
    snprintf(kU, sizeof(kU), "t%du", t);
    snprintf(kL, sizeof(kL), "t%dl", t);
    snprintf(kP, sizeof(kP), "t%dp", t);
    tabs[t].used = prefs.getBool(kU, false);
    String lbl = prefs.getString(kL, "");
    String pwr = prefs.getString(kP, "");
    strncpy(tabs[t].label,    lbl.c_str(), sizeof(tabs[t].label)    - 1);
    strncpy(tabs[t].powerKey, pwr.c_str(), sizeof(tabs[t].powerKey) - 1);
    if (tabs[t].used) anyTab = true;
  }
  if (!anyTab) {
    tabs[0].used = true;
    strncpy(tabs[0].label,    "Silent Hill", sizeof(tabs[0].label) - 1);
    strncpy(tabs[0].powerKey, "power/main",  sizeof(tabs[0].powerKey) - 1);
    tabs[1].used = true;
    strncpy(tabs[1].label,    "Chronomant", sizeof(tabs[1].label) - 1);
    strncpy(tabs[1].powerKey, "",           sizeof(tabs[1].powerKey) - 1);
  }
  prefs.end();
  Serial.println(F("[NVS] Settings loaded"));
}

void loadDeviceTab(DeviceState* dev) {
  if (!dev) return;
  char nvsKey[16];
  strncpy(nvsKey, dev->key, sizeof(nvsKey) - 1);
  nvsKey[sizeof(nvsKey) - 1] = '\0';
  for (char* p = nvsKey; *p; p++) if (*p == '/') *p = '-';
  prefs.begin("qhub", true);
  int stored = prefs.getInt(nvsKey, -99);
  prefs.end();
  if (stored != -99) {
    dev->tabIndex = (int8_t)stored;
    Serial.print(F("[NVS] Load ")); Serial.print(dev->key);
    Serial.print(F(" tab=")); Serial.println(dev->tabIndex);
  } else {
    dev->tabIndex = -1;
    Serial.print(F("[NVS] New device (ungrouped): ")); Serial.println(dev->key);
  }
}

// ====== Devices ======
DeviceState* findDeviceByKey(const char* key) {
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].used && strcmp(devices[i].key, key) == 0)
      return &devices[i];
  }
  return nullptr;
}

DeviceState* findOrCreateDevice(const char* type, const char* name) {
  char key[32];
  snprintf(key, sizeof(key), "%s/%s", type, name);
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].used && strcmp(devices[i].key, key) == 0)
      return &devices[i];
  }
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    if (!devices[i].used) {
      devices[i].used = true;
      strncpy(devices[i].type, type, sizeof(devices[i].type) - 1);
      strncpy(devices[i].name, name, sizeof(devices[i].name) - 1);
      strncpy(devices[i].key,  key,  sizeof(devices[i].key)  - 1);
      devices[i].info[0]     = '\0';
      devices[i].hasState    = false;
      devices[i].on          = false;
      devices[i].rssi        = 0;
      devices[i].rssiMin     = 0;
      devices[i].rssiMax     = -127;
      devices[i].lastSeenMs  = millis();
      devices[i].buttonCount = 0;
      devices[i].tabIndex    = -1;
      loadDeviceTab(&devices[i]);
      return &devices[i];
    }
  }
  return nullptr;
}

bool isDeviceOnline(const DeviceState& d) {
  return d.used && (millis() - d.lastSeenMs) < ONLINE_TIMEOUT_MS;
}

void parseButtonsConfig(DeviceState* dev, const char* config) {
  if (!dev || !config) return;
  dev->buttonCount = 0;
  char buf[512];
  strncpy(buf, config, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* sp1 = nullptr;
  char* bstr = strtok_r(buf, "|", &sp1);
  while (bstr && dev->buttonCount < MAX_BUTTONS) {
    char* sp2 = nullptr;
    char* label   = strtok_r(bstr, ":", &sp2);
    char* topic   = strtok_r(nullptr, ":", &sp2);
    char* payload = strtok_r(nullptr, ":", &sp2);
    if (label && topic && payload) {
      Button& btn = dev->buttons[dev->buttonCount];
      strncpy(btn.label,   label,   sizeof(btn.label)   - 1);
      strncpy(btn.topic,   topic,   sizeof(btn.topic)   - 1);
      strncpy(btn.payload, payload, sizeof(btn.payload) - 1);
      btn.label[sizeof(btn.label)-1]     = '\0';
      btn.topic[sizeof(btn.topic)-1]     = '\0';
      btn.payload[sizeof(btn.payload)-1] = '\0';
      dev->buttonCount++;
    }
    bstr = strtok_r(nullptr, "|", &sp1);
  }
}

// ====== HTML ======
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quest Hub</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#111;color:#eee;min-height:100vh}
.header{background:#1a1a1a;border-bottom:1px solid #333;padding:12px 20px;display:-webkit-box;display:-webkit-flex;display:flex;-webkit-box-align:center;-webkit-align-items:center;align-items:center;-webkit-box-pack:justify;-webkit-justify-content:space-between;justify-content:space-between}
.header h2{color:#0af;font-size:18px}
.btn-settings{background:#333;border:1px solid #555;color:#aaa;padding:8px 16px;border-radius:6px;cursor:pointer;font-size:14px}
.tab-bar{background:#161616;border-bottom:1px solid #2a2a2a;display:-webkit-box;display:-webkit-flex;display:flex;-webkit-box-align:center;-webkit-align-items:center;align-items:center;padding:0 16px;overflow-x:auto;-webkit-overflow-scrolling:touch}
.tab{padding:12px 18px;cursor:pointer;font-size:14px;color:#888;border-bottom:3px solid transparent;white-space:nowrap}
.tab.active{color:#0af;border-bottom-color:#0af}
.power-block{margin:16px 16px 0}
.power-card{background:#1a0808;border:2px solid #8b0000;border-radius:10px;padding:16px 20px}
.power-card.on{background:#081a08;border-color:#006400}
.power-title{font-size:16px;font-weight:bold;color:#ff6666;margin-bottom:6px}
.power-card.on .power-title{color:#66ff66}
.power-sub{font-size:12px;color:#888;margin-bottom:12px}
.power-buttons{display:-webkit-box;display:-webkit-flex;display:flex;-webkit-flex-wrap:wrap;flex-wrap:wrap;gap:8px}
.power-btn{padding:10px 22px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:bold}
.pstart{background:#228B22;color:#fff}
.punlock{background:#B8860B;color:#fff}
.poff{background:#8B0000;color:#fff}
.pdefault{background:#444;color:#fff}
.power-offline{padding:14px;background:#1a1a1a;border:1px dashed #444;border-radius:8px;color:#555;font-size:13px;text-align:center}
.content{padding:16px}
.device-grid{display:-webkit-box;display:-webkit-flex;display:flex;-webkit-flex-wrap:wrap;flex-wrap:wrap;gap:14px}
.device-card{background:#1e1e1e;border:1px solid #333;border-radius:8px;padding:14px;width:280px}
.device-card.offline{opacity:.45;border-color:#500}
.device-header{display:-webkit-box;display:-webkit-flex;display:flex;-webkit-box-pack:justify;-webkit-justify-content:space-between;justify-content:space-between;margin-bottom:10px}
.device-name{font-size:15px;font-weight:bold;color:#0af}
.device-type{font-size:11px;color:#666;text-transform:uppercase;margin-top:2px}
.badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:10px;font-weight:bold}
.bon{background:#006400;color:#fff}
.boff{background:#8B0000;color:#fff}
.dev-info{margin:6px 0;font-size:12px;color:#888}
.state-ind{display:inline-block;padding:3px 10px;border-radius:4px;font-size:11px;font-weight:bold;margin-bottom:8px}
.sind-on{background:#006400;color:#fff}
.sind-off{background:#444;color:#aaa}
.rssi-ind{display:inline-block;padding:2px 7px;border-radius:4px;font-size:10px;font-family:monospace;margin-left:6px}
.r1{background:#132;color:#4c4}
.r2{background:#221;color:#cc4}
.r3{background:#301;color:#e93}
.r4{background:#300;color:#e44}
.btn-group{display:-webkit-box;display:-webkit-flex;display:flex;-webkit-flex-wrap:wrap;flex-wrap:wrap;gap:6px;margin-top:8px}
.dev-btn{padding:7px 12px;border:none;border-radius:4px;cursor:pointer;font-size:12px;font-weight:bold}
.bs{background:#006400;color:#fff}
.bd{background:#8B0000;color:#fff}
.bp{background:#006080;color:#fff}
.bv{background:#5a0080;color:#fff}
.bw{background:#7a5000;color:#fff}
.bx{background:#444;color:#fff}
.empty{text-align:center;color:#444;padding:50px;font-size:13px;width:100%}

/* Modal */
.modal-bg{display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,.85);z-index:100}
.modal-bg.open{display:block}
.modal{background:#1e1e1e;border:1px solid #444;border-radius:10px;padding:20px;width:92%;max-width:520px;max-height:85vh;overflow-y:auto;position:absolute;top:50%;left:50%;-webkit-transform:translate(-50%,-50%);transform:translate(-50%,-50%)}
.modal h3{color:#0af;margin-bottom:16px;font-size:15px}
.sec-title{font-size:11px;color:#555;text-transform:uppercase;letter-spacing:.5px;margin:16px 0 8px}
.tab-item{background:#2a2a2a;border:1px solid #3a3a3a;border-radius:6px;padding:10px;margin-bottom:8px}
.tab-item input,.tab-item select{width:100%;background:#1e1e1e;border:1px solid #444;color:#eee;padding:7px 8px;border-radius:4px;font-size:13px;margin-top:6px;-webkit-appearance:none}
.tab-row{display:-webkit-box;display:-webkit-flex;display:flex;gap:8px;margin-top:6px;-webkit-box-align:center;-webkit-align-items:center;align-items:center}
.tab-row select{-webkit-box-flex:1;-webkit-flex:1;flex:1;background:#1e1e1e;border:1px solid #444;color:#eee;padding:7px 8px;border-radius:4px;font-size:13px;-webkit-appearance:none}
.del-btn{background:#500;border:none;color:#f88;padding:7px 12px;border-radius:4px;cursor:pointer;font-size:12px;white-space:nowrap}
.add-btn{width:100%;background:#1a2a1a;border:1px dashed #444;color:#888;padding:9px;border-radius:5px;cursor:pointer;font-size:13px;margin-bottom:4px}
.assign-row{display:-webkit-box;display:-webkit-flex;display:flex;-webkit-box-align:center;-webkit-align-items:center;align-items:center;gap:8px;margin-bottom:8px;font-size:12px}
.assign-row span{-webkit-box-flex:1;-webkit-flex:1;flex:1;color:#ccc}
.assign-row select{background:#2a2a2a;border:1px solid #444;color:#eee;padding:6px 8px;border-radius:4px;font-size:12px;-webkit-appearance:none}
.modal-footer{display:-webkit-box;display:-webkit-flex;display:flex;gap:10px;-webkit-box-pack:end;-webkit-justify-content:flex-end;justify-content:flex-end;margin-top:16px;border-top:1px solid #333;padding-top:16px}
.btn-save{background:#005a00;color:#fff;border:none;padding:9px 20px;border-radius:5px;cursor:pointer;font-size:13px;font-weight:bold}
.btn-cancel{background:#333;color:#aaa;border:none;padding:9px 16px;border-radius:5px;cursor:pointer;font-size:13px}
</style>
</head>
<body>
<div class="header">
  <h2>Quest Hub</h2>
  <button class="btn-settings" onclick="openSettings()">Settings</button>
</div>
<div class="tab-bar" id="tabBar"></div>
<div id="cycleBar" style="padding:6px 16px;font-size:12px;color:#666;background:#161616;border-bottom:1px solid #2a2a2a"></div>
<div class="power-block" id="powerBlock"></div>
<div class="content"><div class="device-grid" id="devGrid"></div></div>

<div class="modal-bg" id="modalBg">
  <div class="modal">
    <h3>Settings</h3>
    <div class="sec-title">Quest tabs</div>
    <div id="tabList"></div>
    <button class="add-btn" onclick="addTab()">+ Add tab</button>
    <div class="sec-title">Device assignment</div>
    <div id="assignList"></div>
    <div class="modal-footer">
      <button class="btn-cancel" onclick="closeSettings()">Cancel</button>
      <button class="btn-save" onclick="saveSettings()">Save</button>
    </div>
  </div>
</div>

<script>
var state = {devices:[], tabs:[]};
var activeTab = 0;
var modalTabs = [];
var modalDevTabs = {};

function el(id){ return document.getElementById(id); }

function ce(tag, cls, txt) {
  var e = document.createElement(tag);
  if (cls) e.className = cls;
  if (txt !== undefined) e.textContent = txt;
  return e;
}

function rssiClass(v){
  if(v===0) return '';
  if(v>=-60) return 'r1';
  if(v>=-70) return 'r2';
  if(v>=-80) return 'r3';
  return 'r4';
}

// ===== Tabs bar =====
function renderTabs() {
  var bar = el('tabBar');
  bar.innerHTML = '';

  var t0 = ce('div', 'tab' + (activeTab === -1 ? ' active' : ''), 'Ungrouped');
  t0.onclick = function() { activeTab = -1; renderAll(); };
  bar.appendChild(t0);

  for (var i = 0; i < state.tabs.length; i++) {
    (function(idx) {
      var lbl = state.tabs[idx].label || ('Tab ' + (idx + 1));
      var t = ce('div', 'tab' + (activeTab === idx ? ' active' : ''), lbl);
      t.onclick = function() { activeTab = idx; renderAll(); };
      bar.appendChild(t);
    })(i);
  }
}

// ===== Power block =====
function pwrClass(lbl) {
  var l = lbl.toUpperCase();
  if (l === 'START' || l === 'ON') return 'pstart';
  if (l === 'UNLOCK') return 'punlock';
  if (l === 'OFF') return 'poff';
  return 'pdefault';
}

function getPowerKey() {
  if (activeTab < 0 || activeTab >= state.tabs.length) return '';
  return state.tabs[activeTab].powerKey || '';
}

function renderPower() {
  var block = el('powerBlock');
  block.innerHTML = '';
  var pkey = getPowerKey();
  if (!pkey) return;

  var dev = null;
  for (var i = 0; i < state.devices.length; i++) {
    if (state.devices[i].key === pkey) { dev = state.devices[i]; break; }
  }

  if (!dev || !dev.online) {
    block.appendChild(ce('div', 'power-offline', 'Power device [' + pkey + '] is offline'));
    return;
  }

  var card = ce('div', 'power-card' + (dev.on ? ' on' : ''));
  card.appendChild(ce('div', 'power-title', dev.on ? 'QUEST RUNNING' : 'QUEST OFF'));
  card.appendChild(ce('div', 'power-sub', (dev.on ? 'ON' : 'OFF') + (dev.info ? ' — ' + dev.info : '')));

  if (dev.buttons && dev.buttons.length > 0) {
    var bg = ce('div', 'power-buttons');
    for (var b = 0; b < dev.buttons.length; b++) {
      (function(btn, idx) {
        var pb = ce('button', 'power-btn ' + pwrClass(btn.label), btn.label);
        pb.onclick = function() { sendButton(dev.key, idx); };
        bg.appendChild(pb);
      })(dev.buttons[b], b);
    }
    card.appendChild(bg);
  }
  block.appendChild(card);
}

// ===== Device grid =====
function devBtnClass(lbl) {
  var l = lbl.toUpperCase();
  if (l === 'ON' || l === 'OPEN') return 'bs';
  if (l === 'OFF' || l === 'LOCKED') return 'bd';
  if (l.indexOf('HORROR') >= 0) return 'bd';
  if (l.indexOf('SOFT') >= 0 || l.indexOf('WHITE') >= 0) return 'bp';
  if (l.indexOf('FLICKER') >= 0) return 'bv';
  if (l.indexOf('UNLOCK') >= 0) return 'bw';
  return 'bx';
}

function renderDevices() {
  var grid = el('devGrid');
  grid.innerHTML = '';
  var pkey = getPowerKey();

  var list = [];
  for (var i = 0; i < state.devices.length; i++) {
    var dev = state.devices[i];
    if (dev.key === pkey) continue;
    if (activeTab === -1) {
      if (dev.tabIndex === -1 || dev.tabIndex === undefined || dev.tabIndex === null) list.push(dev);
    } else {
      if (dev.tabIndex === activeTab) list.push(dev);
    }
  }

  if (list.length === 0) {
    grid.appendChild(ce('div', 'empty', 'No devices'));
    return;
  }

  for (var j = 0; j < list.length; j++) {
    (function(dev) {
      var card = ce('div', 'device-card' + (dev.online ? '' : ' offline'));

      var hdr = ce('div', 'device-header');
      var left = ce('div');
      left.appendChild(ce('div', 'device-name', dev.name));
      left.appendChild(ce('div', 'device-type', dev.type));
      hdr.appendChild(left);
      var right = ce('div');
      right.appendChild(ce('span', 'badge ' + (dev.online ? 'bon' : 'boff'), dev.online ? 'ONLINE' : 'OFFLINE'));
      if (dev.rssi !== 0) {
        right.appendChild(ce('span', 'rssi-ind ' + rssiClass(dev.rssi), dev.rssi + ' dBm'));
      }
      hdr.appendChild(right);
      card.appendChild(hdr);

      if (dev.hasState) {
        card.appendChild(ce('div', 'state-ind ' + (dev.on ? 'sind-on' : 'sind-off'), dev.on ? 'ON' : 'OFF'));
      }
      if (dev.info) {
        card.appendChild(ce('div', 'dev-info', dev.info));
      }
      if (dev.buttons && dev.buttons.length > 0) {
        var bg = ce('div', 'btn-group');
        for (var b = 0; b < dev.buttons.length; b++) {
          (function(btn, idx) {
            var pb = ce('button', 'dev-btn ' + devBtnClass(btn.label), btn.label);
            pb.onclick = function() { sendButton(dev.key, idx); };
            bg.appendChild(pb);
          })(dev.buttons[b], b);
        }
        card.appendChild(bg);
      }
      grid.appendChild(card);
    })(list[j]);
  }
}

function renderCycle() {
  var r = state.cycleRemaining;
  var m = Math.floor(r / 60), s = r % 60;
  el('cycleBar').textContent = 'Хрономаг: цикл #' + state.cycleNumber +
    ', до следующего сброса ' + m + 'м ' + (s < 10 ? '0' : '') + s + 'с';
}

function renderAll() { renderTabs(); renderCycle(); renderPower(); renderDevices(); }

// ===== API =====
function refresh() {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', '/api/state', true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4 && xhr.status === 200) {
      try {
        state = JSON.parse(xhr.responseText);
        if (state.tabs.length === 0) { activeTab = -1; }
        else if (activeTab !== -1 && activeTab >= state.tabs.length) { activeTab = 0; }
        renderAll();
      } catch(e) {}
    }
  };
  xhr.send();
}

function sendButton(key, idx) {
  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/button?key=' + encodeURIComponent(key) + '&index=' + idx, true);
  xhr.send();
}

// ===== Settings modal =====
function openSettings() {
  modalTabs = [];
  for (var i = 0; i < state.tabs.length; i++) {
    modalTabs.push({ label: state.tabs[i].label || '', powerKey: state.tabs[i].powerKey || '' });
  }
  modalDevTabs = {};
  for (var j = 0; j < state.devices.length; j++) {
    var d = state.devices[j];
    modalDevTabs[d.key] = (d.tabIndex !== undefined && d.tabIndex !== null) ? d.tabIndex : -1;
  }
  renderTabList();
  renderAssignList();
  el('modalBg').className = 'modal-bg open';
}

function closeSettings() { el('modalBg').className = 'modal-bg'; }

function addTab() {
  modalTabs.push({ label: '', powerKey: '' });
  renderTabList();
  renderAssignList();
}

function renderTabList() {
  var list = el('tabList');
  list.innerHTML = '';
  for (var i = 0; i < modalTabs.length; i++) {
    (function(idx) {
      var item = ce('div', 'tab-item');

      var inp = document.createElement('input');
      inp.type = 'text';
      inp.placeholder = 'Tab name';
      inp.value = modalTabs[idx].label;
      inp.onchange = (function(ii) {
        return function() { modalTabs[ii].label = this.value; renderAssignList(); };
      })(idx);
      item.appendChild(inp);

      var row = ce('div', 'tab-row');

      var sel = document.createElement('select');
      var o0 = document.createElement('option');
      o0.value = ''; o0.textContent = '— no power device —';
      if (!modalTabs[idx].powerKey) o0.selected = true;
      sel.appendChild(o0);
      for (var j = 0; j < state.devices.length; j++) {
        var o = document.createElement('option');
        o.value = state.devices[j].key;
        o.textContent = state.devices[j].key;
        if (state.devices[j].key === modalTabs[idx].powerKey) o.selected = true;
        sel.appendChild(o);
      }
      sel.onchange = (function(ii) {
        return function() { modalTabs[ii].powerKey = this.value; };
      })(idx);
      row.appendChild(sel);

      var del = ce('button', 'del-btn', 'Remove');
      del.onclick = (function(ii) {
        return function() { modalTabs.splice(ii, 1); renderTabList(); renderAssignList(); };
      })(idx);
      row.appendChild(del);

      item.appendChild(row);
      list.appendChild(item);
    })(i);
  }
}

function renderAssignList() {
  var list = el('assignList');
  list.innerHTML = '';

  if (state.devices.length === 0) {
    list.appendChild(ce('div', 'empty', 'No devices online yet'));
    return;
  }

  for (var i = 0; i < state.devices.length; i++) {
    (function(dev) {
      var row = ce('div', 'assign-row');
      row.appendChild(ce('span', null, dev.key));

      var sel = document.createElement('select');
      var o0 = document.createElement('option');
      o0.value = '-1'; o0.textContent = 'Ungrouped';
      if (modalDevTabs[dev.key] === -1 || modalDevTabs[dev.key] === undefined) o0.selected = true;
      sel.appendChild(o0);

      for (var t = 0; t < modalTabs.length; t++) {
        var o = document.createElement('option');
        o.value = String(t);
        o.textContent = modalTabs[t].label || ('Tab ' + (t + 1));
        if (modalDevTabs[dev.key] === t) o.selected = true;
        sel.appendChild(o);
      }
      sel.onchange = (function(key) {
        return function() { modalDevTabs[key] = parseInt(this.value, 10); };
      })(dev.key);
      row.appendChild(sel);
      list.appendChild(row);
    })(state.devices[i]);
  }
}

function saveSettings() {
  var body = '{"tabs":[';
  for (var i = 0; i < modalTabs.length; i++) {
    if (i > 0) body += ',';
    body += '{"label":"' + modalTabs[i].label + '","powerKey":"' + modalTabs[i].powerKey + '"}';
  }
  body += '],"deviceTabs":{';
  var first = true;
  for (var key in modalDevTabs) {
    if (!first) body += ',';
    body += '"' + key + '":' + modalDevTabs[key];
    first = false;
  }
  body += '}}';

  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/settings', true);
  xhr.setRequestHeader('Content-Type', 'application/json');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      closeSettings();
      refresh();
    }
  };
  xhr.send(body);
}

setInterval(refresh, 1500);
refresh();
</script>
</body>
</html>
)HTML";

// ====== HTTP handlers ======
void handleRoot() {
  http.send_P(200, "text/html", HTML_PAGE);
}

void handleStateApi() {
  unsigned long cycleRemaining = (CYCLE_MS - (millis() - cycleStartMs)) / 1000UL;

  String json = "{\"cycleNumber\":";
  json += String(cycleNumber);
  json += ",\"cycleRemaining\":";
  json += String(cycleRemaining);
  json += ",\"tabs\":[";
  bool ft = true;
  for (uint8_t t = 0; t < MAX_TABS; t++) {
    if (!tabs[t].used) continue;
    if (!ft) json += ",";
    ft = false;
    json += "{\"label\":\"";   json += tabs[t].label;    json += "\",";
    json += "\"powerKey\":\""; json += tabs[t].powerKey; json += "\"}";
  }
  json += "],\"devices\":[";
  bool fd = true;
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    if (!devices[i].used) continue;
    if (!fd) json += ",";
    fd = false;
    json += "{";
    json += "\"key\":\"";    json += devices[i].key;   json += "\",";
    json += "\"type\":\"";   json += devices[i].type;  json += "\",";
    json += "\"name\":\"";   json += devices[i].name;  json += "\",";
    json += "\"online\":";   json += isDeviceOnline(devices[i]) ? "true" : "false"; json += ",";
    json += "\"hasState\":"; json += devices[i].hasState ? "true" : "false"; json += ",";
    json += "\"on\":";       json += devices[i].on ? "true" : "false"; json += ",";
    json += "\"info\":\"";   json += devices[i].info;  json += "\",";
    json += "\"rssi\":";     json += String(devices[i].rssi); json += ",";
    json += "\"rssiMin\":";  json += String(devices[i].rssiMin); json += ",";
    json += "\"rssiMax\":";  json += String(devices[i].rssiMax == -127 ? 0 : devices[i].rssiMax); json += ",";
    json += "\"tabIndex\":"; json += String((int)devices[i].tabIndex); json += ",";
    json += "\"buttons\":[";
    for (uint8_t b = 0; b < devices[i].buttonCount; b++) {
      if (b > 0) json += ",";
      json += "{\"index\":"; json += String(b); json += ",";
      json += "\"label\":\""; json += devices[i].buttons[b].label; json += "\"}";
    }
    json += "]}";
  }
  json += "]}";
  http.send(200, "application/json", json);
}

void handleButtonApi() {
  if (!http.hasArg("key") || !http.hasArg("index")) {
    http.send(400, "application/json", "{\"error\":\"missing args\"}"); return;
  }
  String key = http.arg("key");
  int idx = http.arg("index").toInt();
  DeviceState* dev = findDeviceByKey(key.c_str());
  if (!dev) { http.send(404, "application/json", "{\"error\":\"not found\"}"); return; }
  if (idx < 0 || idx >= dev->buttonCount) { http.send(400, "application/json", "{\"error\":\"bad index\"}"); return; }
  Button& btn = dev->buttons[idx];
  Serial.print(F("[HTTP] ")); Serial.print(btn.label);
  Serial.print(F(" -> ")); Serial.print(btn.topic);
  Serial.print(F(" = ")); Serial.println(btn.payload);
  mqtt.publish(btn.topic, btn.payload);
  http.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleSettingsApi() {
  if (!http.hasArg("plain")) { http.send(400, "text/plain", "no body"); return; }
  String body = http.arg("plain");

  for (uint8_t t = 0; t < MAX_TABS; t++) {
    tabs[t].used = false;
    tabs[t].label[0] = '\0';
    tabs[t].powerKey[0] = '\0';
  }

  uint8_t tabCount = 0;
  int tabsStart = body.indexOf("\"tabs\":");
  if (tabsStart >= 0) {
    int arrStart = body.indexOf('[', tabsStart);
    int arrEnd   = body.indexOf(']', arrStart);
    if (arrStart >= 0 && arrEnd > arrStart) {
      String arr = body.substring(arrStart, arrEnd + 1);
      int pos = 0;
      while (tabCount < MAX_TABS) {
        int os = arr.indexOf('{', pos); if (os < 0) break;
        int oe = arr.indexOf('}', os);  if (oe < 0) break;
        String obj = arr.substring(os, oe + 1);
        char lbl[32] = {}, pkey[32] = {};
        int li = obj.indexOf("\"label\":\"");
        if (li >= 0) { li += 9; int le = obj.indexOf('"', li); if (le > li) obj.substring(li, le).toCharArray(lbl, sizeof(lbl)); }
        int pi = obj.indexOf("\"powerKey\":\"");
        if (pi >= 0) { pi += 12; int pe = obj.indexOf('"', pi); if (pe > pi) obj.substring(pi, pe).toCharArray(pkey, sizeof(pkey)); }
        if (strlen(lbl) > 0) {
          tabs[tabCount].used = true;
          strncpy(tabs[tabCount].label,    lbl,  sizeof(tabs[tabCount].label)    - 1);
          strncpy(tabs[tabCount].powerKey, pkey, sizeof(tabs[tabCount].powerKey) - 1);
          tabCount++;
        }
        pos = oe + 1;
      }
    }
  }

  int dtStart = body.indexOf("\"deviceTabs\":");
  if (dtStart >= 0) {
    int os = body.indexOf('{', dtStart);
    int oe = body.lastIndexOf('}');
    if (os >= 0 && oe > os) {
      String dt = body.substring(os + 1, oe);
      int pos = 0;
      while (pos < (int)dt.length()) {
        int qi = dt.indexOf('"', pos);      if (qi < 0) break;
        int qe = dt.indexOf('"', qi + 1);   if (qe < 0) break;
        String devKey = dt.substring(qi + 1, qe);
        int ci = dt.indexOf(':', qe);        if (ci < 0) break;
        int ns = ci + 1;
        while (ns < (int)dt.length() && (dt[ns] == ' ' || dt[ns] == '\n')) ns++;
        int ne = ns;
        if (ne < (int)dt.length() && dt[ne] == '-') ne++;
        while (ne < (int)dt.length() && isdigit(dt[ne])) ne++;
        int tabIdx = dt.substring(ns, ne).toInt();
        DeviceState* dev = findDeviceByKey(devKey.c_str());
        if (dev) dev->tabIndex = (int8_t)tabIdx;
        pos = ne;
      }
    }
  }

  saveSettings();
  http.send(200, "application/json", "{\"status\":\"ok\"}");
  Serial.print(F("[Settings] Saved ")); Serial.print(tabCount); Serial.println(F(" tabs"));
}

void handleNotFound() { http.send(404, "text/plain", "Not found"); }

// ====== Цикл хрономага ======
// Раз в CYCLE_MS — общий "старт нового цикла" для всех устройств.
// Между стартами — периодический tick с обратным отсчётом (для индикации).
void cycleCheck() {
  unsigned long now = millis();

  if (now - cycleStartMs >= CYCLE_MS) {
    cycleStartMs = now;
    cycleNumber++;
    lastCycleTickMs = now;

    char payload[32];
    snprintf(payload, sizeof(payload), "{\"cycle\":%lu}", cycleNumber);
    mqtt.publish("quest/chronomage/cycle/start", payload);
    Serial.print(F("[CYCLE] старт #")); Serial.println(cycleNumber);
    return;
  }

  if (now - lastCycleTickMs >= CYCLE_TICK_MS) {
    lastCycleTickMs = now;
    unsigned long remaining = (CYCLE_MS - (now - cycleStartMs)) / 1000UL;

    char payload[32];
    snprintf(payload, sizeof(payload), "{\"remaining\":%lu}", remaining);
    mqtt.publish("quest/chronomage/cycle/tick", payload);
  }
}

// ====== MQTT ======
void mqttMessageHandler(const char* topic, const char* payload) {
  Serial.print(F("[MQTT] ")); Serial.print(topic);
  Serial.print(F(" | ")); Serial.println(payload);
  char buf[64];
  strncpy(buf, topic, sizeof(buf) - 1); buf[sizeof(buf)-1] = '\0';
  char* sp = nullptr;
  char* p0 = strtok_r(buf, "/", &sp);
  char* p1 = strtok_r(nullptr, "/", &sp);
  char* p2 = strtok_r(nullptr, "/", &sp);
  char* p3 = strtok_r(nullptr, "/", &sp);
  if (!p0||!p1||!p2||!p3) return;
  if (strcmp(p0, "quest") != 0) return;
  DeviceState* dev = findOrCreateDevice(p1, p2);
  if (!dev) { Serial.println(F("[MQTT] No slots!")); return; }
  dev->lastSeenMs = millis();
  if (strcmp(p3, "state") == 0) {
    dev->hasState = true;
    dev->on = (strcasecmp(payload,"ON")==0 || strcasecmp(payload,"OPEN")==0 ||
               strcmp(payload,"1")==0       || strcasecmp(payload,"TRUE")==0);
  } else if (strcmp(p3, "info") == 0) {
    strncpy(dev->info, payload, sizeof(dev->info)-1);
    dev->info[sizeof(dev->info)-1] = '\0';
  } else if (strcmp(p3, "buttons") == 0) {
    parseButtonsConfig(dev, payload);
    Serial.print(F("[MQTT] ")); Serial.print(dev->key);
    Serial.print(F(" buttons: ")); Serial.println(dev->buttonCount);
  } else if (strcmp(p3, "rssi") == 0) {
    int v = atoi(payload);
    if (v != 0) {
      dev->rssi = v;
      if (dev->rssiMin == 0 || v < dev->rssiMin) dev->rssiMin = v;
      if (v > dev->rssiMax) dev->rssiMax = v;
    }
  }
}

// ====== setup & loop ======
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Quest Hub ESP32 ==="));
  loadSettings();
  WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
  WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("[WiFi] Connecting"));
  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 60) {
    delay(500); Serial.print("."); retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WiFi] FAILED"));
  }
  mqtt.subscribe("quest/#", [](const char* topic, const char* payload) {
    mqttMessageHandler(topic, payload);
  });
  mqtt.begin();
  Serial.println(F("[MQTT] :1883"));
  http.on("/",             HTTP_GET,  handleRoot);
  http.on("/api/state",    HTTP_GET,  handleStateApi);
  http.on("/api/button",   HTTP_POST, handleButtonApi);
  http.on("/api/settings", HTTP_POST, handleSettingsApi);
  http.onNotFound(handleNotFound);
  http.begin();
  Serial.println(F("[HTTP] :80"));

  // --- OTA ---
  ArduinoOTA.setHostname("quest-hub");
  ArduinoOTA.setPassword("quest2025");
  ArduinoOTA.onStart([]() { Serial.println(F("[OTA] Start")); });
  ArduinoOTA.onEnd([]()   { Serial.println(F("[OTA] Done")); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.print(F("[OTA] Error: ")); Serial.println(e); });
  ArduinoOTA.begin();
  Serial.println(F("[OTA] Ready (quest-hub / quest2025)"));
}

void loop() {
  ArduinoOTA.handle();
  http.handleClient();
  mqtt.loop();
  cycleCheck();
  delay(1);
}
