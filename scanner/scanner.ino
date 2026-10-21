/* -*- tab-width: 2; mode: c++; -*-
 * Generic RF Sniffer - ESP32
 * AP: "DroneScanner" password "12345678"
 * Web: http://192.168.4.1
 */

#if !defined(ARDUINO_ARCH_ESP32)
#error "This program requires an ESP32"
#endif

#define MAX_DRONES 20
#define FW_VERSION "2.1"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "opendroneid.h"
#include "odid_wifi.h"
/* markers */
#define PKT_MARKER_0     { 0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00 }
#define FRAME_TYPE_B     0x80
#define ELEM_TAG         0xdd
#define VND_A_0          0x90
#define VND_A_1          0x3a
#define VND_A_2          0xe6
#define VND_B_0          0xfa
#define VND_B_1          0x0b
#define VND_B_2          0xbc
#define OFFSET_A         36
#define OFFSET_B         7

#include <esp_timer.h>
#include <WiFi.h>
#include <WebServer.h>

// Config
const int SERIAL1_RX_PIN = 7;
const int SERIAL1_TX_PIN = 6;

const char* AP_SSID = "DroneScanner";
const char* AP_PASSWORD = "12345678";
const int AP_CHANNEL = 6;

WebServer server(80);

#define DRONE_MODEL_COUNT 100
typedef struct {
  const char* prefix;
  const char* name;
  const char* brand;
  const char* category;
} DroneModel;

const DroneModel droneModels[DRONE_MODEL_COUNT] PROGMEM = {
  {"1581F45T", "Mavic 3", "DJI", "消费级"},
  {"1581F67P", "Mavic 3 Classic", "DJI", "消费级"},
  {"1581F67Q", "Mavic 3 Pro", "DJI", "消费级"},
  {"1581F6MK", "Mavic 3 Pro Cine", "DJI", "消费级"},
  {"1581F5YH", "Mini 3", "DJI", "消费级"},
  {"1581F4XF", "Mini 3 Pro", "DJI", "消费级"},
  {"1581F6Z9", "Mini 4 Pro", "DJI", "消费级"},
  {"1581F6CD", "Mini 2 SE", "DJI", "消费级"},
  {"1581F8PJ", "Mini 4K", "DJI", "消费级"},
  {"1581F9DE", "Mini 5 Pro", "DJI", "消费级"},
  {"1581FANL", "Mini 5 Pro (4G)", "DJI", "消费级"},
  {"1581F8LQ", "Mavic 4 Pro (64GB)", "DJI", "消费级"},
  {"1581F986", "Mavic 4 Pro (512GB)", "DJI", "消费级"},
  {"1581F6N8", "Air 3", "DJI", "消费级"},
  {"1581F895", "Air 3S", "DJI", "消费级"},
  {"1581F11V", "Phantom 4 Pro V2.0", "DJI", "消费级"},
  {"1581FB34", "Lito X1", "DJI", "消费级"},
  {"1581FBV5", "Lito 1", "DJI", "消费级"},
  {"1581F7V2", "Flip", "DJI", "穿越机"},
  {"1581F4QW", "Avata", "DJI", "穿越机"},
  {"1581F6W8", "Avata 2", "DJI", "穿越机"},
  {"1581FA8J", "Avata 360 (DVN3XT)", "DJI", "穿越机"},
  {"1581FBLK", "Avata 360 (DVN3NT)", "DJI", "穿越机"},
  {"1581F37Q", "FPV", "DJI", "穿越机"},
  {"1581F87L", "Neo", "DJI", "穿越机"},
  {"1581FA6Q", "Neo 2", "DJI", "穿越机"},
  {"1581F5FH", "Mavic 3E", "DJI", "行业级"},
  {"1581F5FJ", "Mavic 3T", "DJI", "行业级"},
  {"1581F5FK", "Mavic 3M", "DJI", "行业级"},
  {"1581F6QA", "Matrice 3D", "DJI", "行业级"},
  {"1581F6Q8", "Matrice 3TD", "DJI", "行业级"},
  {"1581FBEX", "Matrice 3TA", "DJI", "行业级"},
  {"1581F4BN", "经纬M30", "DJI", "行业级"},
  {"1581F5BK", "Matrice 30T", "DJI", "行业级"},
  {"1581F62H", "Matrice 30T", "DJI", "行业级"},
  {"1581F5BL", "Matrice 30（机场版）", "DJI", "行业级"},
  {"1581F5BM", "Matrice 30T（机场版）", "DJI", "行业级"},
  {"1581F6GK", "Matrice 350 RTK", "DJI", "行业级"},
  {"1581F8DB", "Matrice 400", "DJI", "行业级"},
  {"1581F7FV", "Matrice 4E", "DJI", "行业级"},
  {"1581F7K3", "Matrice 4T", "DJI", "行业级"},
  {"1581F8HH", "Matrice 4D", "DJI", "行业级"},
  {"1581F8HG", "Matrice 4TD", "DJI", "行业级"},
  {"1581F4GC", "Mavic 2 Enterprise Advanced", "DJI", "行业级"},
  {"1581F0UY", "Phantom 4 RTK", "DJI", "行业级"},
  {"1581F4Z4", "Inspire 3", "DJI", "行业级"},
  {"1581F446", "Agras T10 / T30", "DJI", "植保级"},
  {"1581F574", "Agras T20P / T40", "DJI", "植保级"},
  {"1581F6BU", "Agras T50", "DJI", "植保级"},
  {"1581F836", "Agras T25P", "DJI", "植保级"},
  {"1581F8ZL", "Agras T100", "DJI", "植保级"},
  {"1581F8ZX", "Agras T70P", "DJI", "植保级"},
  {"1581FAE3", "Agras T70S", "DJI", "植保级"},
  {"1581F7C6", "FlyCart 30", "DJI", "物流级"},
  {"1581FAN4", "FlyCart 100", "DJI", "物流级"},
  {"1748FEV3", "EVO Max / MDX-1", "Autel", "行业级"},
  {"1748FEV4", "Autel Alpha", "Autel", "行业级"},
  {"1748CHL2", "EVO II", "Autel", "消费级"},
  {"1748CHL7", "EVO II", "Autel", "消费级"},
  {"1748CLT0", "EVO Lite", "Autel", "消费级"},
  {"1748CLTG", "EVO Lite", "Autel", "消费级"},
  {"1748CLTC", "EVO Lite+", "Autel", "消费级"},
  {"1748CNA0", "EVO Nano", "Autel", "消费级"},
  {"1748CNAG", "EVO Nano", "Autel", "消费级"},
  {"1748FEV2", "EVO II V3", "Autel", "消费级"},
  {"1748CJD1", "Dragonfish Standard", "Autel", "行业级"},
  {"1748CJD2", "Dragonfish Lite", "Autel", "行业级"},
  {"1748CJD3", "Dragonfish Pro", "Autel", "行业级"},
  {"11223344", "Anafi", "Parrot", "消费级"},
  {"1588E040", "Anafi AI", "Parrot", "行业级"},
  {"44332211", "Anafi USA", "Parrot", "消费级"},
  {"PF723000", "Rolling Spider", "Parrot", "消费级"},
  {"PF727078", "Mambo FLY", "Parrot", "消费级"},
  {"P208904", "Skydio 2 / 2+", "Skydio", "消费级"},
  {"SDR21V1", "Skydio X2E", "Skydio", "行业级"},
  {"SDR35V1", "Skydio X2", "Skydio", "行业级"},
  {"SR47PCV", "Skydio X10", "Skydio", "行业级"},
  {"YUNH5ETWET", "H520 Hexacopter", "Yuneec", "行业级"},
  {"YUNH920202EU", "Tornado H920", "Yuneec", "行业级"},
  {"YUNQ4KPUS", "Q500 4K Typhoon", "Yuneec", "消费级"},
  {"YUNTYH3EU", "Typhoon H3", "Yuneec", "行业级"},
  {"1758X000", "P150 Max", "极目", "植保级"},
  {"1758X001", "P100 Pro", "极目", "植保级"},
  {"1758X002", "P100", "极目", "植保级"},
  {"1758X003", "V40", "极目", "植保级"},
  {"18179000", "Alta X", "Freefly", "影视级"},
  {"18179001", "Alta X Gen2", "Freefly", "影视级"},
  {"18179002", "Alta 8", "Freefly", "影视级"},
  {"1587A000", "eBee X", "senseFly", "测绘级"},
  {"1587A001", "eBee TAC", "senseFly", "测绘级"},
  {"1587A002", "eBee Geo", "senseFly", "测绘级"},
  {"1587A003", "eBee RTK", "senseFly", "测绘级"},
  {"1567A000", "md4-1000", "Microdrones", "测绘级"},
  {"1567A001", "md4-3000", "Microdrones", "测绘级"},
  {"1688W000", "WingtraOne Gen II", "Wingtra", "测绘级"},
  {"1688W001", "WingtraOne GEN I", "Wingtra", "测绘级"},
  {"1701A000", "Percepto Max", "Percepto", "行业级"},
  {"1701A001", "Percepto Sparrow", "Percepto", "行业级"},
  {"1701A002", "Percepto Air", "Percepto", "行业级"},
  {"1778S000", "SplashDrone 4", "SwellPro", "消费级"}
};

// 机型 lookup cache
struct model_cache {
  char name[32];
  char brand[16];
  char category[12];
  bool found;
};

model_cache modelCache[MAX_DRONES];

// Cache

struct uav_data {
  uint8_t mac[6];
  int8_t rssi;
  char op_id[ODID_ID_SIZE + 1];
  char uav_id[ODID_ID_SIZE + 1];
  double lat_d;
  double long_d;
  double base_lat_d;
  double base_long_d;
  int altitude_msl;
  int height_agl;
  int speed;
  int heading;
  int speed_vertical;
  int altitude_pressure;
  int horizontal_accuracy;
  int vertical_accuracy;
  int speed_accuracy;
  int timestamp;
  int status;
  int operator_location_type;
  uint32_t system_timestamp;
  int operator_id_type;
  uint8_t auth_type;
  uint8_t auth_page;
  uint8_t auth_length;
  uint32_t auth_timestamp;
  char auth_data[25];
  uint8_t desc_type;
  char description[ODID_STR_SIZE + 1];
  unsigned long last_seen;
};

uav_data droneCache[MAX_DRONES];
int droneCount = 0;
unsigned long lastHousekeeping = 0;
static unsigned long packetCount = 0;
unsigned long last_status = 0;

ODID_UAS_Data uas;

// Forward declarations
void callback(void *, wifi_promiscuous_pkt_type_t);
void parse_odid(struct uav_data *, ODID_UAS_Data *);
void store_mac(struct uav_data *uav, uint8_t *payload);
void printDroneInfo(int idx);
int findOrAddDrone(uint8_t *mac);
const DroneModel* lookup_drone_model(const char* uav_id);
String jsonEscape(const char* s);
void handleRoot();
void handleApiDrones();
void handleClear();

// Web UI
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>无人机探测</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box}
    html{overflow-x:hidden}
    body{font-family:'PingFang SC','Microsoft YaHei',-apple-system,sans-serif;background:#0f0f1a;color:#e0e0e0;min-height:100vh;-webkit-text-size-adjust:100%}
    .hd{background:rgba(0,0,0,.4);padding:14px 16px;display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:100;backdrop-filter:blur(10px)}
    .hd h1{font-size:18px;font-weight:600;display:flex;align-items:center;gap:8px}
    .hd h1::before{content:'🛸';font-size:22px}
    .badge{background:rgba(76,175,80,.15);color:#4caf50;padding:6px 12px;border-radius:16px;font-size:12px;border:1px solid rgba(76,175,80,.25)}
    .badge::before{content:'● ';animation:bk 1s infinite}
    @keyframes bk{50%{opacity:.3}}
    .stats{display:flex;gap:10px;padding:14px 16px;overflow-x:auto;-webkit-overflow-scrolling:touch}
    .st{flex:0 0 auto;min-width:100px;background:rgba(255,255,255,.04);border-radius:12px;padding:14px 16px;text-align:center;border:1px solid rgba(255,255,255,.08)}
    .st .ic{font-size:22px;margin-bottom:4px}
    .st .v{font-size:26px;font-weight:700;color:#fff}
    .st .l{font-size:11px;color:#666;margin-top:2px}
    .st.d .v{color:#e94560}.st.p .v{color:#4fc3f7}.st.t .v{color:#4caf50}
    .ctrls{padding:0 16px 10px;display:flex;gap:8px}
    .btn{background:linear-gradient(135deg,#e94560,#ff6b6b);color:#fff;border:none;padding:10px 18px;border-radius:8px;cursor:pointer;font-size:13px;font-weight:500;flex-shrink:0}
    .btn2{background:rgba(255,255,255,.08);color:#aaa}
    .btn:active{opacity:.8}
    .list{padding:6px 16px 80px}
    .item{background:rgba(255,255,255,.04);border-radius:12px;margin-bottom:10px;overflow:hidden;border:1px solid rgba(255,255,255,.08);transition:border-color .3s}
    .item:hover{border-color:rgba(233,69,96,.3)}
    .item-head{padding:12px 14px;display:flex;align-items:center;gap:10px;cursor:pointer;-webkit-tap-highlight-color:transparent;user-select:none}
    .item-icon{font-size:28px;flex-shrink:0}
    .item-info{flex:1;min-width:0;overflow:hidden}
    .item-name{font-size:15px;font-weight:600;color:#fff;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .item-name.unknown{color:#ff9800}
    .item-sub{font-size:11px;color:#666;margin-top:2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .item-right{flex-shrink:0;display:flex;flex-direction:column;align-items:flex-end;gap:4px}
    .item-rssi{font-size:11px;color:#888;white-space:nowrap}
    .rbar{width:50px;height:4px;background:rgba(255,255,255,.1);border-radius:2px;overflow:hidden}
    .rfill{height:100%;border-radius:2px}
    .cat-tag{font-size:10px;padding:2px 8px;border-radius:8px;white-space:nowrap}
    .cat-tag.c0{background:rgba(76,175,80,.15);color:#4caf50}
    .cat-tag.c1{background:rgba(33,150,243,.15);color:#42a5f5}
    .cat-tag.c2{background:rgba(156,39,176,.15);color:#ab47bc}
    .cat-tag.c3{background:rgba(255,152,0,.15);color:#ff9800}
    .cat-tag.c4{background:rgba(233,69,96,.15);color:#e94560}
    .cat-tag.c5{background:rgba(38,198,218,.15);color:#26c6da}
    .cat-tag.c6{background:rgba(255,213,79,.15);color:#ffd54f}
    .cat-tag.c7{background:rgba(179,157,219,.15);color:#b39ddb}
    .arrow{color:#555;font-size:12px;transition:transform .3s;flex-shrink:0}
    .item.exp .arrow{transform:rotate(90deg)}
    .detail{display:none;border-top:1px solid rgba(255,255,255,.06);padding:12px 14px;background:rgba(0,0,0,.15)}
    .item.exp .detail{display:block}
    .dg{display:grid;grid-template-columns:1fr 1fr;gap:8px}
    .di{background:rgba(0,0,0,.2);padding:8px 10px;border-radius:6px;overflow:hidden}
    .di .lb{font-size:10px;color:#555;margin-bottom:2px}
    .di .vl{font-size:12px;color:#ccc;font-weight:500;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
    .di.fw{grid-column:span 2}
    .di.sn .vl{white-space:normal;word-break:break-all;font-size:11px}
    .di a{color:#4fc3f7;text-decoration:none;font-size:12px}
    .di a:active{opacity:.7}
    .empty{text-align:center;padding:60px 20px;color:#555}
    .empty .ei{font-size:48px;margin-bottom:16px;opacity:.4}
    .empty h3{font-size:16px;color:#666;margin-bottom:8px}
    .empty p{font-size:13px;color:#444}
    .lu{text-align:center;padding:8px;color:#444;font-size:11px}
  </style>
</head>
<body>
  <div class="hd"><h1>无人机探测</h1><div class="badge">扫描中</div></div>
  <div class="stats">
    <div class="st d"><div class="ic">🛸</div><div class="v" id="dc">0</div><div class="l">无人机</div></div>
    <div class="st p"><div class="ic">📡</div><div class="v" id="pc">0</div><div class="l">数据包</div></div>
    <div class="st t"><div class="ic">⏱️</div><div class="v" id="at">--</div><div class="l">运行时长</div></div>
  </div>
  <div class="ctrls">
    <button class="btn" onclick="load()">🔄 刷新</button>
    <button class="btn btn2" onclick="clr()">🗑️ 清空</button>
  </div>
  <div class="list" id="dl"></div>
  <div class="lu" id="lu">--</div>
<script>
var st=Date.now();
function ft(s){var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=Math.floor(s%60);return(h>0?h+'h ':'')+(m>0?m+'m ':'')+sc+'s'}
function rp(r){return Math.max(0,Math.min(100,((r+100)/70)*100))}
function rc(r){if(r>70)return'#4caf50';if(r>50)return'#8bc34a';if(r>30)return'#ff9800';return'#e94560'}
    function cc(c){if(!c)return'c3';if(c.indexOf('消费级')>=0)return'c0';if(c.indexOf('穿越机')>=0)return'c1';if(c.indexOf('行业级')>=0)return'c2';if(c.indexOf('植保级')>=0)return'c4';if(c.indexOf('物流级')>=0)return'c5';if(c.indexOf('影视级')>=0)return'c6';if(c.indexOf('测绘级')>=0)return'c7';return'c3'}
function tog(id){var e=document.getElementById(id);if(e)e.classList.toggle('exp')}
function load(){
  var expanded={};
  var els=document.querySelectorAll('.item.exp');
  for(var i=0;i<els.length;i++)expanded[els[i].id]=1;
  fetch('/api/drones').then(function(r){return r.json()}).then(function(data){
    document.getElementById('dc').textContent=data.drones?data.drones.length:0;
    document.getElementById('pc').textContent=data.packets||0;
    document.getElementById('at').textContent=ft((Date.now()-st)/1000);
    var ds=data.drones||[],h='';
    if(!ds.length){
      h='<div class="empty"><div class="ei">📡</div><h3>暂无探测记录</h3><p>正在扫描周围的无人机信号...</p></div>';
    }else{
      for(var i=0;i<ds.length;i++){
        var d=ds[i],id='d'+i;
        var mn=d.model_name||'未知机型';
        var mc=d.model_cat||'';
        var uk=!d.model_name;
        var exp=expanded[id]?' exp':'';
        var rp2=rp(d.rssi),rc2=rc(d.rssi);
        var icon=uk?'❓':'🛸';
        var dm=d.lat?'https://maps.google.com/?q='+d.lat.toFixed(6)+','+d.lon.toFixed(6):'';
        var pm=d.base_lat?'https://maps.google.com/?q='+d.base_lat.toFixed(6)+','+d.base_lon.toFixed(6):'';
        h+='<div class="item'+exp+'" id="'+id+'">';
        h+='<div class="item-head" onclick="tog(\''+id+'\')">';
        h+='<div class="item-icon">'+icon+'</div>';
        h+='<div class="item-info">';
        h+='<div class="item-name'+(uk?' unknown':'')+'">'+mn+'</div>';
        h+='<div class="item-sub">'+(d.brand?d.brand+' · ':'')+d.mac+(d.alt?' · '+d.alt+'m':'')+(d.speed?' · '+d.speed+'m/s':'')+'</div>';
        h+='</div>';
        h+='<div class="item-right">';
        if(mc)h+='<span class="cat-tag '+cc(mc)+'">'+mc+'</span>';
        h+='<div class="item-rssi">'+d.rssi+'dBm <span class="rbar"><span class="rfill" style="width:'+rp2+'%;background:'+rc2+'"></span></span></div>';
        h+='</div>';
        h+='<div class="arrow">▶</div>';
        h+='</div>';
        h+='<div class="detail"><div class="dg">';
        if(d.uav_id)h+='<div class="di sn"><div class="lb">Serial</div><div class="vl">'+d.uav_id+'</div></div>';
        if(d.op_id)h+='<div class="di"><div class="lb">操作员</div><div class="vl">'+d.op_id+'</div></div>';
        if(dm)h+='<div class="di fw"><div class="lb">🛸 无人机位置</div><div class="vl"><a href="'+dm+'" target="_blank">📍 '+d.lat.toFixed(6)+', '+d.lon.toFixed(6)+'</a></div></div>';
        if(pm)h+='<div class="di fw"><div class="lb">🎯 飞手位置</div><div class="vl"><a href="'+pm+'" target="_blank">📍 '+d.base_lat.toFixed(6)+', '+d.base_lon.toFixed(6)+'</a></div></div>';
        if(d.alt!==undefined)h+='<div class="di"><div class="lb">海拔高度</div><div class="vl">'+d.alt+' m</div></div>';
        if(d.height_agl!==undefined)h+='<div class="di"><div class="lb">相对高度</div><div class="vl">'+d.height_agl+' m</div></div>';
        if(d.speed!==undefined)h+='<div class="di"><div class="lb">水平速度</div><div class="vl">'+d.speed+' m/s</div></div>';
        if(d.heading!==undefined)h+='<div class="di"><div class="lb">航向</div><div class="vl">'+d.heading+'°</div></div>';
        if(d.speed_vertical!==undefined)h+='<div class="di"><div class="lb">V.水平速度</div><div class="vl">'+d.speed_vertical+' m/s</div></div>';
        if(d.desc&&d.desc.length>0)h+='<div class="di fw"><div class="lb">描述</div><div class="vl">'+d.desc+'</div></div>';
        h+='<div class="di"><div class="lb">最后活跃</div><div class="vl">'+d.last_seen+'秒前</div></div>';
        h+='</div></div></div>';
      }
    }
    document.getElementById('dl').innerHTML=h;
    document.getElementById('lu').textContent='更新: '+new Date().toLocaleTimeString('zh-CN')+(data.version?' · v'+data.version:'');
  });
}
function clr(){fetch('/api/clear').then(function(){load()})}
setInterval(load,2000);
setInterval(function(){document.getElementById('at').textContent=ft((Date.now()-st)/1000)},1000);
window.onload=load;
</script>
</body>
</html>
)rawliteral";

// Setup
void setup() {
  setCpuFrequencyMhz(160);
  memset(droneCache, 0, sizeof(droneCache));
  memset(modelCache, 0, sizeof(modelCache));

  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.printf("=== 无人机探测 v%s ===\n", FW_VERSION);
  Serial.println("========================================");

  nvs_flash_init();

  Serial.println("Starting AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 4);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("AP: %s\n", AP_SSID);
  Serial.printf("Web: http://%s\n", apIP.toString().c_str());

  Serial.println("Init RF...");
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback);
  esp_wifi_set_channel(AP_CHANNEL, WIFI_SECOND_CHAN_NONE);

  server.on("/", handleRoot);
  server.on("/api/drones", handleApiDrones);
  server.on("/api/clear", handleClear);
  server.begin();
  Serial.println("Web server ready");

  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.println("\n正在扫描周围的无人机信号...\n");
}

// Loop
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastHousekeeping > 10000) {
    lastHousekeeping = now;
    for (int i = 0; i < MAX_DRONES; i++) {
      if (droneCache[i].mac[0] != 0 && (now - droneCache[i].last_seen > 120000)) {
        Serial.printf("[exp] stale: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      droneCache[i].mac[0], droneCache[i].mac[1], droneCache[i].mac[2],
                      droneCache[i].mac[3], droneCache[i].mac[4], droneCache[i].mac[5]);
        memset(&droneCache[i], 0, sizeof(uav_data));
        memset(&modelCache[i], 0, sizeof(model_cache));
        droneCount--;
      }
    }
  }

  if (now - last_status > 60000UL) {
    Serial.printf("[state] %d targets | %lu packets\n", droneCount, packetCount);
    last_status = now;
  }

  delay(1);
}

// 机型 lookup
const DroneModel* lookup_drone_model(const char* uav_id) {
  if (!uav_id || uav_id[0] == 0) return NULL;

  for (int i = 0; i < DRONE_MODEL_COUNT; i++) {
    int prefixLen = strlen(droneModels[i].prefix);
    if (strncmp(uav_id, droneModels[i].prefix, prefixLen) == 0) {
      return &droneModels[i];
    }
  }
  return NULL;
}

// Escape untrusted strings before embedding into JSON
String jsonEscape(const char* s) {
  if (!s) return "\"\"";
  String out = "\"";
  for (const char* p = s; *p; p++) {
    char c = *p;
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  out += "\"";
  return out;
}

// Web Handlers
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleApiDrones() {
  String json = "{\"drones\":[";
  int count = 0;

  for (int i = 0; i < MAX_DRONES; i++) {
    if (droneCache[i].mac[0] == 0) continue;
    if (count > 0) json += ",";

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             droneCache[i].mac[0], droneCache[i].mac[1], droneCache[i].mac[2],
             droneCache[i].mac[3], droneCache[i].mac[4], droneCache[i].mac[5]);

    // Lookup model
    const char* modelName = modelCache[i].found ? modelCache[i].name : NULL;
    const char* modelCat = modelCache[i].found ? modelCache[i].category : NULL;
    const char* modelBrand = modelCache[i].found ? modelCache[i].brand : NULL;

    // If not cached, try to lookup
    if (!modelCache[i].found && droneCache[i].uav_id[0] != 0) {
      const DroneModel* m = lookup_drone_model(droneCache[i].uav_id);
      if (m) {
        modelName = m->name;
        modelCat = m->category;
        modelBrand = m->brand;
        strncpy(modelCache[i].name, m->name, sizeof(modelCache[i].name) - 1);
        strncpy(modelCache[i].category, m->category, sizeof(modelCache[i].category) - 1);
        strncpy(modelCache[i].brand, m->brand, sizeof(modelCache[i].brand) - 1);
        modelCache[i].found = true;
        Serial.printf("[match] %s -> %s %s (%s)\n", droneCache[i].uav_id, m->brand, m->name, m->category);
      }
    }

    unsigned long lastSeenSec = (millis() - droneCache[i].last_seen) / 1000;

    json += "{";
    json += "\"mac\":\"" + String(macStr) + "\"";
    json += ",\"rssi\":" + String(droneCache[i].rssi);
    json += ",\"lat\":" + String(droneCache[i].lat_d, 6);
    json += ",\"lon\":" + String(droneCache[i].long_d, 6);
    json += ",\"base_lat\":" + String(droneCache[i].base_lat_d, 6);
    json += ",\"base_lon\":" + String(droneCache[i].base_long_d, 6);
    json += ",\"alt\":" + String(droneCache[i].altitude_msl);
    json += ",\"height_agl\":" + String(droneCache[i].height_agl);
    json += ",\"speed\":" + String(droneCache[i].speed);
    json += ",\"heading\":" + String(droneCache[i].heading);
    json += ",\"speed_vertical\":" + String(droneCache[i].speed_vertical);
    json += ",\"uav_id\":" + jsonEscape(droneCache[i].uav_id);
    json += ",\"op_id\":" + jsonEscape(droneCache[i].op_id);
    json += ",\"desc\":" + jsonEscape(droneCache[i].description);
    json += ",\"model_name\":" + jsonEscape(modelName ? modelName : "");
    json += ",\"model_cat\":" + jsonEscape(modelCat ? modelCat : "");
    json += ",\"brand\":" + jsonEscape(modelBrand ? modelBrand : "");
    json += ",\"last_seen\":" + String(lastSeenSec);
    json += "}";
    count++;
  }

  json += "],\"packets\":" + String(packetCount);
  json += ",\"version\":\"" + String(FW_VERSION) + "\"";
  json += ",\"uptime\":" + String(millis() / 1000);
  json += "}";
  server.send(200, "application/json", json);
}

void handleClear() {
  memset(droneCache, 0, sizeof(droneCache));
  memset(modelCache, 0, sizeof(modelCache));
  droneCount = 0;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
  Serial.println("[clear] list cleared");
}

// Cache
int findOrAddDrone(uint8_t *mac) {
  for (int i = 0; i < MAX_DRONES; i++) {
    if (memcmp(droneCache[i].mac, mac, 6) == 0) return i;
  }
  for (int i = 0; i < MAX_DRONES; i++) {
    if (droneCache[i].mac[0] == 0) {
      memcpy(droneCache[i].mac, mac, 6);
      memset(modelCache[i].name, 0, sizeof(modelCache[i].name));
      memset(modelCache[i].brand, 0, sizeof(modelCache[i].brand));
      memset(modelCache[i].category, 0, sizeof(modelCache[i].category));
      modelCache[i].found = false;
      droneCount++;
      return i;
    }
  }
  return -1;
}

// RX callback
void callback(void *buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
  uint8_t *payload = packet->payload;
  int length = packet->rx_ctrl.sig_len;

  uav_data tempUAV;
  memset(&tempUAV, 0, sizeof(tempUAV));

  store_mac(&tempUAV, payload);
  tempUAV.rssi = packet->rx_ctrl.rssi;

  static const uint8_t pkt_mark_0[6] = PKT_MARKER_0;
  bool parsed = false;

  if (memcmp(pkt_mark_0, &payload[4], 6) == 0) {
    if (rf_recv_pkt_a(&uas,
          (char *)tempUAV.op_id, payload, length) == 0) {
      parse_odid(&tempUAV, &uas);
      parsed = true;
    }
  }
  else if (payload[0] == FRAME_TYPE_B) {
    int offset = OFFSET_A;
    while (offset < length && !parsed) {
      int typ = payload[offset];
      int len = payload[offset + 1];
      if ((typ == ELEM_TAG) &&
          (((payload[offset + 2] == VND_A_0 && payload[offset + 3] == VND_A_1 && payload[offset + 4] == VND_A_2)) ||
           ((payload[offset + 2] == VND_B_0 && payload[offset + 3] == VND_B_1 && payload[offset + 4] == VND_B_2)))) {
        int j = offset + OFFSET_B;
        if (j < length) {
          memset(&uas, 0, sizeof(uas));
          rf_data_decode(&uas, &payload[j], length - j);
          parse_odid(&tempUAV, &uas);
          parsed = true;
        }
      }
      offset += len + 2;
    }
  }

  if (parsed) {
    packetCount++;

    int idx = findOrAddDrone(tempUAV.mac);
    if (idx >= 0) {
      memcpy(&droneCache[idx], &tempUAV, sizeof(uav_data));
      droneCache[idx].last_seen = millis();
      printDroneInfo(idx);
    }
  }
}

void store_mac(uav_data *uav, uint8_t *payload) {
  memcpy(uav->mac, &payload[10], 6);
}

void printDroneInfo(int idx) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           droneCache[idx].mac[0], droneCache[idx].mac[1], droneCache[idx].mac[2],
           droneCache[idx].mac[3], droneCache[idx].mac[4], droneCache[idx].mac[5]);

  Serial.printf("[%lu] %s RSSI:%d", packetCount, macStr, droneCache[idx].rssi);

  if (droneCache[idx].uav_id[0] != 0) {
    const DroneModel* m = lookup_drone_model(droneCache[idx].uav_id);
    if (m) {
      Serial.printf(" | 机型:%s %s(%s)", m->brand, m->name, m->category);
    } else {
      Serial.printf(" | ID:%s", droneCache[idx].uav_id);
    }
  }

  if (droneCache[idx].lat_d != 0.0 && droneCache[idx].long_d != 0.0) {
    Serial.printf(" | %.6f,%.6f alt:%dm", 
                  droneCache[idx].lat_d, droneCache[idx].long_d, droneCache[idx].altitude_msl);
  }

  if (droneCache[idx].op_id[0] != 0) {
    Serial.printf(" | OP:%s", droneCache[idx].op_id);
  }
  Serial.println();
}

// Parse payload
void parse_odid(uav_data *UAV, ODID_UAS_Data *src) {
  memset(UAV->op_id, 0, sizeof(UAV->op_id));
  memset(UAV->uav_id, 0, sizeof(UAV->uav_id));
  memset(UAV->description, 0, sizeof(UAV->description));
  memset(UAV->auth_data, 0, sizeof(UAV->auth_data));

  if (src->BasicIDValid[0]) {
    strncpy(UAV->uav_id, (char *)src->BasicID[0].UASID, ODID_ID_SIZE);
  }

  if (src->LocationValid) {
    UAV->lat_d = src->Location.Latitude;
    UAV->long_d = src->Location.Longitude;
    UAV->altitude_msl = (int)src->Location.AltitudeGeo;
    UAV->height_agl = (int)src->Location.Height;
    UAV->speed = (int)src->Location.SpeedHorizontal;
    UAV->heading = (int)src->Location.Direction;
    UAV->speed_vertical = (int)src->Location.SpeedVertical;
    UAV->altitude_pressure = (int)src->Location.AltitudeBaro;
    UAV->horizontal_accuracy = src->Location.HorizAccuracy;
    UAV->vertical_accuracy = src->Location.VertAccuracy;
    UAV->speed_accuracy = src->Location.SpeedAccuracy;
    UAV->timestamp = (int)src->Location.TimeStamp;
    UAV->status = src->Location.Status;
  }

  if (src->SystemValid) {
    UAV->base_lat_d = src->System.OperatorLatitude;
    UAV->base_long_d = src->System.OperatorLongitude;
    UAV->operator_location_type = src->System.OperatorLocationType;
    UAV->system_timestamp = src->System.Timestamp;
  }

  if (src->AuthValid[0]) {
    UAV->auth_type = src->Auth[0].AuthType;
    UAV->auth_page = src->Auth[0].DataPage;
    UAV->auth_timestamp = src->Auth[0].Timestamp;
    memcpy(UAV->auth_data, src->Auth[0].AuthData, sizeof(UAV->auth_data) - 1);
  }

  if (src->SelfIDValid) {
    UAV->desc_type = src->SelfID.DescType;
    strncpy(UAV->description, src->SelfID.Desc, ODID_STR_SIZE);
  }

  if (src->OperatorIDValid) {
    UAV->operator_id_type = src->OperatorID.OperatorIdType;
    strncpy(UAV->op_id, (char *)src->OperatorID.OperatorId, ODID_ID_SIZE);
  }
}
