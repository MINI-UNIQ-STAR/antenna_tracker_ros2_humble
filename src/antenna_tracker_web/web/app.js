/* Antenna Tracker Web Dashboard — rosbridge client */
'use strict';

const WS_URL       = 'ws://' + window.location.hostname + ':9090';
const MAX_CHART_PTS = 200;

let ros   = null;
let chart = null;

// Compass state
let compass = {
    canvas: null, ctx: null,
    az: 0, tgtAz: 0, el: 0
};

/* ── ROS Connection ──────────────────────────────── */
function connectROS() {
    ros = new ROSLIB.Ros({ url: WS_URL });
    ros.on('connection', () => {
        setStatusUI(true);
        subscribeTopics();
    });
    ros.on('error',   () => setStatusUI(false));
    ros.on('close',   () => {
        setStatusUI(false);
        setTimeout(connectROS, 3000);
    });
}

function setStatusUI(connected) {
    document.getElementById('statusDot').className  = 'status-dot ' + (connected ? 'connected' : 'disconnected');
    document.getElementById('statusText').textContent = connected ? 'Connected' : 'Disconnected';
}

/* ── Subscriptions ───────────────────────────────── */
function subscribeTopics() {
    // Antenna state
    subscribe('/antenna/state', 'antenna_tracker_msgs/msg/AntennaState', (msg) => {
        setText('curAz',   msg.current_azimuth.toFixed(2)   + '°');
        setText('curEl',   msg.current_elevation.toFixed(2) + '°');
        setText('tgtAz',   msg.target_azimuth.toFixed(2)    + '°');
        setText('tgtEl',   msg.target_elevation.toFixed(2)  + '°');

        var azE = msg.az_error, elE = msg.el_error;
        setTextClass('azErr', azE.toFixed(2) + '°', Math.abs(azE) > 2 ? 'metric-value warn' : 'metric-value');
        setTextClass('elErr', elE.toFixed(2) + '°', Math.abs(elE) > 2 ? 'metric-value warn' : 'metric-value');

        setText('azMotor',   msg.az_motor_cmd.toFixed(1));
        setText('elMotor',   msg.el_motor_cmd.toFixed(1));

        // Compass
        compass.az    = msg.current_azimuth;
        compass.el    = msg.current_elevation;
        compass.tgtAz = msg.target_azimuth;
        setText('compassAz',    msg.current_azimuth.toFixed(1)   + '°');
        setText('compassEl',    msg.current_elevation.toFixed(1) + '°');
        setText('compassTgtAz', msg.target_azimuth.toFixed(1)    + '°');
        drawCompass();

        // Mode label
        var modeNames = ['AUTO', 'MANUAL', 'STANDBY', 'EMERGENCY'];
        var modeColors = ['#22d46e', '#3e88ee', '#f0a030', '#e83a55'];
        var modeIdx = msg.mode !== undefined ? msg.mode : 2;
        var el2 = document.getElementById('activeModeDisplay');
        el2.textContent = modeNames[modeIdx] || 'UNKNOWN';
        el2.style.color = modeColors[modeIdx] || '#aaa';

        // Highlight active btn
        ['btnAuto','btnManual','btnStandby','btnEmergency'].forEach((id, i) => {
            document.getElementById(id).classList.toggle('mode-active', i === modeIdx);
        });

        // Show manual panel only in MANUAL mode (modeIdx===1)
        document.getElementById('manualPanel').classList.toggle('disabled', modeIdx !== 1);

        updateChart(msg.current_azimuth, msg.current_elevation,
                    msg.target_azimuth,  msg.target_elevation);
    });

    // Diagnostics
    subscribe('/antenna/diagnostics', 'antenna_tracker_msgs/msg/TrackerDiagnostics', (msg) => {
        setHealth('healthImu',     msg.imu_ok);
        setHealth('healthMag',     msg.mag_ok);
        setHealth('healthGps',     msg.gps_ok);
        setHealth('healthEncoder', msg.encoder_ok);
        setHealth('healthCan',     msg.can_ok);

        var cpuEl = document.getElementById('healthCpu');
        var cpuOk = msg.cpu_temp_c < 80;
        cpuEl.textContent  = 'CPU ' + msg.cpu_temp_c.toFixed(1) + '°C';
        cpuEl.className    = 'health-item ' + (cpuOk ? 'ok' : 'fail');

        setText('loopRate', msg.loop_rate_hz.toFixed(1) + ' Hz');
    });

    // Target GPS  
    subscribe('/antenna/target_gps', 'antenna_tracker_msgs/msg/TargetGPS', (msg) => {
        setText('tgtLat',  msg.latitude.toFixed(6));
        setText('tgtLon',  msg.longitude.toFixed(6));
        setText('tgtAlt',  msg.altitude_m.toFixed(1) + ' m');
        setText('tgtRssi', msg.rssi_dbm.toFixed(0)   + ' dBm');

        // Convert RSSI (-100 dBm worst, -40 dBm best) → 0-100%
        var rssi  = Math.max(-100, Math.min(-40, msg.rssi_dbm));
        var pct   = ((rssi + 100) / 60 * 100).toFixed(1);
        document.getElementById('sigBar').style.width = pct + '%';
    });
}

function subscribe(topic, type, cb) {
    var t = new ROSLIB.Topic({ ros: ros, name: topic, messageType: type });
    t.subscribe(cb);
}

/* ── Mode Service ────────────────────────────────── */
function setMode(mode) {
    if (!ros || !ros.isConnected) return;
    var client  = new ROSLIB.Service({ ros: ros, name: '/antenna/set_mode',
                                       serviceType: 'antenna_tracker_msgs/srv/SetMode' });
    var request = new ROSLIB.ServiceRequest({ mode: mode });
    client.callService(request, (result) => {
        if (!result.success) alert('Mode change failed: ' + result.message);
    });
}

/* ── Manual Target Service ───────────────────────── */
function setManualTarget() {
    if (!ros || !ros.isConnected) return;
    var az = parseFloat(document.getElementById('manualAzSlider').value);
    var el = parseFloat(document.getElementById('manualElSlider').value);
    var client = new ROSLIB.Service({ ros: ros, name: '/antenna/set_manual_target',
                                      serviceType: 'antenna_tracker_msgs/srv/SetManualTarget' });
    var request = new ROSLIB.ServiceRequest({ azimuth_deg: az, elevation_deg: el });
    client.callService(request, (result) => {
        if (!result.success) alert('Manual target failed: ' + result.message);
    });
}

/* ── Helpers ─────────────────────────────────────── */
function setText(id, val)                { document.getElementById(id).textContent = val; }
function setTextClass(id, val, cls)      { var e = document.getElementById(id); e.textContent = val; e.className = cls; }
function setHealth(id, ok)               { document.getElementById(id).className = 'health-item ' + (ok ? 'ok' : 'fail'); }

/* ── Compass Drawing ─────────────────────────────── */
function initCompass() {
    compass.canvas = document.getElementById('compassCanvas');
    compass.ctx    = compass.canvas.getContext('2d');
    drawCompass();
}

function drawCompass() {
    var c = compass.canvas, ctx = compass.ctx;
    var cx = c.width / 2, cy = c.height / 2, r = cx - 10;
    ctx.clearRect(0, 0, c.width, c.height);

    // Background circle
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, 2 * Math.PI);
    ctx.fillStyle   = '#0b0d18';
    ctx.fill();
    ctx.strokeStyle = '#1c2a45';
    ctx.lineWidth   = 2;
    ctx.stroke();

    // Cardinal labels
    var labels = ['N','E','S','W'];
    ctx.fillStyle   = '#4a80bb';
    ctx.font        = '11px Inter, sans-serif';
    ctx.textAlign   = 'center';
    ctx.textBaseline = 'middle';
    labels.forEach((lbl, i) => {
        var ang = (i * 90 - 90) * Math.PI / 180;
        var lr  = r - 14;
        ctx.fillText(lbl, cx + lr * Math.cos(ang), cy + lr * Math.sin(ang));
    });

    // Tick marks
    for (var deg = 0; deg < 360; deg += 10) {
        var ang = (deg - 90) * Math.PI / 180;
        var major = deg % 90 === 0;
        var r1 = major ? r - 22 : r - 14;
        ctx.beginPath();
        ctx.moveTo(cx + r1  * Math.cos(ang), cy + r1  * Math.sin(ang));
        ctx.lineTo(cx + (r-2) * Math.cos(ang), cy + (r-2) * Math.sin(ang));
        ctx.strokeStyle = major ? '#3558aa' : '#1c2a45';
        ctx.lineWidth   = major ? 1.5 : 0.8;
        ctx.stroke();
    }

    // Target azimuth needle (orange dashed)
    drawNeedle(ctx, cx, cy, compass.tgtAz, r * 0.62, '#f0a030', 2, [6, 4]);

    // Actual azimuth needle (cyan solid)
    drawNeedle(ctx, cx, cy, compass.az,    r * 0.78, '#55c8ee', 2.5, []);

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fillStyle = '#55c8ee';
    ctx.fill();

    // Elevation arc on inner ring
    var el   = Math.max(0, Math.min(90, compass.el));
    var elAng = ((el / 90) * (3 * Math.PI / 2)) - Math.PI;
    ctx.beginPath();
    ctx.arc(cx, cy, r * 0.25, -Math.PI, elAng);
    ctx.strokeStyle = '#22d46e';
    ctx.lineWidth   = 2.5;
    ctx.stroke();
}

function drawNeedle(ctx, cx, cy, azDeg, len, color, width, dash) {
    var ang = (azDeg - 90) * Math.PI / 180;
    ctx.save();
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + len * Math.cos(ang), cy + len * Math.sin(ang));
    ctx.strokeStyle = color;
    ctx.lineWidth   = width;
    ctx.lineCap     = 'round';
    ctx.stroke();
    ctx.restore();
}

/* ── Chart ───────────────────────────────────────── */
function initChart() {
    var ctx = document.getElementById('trackingChart').getContext('2d');
    var ds  = (label, color, dash) => ({
        label, data: [], borderColor: color,
        borderDash: dash || [], borderWidth: dash ? 1 : 1.8,
        pointRadius: 0, fill: false, tension: 0.2
    });
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                ds('Azimuth',    '#55c8ee'),
                ds('Elevation',  '#e06070'),
                ds('Target AZ',  '#55c8ee88', [6,4]),
                ds('Target EL',  '#e0607088', [6,4])
            ]
        },
        options: {
            responsive: true, maintainAspectRatio: false, animation: false,
            scales: {
                x: { display: false },
                y: { min: -5, max: 370,
                     ticks: { color: '#4a6888', font: { size: 10 } },
                     grid:  { color: '#151e30' } }
            },
            plugins: {
                legend: { labels: { color: '#8aaabb', boxWidth: 11, font: { size: 11 } } }
            }
        }
    });
}

function updateChart(az, el, tAz, tEl) {
    var now = new Date().toLocaleTimeString();
    chart.data.labels.push(now);
    [az, el, tAz, tEl].forEach((v, i) => chart.data.datasets[i].data.push(v));
    if (chart.data.labels.length > MAX_CHART_PTS) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(d => d.data.shift());
    }
    chart.update();
}

/* ── Init ────────────────────────────────────────── */
initCompass();
initChart();
connectROS();
