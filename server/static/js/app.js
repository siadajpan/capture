/**
 * Edge Sensor Hub - Mobile & Multi-Sensor Web Application Logic
 */

(function () {
  'use strict';

  // Application State
  const state = {
    viewMode: 'device', // 'device' | 'modality'
    selectedModalityFilter: 'all',
    displayEnabled: true,
    isRecording: false,
    recordStartTime: null,
    edges: [],
    systemInfo: null,
    pollTimer: null,
    recClockTimer: null
  };

  // DOM Elements
  const el = {
    serverIpLabel: document.getElementById('serverIpLabel'),
    phoneUrlDisplay: document.getElementById('phoneUrlDisplay'),
    copyUrlBtn: document.getElementById('copyUrlBtn'),
    edgeCountBadge: document.getElementById('edgeCountBadge'),
    onlineEdgesCount: document.getElementById('onlineEdgesCount'),
    totalSensorsCount: document.getElementById('totalSensorsCount'),

    // Control HUD
    recStatusBox: document.getElementById('recStatusBox'),
    recStateText: document.getElementById('recStateText'),
    recTimerText: document.getElementById('recTimerText'),
    toggleDisplayBtn: document.getElementById('toggleDisplayBtn'),
    displayBtnText: document.getElementById('displayBtnText'),
    toggleCollectBtn: document.getElementById('toggleCollectBtn'),
    collectBtnText: document.getElementById('collectBtnText'),
    collectSubText: document.getElementById('collectSubText'),

    // View Switcher
    viewByDeviceBtn: document.getElementById('viewByDeviceBtn'),
    viewByModalityBtn: document.getElementById('viewByModalityBtn'),
    modalityPillsBar: document.getElementById('modalityPillsBar'),
    feedsViewport: document.getElementById('feedsViewport'),

    // Modal
    modalBackdrop: document.getElementById('modalBackdrop'),
    openAddDeviceBtn: document.getElementById('openAddDeviceBtn'),
    closeModalBtn: document.getElementById('closeModalBtn'),
    networkInfoBadge: document.getElementById('networkInfoBadge'),
    addEdgeForm: document.getElementById('addEdgeForm'),
    edgeNameInput: document.getElementById('edgeNameInput'),
    edgeIdInput: document.getElementById('edgeIdInput'),
    edgeHostInput: document.getElementById('edgeHostInput'),
    edgePortInput: document.getElementById('edgePortInput'),
    edgeListContainer: document.getElementById('edgeListContainer')
  };

  // ==========================================================================
  // Initialization
  // ==========================================================================
  async function init() {
    bindEvents();
    await fetchSystemInfo();
    await pollData();
    // Poll loop
    state.pollTimer = setInterval(pollData, 1800);
  }

  function bindEvents() {
    // Master HUD Display toggle
    el.toggleDisplayBtn.addEventListener('click', toggleDisplay);

    // Master HUD Collect / Record toggle
    el.toggleCollectBtn.addEventListener('click', toggleCollection);

    // View Mode Switcher
    el.viewByDeviceBtn.addEventListener('click', () => switchViewMode('device'));
    el.viewByModalityBtn.addEventListener('click', () => switchViewMode('modality'));

    // Modality filter pills
    el.modalityPillsBar.addEventListener('click', (e) => {
      const btn = e.target.closest('.pill');
      if (!btn) return;
      document.querySelectorAll('.pill').forEach(p => p.classList.remove('active'));
      btn.classList.add('active');
      state.selectedModalityFilter = btn.dataset.mod;
      renderFeeds();
    });

    // Modal Events
    el.openAddDeviceBtn.addEventListener('click', openModal);
    el.networkInfoBadge.addEventListener('click', openModal);
    el.closeModalBtn.addEventListener('click', closeModal);
    el.modalBackdrop.addEventListener('click', (e) => {
      if (e.target === el.modalBackdrop) closeModal();
    });

    // Copy Phone URL
    el.copyUrlBtn.addEventListener('click', () => {
      if (navigator.clipboard && state.systemInfo) {
        navigator.clipboard.writeText(state.systemInfo.mobile_url);
        el.copyUrlBtn.textContent = 'Copied!';
        setTimeout(() => el.copyUrlBtn.textContent = 'Copy', 2000);
      }
    });

    // Add Edge Form
    el.addEdgeForm.addEventListener('submit', handleAddEdge);
  }

  // ==========================================================================
  // API & Data Fetching
  // ==========================================================================
  async function fetchSystemInfo() {
    try {
      const res = await fetch('/api/info');
      if (res.ok) {
        state.systemInfo = await res.json();
        el.serverIpLabel.textContent = `${state.systemInfo.server_ip}:${state.systemInfo.server_port}`;
        el.phoneUrlDisplay.textContent = state.systemInfo.mobile_url;
      }
    } catch (err) {
      console.warn('Could not fetch system info', err);
    }
  }

  async function pollData() {
    try {
      const res = await fetch('/api/edges');
      if (!res.ok) return;
      const data = await res.json();
      state.edges = data.edges || [];
      state.displayEnabled = data.display_enabled;

      updateHUDStatus();
      renderFeeds();
      renderEdgeListModal();
    } catch (err) {
      console.warn('Poll error:', err);
    }
  }

  // ==========================================================================
  // HUD Status & Controls
  // ==========================================================================
  function updateHUDStatus() {
    // Counts
    const onlineEdges = state.edges.filter(e => e.online);
    let totalSensors = 0;
    onlineEdges.forEach(e => {
      totalSensors += (e.sensors || []).length;
    });

    el.edgeCountBadge.textContent = state.edges.length;
    el.onlineEdgesCount.textContent = `${onlineEdges.length}/${state.edges.length}`;
    el.totalSensorsCount.textContent = totalSensors;

    // Check if any device is recording
    const isAnyRecording = onlineEdges.some(e => e.collection && e.collection.is_recording);
    state.isRecording = isAnyRecording;

    if (isAnyRecording) {
      el.recStatusBox.classList.add('recording');
      el.recStateText.textContent = 'INTERNAL RECORDING';
      el.toggleCollectBtn.classList.add('recording');
      el.collectBtnText.textContent = 'STOP RECORDING';
      el.collectSubText.textContent = 'Active on edge storage';

      // Pick max duration from edges
      let maxDuration = 0;
      onlineEdges.forEach(e => {
        if (e.collection && e.collection.is_recording) {
          maxDuration = Math.max(maxDuration, e.collection.duration_seconds || 0);
        }
      });
      el.recTimerText.textContent = formatDuration(maxDuration);
    } else {
      el.recStatusBox.classList.remove('recording');
      el.recStateText.textContent = 'IDLE - READY';
      el.recTimerText.textContent = '00:00:00';
      el.toggleCollectBtn.classList.remove('recording');
      el.collectBtnText.textContent = 'COLLECT FEEDS';
      el.collectSubText.textContent = 'Record to internal edge storage';
    }

    // Display Button State
    if (state.displayEnabled) {
      el.toggleDisplayBtn.classList.add('active');
      el.displayBtnText.textContent = 'DISPLAY FEEDS: ON';
    } else {
      el.toggleDisplayBtn.classList.remove('active');
      el.displayBtnText.textContent = 'DISPLAY FEEDS: OFF';
    }
  }

  async function toggleDisplay() {
    const newState = !state.displayEnabled;
    try {
      const res = await fetch('/api/display/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: newState })
      });
      if (res.ok) {
        const d = await res.json();
        state.displayEnabled = d.display_enabled;
        updateHUDStatus();
        renderFeeds();
      }
    } catch (err) {
      console.error('Display toggle error:', err);
    }
  }

  async function toggleCollection() {
    const endpoint = state.isRecording ? '/api/collection/stop' : '/api/collection/start';
    try {
      const res = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ label: 'Mobile App Session' })
      });
      if (res.ok) {
        await pollData();
      }
    } catch (err) {
      console.error('Collection toggle error:', err);
    }
  }

  function switchViewMode(mode) {
    state.viewMode = mode;
    if (mode === 'device') {
      el.viewByDeviceBtn.classList.add('active');
      el.viewByModalityBtn.classList.remove('active');
      el.modalityPillsBar.style.display = 'none';
    } else {
      el.viewByDeviceBtn.classList.remove('active');
      el.viewByModalityBtn.classList.add('active');
      el.modalityPillsBar.style.display = 'flex';
    }
    renderFeeds();
  }

  // ==========================================================================
  // Render Feed Viewport
  // ==========================================================================
  function renderFeeds() {
    if (state.edges.length === 0) {
      el.feedsViewport.innerHTML = `
        <div class="empty-state">
          <p>No edge devices configured.</p>
          <button class="submit-btn" onclick="document.getElementById('openAddDeviceBtn').click()">+ Add Edge Device</button>
        </div>
      `;
      return;
    }

    if (state.viewMode === 'device') {
      renderByDevice();
    } else {
      renderByModality();
    }
  }

  /**
   * View 1: Display each edge device with all of its sensors
   */
  function renderByDevice() {
    let html = '';

    state.edges.forEach(edge => {
      const isOnline = edge.online;
      const sensors = edge.sensors || [];
      const col = edge.collection || {};
      const isRec = col.is_recording;
      const freeGb = col.disk_free_gb !== undefined ? `${col.disk_free_gb} GB Free` : 'SD Card Ready';

      html += `
        <div class="device-card" id="card-${edge.device_id}">
          <div class="device-card-header">
            <div class="device-card-title">
              <div class="status-indicator ${isOnline ? '' : 'offline'}"></div>
              <span class="device-name">${escapeHtml(edge.device_name || edge.device_id)}</span>
              <span class="device-id-tag">${escapeHtml(edge.device_id)}</span>
            </div>
            <div class="device-telemetry">
              ${isRec ? `<span class="rec-pulse-chip">⏺ REC (${col.duration_seconds || 0}s)</span>` : ''}
              <span class="storage-badge">💾 ${freeGb}</span>
            </div>
          </div>
          
          <div class="sensors-grid">
            ${sensors.length > 0 ? sensors.map(sensor => renderSensorBox(edge, sensor)).join('') : `
              <div class="empty-state" style="padding: 20px;">
                <p>${isOnline ? 'No sensors reported' : 'Edge device offline (reconnecting...)'}</p>
              </div>
            `}
          </div>
        </div>
      `;
    });

    el.feedsViewport.innerHTML = html;
  }

  /**
   * View 2: Grouped by Modality (All RGB, All Thermal, All Depth, etc.)
   */
  function renderByModality() {
    // Gather all sensors across all online edges, grouped by modality
    const modalitiesMap = {};

    state.edges.forEach(edge => {
      if (!edge.online) return;
      (edge.sensors || []).forEach(sensor => {
        const mod = (sensor.modality || 'other').toLowerCase();
        if (state.selectedModalityFilter !== 'all' && mod !== state.selectedModalityFilter) {
          return;
        }
        if (!modalitiesMap[mod]) {
          modalitiesMap[mod] = [];
        }
        modalitiesMap[mod].push({ edge, sensor });
      });
    });

    const modalities = Object.keys(modalitiesMap);
    if (modalities.length === 0) {
      el.feedsViewport.innerHTML = `
        <div class="empty-state">
          <p>No sensor feeds found for filter: <strong>${escapeHtml(state.selectedModalityFilter.toUpperCase())}</strong></p>
        </div>
      `;
      return;
    }

    let html = '';
    modalities.forEach(mod => {
      const items = modalitiesMap[mod];
      html += `
        <div class="modality-group-section">
          <div class="modality-section-header">
            <span class="feed-modality-badge modality-${mod}">${mod}</span>
            <h3 class="modality-section-title">All ${mod.toUpperCase()} Sensors (${items.length})</h3>
          </div>
          <div class="sensors-grid">
            ${items.map(item => renderSensorBox(item.edge, item.sensor, true)).join('')}
          </div>
        </div>
      `;
    });

    el.feedsViewport.innerHTML = html;
  }

  function renderSensorBox(edge, sensor, showDeviceTag = false) {
    const streamUrl = `/proxy/stream/${edge.device_id}/${sensor.id || sensor.modality}`;
    const mod = (sensor.modality || 'rgb').toLowerCase();
    const isRec = edge.collection && edge.collection.is_recording;
    const frameCount = (edge.collection && edge.collection.frame_counts && edge.collection.frame_counts[sensor.id]) || 0;

    return `
      <div class="feed-box" id="sensor-${edge.device_id}-${sensor.id}">
        <div class="feed-box-header">
          <div style="display:flex; align-items:center; gap:8px;">
            <span class="feed-modality-badge modality-${mod}">${mod}</span>
            <span class="feed-name">${escapeHtml(sensor.name || sensor.id)}</span>
          </div>
          ${showDeviceTag ? `<span class="device-id-tag">${escapeHtml(edge.device_name || edge.device_id)}</span>` : ''}
        </div>

        <div class="feed-screen">
          ${state.displayEnabled && edge.online ? `
            <img src="${streamUrl}" alt="${sensor.name}" loading="lazy">
          ` : `
            <div class="display-off-overlay">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"></path>
                <line x1="1" y1="1" x2="23" y2="23"></line>
              </svg>
              <span>${!edge.online ? 'Edge Offline' : 'Display Paused (Saving Bandwidth)'}</span>
            </div>
          `}
        </div>

        <div class="feed-footer">
          <span>${sensor.resolution || '640x480'} @ ${sensor.fps || 15}fps</span>
          <span>${isRec ? `<span class="rec-pulse-chip">REC #${frameCount}</span>` : 'READY'}</span>
        </div>
      </div>
    `;
  }

  // ==========================================================================
  // Edge Device Management Modal
  // ==========================================================================
  function openModal() {
    el.modalBackdrop.classList.add('open');
  }

  function closeModal() {
    el.modalBackdrop.classList.remove('open');
  }

  function renderEdgeListModal() {
    if (!el.edgeListContainer) return;
    if (state.edges.length === 0) {
      el.edgeListContainer.innerHTML = '<p style="color:var(--text-muted);font-size:0.8rem;">No devices registered yet.</p>';
      return;
    }

    el.edgeListContainer.innerHTML = state.edges.map(e => `
      <div class="edge-item-row">
        <div class="edge-item-info">
          <span class="edge-item-name">${escapeHtml(e.device_name || e.device_id)}</span>
          <span class="edge-item-url">${escapeHtml(e.host)}:${e.port} &bull; ${e.online ? '<span style="color:var(--accent-green)">ONLINE</span>' : '<span style="color:var(--text-dim)">OFFLINE</span>'}</span>
        </div>
        <button class="delete-edge-btn" onclick="window.removeEdgeDevice('${e.device_id}')">Remove</button>
      </div>
    `).join('');
  }

  async function handleAddEdge(e) {
    e.preventDefault();
    const payload = {
      device_id: el.edgeIdInput.value.trim(),
      device_name: el.edgeNameInput.value.trim(),
      host: el.edgeHostInput.value.trim(),
      port: parseInt(el.edgePortInput.value.trim(), 10) || 5001
    };

    try {
      const res = await fetch('/api/edges', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (res.ok) {
        el.addEdgeForm.reset();
        await pollData();
      }
    } catch (err) {
      alert('Failed to add edge device: ' + err);
    }
  }

  window.removeEdgeDevice = async function(deviceId) {
    if (!confirm(`Remove edge device "${deviceId}"?`)) return;
    try {
      await fetch(`/api/edges/${deviceId}`, { method: 'DELETE' });
      await pollData();
    } catch (err) {
      alert('Failed to remove edge: ' + err);
    }
  };

  // ==========================================================================
  // Utilities
  // ==========================================================================
  function formatDuration(sec) {
    const s = Math.floor(sec);
    const hrs = Math.floor(s / 3600).toString().padStart(2, '0');
    const mins = Math.floor((s % 3600) / 60).toString().padStart(2, '0');
    const secs = (s % 60).toString().padStart(2, '0');
    return `${hrs}:${mins}:${secs}`;
  }

  function escapeHtml(str) {
    if (!str) return '';
    return String(str)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  // Start app
  document.addEventListener('DOMContentLoaded', init);
})();
