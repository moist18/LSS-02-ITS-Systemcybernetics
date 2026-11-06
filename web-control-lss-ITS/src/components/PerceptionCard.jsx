import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useTelemetry } from '../context/TelemetryContext';

export default function PerceptionCard({ className = '' }) {
  const { telemetry } = useTelemetry();

  // Control and display state
  const [missionMode, setMissionMode] = useState('gate'); // 'gate', 'kamikaze', 'docking'
  const [confThreshold, setConfThreshold] = useState(0.50);
  const [showBoxes, setShowBoxes] = useState(true);
  const [showGuidance, setShowGuidance] = useState(true);
  const [showReticle, setShowReticle] = useState(true);
  const [showHud, setShowHud] = useState(true);
  const [isPaused, setIsPaused] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [streamSource, setStreamSource] = useState('canvas'); // 'canvas' or 'live'
  const [liveStreamUrl, setLiveStreamUrl] = useState('');
  const [liveStreamError, setLiveStreamError] = useState(false);
  const [fps, setFps] = useState(29.8);

  const canvasRef = useRef(null);
  const animationFrameRef = useRef(null);
  const cardContainerRef = useRef(null);

  // Fallback backend URL detection for live ROS stream
  useEffect(() => {
    const host = window.location.hostname || 'localhost';
    setLiveStreamUrl(`http://${host}:8000/api/telemetry/vision/stream`);
  }, []);

  // Telemetry references
  const roll = telemetry?.navigation?.roll ?? 0.0;
  const pitch = telemetry?.navigation?.pitch ?? 0.0;
  const heading = telemetry?.navigation?.heading_deg ?? 214.0;
  const targetDist = telemetry?.detections?.target_distance_m ?? 12.8;
  const activeTargetName = telemetry?.detections?.active_target ?? 'GATE_ALPHA';

  // Synthetic CV simulation variables
  const simState = useRef({
    time: 0,
    lastFrameTime: performance.now(),
    frameCount: 0,
  });

  // Snapshot capture handler
  const handleCaptureSnapshot = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    try {
      const dataUrl = canvas.toDataURL('image/png');
      const link = document.createElement('a');
      link.download = `LSS02_Perception_Capture_${Date.now()}.png`;
      link.href = dataUrl;
      link.click();
    } catch (err) {
      console.error('Failed to capture canvas snapshot:', err);
    }
  }, []);

  // Fullscreen toggle
  const toggleFullscreen = useCallback(() => {
    setIsFullscreen((prev) => !prev);
  }, []);

  // Main Canvas Rendering Engine (Adaptive 640x360)
  useEffect(() => {
    if (streamSource === 'live' && !liveStreamError) {
      return;
    }

    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let isSubscribed = true;

    const render = (now) => {
      if (!isSubscribed) return;

      const delta = (now - simState.current.lastFrameTime) / 1000;
      simState.current.lastFrameTime = now;
      simState.current.frameCount++;

      if (simState.current.frameCount % 20 === 0 && delta > 0) {
        setFps(Number((1 / delta).toFixed(1)));
      }

      if (!isPaused) {
        simState.current.time += 0.025;
      }

      const t = simState.current.time;
      const width = canvas.width;
      const height = canvas.height;

      // Clear Canvas
      ctx.clearRect(0, 0, width, height);

      // 1. DYNAMIC WATER & SKY BACKGROUND
      const horizonY = height * 0.40 + Math.sin(t * 0.8) * 3 + pitch * 2.2;
      const rollAngle = (roll * Math.PI) / 180 + Math.sin(t * 0.6) * 0.012;

      ctx.save();
      // Apply subtle vessel pitch/roll transform
      ctx.translate(width / 2, height / 2);
      ctx.rotate(rollAngle);
      ctx.translate(-width / 2, -height / 2);

      // Sky gradient
      const skyGrad = ctx.createLinearGradient(0, 0, 0, horizonY);
      skyGrad.addColorStop(0, '#0f172a');
      skyGrad.addColorStop(0.7, '#1e293b');
      skyGrad.addColorStop(1, '#334155');
      ctx.fillStyle = skyGrad;
      ctx.fillRect(-width * 0.2, -height * 0.2, width * 1.4, horizonY + height * 0.2);

      // Distant mountains silhouette
      ctx.fillStyle = '#090d16';
      ctx.beginPath();
      ctx.moveTo(-50, horizonY);
      for (let x = -50; x <= width + 50; x += 30) {
        const hillH = Math.sin(x * 0.012 + 1.2) * 12 + Math.cos(x * 0.02) * 6;
        ctx.lineTo(x, horizonY - 8 - Math.max(0, hillH));
      }
      ctx.lineTo(width + 50, horizonY);
      ctx.closePath();
      ctx.fill();

      // Sea water gradient
      const seaGrad = ctx.createLinearGradient(0, horizonY, 0, height);
      seaGrad.addColorStop(0, '#0c2236');
      seaGrad.addColorStop(0.35, '#083344');
      seaGrad.addColorStop(0.7, '#0e4359');
      seaGrad.addColorStop(1, '#072533');
      ctx.fillStyle = seaGrad;
      ctx.fillRect(-width * 0.2, horizonY, width * 1.4, height * 1.4);

      // Ocean water ripples
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.16)';
      ctx.lineWidth = 1.0;
      for (let r = 0; r < 5; r++) {
        const yPos = horizonY + 18 + r * 26 + Math.sin(t * 1.2 + r) * 2.5;
        ctx.beginPath();
        ctx.moveTo(0, yPos);
        ctx.bezierCurveTo(
          width * 0.25,
          yPos - 3 + Math.sin(t * 2 + r) * 2.5,
          width * 0.75,
          yPos + 3 - Math.cos(t * 2 + r) * 2.5,
          width,
          yPos
        );
        ctx.stroke();
      }

      // 2. SIMULATED BUOY / TARGET POSITIONS
      const bobY = Math.sin(t * 1.8) * 3;
      const swayX = Math.cos(t * 0.9) * 10;

      let bboxesToDraw = [];
      let leftBuoyMid = null;
      let rightBuoyMid = null;
      let targetCentroid = null;

      if (missionMode === 'gate') {
        const orangeX = 195 + swayX;
        const orangeY = height * 0.46 + bobY;
        const orangeW = 64;
        const orangeH = 84;

        const greenX = 410 + swayX * 0.9;
        const greenY = height * 0.45 - bobY;
        const greenW = 68;
        const greenH = 88;

        const yellowX = 95 + swayX * 1.1;
        const yellowY = height * 0.41 + bobY * 0.6;
        const yellowW = 40;
        const yellowH = 50;

        drawBuoySprite(ctx, orangeX, orangeY, orangeW, orangeH, '#ea580c', '#f97316', '#ffedd5');
        drawBuoySprite(ctx, greenX, greenY, greenW, greenH, '#16a34a', '#22c55e', '#dcfce7');
        drawBuoySprite(ctx, yellowX, yellowY, yellowW, yellowH, '#ca8a04', '#eab308', '#fef9c3');

        leftBuoyMid = { x: orangeX + orangeW / 2, y: orangeY + orangeH / 2 };
        rightBuoyMid = { x: greenX + greenW / 2, y: greenY + greenH / 2 };

        bboxesToDraw = [
          { cls: 'orange_buoy', conf: 0.94, x: orangeX, y: orangeY, w: orangeW, h: orangeH, color: '#f97316', mid: leftBuoyMid },
          { cls: 'green_buoy', conf: 0.91, x: greenX, y: greenY, w: greenW, h: greenH, color: '#22c55e', mid: rightBuoyMid },
          { cls: 'yellow_buoy', conf: 0.82, x: yellowX, y: yellowY, w: yellowW, h: yellowH, color: '#eab308', mid: { x: yellowX + yellowW / 2, y: yellowY + yellowH / 2 } },
        ];
      } else if (missionMode === 'kamikaze') {
        const targetX = 310 + swayX * 1.3;
        const targetY = height * 0.43 + bobY;
        const targetW = 76;
        const targetH = 96;

        drawBuoySprite(ctx, targetX, targetY, targetW, targetH, '#dc2626', '#ef4444', '#fee2e2');
        targetCentroid = { x: targetX + targetW / 2, y: targetY + targetH / 2 };

        bboxesToDraw = [
          { cls: 'red_buoy', conf: 0.96, x: targetX, y: targetY, w: targetW, h: targetH, color: '#ef4444', mid: targetCentroid },
        ];
      } else {
        const rDockX = 180 + swayX * 0.7;
        const rDockY = height * 0.48 + bobY * 0.5;
        const rDockW = 80;
        const rDockH = 56;

        const gDockX = 380 + swayX * 0.7;
        const gDockY = height * 0.48 + bobY * 0.5;
        const gDockW = 80;
        const gDockH = 56;

        drawDockSprite(ctx, rDockX, rDockY, rDockW, rDockH, '#ef4444');
        drawDockSprite(ctx, gDockX, gDockY, gDockW, gDockH, '#22c55e');

        leftBuoyMid = { x: rDockX + rDockW / 2, y: rDockY + rDockH / 2 };
        rightBuoyMid = { x: gDockX + gDockW / 2, y: gDockY + gDockH / 2 };

        bboxesToDraw = [
          { cls: 'red_dock', conf: 0.93, x: rDockX, y: rDockY, w: rDockW, h: rDockH, color: '#ef4444', mid: leftBuoyMid },
          { cls: 'green_dock', conf: 0.89, x: gDockX, y: gDockY, w: gDockW, h: gDockH, color: '#22c55e', mid: rightBuoyMid },
        ];
      }

      ctx.restore();

      // 3. COMPUTER VISION GUIDANCE OVERLAYS
      const centerScreenX = width / 2;
      const centerScreenY = height / 2;
      const deadzone = 48;

      // Draw Deadzone Vertical Guides
      if (showGuidance) {
        ctx.save();
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1;

        ctx.beginPath();
        ctx.moveTo(centerScreenX - deadzone, 0);
        ctx.lineTo(centerScreenX - deadzone, height);
        ctx.moveTo(centerScreenX + deadzone, 0);
        ctx.lineTo(centerScreenX + deadzone, height);
        ctx.stroke();
        ctx.restore();
      }

      // Gate connector & midpoint guidance
      let connectorMidpoint = null;
      let steeringCommand = 'ON COURSE';
      let steeringColor = '#22c55e';

      if (missionMode === 'gate' || missionMode === 'docking') {
        if (leftBuoyMid && rightBuoyMid && showGuidance) {
          connectorMidpoint = {
            x: (leftBuoyMid.x + rightBuoyMid.x) / 2,
            y: (leftBuoyMid.y + rightBuoyMid.y) / 2,
          };

          // White gate connection line
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([5, 3]);
          ctx.beginPath();
          ctx.moveTo(leftBuoyMid.x, leftBuoyMid.y);
          ctx.lineTo(rightBuoyMid.x, rightBuoyMid.y);
          ctx.stroke();
          ctx.setLineDash([]);

          // Steering trajectory line
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(centerScreenX, height - 6);
          ctx.lineTo(connectorMidpoint.x, connectorMidpoint.y);
          ctx.stroke();

          // Pulsing Midpoint Dot
          const pulse = 5 + Math.sin(t * 6) * 1.5;
          ctx.fillStyle = '#ef4444';
          ctx.beginPath();
          ctx.arc(connectorMidpoint.x, connectorMidpoint.y, pulse, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 1.5;
          ctx.stroke();

          // Steering Alert
          if (connectorMidpoint.x < centerScreenX - deadzone) {
            steeringCommand = 'TURN LEFT';
            steeringColor = '#f97316';
          } else if (connectorMidpoint.x > centerScreenX + deadzone) {
            steeringCommand = 'TURN RIGHT';
            steeringColor = '#f97316';
          } else {
            steeringCommand = 'ON COURSE';
            steeringColor = '#22c55e';
          }
        }
      } else if (missionMode === 'kamikaze') {
        if (targetCentroid && showGuidance) {
          connectorMidpoint = targetCentroid;

          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 2;
          ctx.setLineDash([4, 2]);
          ctx.beginPath();
          ctx.moveTo(centerScreenX, height - 6);
          ctx.lineTo(targetCentroid.x, targetCentroid.y);
          ctx.stroke();
          ctx.setLineDash([]);

          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.arc(targetCentroid.x, targetCentroid.y, 14 + Math.sin(t * 8) * 2, 0, Math.PI * 2);
          ctx.stroke();

          if (targetCentroid.x < centerScreenX - deadzone) {
            steeringCommand = 'TURN LEFT';
            steeringColor = '#f97316';
          } else if (targetCentroid.x > centerScreenX + deadzone) {
            steeringCommand = 'TURN RIGHT';
            steeringColor = '#f97316';
          } else {
            steeringCommand = 'LOCKED ON';
            steeringColor = '#ef4444';
          }
        }
      }

      // Draw YOLO Bounding Boxes
      if (showBoxes) {
        bboxesToDraw.forEach((item) => {
          if (item.conf < confThreshold) return;

          ctx.save();
          ctx.strokeStyle = item.color;
          ctx.lineWidth = 1.5;
          ctx.strokeRect(item.x, item.y, item.w, item.h);

          // Corner brackets
          const cornerLen = 8;
          ctx.lineWidth = 2.5;
          // TL
          ctx.beginPath();
          ctx.moveTo(item.x, item.y + cornerLen);
          ctx.lineTo(item.x, item.y);
          ctx.lineTo(item.x + cornerLen, item.y);
          ctx.stroke();
          // TR
          ctx.beginPath();
          ctx.moveTo(item.x + item.w - cornerLen, item.y);
          ctx.lineTo(item.x + item.w);
          ctx.lineTo(item.x + item.w, item.y + cornerLen);
          ctx.stroke();

          // Centroid
          if (item.mid) {
            ctx.fillStyle = item.color;
            ctx.beginPath();
            ctx.arc(item.mid.x, item.mid.y, 3, 0, Math.PI * 2);
            ctx.fill();
          }

          // Label
          const labelText = `${item.cls} ${(item.conf * 100).toFixed(0)}%`;
          ctx.font = 'bold 9px "JetBrains Mono", monospace';
          const labelWidth = ctx.measureText(labelText).width + 8;
          ctx.fillStyle = item.color;
          ctx.fillRect(item.x, item.y - 14, labelWidth, 14);
          ctx.fillStyle = '#0f172a';
          ctx.fillText(labelText, item.x + 4, item.y - 3);

          ctx.restore();
        });
      }

      // Reticle
      if (showReticle) {
        ctx.save();
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(centerScreenX - 18, centerScreenY);
        ctx.lineTo(centerScreenX + 18, centerScreenY);
        ctx.moveTo(centerScreenX, centerScreenY - 18);
        ctx.lineTo(centerScreenX, centerScreenY + 18);
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(centerScreenX, centerScreenY, 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }

      // Steering Command Badge (Top Right)
      if (showGuidance) {
        ctx.save();
        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        const cmdText = `▶ ${steeringCommand}`;
        const cmdW = ctx.measureText(cmdText).width + 16;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
        ctx.strokeStyle = steeringColor;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.roundRect(width - cmdW - 10, 10, cmdW, 24, 6);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = steeringColor;
        ctx.fillText(cmdText, width - cmdW + 1, 26);
        ctx.restore();
      }

      // HUD Metadata (Top Left)
      if (showHud) {
        ctx.save();
        ctx.font = '8.5px "JetBrains Mono", monospace';
        ctx.fillStyle = 'rgba(255, 255, 255, 0.85)';
        ctx.fillText('CAM: /robot/vision/navigation', 10, 18);
        ctx.fillStyle = '#38bdf8';
        ctx.fillText('MODEL: teknov8s-new.engine', 10, 29);
        ctx.restore();
      }

      animationFrameRef.current = requestAnimationFrame(render);
    };

    animationFrameRef.current = requestAnimationFrame(render);

    return () => {
      isSubscribed = false;
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [
    missionMode,
    confThreshold,
    showBoxes,
    showGuidance,
    showReticle,
    showHud,
    isPaused,
    streamSource,
    liveStreamError,
    roll,
    pitch,
  ]);

  function drawBuoySprite(ctx, x, y, w, h, darkColor, lightColor, capColor) {
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.25)';
    ctx.beginPath();
    ctx.ellipse(x + w / 2, y + h - 3, w * 0.45, 5, 0, 0, Math.PI * 2);
    ctx.fill();

    const bodyGrad = ctx.createLinearGradient(x, y, x + w, y);
    bodyGrad.addColorStop(0, darkColor);
    bodyGrad.addColorStop(0.5, lightColor);
    bodyGrad.addColorStop(1, darkColor);
    ctx.fillStyle = bodyGrad;
    ctx.beginPath();
    ctx.roundRect(x + w * 0.15, y + h * 0.2, w * 0.7, h * 0.75, [10, 10, 6, 6]);
    ctx.fill();

    ctx.fillStyle = capColor;
    ctx.beginPath();
    ctx.roundRect(x + w * 0.35, y + 3, w * 0.3, h * 0.22, 3);
    ctx.fill();
    ctx.restore();
  }

  function drawDockSprite(ctx, x, y, w, h, markerColor) {
    ctx.save();
    ctx.fillStyle = '#334155';
    ctx.fillRect(x, y + h * 0.5, w, h * 0.5);
    ctx.fillStyle = markerColor;
    ctx.fillRect(x + w * 0.3, y, w * 0.4, h * 0.5);
    ctx.restore();
  }

  return (
    <div
      ref={cardContainerRef}
      className={`bg-white dark:bg-[#151921] rounded-3xl p-4 sm:p-5 shadow-cockpit border border-slate-200/50 dark:border-slate-800 flex flex-col justify-between gap-3 transition-all duration-300 ${
        isFullscreen ? 'fixed inset-4 z-50 overflow-y-auto max-w-7xl mx-auto backdrop-blur-xl' : 'flex-1'
      } ${className}`}
    >
      {/* 1. CARD HEADER */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2.5">
          <div className="w-8 h-8 rounded-xl bg-sky-50 dark:bg-sky-950/60 border border-sky-100 dark:border-sky-800/60 flex items-center justify-center text-sky-600 dark:text-sky-400">
            <span className="material-symbols-outlined text-base">videocam</span>
          </div>
          <div>
            <h3 className="text-xs font-bold text-slate-900 dark:text-white uppercase tracking-wider flex items-center gap-2">
              <span>Perception Computer Vision</span>
              <span className="inline-flex items-center gap-1 text-[8.5px] font-mono px-2 py-0.5 rounded-full bg-emerald-50 dark:bg-emerald-950/60 text-emerald-600 dark:text-emerald-400 border border-emerald-200 dark:border-emerald-800/50">
                <span className="w-1.5 h-1.5 rounded-full bg-emerald-500 animate-pulse"></span>
                LIVE {fps} FPS
              </span>
            </h3>
            <p className="text-[9.5px] text-slate-400 dark:text-slate-500">
              YOLOv8s-TRT • Dual Buoy Gate Tracking • /robot/vision/navigation
            </p>
          </div>
        </div>

        {/* Header Right Action Badges */}
        <div className="flex items-center gap-1.5">
          <button
            type="button"
            onClick={() => setStreamSource((prev) => (prev === 'canvas' ? 'live' : 'canvas'))}
            className={`text-[9.5px] font-mono px-2.5 py-1 rounded-full border transition-all flex items-center gap-1 ${
              streamSource === 'live'
                ? 'bg-sky-500 text-white border-sky-400'
                : 'bg-slate-50 dark:bg-slate-800 text-slate-600 dark:text-slate-300 border-slate-200 dark:border-slate-700 hover:border-sky-400'
            }`}
            title="Toggle Live ROS Stream / AI Simulation"
          >
            <span className="material-symbols-outlined text-[12px]">
              {streamSource === 'live' ? 'sensors' : 'smart_toy'}
            </span>
            <span>{streamSource === 'live' ? 'ROS STREAM' : 'AI SIM'}</span>
          </button>

          <button
            type="button"
            onClick={toggleFullscreen}
            className="w-7 h-7 rounded-lg bg-slate-100 dark:bg-slate-800 hover:bg-slate-200 dark:hover:bg-slate-700 text-slate-600 dark:text-slate-300 flex items-center justify-center transition-colors"
            title={isFullscreen ? 'Exit Fullscreen' : 'Expand Fullscreen'}
          >
            <span className="material-symbols-outlined text-sm">
              {isFullscreen ? 'fullscreen_exit' : 'fullscreen'}
            </span>
          </button>
        </div>
      </div>

      {/* 2. CAMERA VIEWPORT & HUD (Flexibly adapts to fill remaining height) */}
      <div className="relative w-full flex-1 min-h-[190px] sm:min-h-[220px] rounded-2xl overflow-hidden bg-slate-950 border border-slate-800/60 shadow-inner group">
        {streamSource === 'live' && !liveStreamError ? (
          <img
            src={liveStreamUrl}
            alt="ROS Live Vision Stream"
            className="w-full h-full object-cover"
            onError={() => setLiveStreamError(true)}
          />
        ) : (
          <canvas
            ref={canvasRef}
            width={640}
            height={360}
            className="w-full h-full object-cover select-none"
          />
        )}

        {/* Live Stream Fallback Notification */}
        {streamSource === 'live' && liveStreamError && (
          <div className="absolute top-1.5 left-1.5 right-1.5 bg-amber-500/90 backdrop-blur-md text-slate-950 text-[9px] font-mono font-bold px-2 py-1 rounded-md flex items-center justify-between z-20">
            <span>ROS stream disconnected. Showing AI simulator.</span>
            <button type="button" onClick={() => setLiveStreamError(false)} className="underline hover:text-white">
              Retry
            </button>
          </div>
        )}

        {/* Top Floating Quick Controls Bar */}
        <div className="absolute top-2 left-2 flex items-center gap-1 z-10 opacity-80 group-hover:opacity-100 transition-opacity">
          <button
            type="button"
            onClick={() => setIsPaused((prev) => !prev)}
            className="h-5 px-2 rounded-md bg-slate-900/80 backdrop-blur-md border border-white/10 text-white text-[9.5px] font-mono flex items-center gap-1 hover:bg-slate-800"
          >
            <span className="material-symbols-outlined text-xs">{isPaused ? 'play_arrow' : 'pause'}</span>
            <span>{isPaused ? 'RESUME' : 'FREEZE'}</span>
          </button>
          <button
            type="button"
            onClick={handleCaptureSnapshot}
            className="h-5 px-2 rounded-md bg-slate-900/80 backdrop-blur-md border border-white/10 text-white text-[9.5px] font-mono flex items-center gap-1 hover:bg-slate-800 hover:text-sky-300"
          >
            <span className="material-symbols-outlined text-xs">photo_camera</span>
            <span>SNAPSHOT</span>
          </button>
        </div>

        {/* Bottom Floating Telemetry Strip */}
        <div className="absolute bottom-2 left-2 right-2 flex items-center justify-between text-[9px] font-mono text-white/80 bg-slate-950/80 backdrop-blur-md px-3 py-1 rounded-lg border border-white/10 pointer-events-none">
          <div className="flex items-center gap-3">
            <span>HDG: <strong className="text-sky-300">{heading.toFixed(0)}°</strong></span>
            <span>ROLL: <strong className="text-slate-200">{roll.toFixed(1)}°</strong></span>
            <span>PITCH: <strong className="text-slate-200">{pitch.toFixed(1)}°</strong></span>
          </div>
          <div className="flex items-center gap-3">
            <span>TARGET: <strong className="text-emerald-400">{activeTargetName}</strong></span>
            <span>DIST: <strong className="text-amber-300">{targetDist}m</strong></span>
          </div>
        </div>
      </div>

      {/* 3. TOOLBAR CONTROLS & MISSION MODES */}
      <div className="flex items-center justify-between gap-2 text-[10px]">
        {/* Mission Type Selection */}
        <div className="flex items-center gap-1 bg-slate-100 dark:bg-slate-800/80 p-0.5 rounded-lg">
          <button
            type="button"
            onClick={() => setMissionMode('gate')}
            className={`px-2.5 py-1 text-[9.5px] font-bold rounded-md transition-all ${
              missionMode === 'gate'
                ? 'bg-white dark:bg-slate-900 text-sky-600 dark:text-sky-400 shadow-xs'
                : 'text-slate-500 hover:text-slate-800 dark:text-slate-400'
            }`}
          >
            Gate Pass
          </button>
          <button
            type="button"
            onClick={() => setMissionMode('kamikaze')}
            className={`px-2.5 py-1 text-[9.5px] font-bold rounded-md transition-all ${
              missionMode === 'kamikaze'
                ? 'bg-white dark:bg-slate-900 text-rose-600 dark:text-rose-400 shadow-xs'
                : 'text-slate-500 hover:text-slate-800 dark:text-slate-400'
            }`}
          >
            Kamikaze
          </button>
          <button
            type="button"
            onClick={() => setMissionMode('docking')}
            className={`px-2.5 py-1 text-[9.5px] font-bold rounded-md transition-all ${
              missionMode === 'docking'
                ? 'bg-white dark:bg-slate-900 text-emerald-600 dark:text-emerald-400 shadow-xs'
                : 'text-slate-500 hover:text-slate-800 dark:text-slate-400'
            }`}
          >
            Docking
          </button>
        </div>

        {/* HUD & Overlay Toggles */}
        <div className="flex items-center gap-1.5">
          <button
            type="button"
            onClick={() => setShowBoxes((p) => !p)}
            className={`py-1 px-2.5 rounded-lg font-mono text-[9.5px] border transition-all ${
              showBoxes
                ? 'bg-sky-50 dark:bg-sky-950/50 border-sky-300 dark:border-sky-800 text-sky-700 dark:text-sky-300 font-semibold'
                : 'bg-slate-50 dark:bg-slate-800/40 border-slate-200 dark:border-slate-800 text-slate-400 line-through'
            }`}
          >
            BBoxes
          </button>
          <button
            type="button"
            onClick={() => setShowGuidance((p) => !p)}
            className={`py-1 px-2.5 rounded-lg font-mono text-[9.5px] border transition-all ${
              showGuidance
                ? 'bg-emerald-50 dark:bg-emerald-950/50 border-emerald-300 dark:border-emerald-800 text-emerald-700 dark:text-emerald-300 font-semibold'
                : 'bg-slate-50 dark:bg-slate-800/40 border-slate-200 dark:border-slate-800 text-slate-400 line-through'
            }`}
          >
            Guidance
          </button>
          <button
            type="button"
            onClick={() => setShowReticle((p) => !p)}
            className={`py-1 px-2.5 rounded-lg font-mono text-[9.5px] border transition-all ${
              showReticle
                ? 'bg-indigo-50 dark:bg-indigo-950/50 border-indigo-300 dark:border-indigo-800 text-indigo-700 dark:text-indigo-300 font-semibold'
                : 'bg-slate-50 dark:bg-slate-800/40 border-slate-200 dark:border-slate-800 text-slate-400 line-through'
            }`}
          >
            HUD
          </button>
        </div>
      </div>

      {/* 4. COMPACT FOOTER PIPELINE STATUS */}
      <div className="flex items-center justify-between px-3 py-2 rounded-xl bg-slate-50 dark:bg-slate-800/50 border border-slate-100 dark:border-slate-800 text-[9.5px] font-mono text-slate-500 dark:text-slate-400">
        <span className="flex items-center gap-1.5">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-500"></span>
          <span className="font-bold text-slate-700 dark:text-slate-200">TensorRT FP16</span>
        </span>
        <span>LAT: <strong className="text-emerald-600 dark:text-emerald-400">11.4 ms</strong></span>
        <span>CONF: <strong className="text-sky-600 dark:text-sky-400">{(confThreshold * 100).toFixed(0)}%</strong></span>
        <span>DEV: <strong className="text-slate-700 dark:text-slate-300">Jetson Orin</strong></span>
      </div>
    </div>
  );
}
