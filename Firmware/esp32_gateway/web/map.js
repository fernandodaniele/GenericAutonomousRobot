"use strict";
function mapValidate(data) {
  if (!data || typeof data.connected !== "boolean" || typeof data.calibrated !== "boolean" ||
      !["incomplete","provisional","calibrated"].includes(data.calibration) ||
      data.calibrated !== (data.calibration==="calibrated") ||
      !Number.isInteger(data.epoch) || data.epoch < 0 || data.epoch > 4294967295) return false;
  const p=data.pose,g=data.grid;
  if (!p || !g || !["no_sample","uncalibrated","valid","invalid"].includes(p.state) ||
      typeof p.valid!=="boolean" || typeof p.stale!=="boolean" ||
      (p.valid && (!data.connected || data.calibration==="incomplete" || p.stale || p.state!=="valid"))) return false;
  if(p.state==="uncalibrated"&&data.calibration!=="incomplete") return false;
  if (!Number.isInteger(g.width)||g.width<8||g.width>128||!Number.isInteger(g.height)||g.height<8||g.height>128||
      g.width*g.height>10000||!Number.isInteger(g.resolution_mm)||g.resolution_mm<20||g.resolution_mm>200||
      !Number.isInteger(g.origin_x_mm)||!Number.isInteger(g.origin_y_mm)||
      !Number.isInteger(g.revision)||g.revision<0||g.revision>4294967295||
      typeof g.cells!=="string"||g.cells.length!==g.width*g.height||/[^012]/.test(g.cells)) return false;
  if (p.tick!==null && (!Number.isInteger(p.tick)||p.tick<0||p.tick>4294967295)) return false;
  if (p.age_ms!==null && (!Number.isSafeInteger(p.age_ms)||p.age_ms<0)) return false;
  if(p.valid&&(p.age_ms===null||p.age_ms>500)) return false;
  if (![p.left_counts,p.right_counts].every(n=>n===null||(typeof n==="string"&&/^-?[0-9]{1,19}$/.test(n)))) return false;
  const metrics=[p.x_mm,p.y_mm,p.theta_mrad,p.increment_mm,p.increment_mrad];
  if (p.state==="valid") {
    if(data.calibration==="incomplete" || !metrics.every(Number.isInteger) || Math.abs(p.x_mm)>100000 ||
        Math.abs(p.y_mm)>100000 || Math.abs(p.theta_mrad)>3142 || Math.abs(p.increment_mm)>250 || Math.abs(p.increment_mrad)>3142) return false;
  } else if(!metrics.every(v=>v===null)) return false;
  if (!Array.isArray(data.trajectory_mm)||data.trajectory_mm.length>128||
      !data.trajectory_mm.every(v=>Array.isArray(v)&&v.length===2&&v.every(n=>Number.isInteger(n)&&Math.abs(n)<=100000))) return false;
  return Array.isArray(data.rays)&&data.rays.length===2&&data.rays.every(r=>r&&Number.isInteger(r.state)&&r.state>=0&&r.state<=3&&Number.isSafeInteger(r.age_ms)&&r.age_ms>=0);
}
function mapToCanvas(grid,x,y,width,height) {
  return [(x-grid.origin_x_mm)/(grid.width*grid.resolution_mm)*width,
    height-(y-grid.origin_y_mm)/(grid.height*grid.resolution_mm)*height];
}
// Viewport only: never changes the grid, pose or trajectory received from STM32.
function mapView(grid, zoom, center) {
  return {...grid, width:grid.width/zoom, height:grid.height/zoom,
    origin_x_mm:center[0]-grid.width*grid.resolution_mm/(2*zoom),
    origin_y_mm:center[1]-grid.height*grid.resolution_mm/(2*zoom)};
}
function createMapPanel(doc,request,clock,delay,cancelDelay,Controller) {
  const canvas=doc.getElementById("map-canvas"),ctx=canvas.getContext("2d");
  let last=null,received=0,offline=false,started=false,zoom=1,center=null,drawKey="",showOccupancy=true;
  const node=id=>doc.getElementById(id);
  const text=(id,value)=>{const e=node(id);if(e&&e.textContent!==value)e.textContent=value;};
  const button=(id,fn)=>node(id)?.addEventListener?.("click",fn);
  const occupancy=node("map-occupancy");
  if(occupancy) {
    occupancy.checked=true;
    occupancy.addEventListener?.("change",()=>{showOccupancy=occupancy.checked;drawKey="";render();});
  }
  const gridCenter=g=>[g.origin_x_mm+g.width*g.resolution_mm/2,g.origin_y_mm+g.height*g.resolution_mm/2];
  function changeZoom(factor) { if(last) {zoom=Math.max(1,Math.min(8,zoom*factor));drawKey="";render();} }
  button("map-zoom-in",()=>changeZoom(2));
  button("map-zoom-out",()=>changeZoom(.5));
  button("map-fit",()=>{zoom=1;center=last?gridCenter(last.grid):null;drawKey="";render();});
  button("map-center",()=>{if(last?.pose.state==="valid"){center=[last.pose.x_mm,last.pose.y_mm];drawKey="";render();}});
  function render() {
    if(!last) { text("map-state","Sin datos de localización");text("map-calibration","Sin información de calibración");
      for(const id of ["map-zoom-in","map-zoom-out","map-center","map-fit"])if(node(id))node(id).disabled=true;
      return;
    }
    const p=last.pose,g=last.grid,age=p.age_ms===null?null:p.age_ms+Math.max(0,Math.floor(clock()-received));
    const status=typeof lastStatus==="undefined"?null:lastStatus;
    const statusLost=status&&(!status.valid||!status.connected);
    const old=offline||statusLost||!last.connected||p.stale||age===null||age>500;
    const approximate=last.calibration==="provisional";
    text("map-calibration",last.calibration==="incomplete"?"Parámetros incompletos: falta CPR o geometría válida":
      approximate?"Geometría provisional: estimación aproximada, no calibrada físicamente":"Parámetros físicamente calibrados");
    text("map-state",p.state==="no_sample"?"Sin muestra":p.state==="invalid"?"Odometría inválida: requiere revisión y reinicio":
      last.calibration==="incomplete"?"No calibrada: parámetros incompletos":
      (old?"Información histórica":"Odometría relativa actualizada")+(approximate?" · Estimación aproximada":""));
    text("map-age",age===null?"Sin muestra":`${age} ms${old?" · Sin vigencia actual":""}`);
    text("map-pose",p.state==="valid"?`X ${(p.x_mm/1000).toFixed(3)} m · Y ${(p.y_mm/1000).toFixed(3)} m · θ ${(p.theta_mrad/1000).toFixed(3)} rad`:"Posición métrica no disponible");
    if(p.state==="valid" && (p.x_mm<g.origin_x_mm || p.x_mm>=g.origin_x_mm+g.width*g.resolution_mm ||
        p.y_mm<g.origin_y_mm || p.y_mm>=g.origin_y_mm+g.height*g.resolution_mm))
      text("map-state","Pose fuera de la grilla: se conserva el mapa acotado"+(old?" · Información histórica":""));
    text("map-counts",p.state==="no_sample"?"Sin conteos":`Izquierdo ${p.left_counts} · Derecho ${p.right_counts}`);
    const names=["Sin muestra","Adquisición fallida","Sin pose asociada","Eco asociado"];
    text("map-rays",last.rays.map((r,i)=>`${i?"Trasero":"Frontal"}: ${names[r.state]}${r.state&&r.age_ms>300?" (histórico)":""}`).join(" · "));
    text("map-scale",`${g.width*g.resolution_mm/1000} × ${g.height*g.resolution_mm/1000} m · celdas de ${g.resolution_mm} mm · +X derecha, +Y arriba`);
    text("map-zoom",zoom+"×");
    for(const [id,disabled] of [["map-zoom-in",zoom===8],["map-zoom-out",zoom===1],["map-center",p.state!=="valid"],["map-fit",false]])
      if(node(id))node(id).disabled=disabled;
    if(!ctx)return;
    const desiredHeight=Math.round(canvas.width*g.height/g.width);
    if(canvas.height!==desiredHeight)canvas.height=desiredHeight;
    // Age text refreshes independently; avoid repainting all 6400 cells at 10 Hz.
    const key=[received,old,canvas.width,canvas.height,canvas.clientWidth,zoom,showOccupancy,...(center||[])].join(":");
    if(key===drawKey)return;drawKey=key;
    const w=canvas.width,h=canvas.height,view=mapView(g,zoom,center||gridCenter(g));
    const point=(x,y)=>mapToCanvas(view,x,y,w,h),cell=w/g.width*zoom;
    ctx.clearRect(0,0,w,h);ctx.fillStyle="#172534";ctx.fillRect(0,0,w,h);
    ctx.save();ctx.globalAlpha=old ? .55 : 1;
    const colors=["#233446","#536f7a","rgba(202,180,149,0.28)"];
    for(let row=0;row<g.height;row++)for(let col=0;col<g.width;col++) {
      const [x,y]=point(g.origin_x_mm+col*g.resolution_mm,g.origin_y_mm+(row+1)*g.resolution_mm);
      if(x+cell<0||y+cell<0||x>w||y>h)continue;
      const state=Number(g.cells[row*g.width+col]);
      // Hidden possible occupancy returns to unknown visually, never to free.
      ctx.fillStyle=colors[state===1?1:0];ctx.fillRect(x,y,cell+.2,cell+.2);
      if(state===2 && showOccupancy) {ctx.fillStyle=colors[2];ctx.fillRect(x,y,cell+.2,cell+.2);}
    }
    // Metric guides are a visual reference, never inferred free-space evidence.
    ctx.strokeStyle="#99c5df26";ctx.lineWidth=1;
    const step=zoom>=4?250:500;
    ctx.beginPath();
    for(let x=Math.ceil(view.origin_x_mm/step)*step;x<=view.origin_x_mm+view.width*g.resolution_mm;x+=step) {
      const [px]=point(x,0);ctx.moveTo(px,0);ctx.lineTo(px,h);
    }
    for(let y=Math.ceil(view.origin_y_mm/step)*step;y<=view.origin_y_mm+view.height*g.resolution_mm;y+=step) {
      const [,py]=point(0,y);ctx.moveTo(0,py);ctx.lineTo(w,py);
    }ctx.stroke();
    const [gx,gy]=point(g.origin_x_mm,g.origin_y_mm+g.height*g.resolution_mm);
    ctx.strokeStyle="#7394ac";ctx.strokeRect(gx,gy,g.width*cell,g.height*cell);
    ctx.lineJoin="round";ctx.lineCap="round";ctx.beginPath();
    last.trajectory_mm.forEach(([x,y],i)=>{const q=point(x,y);if(i===0)ctx.moveTo(...q);else ctx.lineTo(...q);});
    ctx.strokeStyle="#0b1825";ctx.lineWidth=5.5;ctx.stroke();
    ctx.strokeStyle="#b4e5ff";ctx.lineWidth=3.5;ctx.stroke();
    if(p.state==="valid") {
      const [x,y]=point(p.x_mm,p.y_mm);
      ctx.save();ctx.translate(x,y);ctx.rotate(-p.theta_mrad/1000);
      const markerScale=Math.max(1,w/(canvas.clientWidth||w));
      ctx.scale(markerScale,markerScale); // Keep the symbol legible on a narrow phone canvas.
      // Recognizable body + wheels + heading; marker is symbolic, not a footprint.
      ctx.fillStyle="#8edaff26";ctx.beginPath();ctx.arc(0,0,24,0,Math.PI*2);ctx.fill();
      ctx.fillStyle="#b8ddf3";ctx.fillRect(-12,-16,19,5);ctx.fillRect(-12,11,19,5);
      ctx.fillStyle="#80c5ff";ctx.fillRect(-13,-10,25,20);
      ctx.strokeStyle="#081827";ctx.lineWidth=2;ctx.strokeRect(-13,-10,25,20);
      ctx.strokeStyle="#f5fbff";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(-3,0);ctx.lineTo(26,0);ctx.lineTo(18,-6);ctx.moveTo(26,0);ctx.lineTo(18,6);ctx.stroke();ctx.restore();
    }
    ctx.restore();
    // Pick a readable metric bar that also fits smaller configurable grids.
    const visibleMm=view.width*g.resolution_mm;
    const scaleMm=[1,2,5,10,20,50,100,250,500,1000,2000,4000].filter(n=>n<=visibleMm*.3).pop()||1;
    const bar=scaleMm/visibleMm*w;
    ctx.fillStyle="#101923ed";ctx.fillRect(12,h-47,bar+24,35);
    ctx.strokeStyle="#d5e9f7";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(24,h-25);ctx.lineTo(24+bar,h-25);ctx.stroke();
    ctx.fillStyle="#d5e9f7";ctx.font="12px sans-serif";ctx.fillText(scaleMm/1000+" m",24,h-32);
  }
  async function poll() {
    const controller=new Controller(),timer=delay(()=>controller.abort(),1500);
    try {
      const response=await request("/api/map",{cache:"no-store",signal:controller.signal});
      if(!response.ok)throw new Error("HTTP");
      const data=await response.json();if(!mapValidate(data))throw new Error("Formato");
      if(!last||last.epoch!==data.epoch||last.grid.width!==data.grid.width||last.grid.height!==data.grid.height||
          last.grid.resolution_mm!==data.grid.resolution_mm||last.grid.origin_x_mm!==data.grid.origin_x_mm||
          last.grid.origin_y_mm!==data.grid.origin_y_mm) {zoom=1;center=gridCenter(data.grid);}
      last=data;received=clock();offline=false;drawKey="";
    } catch {offline=true;}
    finally {cancelDelay(timer);render();delay(poll,500);}
  }
  function refresh() {render();delay(refresh,100);}
  return {start(){if(!started){started=true;poll();refresh();}},render};
}
if(typeof document!=="undefined") createMapPanel(document,fetch,()=>performance.now(),setTimeout,clearTimeout,AbortController).start();
