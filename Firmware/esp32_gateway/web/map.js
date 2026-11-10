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
function createMapPanel(doc,request,clock,delay,cancelDelay,Controller) {
  const canvas=doc.getElementById("map-canvas"),ctx=canvas.getContext("2d");
  let last=null,received=0,offline=false,started=false;
  const text=(id,value)=>{const node=doc.getElementById(id);if(node.textContent!==value)node.textContent=value;};
  function render() {
    if(!last) { text("map-state","Sin datos de localización");text("map-calibration","Sin información de calibración");return; }
    const p=last.pose,g=last.grid,age=p.age_ms===null?null:p.age_ms+Math.max(0,Math.floor(clock()-received));
    const old=offline||!last.connected||p.stale||age===null||age>500;
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
    if(!ctx) return;
    const desiredHeight=Math.round(canvas.width*g.height/g.width);
    if(canvas.height!==desiredHeight)canvas.height=desiredHeight;
    const w=canvas.width,h=canvas.height;
    ctx.clearRect(0,0,w,h);
    const colors=["#dde3eb","#ffffff","#b94d38"];
    for(let row=0;row<g.height;row++)for(let col=0;col<g.width;col++) {
      ctx.fillStyle=colors[Number(g.cells[row*g.width+col])];
      ctx.fillRect(col*w/g.width,h-(row+1)*h/g.height,w/g.width+0.2,h/g.height+0.2);
    }
    ctx.strokeStyle="#16798a";ctx.lineWidth=2;ctx.beginPath();
    last.trajectory_mm.forEach(([x,y],i)=>{const q=mapToCanvas(g,x,y,w,h);if(i===0)ctx.moveTo(...q);else ctx.lineTo(...q);});ctx.stroke();
    if(p.state==="valid") {
      const [x,y]=mapToCanvas(g,p.x_mm,p.y_mm,w,h);
      ctx.save();ctx.globalAlpha=old?0.45:1;ctx.translate(x,y);ctx.rotate(-p.theta_mrad/1000);
      ctx.fillStyle="#173964";ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-7,-7);ctx.lineTo(-7,7);ctx.closePath();ctx.fill();ctx.restore();
    }
    text("map-scale",`${g.width*g.resolution_mm/1000} × ${g.height*g.resolution_mm/1000} m · celdas de ${g.resolution_mm} mm · +X derecha, +Y arriba`);
  }
  async function poll() {
    const controller=new Controller(),timer=delay(()=>controller.abort(),1500);
    try {
      const response=await request("/api/map",{cache:"no-store",signal:controller.signal});
      if(!response.ok)throw new Error("HTTP");
      const data=await response.json();if(!mapValidate(data))throw new Error("Formato");
      last=data;received=clock();offline=false;
    } catch { offline=true; }
    finally {cancelDelay(timer);render();delay(poll,500);}
  }
  return {start(){if(!started){started=true;poll();}},render};
}
if(typeof document!=="undefined") createMapPanel(document,fetch,()=>performance.now(),setTimeout,clearTimeout,AbortController).start();
