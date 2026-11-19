// Anatomie d'une phase a 3 pieds : avant_RL, design S2 'dedans' fixe, xi = 45 vs 77.32, servos S3306 x3 (couple non limitant)
const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const NAMES=M.LEG_NAMES;
function mk(){const p=JSON.parse(JSON.stringify(cfg)); p.chassis.mount='dedans'; return p;}
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
const out={};
for(const xi of [45,77.32]){
  const p=mk(); const ix=n=>p.servos.findIndex(s=>s.name===n); const combo=['S3306','S3306','S3306'].map(ix);
  const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo});
  const pi=0, NS=r.sw.length, ph=r.servo.phases[pi];
  const rows=[]; let cnt={geo:0,stab:0,torque:0,ok:0};
  for(let s=0;s<NS;s++){
    const geoOk=r.geoOk[pi][s]; const nes=r.nesm[pi][s]; const ssm=r.ssmc[pi][s];
    const tau=r.k[pi][s].map(k=>r.servo.Mc*M.G*k/1000);
    const stab=geoOk&&nes>=0&&ssm>=0;
    const tq=tau[0]<=r.servo.Cu[0]&&tau[1]<=r.servo.Cu[1]&&tau[2]<=r.servo.Cu[2];
    if(!geoOk)cnt.geo++; else if(!stab)cnt.stab++; else if(!tq)cnt.torque++; else cnt.ok++;
    rows.push({s,sw:r.sw[s],geoOk,nes,ssm,tau,why:r.why[pi][s]});
  }
  // etat par etat pour la meilleure cellule
  const best=ph.best; const det=[];
  r.detail[pi].forEach((d,j)=>{const v=M.stateView(p,r,pi,j,best); const pm=M.polygonMargins(v.feet,v.Gc,v.hxy,v.zmax);
    det.push({label:d.label,ground:v.order.map(l=>NAMES[l]),hxy:v.hxy,hz:v.hz,Gc:v.Gc,edges:pm.edges,nesm:pm.nesm,ssm:pm.ssm,lam:v.lamLeg,legs:Object.fromEntries(Object.entries(v.legs).map(([l,ps])=>[NAMES[l],{D:ps.D,rh:ps.rh,th1:ps.th1,th2:ps.th2,th3:ps.th3,beta:ps.be*180/Math.PI,reach:ps.reach}]))});});
  out[xi]={cnt,best,sw:ph.sway,nesm:ph.nesm,tau:ph.tau,Mc:r.servo.Mc,det,rows};
  console.log('\n===== xi',xi,'avant_RL | cellules (géo/stab/couple/ok)',Object.values(cnt).join('/'),'| meilleure cellule sway',ph.sway,'NESM',ph.nesm.toFixed(2),'tau',ph.tau.map(v=>v.toFixed(3)).join('/'),'Mc',r.servo.Mc.toFixed(3));
  for(const d of det){console.log(' état:',d.label,'| pieds',d.ground.join(','),'| demi-boîte CdG hxy',d.hxy.map(v=>v.toFixed(1)).join(','),'hz',d.hz.toFixed(1),'| NESM état',d.nesm.toFixed(2),'| SSM',d.ssm.toFixed(2),'| marges arêtes',d.edges.map(v=>v.toFixed(1)).join(', '));
    console.log('    λmax:',Object.entries(d.lam).map(([l,v])=>NAMES[l]+'='+v.toFixed(2)).join(' '),'| D:',Object.entries(d.legs).map(([n,q])=>n+'='+q.D.toFixed(0)).join(' '));}
}
require('fs').writeFileSync('/home/claude/gr2_dim/diag/diag1.json',JSON.stringify(out));
