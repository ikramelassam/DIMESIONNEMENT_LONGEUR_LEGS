const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
const relax={'aucune':null,'marge pied/châssis→−∞':p=>{p.margins.chassis_clearance=-1000;},'pieds/pattes 30→0':p=>{p.margins.feet_clearance=0;},'portée 5→0':p=>{p.margins.reach_margin=0;},'θ3 libre':p=>{p.margins.theta3_min=-180;p.margins.theta3_max=360;},'débord C2 libre':p=>{p.chassis.overhang_C2=1e4;}};
const XI=[20,27.5,30,37.5,40,45,50,52.5,60,77.32];
console.log('cellules atteignables (sur 121), phase avant_RL, par relaxation et ξ');
console.log('relaxation'.padEnd(26),XI.map(x=>String(x).padStart(6)).join(''));
for(const [k,mod] of Object.entries(relax)){
  let line=k.padEnd(26);
  for(const xi of XI){const p=JSON.parse(JSON.stringify(cfg));p.chassis.mount='dedans';if(mod)mod(p);const ix=n=>p.servos.findIndex(s=>s.name===n);
    const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});line+=String(r.geoOk[0].filter(Boolean).length).padStart(6);}
  console.log(line);}
// causes bit par bit pour xi=20,30,45,77
for(const xi of [20,30,45,77.32]){const p=JSON.parse(JSON.stringify(cfg));p.chassis.mount='dedans';const ix=n=>p.servos.findIndex(s=>s.name===n);
  const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});
  const bits=new Array(7).fill(0);for(const w of r.why[0])for(let b=0;b<7;b++)if(w&(1<<b))bits[b]++;
  console.log('xi',xi,M.WHY.map((n,i)=>n+'='+bits[i]).join(' ; '));}
