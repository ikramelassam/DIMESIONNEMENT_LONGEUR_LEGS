const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76}; const out={};
for(const n of [11,21,41]){ for(const xi of [45,77.32]){const p=JSON.parse(JSON.stringify(cfg));p.chassis.mount='dedans';p.task.sway_n=n;const ix=nm=>p.servos.findIndex(s=>s.name===nm);
  const t=Date.now();const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});const s=r.servo;
  out[n+'_'+xi]={f2:s.f2,f3:s.f3,tmin:s.tmin,crit:s.critical,ph:s.phases.filter(q=>q.ok).length};
  console.log('grille',n+'x'+n,'(pas',(120/(n-1)).toFixed(1),'mm) xi',xi,'phases',s.phases.filter(q=>q.ok).length,'f2',s.f2.toFixed(2),'critique',s.critical,'tmin',s.tmin.map(v=>v.toFixed(3)).join('/'),((Date.now()-t)/1000).toFixed(1)+'s');}}
require('fs').writeFileSync('/home/claude/gr2_dim/diag/grid.json',JSON.stringify(out));
