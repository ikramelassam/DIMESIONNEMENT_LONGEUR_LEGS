const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
const out=[];
for(let xi=0;xi<=90.001;xi+=2.5){
  const row={xi};
  for(const sc of ['S2','S1']){
    const p=JSON.parse(JSON.stringify(cfg)); p.chassis.mount='dedans'; const ix=n=>p.servos.findIndex(s=>s.name===n);
    for(const [tag,names] of [['sss',['S3306','S3306','S3306']],['std',['SG90','S3306','S-U400']]]){
      const r=M.evaluate(p,{...base,xi},sc,{mode:'exact',combo:names.map(ix)}); const s=r.servo;
      row[sc+'_'+tag]={ph:s.phases.filter(q=>q.ok).length,f2:Number.isFinite(s.f2)?s.f2:null,f3:Number.isFinite(s.f3)&&s.f3<1e3?s.f3:null,adm:s.admissible,t:s.tmin.map(v=>Number.isFinite(v)&&v<1e3?v:null)};
      if(tag==='sss'){ // cellules géométriquement faisables de avant_RL
        row[sc+'_geo_avantRL']=r.geoOk[0].filter(Boolean).length;}
    }
  }
  out.push(row);
}
require('fs').writeFileSync('/home/claude/gr2_dim/diag/sweep_xi.json',JSON.stringify(out));
console.log('xi | S2 sss: ph f2 tauC3 | S2 std: ph | S1 sss: ph f2 | geo cells avant_RL');
for(const r of out) console.log(String(r.xi).padStart(5),'|',r.S2_sss.ph,(r.S2_sss.f2??NaN).toFixed(1).padStart(5),(r.S2_sss.t[2]??NaN).toFixed(2),'|',r.S2_std.ph,'|',r.S1_sss.ph,(r.S1_sss.f2??NaN).toFixed(1).padStart(5),'|',r.S2_geo_avantRL);
