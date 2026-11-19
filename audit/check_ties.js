// Les ecarts residuels de l'audit portent sur des grandeurs lues "a la position retenue" (tau_C1, tau_C3, course).
// Hypothese : plusieurs positions du corps ont le meme NESM maximal (ex aequo, symetrie) ; chaque programme en retient une.
// Test : la valeur Python est-elle atteinte par un autre ex aequo du simulateur ?
const fs = require("fs"), path = require("path"), M = require("../sim/model.js");
const cfg = require("./config.json"), G = 9.81;
const files = ["dedans_S2", "dedans_S1", "dehors_S2", "dehors_S1"].map((c) => JSON.parse(fs.readFileSync(path.join(__dirname, `ref_audit_${c}.json`))));
const A = [].concat(...files), res = JSON.parse(fs.readFileSync(path.join(__dirname, "audit_result.json")));
const KEYS = { tau_C1: 0, tau_C2: 1, tau_C3: 2 };
let n = 0, explained = 0; const out = [];
for (const d of A) {
  const p = JSON.parse(JSON.stringify(cfg)); p.chassis.mount = d.mount; const S0 = M.servoList(p), nm = S0.map((s) => s.name);
  for (const [cn, ent] of Object.entries(d.combos)) {
    if (!ent.adm) continue;
    const cb = cn.split("/").map((q) => nm.indexOf(q)); const r = M.evaluate(p, d.x, d.sc, { mode: "exact", combo: cb }), s = r.servo;
    const bad = Object.entries(KEYS).filter(([k, j]) => Math.abs(s.tauMaxBest[j] - ent.v[k]) / Math.max(1, Math.abs(ent.v[k])) > 1e-6);
    const badSpan = [0, 1, 2].filter((j) => Math.abs(s.span[j] - ent.v["span_C" + (j + 1)]) / Math.max(1, ent.v["span_C" + (j + 1)]) > 1e-6);
    if (!bad.length && !badSpan.length) continue;
    n++;
    // pour chaque phase : positions ex aequo (NESM a 1e-9 pres du meilleur, parmi les faisables) et leurs couples
    const Cu = s.Cu, tie = s.phases.map((ph, pi) => {
      if (!ph.ok) return null;
      const out = [];
      for (let q = 0; q < r.sw.length; q++) {
        const tau = r.k[pi][q].map((kj) => (s.Mc * G * kj) / 1000);
        const stab = r.geoOk[pi][q] && r.nesm[pi][q] >= 0 && r.ssmc[pi][q] >= 0;
        const swC2 = r.swA2[pi][q] + r.swB2[pi][q] * S0[cb[2]].mass;
        const tq = tau[0] <= Cu[0] && tau[1] <= Cu[1] && tau[2] <= Cu[2] && swC2 <= Cu[1] && r.swA3[pi][q] <= Cu[2];
        if (stab && tq && Math.abs(r.nesm[pi][q] - ph.nesm) <= 1e-9 * Math.max(1, ph.nesm)) out.push(tau);
      }
      return out;
    });
    const ok = bad.every(([k, j]) => {
      const vals = tie.filter(Boolean).map((t) => t.map((x) => x[j]));
      const lo = Math.max(...vals.map((v) => Math.min(...v))), hi = Math.max(...vals.map((v) => Math.max(...v)));
      return ent.v[k] >= lo - 1e-9 && ent.v[k] <= hi + 1e-9;
    });
    if (ok) explained++;
    out.push({ case: d.case, tag: d.tag, combo: cn, bad: bad.map(([k]) => k), badSpan, nties: tie.filter(Boolean).map((t) => t.length).join(","), explained: ok });
  }
}
console.log(JSON.stringify({ cases_with_diff: n, explained_by_ties: explained, details: out }, null, 1));
fs.writeFileSync(path.join(__dirname, "ties_result.json"), JSON.stringify({ cases_with_diff: n, explained_by_ties: explained, details: out }));
const vp = path.join(__dirname, "../sim/validation_audit.json"), v = JSON.parse(fs.readFileSync(vp)); v.ties_cases = n; v.ties_explained = explained; fs.writeFileSync(vp, JSON.stringify(v));
