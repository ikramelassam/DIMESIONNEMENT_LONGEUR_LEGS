// Audit de coherence cote simulateur : rejoue chaque design de audit/ref_audit.json avec sim/model.js
// (mode exact) et compare a la reponse du programme Python. Usage : node audit/audit_run.js
const fs = require("fs"), path = require("path");
const M = require("../sim/model.js"), C = require("../sim/compare.js");
const cfg = require("./config.json"), refs = require("../sim/ref_fronts.json");
const files = ["dedans_S2", "dedans_S1", "dehors_S2", "dehors_S1"].map((c) => path.join(__dirname, `ref_audit_${c}.json`));
const A = files.every(fs.existsSync) ? [].concat(...files.map((f) => JSON.parse(fs.readFileSync(f)))) : JSON.parse(fs.readFileSync(path.join(__dirname, "ref_audit.json")));
const rel = (a, b) => Math.abs(a - b) / Math.max(1, Math.abs(b));
const KEYS = { f1_masse_pattes: (s) => s.f1, f2_nesm: (s) => s.f2, f3_couple_C2: (s) => s.f3, tau_C1: (s) => s.tauMaxBest[0], tau_C2: (s) => s.tauMaxBest[1], tau_C3: (s) => s.tauMaxBest[2],
  tau_min_C1: (s) => s.tmin[0], tau_min_C3: (s) => s.tmin[2], span_C1: (s) => s.span[0], span_C2: (s) => s.span[1], span_C3: (s) => s.span[2], M_total: (s) => s.Mc,
  frac_bloc_C1: (s) => s.fracBloc[0], frac_bloc_C2: (s) => s.fracBloc[1], frac_bloc_C3: (s) => s.fracBloc[2] };
const byTag = {}, tot = { designs: 0, pre_agree: 0, pre_diff: 0, viol_err: 0, pairs: 0, adm_agree: 0, adm_diff: 0, both_adm: 0, both_inf: 0, val_err: 0, crit_diff: 0, dom_agree: 0, dom_diff: 0, dom_strict_vs_tol: 0, pre_inf: 0 };
const perKey = {}; const diffs = []; const pairs = [];
const t0 = Date.now();
for (const d of A) {
  const p = JSON.parse(JSON.stringify(cfg)); p.chassis.mount = d.mount;
  const S0 = M.servoList(p), nm = S0.map((s) => s.name);
  const geo = M.geometry(p, d.x), pre = M.prefilter(p, geo);
  tot.designs++;
  const T = (byTag[d.tag.replace(/_(L2|L3|alpha|beta|xi)$/, "")] ||= { designs: 0, pre_ok: 0, pairs: 0, adm: 0, diff: 0 }); T.designs++;
  if (pre.ok === d.pre_ok) tot.pre_agree++; else { tot.pre_diff++; diffs.push(["prefiltre", d.case, d.tag, d.x]); }
  const vmap = { G5_garde_sol: "G5_garde_sol", G3_theta3_repos: "G3_theta3_repos", G6_gabarit: "G6_gabarit", G7_axes_C1: "G7_axes_C1", G10_chassis: "G10_chassis" };
  for (const k of Object.keys(vmap)) tot.viol_err = Math.max(tot.viol_err, rel(pre.viol[k], d.pre_viol[k]));
  if (pre.ok) T.pre_ok++;
  for (const [cn, ent] of Object.entries(d.combos)) {
    tot.pairs++; T.pairs++;
    if (!pre.ok) { tot.pre_inf++; if (ent.adm) { tot.adm_diff++; T.diff++; diffs.push(["adm", d.case, d.tag, cn]); } else tot.adm_agree++; continue; }
    const cb = cn.split("/").map((n) => nm.indexOf(n));
    const r = M.evaluate(p, d.x, d.sc, { mode: "exact", combo: cb }), s = r.servo;
    if (s.admissible === ent.adm) { tot.adm_agree++; if (ent.adm) { tot.both_adm++; T.adm++; } else tot.both_inf++; }
    else { tot.adm_diff++; T.diff++; diffs.push(["adm", d.case, d.tag, cn, s.admissible, ent.adm, s.names]); continue; }
    if (!ent.adm) continue;
    for (const [k, fn] of Object.entries(KEYS)) { const e = rel(fn(s), ent.v[k]); perKey[k] = Math.max(perKey[k] || 0, e); tot.val_err = Math.max(tot.val_err, e); if (e > 1e-6) diffs.push(["val", d.case, d.tag, cn, k, fn(s), ent.v[k]]); }
    if (s.critical !== ent.v.phase_critique) tot.crit_diff++;
    pairs.push([d.case, d.tag.replace(/_(L2|L3|alpha|beta|xi)$/, ""), ent.v.f1_masse_pattes, s.f1, ent.v.f2_nesm, s.f2, ent.v.f3_couple_C2, s.f3]);
    // dominance par rapport au front de reference
    const cmp = C.compare(refs[d.case], [s.f1, s.f2, s.f3], d.x, null);
    const pf = [ent.v.f1_masse_pattes, ent.v.f2_nesm, ent.v.f3_couple_C2], pyDom = refs[d.case].front.filter((q) => C.dom(C.FK.map((k) => q[k]), pf)).length;
    if (cmp.dominators === pyDom) tot.dom_agree++; else { tot.dom_diff++; diffs.push(["dom", d.case, d.tag, cn, cmp.dominators, pyDom, ent.dom]); }
    if (pyDom !== ent.dom) tot.dom_strict_vs_tol++;
  }
}
const out = { sec: Math.round((Date.now() - t0) / 1000), tot, perKey, byTag, nDiffs: diffs.length, diffsSample: diffs.slice(0, 15) };
console.log(JSON.stringify(out, null, 1));
fs.writeFileSync(path.join(__dirname, "audit_result.json"), JSON.stringify(out));
fs.writeFileSync(path.join(__dirname, "audit_pairs.json"), JSON.stringify(pairs));
fs.writeFileSync(path.join(__dirname, "../sim/validation_audit.json"), JSON.stringify({ tot, perKey, nDiffs: diffs.length, designs: A.length, err_obj: Math.max(perKey.f1_masse_pattes, perKey.f2_nesm, perKey.f3_couple_C2) }));
