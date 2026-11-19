// Pipeline JS de bout en bout vs Python sur le MEME echantillon (sans raffinement ni Monte-Carlo).
const O = require("./opt.js"), ref = require("./ref_pipe.json");
(async () => {
  const out = [];
  for (const c of ref.cases) {
    const r = await O.optimize(ref.config, { mount: c.mount, sc: c.sc, allowed: ref.allowed, X: ref.X, refineRounds: 0, nMc: 0 }, O.syncExec);
    const key = (q) => [q.L2, q.L3, q.alpha, q.beta, q.xi].map((v) => +v.toFixed(5)).concat([q.servo_C1, q.servo_C2, q.servo_C3]).join("|");
    const jsP = r.front.map(key), pyP = c.pareto.map((q) => q.join("|"));
    out.push({ cas: `${c.mount} ${c.sc}`, admissibles: [r.res.admissibles, c.adm], front_identique: JSON.stringify(jsP) === JSON.stringify(pyP), front_n: jsP.length,
      coude_L2: [r.res.choix.coude.L2, c.knee], eps_L2: [r.res.choix.eps_contrainte?.L2, c.eps], stats_identiques: JSON.stringify(Object.values(r.res.stats).map((s) => s.apres_raffinement)) === JSON.stringify(Object.values(c.stats).map((s) => s.apres_raffinement)) });
  }
  console.log(JSON.stringify(out, null, 1));
  require("fs").writeFileSync(__dirname + "/validation_pipe.json", JSON.stringify(out));
})();
